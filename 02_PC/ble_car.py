"""Windows BLE transport using a hidden Windows PowerShell/WinRT worker.

No third-party BLE package or virtual COM driver is required. A GATT write
result is not an acknowledgment from the STM32 application.
"""
import json
import os
from pathlib import Path
import re
import subprocess
import threading
import time


def known_ble_devices():
    """Read Windows BLE device names/addresses; do not scan, pair or connect."""
    if os.name != "nt":
        return []
    import winreg
    devices = []
    try:
        root = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                             r"SYSTEM\CurrentControlSet\Enum\BTHLE")
    except OSError:
        return devices
    with root:
        for i in range(winreg.QueryInfoKey(root)[0]):
            key_name = winreg.EnumKey(root, i)
            if not re.fullmatch(r"Dev_[0-9a-fA-F]{12}", key_name):
                continue
            with winreg.OpenKey(root, key_name) as device:
                for j in range(winreg.QueryInfoKey(device)[0]):
                    try:
                        with winreg.OpenKey(device, winreg.EnumKey(device, j)) as instance:
                            name = winreg.QueryValueEx(instance, "FriendlyName")[0]
                    except OSError:
                        continue
                    address = ":".join(key_name[4:][n:n+2] for n in range(0, 12, 2)).upper()
                    devices.append((name, address))
    return sorted(set(devices))


class BleCarLink:
    def __init__(self, address, on_event, popen_factory=None, clock=None):
        compact = address.replace(":", "").replace("-", "")
        if not re.fullmatch(r"[0-9a-fA-F]{12}", compact):
            raise ValueError("Invalid BLE address")
        self.address = ":".join(compact[n:n+2] for n in range(0, 12, 2)).upper()
        self.on_event = on_event
        self.ready = False
        self.process = None
        self._popen = popen_factory or subprocess.Popen
        self._lock = threading.Lock()
        self._sequence = 0
        self._closing = False
        self._clock = clock or time.monotonic
        self._pending = None
        self._manual_pending = []
        self._inflight = None
        self._auto_session = None
        self._worker_ready = False
        self._close_sent = False

    def start(self):
        worker = Path(__file__).resolve().with_name("ble_car_worker.ps1")
        system_root = os.environ.get("SystemRoot", r"C:\Windows")
        powershell = os.path.join(system_root, "System32", "WindowsPowerShell", "v1.0", "powershell.exe")
        self.process = self._popen(
            [powershell, "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass",
             "-File", str(worker), "-Address", self.address],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            encoding="utf-8", errors="replace", bufsize=1,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        threading.Thread(target=self._read_events, daemon=True).start()

    def _read_events(self):
        try:
            for line in self.process.stdout:
                try:
                    event = json.loads(line.lstrip("\ufeff"))
                    if not isinstance(event, dict):
                        raise ValueError("Not an event object")
                except (ValueError, TypeError):
                    self.on_event(self, {"kind": "info", "message": line.strip()[:300]})
                    continue
                self._receive_event(event)
        except (OSError, ValueError) as error:
            self.on_event(self, {"kind": "error", "message": str(error)})
        finally:
            with self._lock:
                self.ready = self._worker_ready = False
                self._pending = self._inflight = None
                self._manual_pending.clear()
            self.on_event(self, {"kind": "closed"})

    def _receive_event(self, event):
        """Called by the worker reader; also the injection point for fake I/O tests."""
        event = dict(event)
        event["received_at"] = self._clock()
        with self._lock:
            if event.get("kind") == "ready":
                self._worker_ready = True
                self.ready = not self._closing
            elif event.get("kind") == "error":
                self.ready = self._worker_ready = False
                self._pending = None
                self._manual_pending.clear()
            elif event.get("kind") == "written":
                request = self._inflight
                matched = bool(request and event.get("id") == request["id"] and
                               event.get("command") == request["command"])
                event["matched"] = matched
                if matched:
                    event.update(source=request["source"], session=request["session"], sent_at=request["sent_at"])
                    self._inflight = None
                    if event.get("status") != "Success":
                        self.ready = self._worker_ready = False
                        self._pending = None
                        self._manual_pending.clear()
            # Write at most one request to the pipe. A pending S is always next.
            self._pump_locked()
        self.on_event(self, event)

    def _pump_locked(self):
        if not self._worker_ready or self._inflight is not None:
            return
        request, self._pending = self._pending, None
        if self._manual_pending:
            self._pending = self._manual_pending.pop(0)
        if request and request["command"] != "S":
            expired = request["valid_until"] is not None and self._clock() >= request["valid_until"]
            obsolete = request["source"] == "auto" and request["session"] != self._auto_session
            if expired or obsolete or self._closing:
                request = None
        if request:
            request["sent_at"] = self._clock()
            self._inflight = request
            self._write_request({"action": "write", "command": request["command"],
                                 "id": request["id"], "timeout_ms": request["timeout_ms"]})
            # Callbacks enqueue data; they must not re-enter this link's lock.
            self.on_event(self, {"kind": "dispatched", "command": request["command"],
                                "id": request["id"], "source": request["source"],
                                "session": request["session"], "sent_at": request["sent_at"],
                                "received_at": request["sent_at"]})
        elif self._closing and not self._close_sent:
            self._write_request({"action": "close"})
            self._close_sent = True

    def begin_auto(self, session):
        with self._lock:
            if not self.ready or self._closing:
                raise OSError("BLE is not ready")
            self._auto_session = session
            self._pending = None
            self._manual_pending.clear()

    def cancel_auto(self):
        with self._lock:
            self._auto_session = None
            self._pending = None
            self._manual_pending.clear()

    def service(self):
        """Bound a wedged bridge without submitting more writes behind it."""
        failure = None
        with self._lock:
            request = self._inflight
            if request and self.ready:
                limit = request["timeout_ms"] / 1000.0 + 0.15
                if self._clock() - request["sent_at"] > limit:
                    self.ready = self._worker_ready = False
                    self._pending = None
                    self._manual_pending.clear()
                    failure = {"kind": "error", "message": "BLE 写入结果超时，停车送达未知"}
        if failure:
            self.on_event(self, failure)

    def _write_request(self, value):
        if self.process is None or self.process.poll() is not None:
            raise OSError("BLE worker is not running")
        self.process.stdin.write(json.dumps(value, ensure_ascii=True) + "\n")
        self.process.stdin.flush()

    def send(self, command, source="manual", session=None, valid_until=None, timeout_ms=None):
        if len(command) != 1 or command not in "FBLRS+-0123456789":
            raise ValueError("Unsupported car command")
        with self._lock:
            if not self.ready or self._closing:
                raise OSError("BLE is not ready")
            if source == "auto" and session != self._auto_session:
                raise ValueError("Obsolete automatic session")
            if command != "S" and self._pending and self._pending["command"] == "S":
                return None  # Never replace a queued stop with motion/speed.
            if source == "manual" and len(self._manual_pending) >= 8 and command != "S":
                return None
            self._sequence += 1
            request = {"id": self._sequence, "command": command, "source": source,
                       "session": session, "valid_until": valid_until,
                       "timeout_ms": timeout_ms or (300 if source != "manual" else 3000)}
            if command == "S":
                self._manual_pending.clear()
                self._pending = request
            elif source == "manual" and self._pending:
                # Preserve speed -> motion order. Only automatic repeated F
                # is coalesced; manual commands have a small bounded FIFO.
                self._manual_pending.append(request)
            else:
                self._pending = request
            self._pump_locked()
            return self._sequence

    def close(self, stop=True, timeout=4.0):
        with self._lock:
            self._closing = True
            was_ready = self.ready
            self.ready = False
            self._auto_session = None
            self._pending = None
            self._manual_pending.clear()
            try:
                if was_ready and stop:
                    self._sequence += 1
                    self._pending = {"id": self._sequence, "command": "S", "source": "shutdown",
                                     "session": None, "valid_until": None, "timeout_ms": 300}
                self._pump_locked()
            except (OSError, ValueError):
                pass
        process = self.process
        if process is None:
            return True
        if not was_ready and process.poll() is None:
            # Give a failing worker time for its own best-effort STOP/finally.
            timeout = min(timeout, 1.5)
        clean = True
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            clean = False
            process.terminate()
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1)
        finally:
            if process.stdin:
                process.stdin.close()
        return clean
