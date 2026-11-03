"""Traffic-light interlock. No UI, transport, wall clock, or hardware I/O."""
import re
import time
from dataclasses import dataclass


def target_color(text):
    """Return R/Y/G/? or raise ValueError for a malformed target record.

    Extended diagnostic fields are deliberately not control inputs. The old
    five-field record remains supported, including numeric light types 4/5/6.
    """
    fields = text.split(",")
    if len(fields) < 5 or fields[0] != "$T":
        raise ValueError("目标报文字段不完整")
    if not re.fullmatch(r"[0-3]", fields[1]):
        raise ValueError("目标 ID 无效")
    if not all(re.fullmatch(r"[0-9a-fA-F]{3}", v) for v in fields[3:5]):
        raise ValueError("目标坐标格式无效")
    x, y = (int(v, 16) for v in fields[3:5])
    if not (0 <= x < 1280 and 0 <= y < 720):
        raise ValueError("目标坐标越界")
    return {"4": "R", "5": "Y", "6": "G"}.get(fields[2], fields[2])


@dataclass(frozen=True)
class Intent:
    command: str
    session: int
    reason: str
    valid_until: float = None


class AutoInterlock:
    START_TIMEOUT = 2.0
    TARGET_TIMEOUT = 1.0
    MAX_AGE = 0.5
    GREEN_MIN = 0.2
    GREEN_MAX = 0.7
    REFRESH = 0.1
    STATES = {"OFF": "手动模式", "ARMING": "启动检查", "WAIT_GREEN": "等待绿灯",
              "RUNNING": "自动前进", "RECOVERING": "停车待恢复"}

    def __init__(self, clock=None):
        self.clock = clock or time.monotonic
        self.state = "OFF"
        self.reason = "自动联动未开启"
        self.session = 0
        self.generation = None
        self.started = 0.0
        self.last_target = None
        self.first_green = None
        self.green_after = 0.0
        self.stop_id = None
        self.stop_confirmed = False
        self.lp_confirmed = False
        self.next_forward = 0.0
        self.links_ready = False
        self.recovery_retry = 0.0
        self._intents = []

    @property
    def active(self):
        return self.state != "OFF"

    @property
    def locked(self):
        return self.state != "OFF"

    @property
    def green_count(self):
        return 2 if self.state == "RUNNING" else int(self.first_green is not None)

    def drain(self):
        result, self._intents = self._intents, []
        return result

    def _stop(self, reason):
        # A stop supersedes any forward intent not yet handed to transport.
        self._intents = [Intent("S", self.session, reason)]
        self.stop_confirmed = False
        self.stop_id = None
        self.first_green = None

    def arm(self, generation):
        self.session += 1
        self.generation = generation
        self.started = self.clock()
        self.links_ready = True
        self.state = "ARMING"
        self.reason = "等待停车写入与新的 LP 全速回传"
        self.last_target = None
        self.lp_confirmed = False
        self._stop(self.reason)
        self._intents.append(Intent("L0", self.session, "请求 FPGA 全速识别"))

    def disarm(self, reason="用户关闭自动联动"):
        self.session += 1
        self.state = "OFF"
        self.reason = reason
        self._stop(reason)

    def pause(self, reason, need_lp=False, disconnected=False):
        """Stop without disarming; only an explicit user stop cancels auto mode.

        Repeated errors while already stopped retain the outstanding stop ID,
        rather than continually replacing it and preventing acknowledgment.
        """
        if not self.active:
            return
        already_stopped = (self.state == "RECOVERING" or
                           (self.state == "WAIT_GREEN" and self.last_target is None and
                            self.first_green is None))
        if disconnected:
            self.links_ready = False
        if need_lp or disconnected:
            self.lp_confirmed = False
        self.state = "RECOVERING" if not (self.lp_confirmed and self.links_ready) else "WAIT_GREEN"
        self.reason = reason
        self.last_target = None
        if not already_stopped:
            self._stop(reason)
            self.recovery_retry = self.clock() + 1.0
            if need_lp and self.links_ready:
                self._intents.append(Intent("L0", self.session, "恢复全速识别"))

    def stop_requested(self, request_id, session):
        if self.active and session == self.session:
            self.stop_id = request_id

    def written(self, command, request_id, session):
        self.tick()
        if not self.active or session != self.session:
            return
        if command == "S" and request_id is not None and request_id == self.stop_id:
            self.stop_confirmed = True
            self.green_after = self.clock()
            self.first_green = None
            self._finish_start()

    def _finish_start(self):
        if (self.state in ("ARMING", "RECOVERING") and self.links_ready and
                self.stop_confirmed and self.lp_confirmed):
            self.state = "WAIT_GREEN"
            self.reason = "检查通过，等待两次新绿灯"
            self.last_target = self.clock()
            self.green_after = self.clock()

    def _eligible(self, received_at, generation):
        if (not self.active or not self.links_ready or generation != self.generation or
                received_at < self.started):
            return False
        age = self.clock() - received_at
        if age < -1e-6 or age > self.MAX_AGE + 1e-9:
            self.pause("报文积压或接收时间异常，停车等待新绿灯")
            return False
        return True

    def lp(self, sleeping, received_at, generation):
        self.tick()
        if not self._eligible(received_at, generation):
            return
        if sleeping:
            self.pause("FPGA 低功耗，停车等待全速恢复", need_lp=True)
        else:
            self.lp_confirmed = True
            self._finish_start()

    def target(self, color, received_at, generation):
        self.tick()
        if not self._eligible(received_at, generation):
            return
        if color not in ("R", "Y", "G"):
            self.pause("未知或坏目标报文，停车等待新绿灯")
            return
        if self.state in ("ARMING", "RECOVERING"):
            return
        if self.last_target is not None and received_at < self.last_target:
            return
        self.last_target = received_at
        if color in ("R", "Y"):
            wait_reason = "红灯等待" if color == "R" else "黄灯等待"
            # Repeated red/yellow reports must not replace the pending stop ID.
            if self.state == "RUNNING" or self.first_green is not None or self.reason != wait_reason:
                self._stop("红灯停车" if color == "R" else "黄灯停车")
            self.state = "WAIT_GREEN"
            self.reason = wait_reason
            self.first_green = None
            return
        if self.state == "RUNNING":
            return
        if not self.stop_confirmed or received_at < self.green_after:
            return
        if self.first_green is None:
            self.first_green = received_at
        else:
            gap = received_at - self.first_green
            if gap > self.GREEN_MAX + 1e-9:
                self.first_green = received_at
            elif gap >= self.GREEN_MIN - 1e-9:
                self.state = "RUNNING"
                self.reason = "两次绿灯确认，前进意图生效（车端未确认）"
                self.next_forward = self.clock()

    def tick(self):
        now = self.clock()
        if self.state == "ARMING" and now - self.started >= self.START_TIMEOUT:
            self.pause("启动检查超时，停车等待条件恢复", need_lp=True)
        elif self.state == "RECOVERING" and self.links_ready and now >= self.recovery_retry:
            # Retry recovery checks slowly, never retry F in this state.
            if not self.stop_confirmed and self.stop_id is None:
                self._stop(self.reason)
            if not self.lp_confirmed:
                self._intents.append(Intent("L0", self.session, "重试全速恢复检查"))
            self.recovery_retry = now + 1.0
        elif self.state in ("WAIT_GREEN", "RUNNING"):
            if self.last_target is not None and now - self.last_target >= self.TARGET_TIMEOUT:
                self.pause("超过 1 秒没有有效目标，停车等待新绿灯")
            elif self.state == "RUNNING" and now >= self.next_forward:
                # Do not leave a long-lived F waiting behind a slow GATT write.
                self._intents = [v for v in self._intents if v.command != "F"]
                self._intents.append(Intent("F", self.session, "绿灯前进刷新",
                                            min(now + 0.15, self.last_target + self.TARGET_TIMEOUT)))
                self.next_forward = now + self.REFRESH
