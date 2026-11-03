# -*- coding: utf-8 -*-
"""
FPGA 智慧路口 · 上位机监控台（强化版）

基于 pc_monitor_gui.py 增强 5 项（对应"面向实际应用的监控中心"）：
  1. 当前状态大色块 —— 红/黄/绿一眼看到，识别丢失自动变灰
  2. 事件计数大数字 —— 红/黄/绿/其它各多少次
  3. 日志自动存 CSV —— 每条事件带时间戳落盘，可另存
  4. 告警 —— 识别到重点目标时提示音 + 色块闪 + 日志标红（不弹窗、不打断）
  5. 串口友好名称 + 端口占用友好提示 —— 修掉 PermissionError 一长串 traceback

协议（与 FPGA 端一致，未改动）：
   收: $T,<id>,<类型字符 R/Y/G/?>,<x:3位hex>,<y:3位hex>
   发: M<n>          n = 0~5，切换显示档位

批 4 扩展（2026-10-15/16）：
    收: $T,<id>,<类型>,<x3hex>,<y3hex>,<红>,<绿>,<黄>,<圆>,<置信度>,
        <蓝牙命令2hex>,K<跳过帧数4hex>,<R2><r2>,<G2><g2>,<Y2><y2>,<B2><b2>,<b2>,<m2>
    发: C<1字节>   小车遥控（前进 F / 后退 B / 左转 L / 右转 R / 停车 S /
                  速度 '0'~'9' / 加速 '+' / 减速 '-'）—— 必须带 C 前缀，
                  否则裸单字节和阈值命令字母撞车（R=红阈值 vs 右转 …）
        L0 / L1    低功耗按需降频开关（关 / 开）

2026-10-20：增加电脑蓝牙直连小车。板载串口仍接收 FPGA 遥测；
选择“电脑蓝牙直连”后，小车按钮只向独立蓝牙串口发单字节，不带 C 前缀。
2026-10-22：箭头触发一次连续定时转向，停车成功后1秒恢复箭头和灯色。
旧三段参数自动换算为总时长；任务执行期间识别仅更新显示，不排队补执行。
本机 HC05_SLAVE 实测为 BLE，增加 Windows 原生 GATT 入口：FFE0/FFE1。
BLE 不创建 COM 口；需要同时保留 ble_car.py 和 ble_car_worker.ps1。

依赖：tkinter（标准库）、pyserial（已安装）
"""

import csv
import json
import os
import queue
import re
import threading
import time
import sys
from pathlib import Path
from dataclasses import replace
import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog

import serial
import serial.tools.list_ports
from ble_car import BleCarLink, known_ble_devices
from direct_control import DirectControl
from turn_settings import TurnSettings
from vision_protocol import parse as parse_vision, config as vision_config,CONFIGURED,LAMP_VALID,SIGN_VALID,CONFLICT
from auto_interlock import target_color
from detection_control import DetectionControl,PROFILES,parse_status
from camera_protocol import parse_camera

BAUD = 115200
MAX_POINTS = 200

TYPE_NAMES = {
    "0": "红色物体",
    "1": "蓝牌", "2": "黄牌", "3": "绿牌",
    "4": "红灯", "5": "黄灯", "6": "绿灯",
    "R": "红灯", "Y": "黄灯", "G": "绿灯",
    "?": "其它",
}

MODE_NAMES = ["原图", "二值化", "线稿", "卡通", "伪彩", "掩码", "色阶检查"]

# 大色块的颜色（按识别类型）
STATE_COLORS = {
    "红灯": "#d32f2f",
    "黄灯": "#f9a825",
    "绿灯": "#2e7d32",
    "左转": "#1565c0",
    "右转": "#6a1b9a",
    "其它": "#37474f",
}
STATE_IDLE = "#9e9e9e"      # 长时间没数据 → 灰（表示识别丢失）
STATE_TIMEOUT_S = 2.0       # 超过这么久没收到数据就算"丢失"

ALARM_TYPES = {"红灯"}
ALARM_COOLDOWN_S = 3.0      # 同一告警最短间隔（防止刷屏式响铃）

# ---- 批 4：小车命令回读用 ----
# FPGA 把"最后发给小车的那个字节"随 $T 行回发，这里翻成人话
BT_NAMES = {
    0x46: "前进", 0x53: "停车", 0x42: "后退", 0x4C: "左转", 0x52: "右转",
    0x47: "对角弯", 0x49: "对角弯", 0x48: "对角弯", 0x4A: "对角弯",
    0x58: "原地掉头", 0x59: "原地掉头",
    0x2B: "加速 +10%", 0x2D: "减速 -10%",
}

# 阈值回读的 9 个字段顺序（和 thr_regs 的字母一一对应）
THR_ORDER = "RrGgYyBbm"


def bt_name(b):
    """把回读到的命令字节翻成中文。"""
    if b == 0:
        return "（还没发过）"
    if 0x30 <= b <= 0x39:
        return "速度 %d0%%" % (b - 0x30)
    return BT_NAMES.get(b, "未知 0x%02X" % b)


class MonitorApp:
    def __init__(self, root):
        self.root = root
        root.title("V5 运动检测 · 交通识别手动开启")
        root.geometry("1240x1000")
        root.minsize(1060, 870)

        self.ser = None
        self.read_thread = None
        self.running = False
        self.q = queue.Queue()
        self.history = []                 # (x, y, name)
        self.port_var = tk.StringVar()
        self.fpga_port_info = {}
        self.fpga_opened_at = 0.0
        self.fpga_last_telemetry = 0.0
        self.fpga_seen = False
        self.fpga_warned = False
        self.fpga_generation = 0
        self.car_ser = None
        self.car_port_var = tk.StringVar()
        self.car_route_var = tk.StringVar(value="FPGA转发")
        self.ble_link = None
        self.car_com_values = []
        self.car_ble_values = []
        self.auto = DirectControl()
        self.turn_settings_path=Path(os.environ.get('ARROW_TURN_SETTINGS_PATH',
                                                    str(Path(__file__).with_name('turn_settings.json'))))
        self.turn_settings_error=None
        try:
            self.auto.configure(TurnSettings.load(self.turn_settings_path))
        except (OSError,ValueError,TypeError,KeyError) as error:
            self.turn_settings_error=str(error)
        self.detection=DetectionControl()
        self.traffic_vision_epoch=None
        self.arrow_config_generation = None
        self.arrow_seen_generation = None
        self.arrow_seen_at = None
        self.auto_link = None
        self.auto_blocked_widgets = []
        self.auto_snapshot = None
        self.closing = False

        # --- 强化项的状态 ---
        self.counts = {name: 0 for name in ("红灯", "黄灯", "绿灯", "左转", "右转", "其它")}
        self.last_state = None
        self.mt_counts = None      # 颜色块多目标个数 (红, 绿, 黄)，旧固件下为 None
        self.last_rx_time = 0.0
        self.last_alarm_time = 0.0
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.diag_file = None
        self.diag_path = None
        self.alarm_enabled = tk.BooleanVar(value=True)
        self.flash_on = False
        self.skip_last = None       # 上一次回读到的累计跳过帧数
        self.camera_status = None
        self.lp_status = None       # 新固件 $L 行中的真实 LP 状态；旧固件时为 None
        self.last_bt = None         # 上一次回读到的"发给小车的命令字节"

        self._build_ui()
        self._refresh_ports()
        if any("HC05" in value.upper() or "HC-05" in value.upper()
               for value in self.car_ble_values):
            self.car_route_var.set("BLE直连")
            self._update_car_port_choices()
        self._open_csv()                  # 启动就开一个 CSV，边收边写
        self._open_diagnostics()
        if self.turn_settings_error:
            self.log('标定文件读取失败，已使用空白时长：'+self.turn_settings_error,'alarm')
        self.log('连续转向标定文件：'+str(self.turn_settings_path),'info')
        self.root.after(60, self._poll)
        self.root.after(200, self._tick_state)   # 状态超时检查
        self.root.after(20, self._auto_tick)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- UI ----------------
    def _build_ui(self):
        # ---- 顶部：串口 ----
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="FPGA 串口:").pack(side="left")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=34)
        self.port_combo.config(state="readonly")
        self.port_combo.pack(side="left", padx=4)
        ttk.Button(top, text="刷新", command=self._refresh_ports).pack(side="left", padx=2)
        self.btn_connect = ttk.Button(top, text="连接", command=self._toggle_connect)
        self.btn_connect.pack(side="left", padx=4)
        self.lbl_status = ttk.Label(top, text="未连接", foreground="red")
        self.lbl_status.pack(side="left", padx=8)
        self.lbl_rate = ttk.Label(top, text="", foreground="#555")
        self.lbl_rate.pack(side="right")

        # HC-05 在小车上时，可由电脑的蓝牙传出 COM 直接控制。
        # 两个串口独立；显式选择发送路径，不向两路同时发送。
        bt_top = ttk.Frame(self.root, padding=(8, 0, 8, 6))
        bt_top.pack(fill="x")
        ttk.Label(bt_top, text="小车路径:").pack(side="left")
        self.car_route_combo = ttk.Combobox(bt_top, textvariable=self.car_route_var,
                                            values=("FPGA转发", "电脑蓝牙直连", "BLE直连"),
                                            state="readonly", width=14)
        self.car_route_combo.pack(side="left", padx=4)
        self.car_route_combo.bind("<<ComboboxSelected>>", lambda event: self._update_car_port_choices())
        ttk.Label(bt_top, text="小车设备/串口:").pack(side="left")
        self.car_port_combo = ttk.Combobox(bt_top, textvariable=self.car_port_var,
                                          width=31, state="readonly")
        self.car_port_combo.pack(side="left", padx=4)
        self.btn_car_connect = ttk.Button(bt_top, text="连接小车",
                                          command=self._toggle_car_connect)
        self.btn_car_connect.pack(side="left", padx=4)
        self.lbl_car_link = ttk.Label(bt_top, text="蓝牙未连接", foreground="#a00")
        self.lbl_car_link.pack(side="left", padx=6)

        auto_box = ttk.LabelFrame(self.root, text="识别 → BLE 小车自动联动", padding=(8, 4))
        auto_box.pack(fill="x", padx=8, pady=(0, 2))
        self.btn_auto = ttk.Button(auto_box, text="开启自动联动", command=self._toggle_auto)
        self.btn_auto.pack(side="left", padx=(0, 8))
        self.lbl_auto = ttk.Label(auto_box, text=self.auto.status_text, foreground="#555")
        self.lbl_auto.pack(side="left", fill="x", expand=True)
        ttk.Label(auto_box, text="连续转向 → 停车1秒 → 恢复识别触发", foreground="#1565c0").pack(side="right")

        calibration=ttk.LabelFrame(self.root,text='转向标定（固定速度；一次连续转动约90°）',padding=(8,4))
        calibration.pack(fill='x',padx=8,pady=(0,3))
        fields=ttk.Frame(calibration);fields.pack(fill='x')
        settings=self.auto.settings
        self.turn_speed_var=tk.StringVar(value=str(settings.speed))
        self.turn_left_var=tk.StringVar(value='' if settings.left_ms is None else str(settings.left_ms))
        self.turn_right_var=tk.StringVar(value='' if settings.right_ms is None else str(settings.right_ms))
        for label,var,lo,hi,step in (('转向速度',self.turn_speed_var,0,9,1),
                                    ('左90°总时长 ms',self.turn_left_var,150,9000,10),
                                    ('右90°总时长 ms',self.turn_right_var,150,9000,10)):
            ttk.Label(fields,text=label).pack(side='left',padx=(0,4))
            entry=ttk.Spinbox(fields,textvariable=var,from_=lo,to=hi,increment=step,width=7)
            entry.pack(side='left',padx=(0,12));self.auto_blocked_widgets.append(entry)
        self.btn_turn_save=ttk.Button(fields,text='保存标定',command=self._save_turn_settings)
        self.btn_turn_save.pack(side='left');self.auto_blocked_widgets.append(self.btn_turn_save)
        tests=ttk.Frame(calibration);tests.pack(fill='x',pady=(4,0))
        self.turn_test_buttons=[]
        for label,direction in (('试左连续约90°','L'),('试右连续约90°','R')):
            button=ttk.Button(tests,text=label,command=lambda d=direction:self._test_turn(d))
            button.pack(side='left',padx=(0,6));self.auto_blocked_widgets.append(button)
            self.turn_test_buttons.append(button)
        ttk.Label(calibration,text='旧单段时长已自动乘3；此处填写连续转向的总时长，中途无停车。转完停车后固定1秒恢复。',
                  foreground='#555').pack(anchor='w',pady=(3,0))

        arrow_box = ttk.LabelFrame(self.root, text="运动检测常开 / 交通识别手动开启", padding=6)
        arrow_box.pack(fill="x", padx=8)
        detect_row=ttk.Frame(arrow_box);detect_row.pack(fill="x")
        self.btn_traffic=ttk.Button(detect_row,text="开启交通灯/箭头",width=20,command=self._toggle_traffic)
        self.btn_traffic.pack(side="left",padx=(0,8))
        ttk.Label(detect_row,text="运动灵敏度:").pack(side="left")
        self.motion_profile_var=tk.StringVar(value=PROFILES[0])
        self.motion_profile_combo=ttk.Combobox(detect_row,textvariable=self.motion_profile_var,values=PROFILES,state="readonly",width=8)
        self.motion_profile_combo.pack(side="left",padx=4)
        self.motion_profile_combo.bind("<<ComboboxSelected>>",lambda event:self._change_motion_profile())
        self.lbl_motion=ttk.Label(detect_row,text="运动数量: -- | 等待 V5 回读",foreground="#555")
        self.lbl_motion.pack(side="left",padx=8)
        self.lbl_direction=ttk.Label(arrow_box,text="交通识别：关闭；默认只显示运动框")
        self.lbl_direction.pack(side="left",fill="x",expand=True)

        mid = ttk.Frame(self.root, padding=8)

        left = ttk.Frame(mid)
        left.pack(side="left", fill="both", expand=True)

        # ---- 强化项 1：当前状态大色块 ----
        box_state = ttk.LabelFrame(left, text="当前识别状态", padding=6)
        box_state.pack(fill="x")
        self.cv_state = tk.Canvas(box_state, height=90, highlightthickness=0)
        self.cv_state.pack(fill="x")
        self.cv_state.bind("<Configure>", lambda e: self._draw_state())

        # ---- 强化项 2：事件计数大数字 ----
        box_cnt = ttk.LabelFrame(left, text="交通识别事件计数", padding=6)
        box_cnt.pack(fill="x", pady=(6, 0))
        self.cnt_labels = {}
        for i, k in enumerate(["红灯", "黄灯", "绿灯", "左转", "右转", "其它"]):
            f = ttk.Frame(box_cnt)
            f.grid(row=0, column=i, padx=10)
            tk.Label(f, text=k, fg=STATE_COLORS[k], font=("", 11, "bold")).pack()
            lab = tk.Label(f, text="0", font=("Consolas", 24, "bold"),
                           fg=STATE_COLORS[k])
            lab.pack()
            self.cnt_labels[k] = lab

        # ---- 识别日志 ----
        box_log = ttk.LabelFrame(left, text="识别日志（同时写入 CSV）", padding=4)
        box_log.pack(fill="both", expand=True, pady=(6, 0))
        self.log_box = scrolledtext.ScrolledText(box_log, height=6, state="disabled",
                                                 font=("Consolas", 10))
        self.log_box.pack(fill="both", expand=True)
        self.log_box.tag_config("alarm", foreground="#d32f2f")
        self.log_box.tag_config("info", foreground="#555555")

        # ---- 右侧：坐标曲线 ----
        right = ttk.LabelFrame(mid, text="交通目标坐标（开启交通识别后更新）", padding=4)
        right.pack(side="right", fill="both", expand=True)
        self.canvas = tk.Canvas(right, bg="black", height=300, width=440)
        self.canvas.pack(fill="both", expand=True)

        # ---- 批 4：小车遥控 + 状态回读 ----
        #   遥控命令必须带 "C" 前缀（C+<1字节>）：小车协议的裸单字节 R/B/Y 和阈值命令
        #   字母撞车（R=红阈值 vs 右转、B=黄里蓝上限 vs 后退、Y=黄下限 vs 原地掉头），
        #   FPGA 侧只在收到 "C" 前缀时把后面那个字节转给蓝牙。
        ctrl = ttk.LabelFrame(self.root, text="小车手动遥控 / 前进10s、后退1s、转向813ms", padding=8)
        ctrl.pack(side='bottom',fill="x", padx=8, pady=4)

        self.car_fpga_unsupported=[]
        car = ttk.Frame(ctrl)
        car.pack(side="left")
        for txt, ch in (("前进", "F"), ("后退", "B"), ("左转", "L"),
                        ("右转", "R"), ("停车", "S")):
            button = ttk.Button(car, text=txt, width=6, command=lambda c=ch: self._send_car(c))
            button.pack(side="left", padx=2)
            if ch != "S":
                self.auto_blocked_widgets.append(button)
        ttk.Label(car, text="  速度").pack(side="left")
        self.spd_var = tk.StringVar(value="5")
        speed_entry = ttk.Entry(car, textvariable=self.spd_var, width=3)
        speed_entry.pack(side="left", padx=2)
        self.car_fpga_unsupported.append(speed_entry)
        self.auto_blocked_widgets.append(speed_entry)
        for text, callback in (("设定", self._send_speed),
                               ("+", lambda: self._send_car("+")), ("-", lambda: self._send_car("-"))):
            button = ttk.Button(car, text=text, width=5 if text == "设定" else 3, command=callback)
            button.pack(side="left", padx=2)
            self.car_fpga_unsupported.append(button)
            self.auto_blocked_widgets.append(button)
        for widget in self.car_fpga_unsupported: widget.config(state="disabled")

        ttk.Separator(ctrl, orient="vertical").pack(side="left", fill="y", padx=10)

        rd = ttk.Frame(ctrl)
        rd.pack(side="left", fill="x", expand=True)
        row_lp = ttk.Frame(rd)
        row_lp.pack(fill="x")
        lp_on = ttk.Button(row_lp, text="低功耗 开 (L1)", width=15,
                           command=lambda: self._send_lp(True))
        lp_on.pack(side="left", padx=2)
        self.auto_blocked_widgets.append(lp_on)
        ttk.Button(row_lp, text="低功耗 关 (L0)", width=15,
                   command=lambda: self._send_lp(False)).pack(side="left", padx=2)
        # 把遥测移到按钮下一整行并加大字号；显式标出 LP 与十六进制 K。
        self.lbl_lp = ttk.Label(rd, text="LP: 等待遥测  |  K=----",
                                font=("Consolas", 12, "bold"), foreground="#9A3412")
        self.lbl_lp.pack(anchor="w", padx=4, pady=(3, 2))
        self.lbl_car = ttk.Label(rd, text="FPGA 命令记录（非小车应答）: —", font=("Consolas", 10))
        self.lbl_car.pack(anchor="w")
        self.lbl_thr = ttk.Label(rd, text="阈值回读: —",
                                 font=("Consolas", 9), foreground="#333333")
        self.lbl_thr.pack(anchor="w")

        # ---- 底部：回控 + 强化项 3/4/5 的控件 ----
        bottom = ttk.LabelFrame(self.root, text="回控与工具", padding=8)
        bottom.pack(side='bottom',before=ctrl,fill="x", padx=8, pady=4)
        mode_row = ttk.Frame(bottom)
        mode_row.pack(fill="x")

        self.mode_btns = []
        for i, name in enumerate(MODE_NAMES):
            b = ttk.Button(mode_row, text="%d %s" % (i, name), width=9,
                           command=lambda m=i: self._send_mode(m))
            b.pack(side="left", padx=3)
            self.mode_btns.append(b)

        file_row=ttk.Frame(bottom)
        file_row.pack(fill="x",pady=(3,0))
        ttk.Checkbutton(file_row, text="告警", variable=self.alarm_enabled).pack(side="left")
        ttk.Button(file_row, text="另存 CSV 为…", command=self._save_csv_as).pack(side="left", padx=6)
        ttk.Button(file_row, text="另存诊断日志", command=self._save_diagnostics_as).pack(side="left", padx=3)
        self.lbl_csv = ttk.Label(file_row, text="", foreground="#555")
        self.lbl_csv.pack(side="left", padx=6)

        # ---- 颜色块阈值在线下发（不用重编 FPGA）----
        # 命令 = 一个字母 + 三位数字，例如 "R120 G090 y045 b015 m008"
        thr_row = ttk.Frame(bottom)
        thr_row.pack(fill="x", pady=(4, 0))
        ttk.Label(thr_row, text="阈值:").pack(side="left")
        self.thr_var = tk.StringVar(value="R120")
        thr_entry = ttk.Entry(thr_row, textvariable=self.thr_var, width=10)
        thr_entry.pack(side="left", padx=2)
        thr_send = ttk.Button(thr_row, text="发送", command=self._send_thr)
        thr_send.pack(side="left", padx=2)
        self.auto_blocked_widgets.extend((thr_entry, thr_send))
        ttk.Label(thr_row,
                  text="交通灯阈值: R/r红 G/g绿 Y/y/B黄；运动灵敏度使用上方三档",
                  foreground="#555").pack(side="left", padx=4)
        # Reserve both control bars before giving remaining space to charts.
        mid.pack(fill='both',expand=True)

    # ---------------- 自动联动：所有运动意图经同一个 BLE 调度器 ----------------
    def _save_turn_settings(self):
        try:
            if self.auto.active:raise ValueError('先关闭联动或完成试转，再修改标定')
            settings=TurnSettings.from_fields(self.turn_speed_var.get(),self.turn_left_var.get(),
                                              self.turn_right_var.get())
            settings.save(self.turn_settings_path)
            self.auto.configure(settings)
        except (ValueError,OSError) as error:
            self.log('标定未保存：'+str(error),'alarm')
            return False
        self.log('已保存连续转向标定：速度%d，左总时长%s ms，右总时长%s ms；停车后固定1000 ms恢复' % (
            settings.speed,settings.left_ms,settings.right_ms),'info')
        return True

    def _test_turn(self,direction):
        if self.auto.active:
            self.log('先关闭自动联动或等待当前试转完成','alarm');return
        if self.car_route_var.get()!='BLE直连' or not self.ble_link or not self.ble_link.ready:
            self.log('先连接BLE小车，试转不需要FPGA在线','alarm');return
        if not self._save_turn_settings():return
        try:
            self.auto.start_test(direction,self.fpga_generation)
            self.auto_link=self.ble_link
            self.auto_link.begin_auto(self.auto.session)
        except (ValueError,OSError) as error:
            self.auto.pause('试转未启动：'+str(error),disconnected=True)
            self.log(str(error),'alarm')
        self._auto_service()

    def _configure_arrow_stream(self):
        if not self.running or not self.ser:return False
        import secrets
        full=(0,0,1279,719)
        self.traffic_vision_epoch=secrets.randbits(32) or 1
        payload=vision_config(self.traffic_vision_epoch,full,full)
        if self._send_raw(b"L0","保持交通识别全速") and self._send_raw(payload.encode("ascii"),"配置灯与箭头全画面识别"):
            self.arrow_config_generation=self.fpga_generation
            return True
        self.traffic_vision_epoch=None
        return False

    def _request_detection(self,traffic,profile):
        if not self.running or not self.ser:
            self.log("先连接 FPGA 串口","alarm");return False
        if not traffic:
            if self.auto.active:
                self.auto.disarm("关闭交通识别，取消自动联动并停车")
                self._auto_service()
            self.traffic_vision_epoch=None
            self.last_state=None
            self.lbl_direction.config(text="交通识别：关闭，等待板卡回读")
            self._draw_state()
        elif not self._configure_arrow_stream():return False
        payload=self.detection.request(traffic,profile,self.auto.clock())
        sent=self._send_raw(payload.encode("ascii"),"设置交通识别/运动灵敏度；等待匹配回读")
        if not sent:
            self.detection.pending=None;self.detection.applied=None;self.detection.error="配置写入失败"
        self._render_detection()
        return sent

    def _toggle_traffic(self):
        if self.detection.pending:
            self.log("正在等待交通开关回读","info");return
        if not self.detection.supported or self.detection.applied is None:
            self.log("需要连接 V5 码流并收到匹配状态回读","alarm");return
        profile=PROFILES.index(self.motion_profile_var.get())
        self._request_detection(not self.detection.traffic_allowed,profile)

    def _change_motion_profile(self):
        if self.motion_profile_var.get() not in PROFILES:return
        if self.auto.active or self.detection.pending or self.detection.applied is None:
            if self.detection.applied:self.motion_profile_var.set(PROFILES[self.detection.applied.profile])
            self.log("先停止联动并等待配置回读，再调整运动灵敏度","alarm");return
        self._request_detection(self.detection.traffic_allowed,PROFILES.index(self.motion_profile_var.get()))

    def _render_detection(self):
        d=self.detection
        self.btn_traffic.config(text="关闭交通灯/箭头" if d.traffic_allowed else "开启交通灯/箭头",
                                state="disabled" if d.pending or not d.supported or d.applied is None else "normal")
        if d.pending:
            self.lbl_motion.config(text="运动数量: -- | 配置回读中",foreground="#9A3412")
        elif d.error:
            self.lbl_motion.config(text=d.error,foreground="#a00")
        elif d.applied:
            self.motion_profile_var.set(PROFILES[d.applied.profile])
            if not d.applied.traffic:
                self.lbl_direction.config(text="交通识别：关闭；只显示运动框")
            elif self.lbl_direction.cget("text").startswith("交通识别：关闭"):
                self.lbl_direction.config(text="交通识别已开启，等待新的灯/箭头结果")
            self.lbl_motion.config(text="运动数量: %d | %s | 交通%s" % (d.applied.count,
                "已就绪" if d.applied.ready else "预热中","开启" if d.applied.traffic else "关闭"),foreground="#1565c0")
        else:
            self.lbl_motion.config(text="运动数量: -- | 等待 V5 回读",foreground="#555")
        self._draw_state()

    def _detection_tick(self):
        if self.detection.tick(self.auto.clock()):
            if self.auto.active:self.auto.disarm("检测配置/遥测失效，取消自动联动")
            self.traffic_vision_epoch=None
            self._render_detection()

    def _toggle_auto(self):
        if self.auto.active:
            self.auto.disarm()
            self._auto_service()
            return
        if not self.running or not self.ser:
            self.log("先连接 FPGA 串口", "alarm")
            return
        if self.car_route_var.get() != "BLE直连" or not self.ble_link or not self.ble_link.ready:
            self.log("先连接 BLE 小车", "alarm")
            return
        if not self.detection.traffic_allowed:
            self.log("先开启交通灯/箭头识别并等待板卡回读，再开启自动联动","alarm")
            return
        self.auto.arm(self.fpga_generation)
        self.auto_link = self.ble_link
        try:
            self.auto_link.begin_auto(self.auto.session)
        except Exception as error:
            self.auto.pause("BLE 提交失败：%s" % error, disconnected=True)
        self._auto_service()

    def _auto_service(self):
        if self.auto.active and not self.auto.testing and not self.detection.traffic_allowed:
            self.auto.disarm("交通识别未确认开启，已取消自动联动")
        self.auto.tick()
        link = self.auto_link or (self.ble_link if self.car_route_var.get() == "BLE直连" else None)
        available = bool(self.car_route_var.get() == "BLE直连" and self.ble_link and self.ble_link.ready
                         and (self.auto.testing or (self.running and self.ser)))
        if self.auto.active:
            if not available:
                if self.auto.links_ready:
                    self.auto.pause("连接断开，已请求停车", disconnected=True)
            elif self.auto_link is not self.ble_link or (not self.auto.testing and self.auto.generation != self.fpga_generation):
                self.auto.pause('连接已更换，取消当前任务',disconnected=True)
        if not self.auto.active and self.auto_link:
            self.auto_link.cancel_auto()
            self.auto_link=None
        for intent in self.auto.drain():
            if intent.session != self.auto.session:
                continue
            if intent.command == "L0":
                if self.running and self.ser:
                    self._send_raw(b"L0", "识别直接联动全速")
                continue
            if intent.command != "S" and (not self.auto.active or not available):
                continue
            try:
                if not link or not link.ready:
                    continue
                if not self.auto.active:
                    link.cancel_auto()
                request_id=link.send(intent.command, source="auto" if self.auto.active else "safety",
                                     session=intent.session, valid_until=intent.valid_until, timeout_ms=300)
                self.auto.submitted(intent.command,request_id,intent.session)
                self._diag_write("AUTO_REQUEST", json.dumps({"command":intent.command,
                    "id":request_id,"session":intent.session,"reason":intent.reason,
                    "task":self.auto.task_id,"turn_ms":self.auto.turn_ms},ensure_ascii=False))
            except Exception as error:
                self.auto.pause("发送失败：%s" % error, disconnected=True)
                if link:
                    link.cancel_auto()
                    link.ready=False
                self.auto_link=None
        self._render_auto()

    def _render_auto(self):
        snapshot=(self.auto.state,self.auto.reason)
        self.btn_auto.config(text='停止试转' if self.auto.testing else ('关闭自动联动' if self.auto.active else '开启自动联动'))
        self.lbl_auto.config(text=self.auto.status_text,wraplength=640)
        for widget in self.auto_blocked_widgets:
            unsupported=self.car_route_var.get()=="FPGA转发" and widget in self.car_fpga_unsupported
            widget.config(state="disabled" if self.auto.locked or unsupported else "normal")
        if snapshot != self.auto_snapshot:
            self.auto_snapshot=snapshot
            self._diag_write("AUTO_STATE",json.dumps({"state":self.auto.state,"reason":self.auto.reason,
                "session":self.auto.session,"task":self.auto.task_id,"turn_ms":self.auto.turn_ms,
                "deadline":self.auto.deadline},ensure_ascii=False))

    def _auto_tick(self):
        if self.closing:
            return
        if self.ble_link and hasattr(self.ble_link, "service"):
            self.ble_link.service()
        self._detection_tick()
        self._auto_service()
        self.root.after(20, self._auto_tick)

    # ---------------- 强化项 5：串口友好名称 + 占用提示 ----------------
    def _refresh_ports(self):
        vals = []
        bt_vals = []
        ranks = []
        self.fpga_port_info = {}
        try:
            for p in serial.tools.list_ports.comports():
                desc = (p.description or "").strip()
                # 去掉描述里重复的端口名，如 "USB Serial Port (COM5)" → "USB Serial Port"
                desc = re.sub(r"\s*\(COM\d+\)\s*$", "", desc).strip()
                if desc and desc.upper().startswith(p.device.upper()):
                    desc = ""
                label = "%s  (%s)" % (p.device, desc) if desc else p.device
                vals.append(label)
                self.fpga_port_info[p.device.upper()] = p
                # This board's USB-UART is FTDI 0403:6042. Do not prefer an
                # unconnected CH340 solely because it appears earlier in Windows.
                hwid = (p.hwid or "").upper()
                is_board = "VID:PID=0403:6042" in hwid
                rank = 0 if is_board else (3 if "BTHENUM" in hwid else 1)
                ranks.append(rank)
                if "BTHENUM" in (p.hwid or "").upper():
                    bt_vals.append(label)
        except Exception:
            vals = []
            bt_vals = []
        self.port_combo["values"] = vals
        self.car_com_values = bt_vals
        try:
            self.car_ble_values = ["%s [%s]" % item for item in known_ble_devices()]
        except OSError as error:
            self.car_ble_values = []
            self.log("无法读取 Windows BLE 设备列表：%s" % error, "alarm")
        self._update_car_port_choices()
        if vals and not self.running and self.port_var.get() not in vals:
            candidates = [index for index in range(len(vals)) if ranks[index] < 3]
            if candidates:
                pick = min(candidates, key=lambda index: ranks[index])
                self.port_var.set(vals[pick])
            else:
                self.port_var.set("")
                self.lbl_status.config(text="未检测到 USB 串口，FPGA 未连接", foreground="#a00")
        if not vals:
            self.lbl_status.config(text="没检测到串口", foreground="#a00")

    def _cur_port(self):
        s = self.port_var.get().strip()
        return s.split("  (")[0].strip() if s else ""

    def _toggle_connect(self):
        if self.running:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self._cur_port()
        if not port:
            self.log("请先选择串口", "info")
            return
        try:
            if self.car_ser and port.upper() == self.car_ser.port.upper():
                self.log("此端口已用于小车蓝牙，请为 FPGA 选择板载 USB-UART", "alarm")
                return
            self.ser = serial.Serial(port, BAUD, timeout=0.3, write_timeout=0.3)
        except PermissionError:
            self.log("【串口被占用】%s 打不开。" % port, "alarm")
            self.log("  常见原因：另一个上位机窗口 / 串口助手 还开着。", "info")
            self.log("  处理：把那些程序关掉，再点『连接』。", "info")
            self.lbl_status.config(text="%s 被占用" % port, foreground="#a00")
            return
        except Exception as e:
            self.log("打开串口失败：%s" % e, "alarm")
            self.lbl_status.config(text="打开失败", foreground="#a00")
            return

        self.running = True
        self.fpga_generation += 1
        self._reset_fpga_state()
        self.fpga_opened_at = time.monotonic()
        self.btn_connect.config(text="断开")
        self.port_combo.config(state="disabled")
        self.lbl_status.config(text="%s 已打开，等待 FPGA" % port, foreground="#9A3412")
        self.read_thread = threading.Thread(target=self._read_loop,
                                            args=(self.ser, self.fpga_generation), daemon=True)
        self.read_thread.start()
        self._request_detection(False,0)
        self.log("已打开 FPGA 串口 %s @ %d；收到有效遥测后才确认在线" % (port, BAUD), "info")

    def _disconnect(self):
        if self.running and self.ser:
            self.manual_car_token=getattr(self,"manual_car_token",0)+1
            self._send_raw(b"CS","断开 FPGA 前停车")
            self._request_detection(False,0)
        self.running = False
        self.fpga_generation += 1
        self.auto.pause("FPGA 串口断开，停车等待重连", disconnected=True)
        self._auto_service()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.btn_connect.config(text="连接")
        self.port_combo.config(state="readonly")
        self._reset_fpga_state()
        self.lbl_status.config(text="FPGA 串口未连接", foreground="red")
        self.log("FPGA 串口已断开；小车 BLE 连接独立", "info")

    def _read_loop(self, uart, generation):
        pending = b""
        while self.running and self.ser is uart and generation == self.fpga_generation:
            try:
                line = uart.readline()
            except Exception as e:
                if self.running and self.ser is uart and generation == self.fpga_generation:
                    self.q.put(("fpga_error", (generation, str(e))))
                break
            if line:
                pending += line
                while b"\n" in pending:
                    complete, pending = pending.split(b"\n", 1)
                    self.q.put(("fpga_line", (generation, time.monotonic(),
                                             complete.decode("ascii", errors="replace").strip())))
                if len(pending) > 4096:
                    self.q.put(("fpga_line", (generation, time.monotonic(), "$T,INVALID")))
                    pending = b""

    def _poll(self):
        try:
            for unused in range(200):
                kind, data = self.q.get_nowait()
                if kind in ("fpga_line", "fpga_error"):
                    generation, payload = data[0], data[-1]
                    if not self.running or generation != self.fpga_generation:
                        continue
                    if kind == "fpga_line":
                        self._handle_line(payload, received_at=data[1] if len(data) == 3 else None)
                    else:
                        self.log("FPGA 串口错误: " + payload, "alarm")
                        self._disconnect()
                elif kind == "ble":
                    self._handle_ble_event(*data)
        except queue.Empty:
            pass
        self._detection_tick()
        self._auto_service()
        if not self.closing:
            self.root.after(20, self._poll)

    # ---------------- 数据 ----------------
    def _reset_fpga_state(self):
        self.detection=DetectionControl()
        self.traffic_vision_epoch=None
        self.motion_profile_var.set(PROFILES[0])
        self._render_detection()
        self.fpga_opened_at = 0.0
        self.fpga_last_telemetry = 0.0
        self.fpga_seen = False
        self.fpga_warned = False
        self.skip_last = None
        self.lp_status = None
        self.last_bt = None
        self.last_rx_time = 0.0
        self.last_state = None
        self.lbl_lp.config(text="LP: 等待遥测  |  K=----", foreground="#9A3412")
        self.lbl_car.config(text="FPGA 命令记录（非小车应答）: —")
        self.lbl_thr.config(text="阈值回读: —")
        self.camera_status = None

    def _mark_fpga_telemetry(self, received_at=None):
        first_or_recovered = not self.fpga_seen or self.fpga_warned
        self.fpga_last_telemetry = time.monotonic() if received_at is None else received_at
        self.fpga_seen = True
        self.fpga_warned = False
        if self.running and self.ser:
            self.lbl_status.config(text="FPGA 在线 %s" % self.ser.port, foreground="green")
            if first_or_recovered:
                self.log("收到有效 FPGA 遥测，已确认 %s 回传在线" % self.ser.port, "info")

    def _check_fpga_telemetry(self, now=None):
        if not self.running or not self.ser:
            return
        now = time.monotonic() if now is None else now
        reference = self.fpga_last_telemetry if self.fpga_seen else self.fpga_opened_at
        if now - reference < 3.0:
            return
        self.lbl_status.config(text="%s 无 FPGA 遥测" % self.ser.port, foreground="#a00")
        self.lbl_lp.config(text="LP: 未收到有效遥测  |  K=----", foreground="#a00")
        self.camera_status = None
        if not self.fpga_warned:
            self.fpga_warned = True
            self.log("串口已打开，但超过 3 秒无有效 FPGA 遥测；核对板载串口、TTL 接线及断电后的 SRAM 码流。BLE 小车连接与此独立。", "alarm")

    def _handle_line(self, text, received_at=None):
        if not text:
            return
        self._diag_write("FPGA_RX", text)
        received_at = time.monotonic() if received_at is None else received_at
        if text.startswith("$P,"):
            try:
                camera=parse_camera(text)
            except ValueError as error:
                self.log(str(error),"alarm")
                return
            self._mark_fpga_telemetry(received_at)
            state="I²C故障" if camera.fault else "调整中" if camera.settling else "光照极限" if camera.at_limit else "自动"
            self.camera_status=camera
            self.lbl_status.config(text="FPGA 在线 %s | 相机%s" % (self.ser.port if self.ser else "",state),
                                   foreground="#a00" if camera.fault else "green")
            return
        if text.startswith("$S"):
            try:status=parse_status(text)
            except ValueError as error:
                if self.auto.active:self.auto.disarm("运动状态报文错误，停车")
                self.log(str(error),"alarm");return
            first=not self.detection.supported
            accepted=self.detection.observe(status,received_at)
            self._mark_fpga_telemetry(received_at)
            if not accepted and first and self.detection.pending is None:
                self._request_detection(False,0)
            elif not accepted and self.detection.pending is None and self.detection.applied is not None:
                self._request_detection(False,0)
            self._render_detection()
            return
        if text.startswith("$V"):
            try:
                observation=parse_vision(text)
            except ValueError as error:
                self.auto.pause("方向报文错误：%s" % error)
                self.log(str(error),"alarm")
                return
            self.arrow_seen_at=received_at
            self.arrow_seen_generation=self.fpga_generation
            self._mark_fpga_telemetry(received_at)
            if not self.detection.traffic_allowed or observation.epoch!=self.traffic_vision_epoch or not observation.flags&CONFIGURED:
                return
            observation=replace(observation,
                color=observation.color if observation.flags&LAMP_VALID and not observation.flags&CONFLICT else ("U" if observation.flags&CONFLICT else "N"),
                direction=observation.direction if observation.flags&SIGN_VALID else "N")
            direction_name={"L":"左转","R":"右转","N":"无箭头","U":"未识别"}[observation.direction]
            self.lbl_direction.config(text="方向：%s | 匹配 %d | 灯色 %s" %
                (direction_name,observation.score,observation.color))
            name=direction_name if observation.direction in ("L","R") else {"R":"红灯","Y":"黄灯","G":"绿灯"}.get(observation.color,"其它")
            self._show_vision_result(name,observation)
            self.auto.observe(observation,received_at,self.fpga_generation)
            self._auto_service()
            return
        if text.startswith("$L"):
            self._handle_lp_line(text, received_at)
            return
        if text.startswith("$T"):
            if not self.detection.traffic_allowed:
                try:target_color(text)
                except ValueError:return
                self._mark_fpga_telemetry(received_at)
                return
            try:
                color = target_color(text)
            except ValueError:
                color = None
            # Legacy T remains readable, but V5 automatic control uses only
            # the current Q epoch in a CRC-valid V observation.
            if not self.detection.supported and self.arrow_seen_generation != self.fpga_generation:
                self.auto.target(color, received_at, self.fpga_generation)
            parts = text.split(",")
            # 新版固件在 x/y 后面多带 3 个字段：红/绿/黄 的"颜色块目标个数"
            # （旧固件只有 5 个字段，这里兼容两种）
            if len(parts) >= 5:
                _, tid, ttype, xhex, yhex = parts[:5]
                mt_r = mt_g = mt_y = None
                circ = conf = None
                if len(parts) >= 8:
                    try:
                        mt_r, mt_g, mt_y = int(parts[5]), int(parts[6]), int(parts[7])
                    except ValueError:
                        mt_r = mt_g = mt_y = None
                if len(parts) >= 10:
                    try:
                        circ, conf = int(parts[8]), int(parts[9])
                    except ValueError:
                        circ = conf = None
                try:
                    x = int(xhex, 16)
                    y = int(yhex, 16)
                except ValueError:
                    self.log(text, "info")
                    return

                self._mark_fpga_telemetry(received_at)
                name = TYPE_NAMES.get(ttype, "其它")
                ts = time.strftime("%H:%M:%S")
                extra = ""
                if mt_r is not None:
                    extra = "  颜色块数量: 红%d 绿%d 黄%d (共%d)" % (
                        mt_r, mt_g, mt_y, mt_r + mt_g + mt_y)
                    self.mt_counts = (mt_r, mt_g, mt_y)
                    if circ is not None:
                        extra += "  圆形%d 置信度%d" % (circ, conf)
                self._update_readback(parts)
                if self.arrow_seen_generation == self.fpga_generation:
                    self.history.append((x,y,name))
                    if len(self.history)>MAX_POINTS:
                        self.history.pop(0)
                    self._redraw()
                    return  # V is the single display/control source; T retains diagnostics.
                self.log("[%s] ID=%s  类型=%s  坐标=(%d, %d)%s" % (ts, tid, name, x, y, extra),
                         "alarm" if name in ALARM_TYPES else None)

                # 强化项 2：计数
                key = name if name in self.counts else "其它"
                self.counts[key] += 1
                self.cnt_labels[key].config(text=str(self.counts[key]))

                # 强化项 1：状态色块
                self.last_state = name if name in STATE_COLORS else "其它"
                self.last_rx_time = time.time()
                self._draw_state()

                # 强化项 3：写 CSV
                self._csv_write(ts, tid, name, x, y)

                # 强化项 4：告警
                self._maybe_alarm(name)

                # 曲线
                self.history.append((x, y, name))
                if len(self.history) > MAX_POINTS:
                    self.history.pop(0)
                self._redraw()

                # 速率
                self.lbl_rate.config(text="收到 %d 条" %
                                     sum(self.counts.values()))
                return
        self.log(text, "info")

    def _show_vision_result(self, name, observation):
        ts=time.strftime("%H:%M:%S")
        self.last_state=name
        self.last_rx_time=time.time()
        self._draw_state()
        self.counts[name]+=1
        self.cnt_labels[name].config(text=str(self.counts[name]))
        self.log("[%s] 识别=%s | 匹配=%d；动作见联动状态" % (ts,name,observation.score),
                 "alarm" if name in ALARM_TYPES else "info")
        self._csv_write(ts,observation.sign_frame if name in ("左转","右转") else observation.lamp_frame,name,"","")
        self._maybe_alarm(name)
        self.lbl_rate.config(text="收到 %d 条" % sum(self.counts.values()))

    # ---------------- 强化项 1：大色块 ----------------
    # ---------------- 批 4：状态回读（小车上行命令 / 低功耗 / 阈值） ----------------
    def _handle_lp_line(self, text, received_at=None):
        """解析新固件周期发送的独立 LP 遥测：$L,<0|1>,Khhhh。"""
        parts = text.strip().split(",")
        if (len(parts) != 3 or parts[0] != "$L" or parts[1] not in ("0", "1")
                or len(parts[2]) != 5 or parts[2][0].upper() != "K"
                or any(ch not in "0123456789abcdefABCDEF" for ch in parts[2][1:])):
            self.log(text, "info")
            self.auto.target(None, self.auto.clock() if received_at is None else received_at,
                             self.fpga_generation)
            return
        skip = int(parts[2][1:], 16)
        self._mark_fpga_telemetry(received_at)
        delta = None if self.skip_last is None else ((skip - self.skip_last) & 0xFFFF)
        self.skip_last = skip
        self.lp_status = (parts[1] == "1")
        self.auto.lp(self.lp_status, self.auto.clock() if received_at is None else received_at,
                     self.fpga_generation)
        self._render_lp_status(self.lp_status, skip, delta)

    def _render_lp_status(self, active, skip, delta=None):
        state = "降频中" if active else "全速"
        delta_text = "" if delta is None else "  |  ΔK=%+d/周期" % delta
        self.lbl_lp.config(
            text="LP: %s  |  K=0x%04X (%d 帧)%s" % (state, skip, skip, delta_text),
            foreground="#B45309" if active else "#15803D")

    def _update_readback(self, parts):
        """
        从 $T 行的后半段更新三块回读显示：
          [10] 蓝牙命令 2hex   [11] K<4hex> 累计跳过帧数   [12..20] 9 个阈值 2hex
        旧固件没有这些字段，长度不够就跳过（不影响前面已经显示的日志/计数）。
        """
        # 发给小车的命令字节
        if len(parts) >= 11:
            try:
                b = int(parts[10], 16)
                self.last_bt = b
                self.lbl_car.config(text="FPGA 命令记录（非小车应答）: 0x%02X  %s" % (b, bt_name(b)))
            except ValueError:
                pass

        # 旧固件兼容：只有 $T/K 时显示 K，并描述本周期是否观察到新增跳帧；
        # 不能把“本周期 K 没变”误称为 LP 状态关闭。新固件以 $L 状态位为准。
        if len(parts) >= 12 and parts[11][:1].upper() == "K":
            try:
                skip = int(parts[11][1:], 16)
                prev = self.skip_last
                d = None if prev is None else ((skip - prev) & 0xFFFF)
                self.skip_last = skip
                if self.lp_status is None:
                    state = "状态未知" if d is None else ("本周期有跳帧" if d > 0 else "本周期无跳帧")
                    self.lbl_lp.config(text="LP: %s  |  K=0x%04X (%d 帧)" % (state, skip, skip),
                                        foreground="#555555")
                # 新固件先发独立 $L；紧随其后的 $T 可能只差数毫秒，
                # 不要用同一个 K 再覆盖 $L 显示的周期增量（会误显示 ΔK=0）。
            except ValueError:
                pass

        # 阈值回读：9 个字段各 2 位十六进制
        if len(parts) >= 21:
            try:
                thr = [int(v, 16) for v in parts[12:21]]
                self.lbl_thr.config(text="阈值回读: " + "  ".join(
                    "%s=%d" % (n, v) for n, v in zip(THR_ORDER, thr)))
            except ValueError:
                pass

    def _draw_state(self):
        c = self.cv_state
        c.delete("all")
        w = max(c.winfo_width(), 200)
        h = max(c.winfo_height(), 110)

        lost = (time.time() - self.last_rx_time) > STATE_TIMEOUT_S
        name = self.last_state if (self.last_state and not lost) else None
        color = STATE_COLORS.get(name, STATE_IDLE) if name else STATE_IDLE

        # 告警时闪一下（边框加粗）
        outline = "#ffffff"
        width = 6 if (self.flash_on and name in ALARM_TYPES) else 2

        big = name if name else ("识别丢失" if lost else "等待数据")
        if not self.detection.traffic_allowed:
            d=self.detection
            if d.pending:big="等待配置回读"
            elif d.applied and d.applied.ready:big="运动目标：%d" % d.applied.count
            elif d.applied:big="运动检测预热中"
            else:big="等待运动状态"
            color="#287c99" if d.applied and d.applied.count else STATE_IDLE
        c.create_rectangle(2, 2, w - 2, h - 2, fill=color, outline=outline, width=width)
        c.create_text(w // 2, h // 2, text=big, fill="white",
                      font=("", 30, "bold"))

    def _tick_state(self):
        """定时检查：长时间没数据就把色块变灰（表示识别丢失）"""
        self.flash_on = not self.flash_on
        self._check_fpga_telemetry()
        self._draw_state()
        self.root.after(400, self._tick_state)

    # ---------------- 强化项 3：CSV ----------------
    def _open_diagnostics(self):
        self.diag_path = os.path.abspath("diagnostic_log_%s.log" % time.strftime("%Y%m%d_%H%M%S"))
        try:
            self.diag_file = open(self.diag_path, "w", encoding="utf-8")
        except OSError as error:
            self.diag_file = None
            self.log("诊断日志无法创建：%s" % error, "alarm")
            return
        self._diag_write("START", json.dumps({
            "revision": "motion_v5_20261023", "source": os.path.abspath(__file__),
            "python": sys.executable, "cwd": os.getcwd(),
            "route": self.car_route_var.get(), "selected_car": self.car_port_var.get(),
        }, ensure_ascii=False))
        self.log("完整连接/发送诊断日志：%s；事件 CSV 只保存识别结果" % self.diag_path, "info")

    def _diag_write(self, kind, message):
        if not self.diag_file:
            return False
        try:
            stamp = time.strftime("%Y-%m-%d %H:%M:%S")
            self.diag_file.write("[%s] %s %s\n" % (stamp, kind, message))
            self.diag_file.flush()
            return True
        except (OSError, ValueError):
            return False

    def _save_diagnostics_as(self):
        if not self.diag_file or not self.diag_path:
            self.log("没有可保存的诊断日志", "alarm")
            return
        dst = filedialog.asksaveasfilename(defaultextension=".log",
                                          initialfile=os.path.basename(self.diag_path))
        if not dst:
            return
        if os.path.normcase(os.path.realpath(dst)) == os.path.normcase(os.path.realpath(self.diag_path)):
            self.log("诊断日志已实时保存在此路径", "info")
            return
        try:
            self.diag_file.flush()
            with open(self.diag_path, "rb") as source, open(dst, "wb") as target:
                target.write(source.read())
            self.log("已另存诊断日志：%s" % dst, "info")
        except OSError as error:
            self.log("保存诊断日志失败：%s" % error, "alarm")

    def _open_csv(self):
        fn = "event_log_%s.csv" % time.strftime("%Y%m%d_%H%M%S")
        try:
            self.csv_path = os.path.abspath(fn)
            self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8-sig")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["时间", "ID", "类型", "x", "y"])
            self.csv_file.flush()
            self.lbl_csv.config(text="记录中: %s" % fn)
        except Exception as e:
            self.csv_file = None
            self.csv_writer = None
            self.lbl_csv.config(text="CSV 打不开: %s" % e)

    def _csv_write(self, ts, tid, name, x, y):
        if not self.csv_writer:
            return
        try:
            self.csv_writer.writerow([ts, tid, name, x, y])
            self.csv_file.flush()          # 实时落盘，断电不丢
        except Exception:
            pass

    def _save_csv_as(self):
        if not self.csv_path:
            return
        dst = filedialog.asksaveasfilename(defaultextension=".csv",
                                           initialfile=os.path.basename(self.csv_path))
        if not dst:
            return
        try:
            self.csv_file.flush()
            with open(self.csv_path, "rb") as s, open(dst, "wb") as d:
                d.write(s.read())
            self.log("已另存为 %s" % dst, "info")
        except Exception as e:
            self.log("另存失败: %s" % e, "alarm")

    # ---------------- 强化项 4：告警 ----------------
    def _maybe_alarm(self, name):
        if not self.alarm_enabled.get():
            return
        if name not in ALARM_TYPES:
            return
        now = time.time()
        if now - self.last_alarm_time < ALARM_COOLDOWN_S:
            return
        self.last_alarm_time = now
        try:
            self.root.bell()               # 系统提示音（不打断）
        except Exception:
            pass
        self.flash_on = True

    # ---------------- 曲线 ----------------
    def _redraw(self):
        c = self.canvas
        c.delete("all")
        w = max(c.winfo_width(), 400)
        h = max(c.winfo_height(), 350)
        # 画场地边框
        c.create_rectangle(8, 8, w - 8, h - 8, outline="#404040")
        if len(self.history) < 2:
            return
        n = len(self.history)
        xs = [p[0] for p in self.history]
        ys = [p[1] for p in self.history]

        def sx(v):
            return 8 + v / 1280.0 * (w - 16)

        def sy(v):
            return 8 + v / 720.0 * (h - 16)

        px = [sx(v) for v in xs]
        py = [sy(v) for v in ys]
        for i in range(n - 1):
            c.create_line(px[i], py[i], px[i + 1], py[i + 1], fill="#3fa7ff", width=2)
        if n:
            c.create_oval(px[-1] - 4, py[-1] - 4, px[-1] + 4, py[-1] + 4,
                          fill="#ff5a5a", outline="")

    # ---------------- 日志 ----------------
    def log(self, text, tag=None):
        self._diag_write("LOG", text)
        self.log_box.config(state="normal")
        if tag:
            self.log_box.insert("end", text + "\n", tag)
        else:
            self.log_box.insert("end", text + "\n")
        self.log_box.see("end")
        self.log_box.config(state="disabled")

    # ---------------- 回控 ----------------
    def _send_mode(self, m):
        if not self.ser or not self.running:
            self.log("未连接，无法回控", "alarm")
            return
        try:
            self.ser.write(("M%d" % m).encode("ascii"))
            self.log("已发送回控指令: M%d (%s)" % (m, MODE_NAMES[m]), "info")
        except Exception as e:
            self.log("发送失败: %s" % e, "alarm")

    def _send_thr(self):
        """把输入框里的阈值命令原样发给 FPGA（字母+三位数字，可发多组）。"""
        if self.auto.locked:
            self.log("请先停车退出自动联动，再修改阈值", "alarm")
            return
        if not self.ser or not self.running:
            self.log("未连接，无法下发阈值", "alarm")
            return
        cmd = (self.thr_var.get() or "").strip()
        if not cmd:
            return
        try:
            self.ser.write(cmd.encode("ascii"))
            self.log("已下发阈值: %s" % cmd, "info")
        except Exception as e:
            self.log("下发失败: %s" % e, "alarm")

    # ---------------- 批 4：遥控 / 低功耗开关 ----------------
    def _update_car_port_choices(self):
        for widget in self.car_fpga_unsupported:
            widget.config(state="disabled" if self.car_route_var.get()=="FPGA转发" or self.auto.locked else "normal")
        if self.car_ser or self.ble_link:
            return
        is_ble = self.car_route_var.get() == "BLE直连"
        values = self.car_ble_values if is_ble else self.car_com_values
        self.car_port_combo["values"] = values
        if self.car_port_var.get() not in values:
            pick = next((value for value in values if "HC05" in value.upper() or "HC-05" in value.upper()), "") if is_ble else ""
            self.car_port_var.set(pick)

    def _toggle_car_connect(self):
        if self.ble_link:
            self._disconnect_ble()
        elif self.car_ser:
            self._disconnect_car()
        elif self.car_route_var.get() == "BLE直连":
            self._connect_ble()
        else:
            self._connect_car()

    def _connect_ble(self):
        if self.car_ser or self.ble_link:
            return
        match = re.search(r"\[([0-9A-Fa-f:]{17})\]$", self.car_port_var.get())
        if not match:
            self.log("请选择 BLE 设备（例如 HC05_SLAVE）；BLE 不使用 COM 端口", "alarm")
            return
        link = BleCarLink(match.group(1), lambda sender, event: self.q.put(("ble", (sender, event))))
        self.ble_link = link
        try:
            link.start()
        except Exception as error:
            self.ble_link = None
            self.log("启动 BLE 接口失败：%s" % error, "alarm")
            return
        self.car_route_var.set("BLE直连")
        self.car_route_combo.config(state="disabled")
        self.car_port_combo.config(state="disabled")
        self.btn_car_connect.config(text="取消 BLE")
        self.lbl_car_link.config(text="正在连接 BLE…", foreground="#9A3412")
        self.log("正在连接 BLE %s；仅发现服务，不发送运动命令" % link.address, "info")

    def _handle_ble_event(self, link, event):
        self._diag_write("BLE_EVENT", json.dumps({"address": link.address, "event": event}, ensure_ascii=False))
        if link is not self.ble_link:
            return
        kind = event.get("kind")
        if kind == "services":
            self.log("BLE 接口：" + ", ".join("%s (%s)" % (item["uuid"], item["properties"])
                     for item in event.get("characteristics", [])), "info")
        elif kind == "ready":
            self.btn_car_connect.config(text="断开 BLE")
            self.lbl_car_link.config(text="BLE 已连接 " + event.get("name", ""), foreground="green")
            self.log("BLE 写入接口已就绪：%s；车端执行仍需实测" % event.get("characteristic", ""), "info")
        elif kind == 'dispatched':
            if link is self.auto_link and event.get('source')=='auto':
                self.auto.dispatched(event.get('command'),event.get('id'),event.get('session'),event['sent_at'])
        elif kind == "written":
            if event.get("matched") and link is self.auto_link and event.get("source") == "auto":
                if event.get("status") == "Success":
                    self.auto.written(event.get("command"), event.get("id"), event.get("session"),event.get('received_at'))
                else:
                    self.auto.pause("BLE 写入失败，停车等待恢复", disconnected=True)
            if event.get("source") != "auto" or event.get("command") != "F":
                self.log("BLE 已写入 %s（GATT %s，未确认车端执行）" %
                         (event.get("command", ""), event.get("status", "")), "info")
        elif kind == "shutdown_stop":
            self.log("BLE 退出停车写入状态：%s，未确认车端执行" % event.get("status", ""), "info")
        elif kind == "shutdown_stop_failed":
            self.log("BLE 退出停车写入失败：%s" % event.get("message", ""), "alarm")
        elif kind == "error":
            self.log("BLE 错误：" + event.get("message", "未知错误"), "alarm")
            self._disconnect_ble(stop=False)
        elif kind == "closed":
            self.log("BLE 接口已关闭", "info")
            self._disconnect_ble(stop=False)
        elif event.get("message"):
            self.log("BLE: " + event["message"], "info")

    def _disconnect_ble(self, stop=True):
        link = self.ble_link
        self.ble_link = None
        self.auto.pause("BLE 断开或写入失败，停车等待重连", disconnected=True)
        self._auto_service()
        if link:
            try:
                if not link.close(stop=stop):
                    self.log("BLE 关闭超时；停车送达状态未知", "alarm")
            except Exception as error:
                self.log("BLE 关闭异常：%s" % error, "alarm")
        self.car_route_combo.config(state="readonly")
        self.car_port_combo.config(state="readonly")
        self.btn_car_connect.config(text="连接小车")
        self.lbl_car_link.config(text="BLE 未连接", foreground="#a00")

    def _connect_car(self):
        port = self.car_port_var.get().split("  (")[0].strip()
        if not port:
            self.log("请先选 HC-05 的蓝牙传出 COM；配对后在 Windows 更多蓝牙设置的 COM 端口中查看", "alarm")
            return
        if port.upper() == self._cur_port().upper() or (
                self.ser and port.upper() == self.ser.port.upper()):
            self.log("小车蓝牙串口不能与 FPGA 串口相同", "alarm")
            return
        if self.car_ser:
            return
        try:
            # Windows 蓝牙 SPP 经系统转换为虚拟串口。这里的 9600 不是
            # 对 HC-05 无线侧/模块 UART 的 AT 配置，不能据此证明车端波特率。
            self.lbl_car_link.config(text="正在打开蓝牙串口…", foreground="#9A3412")
            self.root.update_idletasks()
            self.car_ser = serial.Serial(port, 9600, timeout=0.1, write_timeout=0.5)
        except Exception as e:
            self.car_ser = None
            self.lbl_car_link.config(text="蓝牙连接失败", foreground="#a00")
            self.log("打开小车蓝牙串口失败：%s" % e, "alarm")
            return
        self.car_route_var.set("电脑蓝牙直连")
        self.car_route_combo.config(state="disabled")
        self.car_port_combo.config(state="disabled")
        self.btn_car_connect.config(text="断开小车")
        self.lbl_car_link.config(text="蓝牙串口已打开 %s" % port, foreground="green")
        self.log("小车蓝牙串口已打开：%s；路径切换为电脑蓝牙直连，按钮发单字节，不带 C。未发送运动命令。" % port, "info")

    def _disconnect_car(self, send_stop=True):
        if self.ble_link:
            self._disconnect_ble(stop=send_stop)
        uart = self.car_ser
        self.car_ser = None
        if uart:
            try:
                if send_stop:
                    if uart.write(b"S") != 1:
                        raise IOError("停车字节未完整写入")
                    self.log("蓝牙断开前已写入停车 S；未确认车端执行", "info")
            except Exception as e:
                self.log("蓝牙停车写入失败：%s" % e, "alarm")
            finally:
                try:
                    uart.close()
                except Exception:
                    pass
        self.car_port_combo.config(state="readonly")
        self.car_route_combo.config(state="readonly")
        self.btn_car_connect.config(text="连接小车")
        self.lbl_car_link.config(text="蓝牙未连接", foreground="#a00")
        # 保留用户选择的路径，断线后不自动回退到 FPGA。

    def _send_raw(self, data, what):
        self._diag_write("FPGA_TX_REQUEST", repr(data))
        if not self.ser or not self.running:
            self.log("未连接，无法发送", "alarm")
            return False
        try:
            if self.ser.write(data) != len(data):
                raise IOError("FPGA 指令字节未完整写入")
        except Exception as e:
            self.log("发送失败: %s" % e, "alarm")
            self.auto.pause("FPGA 指令写入失败，停车等待恢复", need_lp=True)
            return False
        if what:
            self.log("已发送: %s" % what, "info")
        return True

    def _send_car(self, ch):
        """按显式路径发送：FPGA 用 C 前缀；HC-05 直连只发原命令字节。"""
        if ch == "S" and self.auto.locked:
            self.auto.disarm("手动停车，已退出自动联动")
            self._auto_service()
            return True
        if self.auto.locked:
            self.log("自动联动或等待恢复期间仅允许停车；先退出自动联动再手动遥控", "alarm")
            return False
        self._diag_write("CAR_REQUEST", json.dumps({"command": ch, "route": self.car_route_var.get(),
                        "selected_car": self.car_port_var.get()}, ensure_ascii=False))
        if len(ch) != 1 or ch not in "FBLRS+-0123456789":
            self.log("不支持的小车按钮命令", "alarm")
            return False
        if self.car_route_var.get() == "BLE直连":
            if not self.ble_link or not self.ble_link.ready:
                self.log("BLE 尚未连接就绪，指令未发送", "alarm")
                return False
            try:
                if ch == "S" and hasattr(self.ble_link, "cancel_auto"):
                    self.ble_link.cancel_auto()
                request_id = self.ble_link.send(ch)
                if request_id is None and isinstance(self.ble_link, BleCarLink):
                    self.log("BLE 队列忙或停车待写入，命令未排队", "info")
                    return False
            except Exception as error:
                self.log("BLE 指令提交失败：%s" % error, "alarm")
                self._disconnect_ble(stop=False)
                return False
            self.log("已提交 BLE 命令 %s，等待写入结果" % ch, "info")
            return True
        if self.car_route_var.get() == "电脑蓝牙直连":
            if not self.car_ser:
                self.log("小车蓝牙未连接，指令未发送；先选择 HC-05 传出串口并连接", "alarm")
                return False
            try:
                if self.car_ser.write(ch.encode("ascii")) != 1:
                    raise IOError("命令字节未完整写入")
            except Exception as e:
                self.log("小车蓝牙发送失败：%s" % e, "alarm")
                self._disconnect_car(send_stop=False)
                return False
            self.log("已写入小车蓝牙 %s：%s (0x%02X)，未确认车端执行" %
                     (self.car_ser.port, ch, ord(ch)), "info")
            return True
        return self._send_fpga_car(ch)

    def _send_fpga_car(self, ch):
        # Frozen FPGA normalizes B/speed/other bytes to S before transmission.
        if ch not in "FBSLR":
            self.log("当前 FPGA 不透传调速字节；请使用前进、后退、停车、左转、右转", "alarm")
            return False
        self.manual_car_token=getattr(self,"manual_car_token",0)+1
        token=self.manual_car_token
        d=self.detection
        if ch=="S":
            sent=self._send_raw(b"CS","立即停车")
            if time.monotonic()<getattr(self,"manual_escape_until",0):
                def clear_escape():
                    if token==self.manual_car_token and self.running and self.ser and self.car_route_var.get()=="FPGA转发":
                        self._send_raw(b"CS",None)
                        self.manual_escape_until=0
                self.root.after(80,clear_escape)
            if self.running and self.ser and (d.pending is not None or not d.applied or d.applied.traffic):
                self._request_detection(False,d.applied.profile if d.applied else 0)
            return sent
        if (d.supported and d.applied and not d.applied.traffic and
                d.pending is None and d.error is None and d.last_status_at is not None and
                self.auto.clock()-d.last_status_at<=1.0):
            return self._write_fpga_action(ch,token)
        # Automatic drawing events share the same untagged action bytes. Stop
        # first, then wait for a matching traffic-off acknowledgment to give
        # manual commands sole ownership of the car link.
        if not self._send_raw(b"CS","手动接管前停车"): return False
        if not (d.pending and d.pending[1] is False):
            if not self._request_detection(False,d.applied.profile if d.applied else 0): return False
        epoch=d.pending[0]
        generation=self.fpga_generation
        deadline=time.monotonic()+2.0
        def finish():
            if token!=self.manual_car_token or generation!=self.fpga_generation or self.closing: return
            if self.car_route_var.get()!="FPGA转发" or not self.running or not self.ser: return
            if d.pending is None and d.applied and d.applied.epoch==epoch and not d.applied.traffic and d.error is None:
                self._write_fpga_action(ch,token)
            elif time.monotonic()>=deadline or d.error:
                self._send_raw(b"CS","接管未确认，保持停车")
                self.log("未收到交通关闭回读，运动指令已取消；请检查 FPGA 连接", "alarm")
            else: self.root.after(20,finish)
        self.root.after(20,finish)
        return True

    def _write_fpga_action(self,ch,token,reset_escape=False):
        if time.monotonic()<getattr(self,"manual_escape_until",0) and not reset_escape:
            # Cancel an incomplete escape immediately, then leave enough quiet
            # time for the next S to be an unambiguous ordinary stop.
            self._send_raw(b"CS","中止未完成的后退编码")
            generation=self.fpga_generation
            def restart():
                d=self.detection
                if (token==self.manual_car_token and generation==self.fpga_generation and
                    self.running and self.ser and self.car_route_var.get()=="FPGA转发" and
                    d.pending is None and d.applied and not d.applied.traffic and not d.error):
                    self._send_raw(b"CS",None)
                    self.manual_escape_until=0
                    self._write_fpga_action(ch,token,True)
            self.root.after(80,restart)
            return True
        if ch!="B":
            return self._send_raw(b"C"+ch.encode("ascii"),"小车遥控 C%s" % ch)
        # The frozen relay accepts only F/S/L/R. The MCU consumes this reserved
        # escape without energizing intermediate directions; H is ignored.
        # Pace one C-prefixed byte per 12 ms; do not fill the FPGA's 16 slots.
        signature="SSSLRLRFRLF"
        self.manual_escape_until=time.monotonic()+.8
        generation=self.fpga_generation
        def step(index):
            if token!=self.manual_car_token or generation!=self.fpga_generation or self.closing: return
            if self.car_route_var.get()!="FPGA转发" or not self.running or not self.ser: return
            d=self.detection
            if d.pending is not None or not d.applied or d.applied.traffic or d.error:
                self._send_raw(b"CS","后退编码被交通配置变化中止")
                return
            if not self._send_raw(b"C"+signature[index].encode("ascii"),None): return
            if index+1<len(signature): self.root.after(12,lambda:step(index+1))
            else:
                self.manual_escape_until=time.monotonic()+.08
                self.log("已发送后退编码：C8T6 执行一次1000ms后退，停车可随时打断", "info")
        if not self._send_raw(b"CS",None): return False
        # Isolate the prefix from an earlier STOP, allowing for radio packet
        # batching. All motor inputs remain stopped while the escape is sent.
        self.root.after(180,lambda:step(0))
        return True

    def _send_speed(self):
        s = (self.spd_var.get() or "").strip()
        if len(s) != 1 or not ("0" <= s <= "9"):
            self.log("速度只能是 0~9 一位数字", "alarm")
            return
        self._send_car(s)

    def _send_lp(self, on):
        """低功耗按需降频开关：L1 开 / L0 关。关掉之后 OSD 上的 LP 就不再出现。"""
        if on and self.auto.locked:
            self.log("自动联动期间保持 L0 全速；请先停车退出联动", "alarm")
            return
        self._send_raw(b"L1" if on else b"L0",
                       "低功耗 %s" % ("开 (L1)" if on else "关 (L0)"))

    def _on_close(self):
        self.closing = True
        if self.auto.locked:
            self.auto.disarm("关闭上位机，撤销自动运动")
            self._auto_service()
        self.log("正在关闭应用；将尽力发送停车并释放连接", "info")
        if self.running and self.ser:
            self.manual_car_token=getattr(self,"manual_car_token",0)+1
            self._send_raw(b"CS","关闭上位机前停车")
            self._request_detection(False,0)
        self._disconnect_car()
        self.running = False
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        try:
            if self.csv_file:
                self.csv_file.close()
        except Exception:
            pass
        try:
            if self.diag_file:
                self.diag_file.close()
        except Exception:
            pass
        try:
            for callback in self.root.tk.splitlist(self.root.tk.call("after","info")):
                self.root.after_cancel(callback)
            self.root.update_idletasks()
        except tk.TclError:
            pass
        self.root.destroy()


def main():
    root = tk.Tk()
    MonitorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
