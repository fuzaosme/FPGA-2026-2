"""IQ1 read-only telemetry and developer display-source command. No car I/O.

P1 has one 352-bit snapshot, 88 uppercase hex digits. Packed field layout is
shared with camera_iq.v, isp_top.v and design_top_wrapper.v. CRC is the existing
CCITT-FALSE wire CRC, excluding '$' and '*'.
"""
from dataclasses import dataclass
import re
from vision_protocol import frame, body_of

VIEWS = ('normal', 'raw', 'demosaic', 'awb', 'quant', 'edges')
STAGES = ('RAW灰度', '去马赛克', '识别输入（白平衡后）', '最终显示')


def debug_view(view):
    if isinstance(view, str):
        if view not in VIEWS:
            raise ValueError('未知诊断画面')
        view = VIEWS.index(view)
    if type(view) is not int or view not in range(6):
        raise ValueError('诊断画面必须为 0..5')
    return frame('I,1,%d' % view)


@dataclass(frozen=True)
class CameraStatus:
    build_id: str
    exposure: int
    gain_index: int
    ready: bool
    fault: bool
    settling: bool
    at_limit: bool
    i2c_errors: int
    stage: int
    view: int
    frame_id: int
    mean_rgb: tuple
    min_y: int
    max_y: int
    sample_count: int
    histogram: tuple
    statistics_valid: bool
    neutral_samples: int
    awb_valid: bool
    gain_r: float
    gain_b: float
    quant_black: int
    edge_pixels: int

    @property
    def analog_gain(self):
        i = self.gain_index - 16
        return 1 + i / 32 if i < 32 else 2 + (i - 32) / 16 if i < 64 else 4 + (i - 64) / 8


def parse_camera(text):
    parts = body_of(text).split(',')
    if len(parts) != 3 or parts[:2] != ['P', '1'] or not re.fullmatch('[0-9A-F]{88}', parts[2]):
        raise ValueError('相机状态格式错误')
    w = int(parts[2], 16)

    def bits(lo, width):
        return (w >> lo) & ((1 << width) - 1)

    stage, view = bits(268, 3), bits(271, 3)
    if stage > 3 or view > 5:
        raise ValueError('相机诊断阶段越界')
    count = bits(72, 16)
    histogram = tuple(bits(s, 16) for s in (56, 40, 24, 8))
    valid = bool(bits(0, 1))
    if bits(312,1) and not (3<=bits(288,16)<=1990 and 16<=bits(304,8)<=112):
        raise ValueError('相机已读回参数越界')
    if valid and (not count or sum(histogram) != count):
        raise ValueError('相机直方图计数不一致')
    return CameraStatus(
        '%08X' % bits(320, 32), bits(288, 16), bits(304, 8),
        bool(bits(312, 1)), bool(bits(313, 1)), bool(bits(314, 1)), bool(bits(315, 1)), bits(316, 4),
        stage, view, bits(128, 32), tuple(bits(s, 8) for s in (120, 112, 104)),
        bits(96, 8), bits(88, 8), count, histogram, valid,
        bits(160, 20), bool(bits(260, 1)), bits(180, 20) / 65536, bits(200, 20) / 65536,
        bits(220, 20), bits(240, 20))
