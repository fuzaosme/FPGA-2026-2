"""Arrow V1 wire contract; no hardware I/O. CRC excludes '$' and '*'."""
import binascii
import re
from dataclasses import dataclass

LAMP_VALID, SIGN_VALID, CONFLICT, CONFIGURED = 1, 2, 4, 8


def frame(body):
    crc = binascii.crc_hqx(body.encode('ascii'), 0xffff)
    return '${}*{:04X}\r\n'.format(body, crc)


def body_of(text):
    text = text.rstrip('\r\n')
    if len(text) > 100 or not re.fullmatch(r'\$[\x20-\x7e]+\*[0-9A-F]{4}', text):
        raise ValueError('观测格式错误')
    body, crc = text[1:].rsplit('*', 1)
    if binascii.crc_hqx(body.encode('ascii'), 0xffff) != int(crc, 16):
        raise ValueError('观测 CRC 错误')
    return body


def hexnum(text, width):
    if not re.fullmatch('[0-9A-F]{%d}' % width, text):
        raise ValueError('十六进制字段错误')
    return int(text, 16)


def newer(value, previous):
    return previous is None or 0 < ((value - previous) & 0xffffffff) < 0x80000000


def config(epoch, lamp=(0, 0, 639, 719), sign=(640, 0, 1279, 719)):
    for box in (lamp, sign):
        x0, y0, x1, y1 = box
        if not (0 <= x0 < x1 < 1280 and 0 <= y0 < y1 < 720):
            raise ValueError('ROI 超出范围或过小')
    if not 0 <= epoch <= 0xffffffff:
        raise ValueError('会话越界')
    return frame('Q,1,{:08X},{}'.format(epoch, ','.join('{:03X}'.format(v) for v in lamp + sign)))


@dataclass(frozen=True)
class Observation:
    epoch: int
    seq: int
    lamp_frame: int
    color: str
    lamp_age: int
    sign_frame: int
    direction: str
    score: int
    sign_age: int
    flags: int


def parse(text):
    f = body_of(text).split(',')
    if len(f) != 12 or f[:2] != ['V', '1']:
        raise ValueError('需要 V1 观测')
    if f[5] not in ('R', 'Y', 'G', 'N', 'U') or f[8] not in ('L', 'R', 'N', 'U'):
        raise ValueError('识别类别错误')
    if not re.fullmatch(r'[0-9]{3}', f[9]) or int(f[9]) > 100:
        raise ValueError('分数错误')
    o = Observation(hexnum(f[2], 8), hexnum(f[3], 8), hexnum(f[4], 8), f[5],
                    hexnum(f[6], 4), hexnum(f[7], 8), f[8], int(f[9]),
                    hexnum(f[10], 4), hexnum(f[11], 2))
    if o.flags & ~15:
        raise ValueError('未知标志位')
    if o.flags & LAMP_VALID and o.color not in ('R', 'Y', 'G'):
        raise ValueError('灯有效位不一致')
    if o.flags & SIGN_VALID and o.direction not in ('L', 'R'):
        raise ValueError('方向有效位不一致')
    return o


def encode(o):
    return frame('V,1,{:08X},{:08X},{:08X},{},{:04X},{:08X},{},{:03d},{:04X},{:02X}'.format(
        o.epoch, o.seq, o.lamp_frame, o.color, o.lamp_age, o.sign_frame,
        o.direction, o.score, o.sign_age, o.flags))
