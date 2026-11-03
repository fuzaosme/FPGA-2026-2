"""Motion V5 configuration/status contract. No I/O or vehicle commands."""
from dataclasses import dataclass
import secrets
from vision_protocol import frame,body_of,hexnum

PROFILES=("标准","高灵敏","低灵敏")

@dataclass(frozen=True)
class Status:
    epoch:int
    traffic:bool
    profile:int
    ready:bool
    count:int

def configuration(epoch,traffic,profile):
    if type(epoch) is not int or not 0<=epoch<=0xffffffff:
        raise ValueError("配置会话无效")
    if type(traffic) is not bool or type(profile) is not int or profile not in range(3):
        raise ValueError("交通开关或灵敏度无效")
    return frame("D,1,{:08X},{},{}".format(epoch,int(traffic),profile))

def parse_status(text):
    p=body_of(text).split(",")
    if len(p)!=7 or p[:2]!=["S","1"] or p[3] not in ("0","1") or p[4] not in ("0","1","2") or p[5] not in ("0","1"):
        raise ValueError("运动状态报文无效")
    epoch=hexnum(p[2],8)
    count=hexnum(p[6],1)
    if count>6:raise ValueError("运动区域数超出范围")
    return Status(epoch,p[3]=="1",int(p[4]),p[5]=="1",count)

def encode_status(status):
    return frame("S,1,{:08X},{},{},{},{:X}".format(status.epoch,int(status.traffic),status.profile,int(status.ready),status.count))

class DetectionControl:
    def __init__(self):
        self.supported=False
        self.applied=None
        self.pending=None
        self.deadline=None
        self.error=None
        self.last_status_at=None

    @property
    def traffic_allowed(self):
        return bool(self.supported and self.applied and self.applied.traffic and self.pending is None and self.error is None)

    def request(self,traffic,profile,now):
        epoch=secrets.randbits(32) or 1
        if self.applied and epoch==self.applied.epoch:epoch=(epoch+1)&0xffffffff or 1
        payload=configuration(epoch,traffic,profile)
        self.pending=(epoch,traffic,profile)
        self.deadline=now+2.0
        self.error=None
        return payload

    def observe(self,status,now):
        self.supported=True
        self.last_status_at=now
        if self.pending:
            if (status.epoch,status.traffic,status.profile)!=self.pending:return False
            self.applied=status
            self.pending=None
            self.deadline=None
            self.error=None
            return True
        if self.applied and status.epoch==self.applied.epoch and status.traffic==self.applied.traffic and status.profile==self.applied.profile:
            self.applied=status
            return True
        return False

    def tick(self,now):
        if self.pending and now>=self.deadline:
            self.pending=None
            self.deadline=None
            self.applied=None
            self.error="配置未收到匹配回读；交通识别状态未知"
            return True
        if self.applied and self.last_status_at is not None and now-self.last_status_at>1.0:
            self.applied=None
            self.error="运动状态遥测超时；请重新连接"
            return True
        return False
