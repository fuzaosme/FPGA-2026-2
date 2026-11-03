"""Continuous-turn total durations; migrate old per-segment values once."""
import json
import os
import hashlib
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class TurnSettings:
    speed: int = 5
    left_ms: int = None
    right_ms: int = None

    def __post_init__(self):
        if type(self.speed) is not int or not 0 <= self.speed <= 9:
            raise ValueError('转向速度须为0～9档')
        for label,value in (('左转',self.left_ms),('右转',self.right_ms)):
            if value is not None and (type(value) is not int or not 150 <= value <= 9000 or value % 10):
                raise ValueError('%s90°总时长须为150～9000毫秒、10毫秒的整数倍；未标定可留空' % label)

    @classmethod
    def from_fields(cls,speed,left_ms,right_ms):
        def number(value,optional=False):
            text=str(value).strip()
            if optional and not text:return None
            if not text.isascii() or not text.isdecimal():
                raise ValueError('标定参数须填写整数，左右时长可留空')
            return int(text)
        return cls(number(speed),number(left_ms,True),number(right_ms,True))

    def duration(self,direction):
        if direction not in ('L','R'):raise ValueError('需要左或右方向')
        value=self.left_ms if direction=='L' else self.right_ms
        if value is None:raise ValueError('请先填写并保存%s90°总时长，再试转或自动转向' % ('左转' if direction=='L' else '右转'))
        if self.speed==0:raise ValueError('转向速度为0档，请设置非零档位并重新标定')
        return value/1000.

    @classmethod
    def load(cls,path):
        path=Path(path)
        if not path.exists():return cls()
        raw=path.read_bytes()
        data=json.loads(raw.decode('utf-8-sig'))
        if not isinstance(data,dict) or type(data.get('version')) is not int or data['version'] not in (1,2):
            raise ValueError('不支持的转向标定文件')
        values={key:data[key] for key in ('speed','left_ms','right_ms')}
        if data['version']==2:return cls(**values)
        # V1 stored 30-degree segment durations; V2 stores one total duration.
        for key in ('left_ms','right_ms'):
            value=values[key]
            if value is not None:
                if type(value) is not int or not 50 <= value <= 3000 or value % 10:
                    raise ValueError('旧版单段时长无效，未迁移标定文件')
                values[key]=value*3
        settings=cls(**values)
        backup=path.with_name(path.stem+'.v1_backup.json')
        if backup.exists() and backup.read_bytes()!=raw:
            backup=path.with_name(path.stem+'.v1_backup_'+hashlib.sha256(raw).hexdigest()[:12]+'.json')
        if not backup.exists():backup.write_bytes(raw)
        settings.save(path)
        return settings

    def save(self,path):
        path=Path(path)
        temporary=path.with_name(path.name+'.tmp')
        temporary.write_text(json.dumps(dict(version=2,**asdict(self)),indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
        os.replace(str(temporary),str(path))
