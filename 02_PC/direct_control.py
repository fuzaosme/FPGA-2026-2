"""One timed continuous turn then 1s cooldown; no hardware I/O.

Recognition never queues jobs during a turn or the final cooldown.
Timers start at actual transport dispatch, rather than queue admission.
"""
import time
from auto_interlock import Intent
from turn_settings import TurnSettings


class DirectControl:
    COOLDOWN=1.0
    TASK_STATES={'PRE_STOP','SET_SPEED','WAIT_TURN','TURNING','STOPPING','COOLDOWN'}
    STATES={'OFF':'联动关闭','READY':'等待新识别','FORWARD':'前进 F','STOPPED':'停车 S',
            'NEEDS_SETUP':'箭头时长未设置','PRE_STOP':'转向准备停车','SET_SPEED':'设置转向速度',
            'WAIT_TURN':'等待转向发送','TURNING':'连续转向约90°','STOPPING':'转向停车写入中',
            'COOLDOWN':'停车，1秒后恢复联动'}

    def __init__(self,clock=None,settings=None):
        self.clock=clock or time.monotonic
        self.settings=settings or TurnSettings()
        self.state='OFF'
        self.reason='箭头连续转向；结束停车1秒后恢复箭头和灯色'
        self.session=0
        self.generation=None
        self.links_ready=False
        self.vision_generation=None
        self.testing=False
        self.task_id=0
        self.direction=None
        self.turn_ms=None
        self.deadline=None
        self._intents=[]
        self._expected=None
        self._request_id=None
        self._resume_after=float('-inf')
        self._job_settings=None

    @property
    def active(self):return self.state!='OFF'

    @property
    def locked(self):return self.active

    @property
    def green_count(self):return 0

    @property
    def busy(self):return self.state in self.TASK_STATES

    @property
    def status_text(self):
        if self.busy:
            text='任务#%d %s 总时长%d ms | %s' % (self.task_id,'左转' if self.direction=='L' else '右转',
                                                self.turn_ms,self.STATES[self.state])
            if self.deadline is not None:text+=' %.1f秒' % max(0.,self.deadline-self.clock())
            return text
        return self.STATES[self.state]+' | '+self.reason

    @staticmethod
    def command_for(observation):
        if observation.direction in ('L','R'):return observation.direction
        return 'F' if observation.color=='G' else 'S'

    def configure(self,settings):
        if self.active:raise ValueError('先关闭联动或完成试转，再修改标定')
        if not isinstance(settings,TurnSettings):raise ValueError('无效的转向标定')
        self.settings=settings

    def drain(self):
        intents,self._intents=self._intents,[]
        return intents

    def _clear_task(self):
        self.deadline=None
        self._expected=self._request_id=None
        self.direction=None
        self.turn_ms=None
        self._job_settings=None

    def arm(self,generation):
        self.session+=1
        self.generation=generation
        self.links_ready=True
        self.testing=False
        self._clear_task()
        self._resume_after=self.clock()
        self.state='READY'
        self.reason='箭头连续转向；无箭头时绿→F，红/黄/未知→S'
        self._intents=[Intent('L0',self.session,'识别全速')]

    def disarm(self,reason='已关闭自动联动'):
        self.session+=1
        self.state='OFF'
        self.reason=reason
        self.testing=False
        self.links_ready=False
        self._clear_task()
        self._intents=[Intent('S',self.session,reason)]

    def pause(self,reason,need_lp=False,disconnected=False):
        if self.active:self.disarm(reason+'；任务已取消，请重新开启')

    def _request(self,command,state,reason):
        self.state=state
        self.reason='任务#%d %s' % (self.task_id,reason)
        self._expected=command
        self._request_id=None
        self.deadline=None
        self._intents=[Intent(command,self.session,self.reason)]

    def _begin_task(self,direction):
        self.settings.duration(direction)
        self.task_id+=1
        self.direction=direction
        self.turn_ms=self.settings.left_ms if direction=='L' else self.settings.right_ms
        self._job_settings=self.settings
        self._request('S','PRE_STOP','准备停车')

    def start_test(self,direction,generation):
        if self.active:raise ValueError('先关闭自动联动或等待当前试转完成')
        self.settings.duration(direction)
        self.arm(generation)
        self.testing=True
        self._begin_task(direction)

    def submitted(self,command,request_id,session):
        if session!=self.session or command!=self._expected:return
        if request_id is None:self.pause('BLE命令未排队')
        else:self._request_id=request_id

    def dispatched(self,command,request_id,session,sent_at):
        if (not self.active or session!=self.session or request_id!=self._request_id
                or command!=self._expected or self.state!='WAIT_TURN'):return
        self.state='TURNING'
        self.deadline=sent_at+self._job_settings.duration(self.direction)
        self.reason='任务#%d 已提交%s，连续转向%d ms；不发中间停车' % (
            self.task_id,self.direction,self.turn_ms)
        self.tick()

    def written(self,command,request_id,session,received_at=None):
        if (not self.active or session!=self.session or request_id is None
                or request_id!=self._request_id or command!=self._expected):return
        at=self.clock() if received_at is None else received_at
        if self.state=='PRE_STOP':
            self._request(str(self._job_settings.speed),'SET_SPEED','设置固定转向速度')
        elif self.state=='SET_SPEED':
            self._request(self.direction,'WAIT_TURN','提交一次连续转动')
        elif self.state=='STOPPING':
            self._expected=self._request_id=None
            self.state='COOLDOWN'
            self.deadline=at+self.COOLDOWN
            self.reason='任务#%d 已写入转向结束停车；1秒后恢复%s' % (self.task_id,'手动模式' if self.testing else '箭头和红绿灯触发')

    def tick(self):
        if self.deadline is None:return
        now=self.clock()
        if now+1e-9<self.deadline:return
        if self.state=='TURNING':
            self._request('S','STOPPING','标定时间到，停车')
        elif self.state=='COOLDOWN':
            self._resume_after=self.deadline
            self._clear_task()
            if self.testing:
                self.testing=False
                self.links_ready=False
                self.state='OFF'
                self.reason='试转结束并停车；请按实测角度调整标定'
            else:
                self.state='READY'
                self.reason='已恢复箭头和灯色触发，等待新报告'

    def _eligible(self,received_at,generation):
        return (self.active and self.links_ready and not self.testing and not self.busy
                and generation==self.generation and received_at>=self._resume_after)

    def observe(self,observation,received_at,generation):
        self.tick()
        if generation!=self.generation:return
        self.vision_generation=generation
        if not self._eligible(received_at,generation):return
        command=self.command_for(observation)
        if command in ('L','R'):
            try:self._begin_task(command)
            except ValueError as error:
                self.state='NEEDS_SETUP'
                self.reason=str(error)
                self._intents=[Intent('S',self.session,self.reason)]
        else:self._emit_light(command)

    def _emit_light(self,command):
        self.state='FORWARD' if command=='F' else 'STOPPED'
        self.reason='新灯色识别 → '+command
        self._intents=[Intent(command,self.session,self.reason)]

    def target(self,color,received_at,generation):
        self.tick()
        if self.vision_generation==generation:return
        if self._eligible(received_at,generation):self._emit_light('F' if color=='G' else 'S')

    def lp(self,*args):pass
    def stop_requested(self,*args):pass
