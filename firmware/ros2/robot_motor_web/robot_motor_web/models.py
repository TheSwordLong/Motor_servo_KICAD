"""HTTP validation deliberately rejects ambiguous units and non-finite numbers."""
import math
from typing import Dict, List, Optional
from pydantic import BaseModel, Field, StrictBool, StrictInt, validator

class Model(BaseModel):
    class Config:
        extra = 'forbid'
        allow_inf_nan = False

class Move(Model):
    mode: StrictInt = 0
    angle_deg: float
    motor_rpm: StrictInt = 0
    timeout_sec: float = 30.0
    @validator('mode')
    def mode_check(cls, v):
        if v not in (0, 1): raise ValueError('mode must be 0 or 1')
        return v
    @validator('angle_deg')
    def angle_check(cls, v):
        if not math.isfinite(v) or abs(v) > 214748364.7: raise ValueError('invalid degree coordinate')
        return v
    @validator('motor_rpm')
    def rpm_check(cls, v):
        if v != 0 and not 400 <= v <= 8000: raise ValueError('RPM must be 0 or 400..8000')
        return v
    @validator('timeout_sec')
    def timeout_check(cls, v):
        if not math.isfinite(v) or not .5 <= v <= 300: raise ValueError('timeout must be .5..300 seconds')
        return v

class Brake(Model):
    engage: StrictBool
class Direction(Model):
    high: StrictBool
class Manual(Model):
    enable: StrictBool
class Confirm(Model):
    confirm: StrictBool
    @validator('confirm')
    def confirmed(cls, v):
        if not v: raise ValueError('explicit confirmation required')
        return v
class Probe(Model):
    joint_name: str = ''
    scope: StrictInt = 0
    direction: StrictInt = 0
    timeout_sec: float = 60.0
    @validator('scope', 'direction')
    def enum_check(cls, v):
        if v not in (0, 1): raise ValueError('must be 0 or 1')
        return v
    _timeout = validator('timeout_sec', allow_reuse=True)(Move.timeout_check.__func__)

class Channel(Model):
    channel: StrictInt
    enabled: StrictBool
    role: StrictInt
    gear_ratio: StrictInt
    inverted: StrictInt
    rpm: StrictInt
    @validator('channel')
    def channel_check(cls, v):
        if not 0 <= v <= 3: raise ValueError('protocol channel must be 0..3')
        return v
    @validator('role')
    def role_check(cls, v):
        if not 0 <= v <= 7: raise ValueError('role must be 0..7')
        return v
    @validator('gear_ratio')
    def gear_check(cls, v):
        if v not in (0, 82, 290, 516): raise ValueError('unsupported gear ratio')
        return v
    @validator('inverted')
    def inverted_check(cls, v):
        if v not in (0, 1): raise ValueError('inverted must be 0 or 1')
        return v
    @validator('rpm')
    def rpm_check(cls, v):
        if not 400 <= v <= 8000: raise ValueError('RPM must be 400..8000')
        return v
class ConfigApply(Model):
    channels: List[Channel]
    confirm: bool = False
    @validator('channels')
    def unique(cls, v):
        if len(v) > 4 or len({x.channel for x in v}) != len(v): raise ValueError('duplicate channel')
        return v

class RuntimeOptions(Model):
    board_name: Optional[str] = None
    namespace: str
    mode: str = 'read_only'
    allowed_motion_joints: List[str] = Field(default_factory=list)
    allow_unreferenced_relative: StrictBool = False
    commissioning: StrictBool = False
    max_commissioning_step_deg: float = 10.0
    polling_hz: float = 5.0
    @validator('board_name')
    def name_check(cls, v):
        import re
        if v is not None and not re.fullmatch('[a-zA-Z][a-zA-Z0-9_]*', v): raise ValueError('invalid board name')
        return v
    @validator('namespace')
    def namespace_check(cls, v):
        import re
        if not re.fullmatch('[a-zA-Z][a-zA-Z0-9_]*', v): raise ValueError('invalid namespace')
        return v
    @validator('mode')
    def mode_check(cls, v):
        if v not in ('read_only', 'maintenance', 'motion', 'save_recovery'): raise ValueError('invalid mode')
        return v
    @validator('max_commissioning_step_deg')
    def step_check(cls, v):
        if not math.isfinite(v) or not .1 <= v <= 360: raise ValueError('step must be .1..360 degrees')
        return v
    @validator('polling_hz')
    def poll_check(cls, v):
        if not math.isfinite(v) or not 0 < v <= 100: raise ValueError('poll must be (0,100] Hz')
        return v
class RuntimeEdit(Model):
    revision: StrictInt
    options: RuntimeOptions
    board: dict
    joints: Dict[str, dict]
