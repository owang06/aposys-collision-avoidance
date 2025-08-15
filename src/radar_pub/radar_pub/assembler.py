from __future__ import annotations
import time, struct
from typing import Dict, List, Optional
from dataclasses import dataclass, asdict

# keep upper 12 bits, drop lower 4 (= sensor index nibble)
FamilyMask = 0xF0F            # 1111-0000-1111
HEADER_BASE      = 0x60A    # short-range profile
OBJ_GENERAL_BASE = 0x60B
OBJ_WARN_BASE    = 0x60E

@dataclass(slots=True)
class HeaderMsg:
    sensor: int
    cycle:  int

@dataclass(slots=True)
class ObjectMsg:
    sensor: int
    obj_id: int
    dist_long_m: float
    dist_lat_m:  float
    v_long_mps:  float
    v_lat_mps:   float
    dyn_prop:    int
    obj_class:   int
    rcs_dbm2:    float
    warn_region_bits: Optional[int] = None
    wall_ts: float = 0.0

    @property
    def x_m(self) -> float:          # Cartesian fwd
        return self.dist_long_m

    @property
    def y_m(self) -> float:          # Cartesian left (+)
        return -self.dist_lat_m

    @property
    def is_collision(self) -> bool:  # convenient flag
        return bool(self.warn_region_bits)

@dataclass(slots=True)
class WarnMsg:
    sensor: int
    obj_id: int
    warn_region_bits: int

@dataclass(slots=True)
class BurstBundle:
    sensor: int
    cycle:  int
    start_ts: float
    objects: List[ObjectMsg]

def parse_frame(fid: int, data:bytes):
    base = fid & FamilyMask
    sid  = (fid >> 4) & 0x7

    if base == HEADER_BASE :
        return HeaderMsg(
            sensor=sid,
            cycle = int.from_bytes(data[1:3], "little"),
        )

    if base == OBJ_GENERAL_BASE:
        obj_id = data[0]
        dist_long = (((data[1] << 5) | (data[2] >> 3)) * 2) / 10 - 500
        dist_lat = (((data[2] & 0x07) << 8) | data[3]) * 2 / 10 - 204.6
        v_long = ((((data[4] << 8) | data[5]) >> 6) * 25) / 100 - 128
        dyn_prop  =  data[5] & 0x07
        v_lat     = (((data[5] & 0x3F) << 3) | (data[6] >> 5)) * 25 / 100 - 64
        obj_cls   = (data[6] >> 3) & 0x03
        rcs       =  data[7] * 0.5 - 64
        return ObjectMsg(
            sensor=sid,
            obj_id=obj_id,
            dist_long_m=dist_long,
            dist_lat_m=dist_lat,
            v_long_mps=v_long,
            v_lat_mps=v_lat,
            dyn_prop=dyn_prop,
            obj_class=obj_cls,
            rcs_dbm2=rcs,
        )

    if base == OBJ_WARN_BASE:
        return WarnMsg(
            sensor=sid,
            obj_id=data[0],
            warn_region_bits=data[1],
        )

class CycleAssembler:
    """Accumulates *ObjectMsg*s between successive headers.

    Parameters
    ----------
    init_period : float
        Starting guess of radar period in seconds (default 0.06).
    """
    def __init__(self, init_period:float= 0.06):
        self._period = init_period
        self._cycle: int | None = None
        self._start_ts = 0.0
        self._last_header_ts = 0.0
        self._buf: Dict[int, ObjectMsg] = {}
        self._ready: List[List[ObjectMsg]] = []

    def update_period(self, wall_ts:float):
        # update period estimate
        if self._last_header_ts:
            dt = wall_ts - self._last_header_ts
            self._period = 0.9 * self._period + 0.1 * dt

        self._last_header_ts = wall_ts

    def push(self, msg, wall_ts: float)-> List[List[ObjectMsg]]:
        if isinstance(msg, HeaderMsg):
            # update period estimate
            self.update_period(wall_ts)
            # flush previous burst (if any)
            if self._cycle is not None and msg.cycle != self._cycle:
                self._flush()
            self._cycle = msg.cycle
            self._start_ts = wall_ts
            self._sensor = msg.sensor
            return

        if self._cycle is None:
            return  # ignore until first header

        if isinstance(msg, ObjectMsg):
            msg.wall_ts = wall_ts
            self._buf[msg.obj_id] = msg

        elif isinstance(msg, WarnMsg):
            if msg.obj_id in self._buf:
                self._buf[msg.obj_id].warn_region_bits = msg.warn_region_bits

        # timeout guard: 2 × current period
        if wall_ts - self._start_ts > 2.0 * self._period:
            self._flush()

    def _flush(self):
        if not self._buf:
            self._cycle = None
            return

        bundle = BurstBundle(
            sensor   = self._sensor,
            cycle    = self._cycle,
            start_ts = self._start_ts,
            objects  = list(self._buf.values()),
        )
        self._ready.append(bundle)

        self._buf.clear()
        self._cycle = None

    def take_ready(self) -> List[BurstBundle]:
        """Return all completed bursts since last call and empty the queue."""
        out, self._ready = self._ready, []
        return out
