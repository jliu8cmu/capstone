from collections import deque
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from swimtrack.geometry import AnchorConfig, PoolConfig

@dataclass
class WallConfig:
    near_zone_m: float = 3.0
    far_zone_m: float = 4.5
    reverse_speed: float = 0.3
    min_length_s: float = 8.0
    pushoff_g: float = 1.8
    pushoff_search_s: float = 3.0
    suppress_before_s: float = 0.5
    suppress_after_s: float =1.0
    suppress_default_s: float = 2.5


@dataclass
class WallEvent:
    end: int
    touch_ms: float
    pushoff_ms: Optional[float]
    suppress_from_ms: float
    suppress_to_ms: float



class WallDetector:
    def __init__(self, pool: PoolConfig, anchor: AnchorConfig, cfg: Optional[WallConfig] = None):
        self.cfg =cfg or WallConfig()
        self.ends = (0.0, pool.length)
        near = 0 if abs(anchor.x - 0.0) <= abs(anchor.x - pool.length) else 1
        self.zones = tuple(self.cfg.near_zone_m if e == near else self.cfg.far_zone_m for e in (0,1))

        self.heading = 0
        self.zone_end: Optional[int] = None
        self.closest_d = np.inf
        self.closest_t = 0.0
        self.last_wall_t = -np.inf
        self.peaks = deque(maxlen=512)
        self.events: List[WallEvent] = []

    def update_imu(self, t_ms, accel_g) -> None:
        mag = np.linalg.norm(np.asarray(accel_g, dtype= float).reshape(-1,3), axis = 1)
        for t, g in zip(np.asarray(t_ms, dtype=float), mag):
            if g >= self.cfg.pushoff_g:
                self.peaks.append((t,g))
        if self.events:
            self._refine(self.events[-1])

    def _refine(self, ev: WallEvent) -> None:
        pushoff = self._pushoff(ev.touch_ms)
        if pushoff is not None and pushoff != ev.pushoff_ms:
            ev.pushoff_ms = pushoff
            ev.suppress_to_ms = pushoff + self.cfg.suppress_after_s * 1000

    def _zone(self, x:float)->Optional[int]:
        for end in (0,1):
            if abs(x-self.ends[end]) < self.zones[end]:
                return end
        return None

    def _pushoff(self, touch_ms: float) -> Optional[float]:
        end = touch_ms + self.cfg.pushoff_search_s * 1000
        found = [(g,t) for t, g in self.peaks if touch_ms <= t <= end]
        return max(found)[1] if found else None

    def update_position(self, t_ms: float, x:float, vx:float) ->Optional[WallEvent]:
        end = self._zone(x)
        if end is None:
            self.zone_end = None
            self.closest_d = np.inf
        else:
            d = abs(x - self.ends[end])
            if end != self.zone_end:
                self.zone_end, self.closest_d, self.closest_t = end, d, t_ms
            elif d < self.closest_d:
                self.closest_d, self.closest_t = d, t_ms


        sign = int(np.sign(vx)) if abs(vx) > self.cfg.reverse_speed else 0
        if sign == 0:
            return None

        reversed_here = (self.heading != 0 and sign != self.heading and self.zone_end is not None and t_ms - self.last_wall_t >= self.cfg.min_length_s * 1000)

        if reversed_here:
            self.heading = sign
            self.last_wall_t = t_ms
            return self._make_event(self.zone_end, self.closest_t)
        if self.zone_end is None or self.heading == 0:
            self.heading = sign

        return None



    def _make_event(self, end: int, touch_ms: float) -> WallEvent:
        ev = WallEvent(end, touch_ms, None, touch_ms - self.cfg.suppress_before_s * 1000,
                       touch_ms + self.cfg.suppress_default_s * 1000)
        self.events.append(ev)
        self._refine(ev)
        return ev

    def strokes_in_length(self, stroke_times_ms) -> int:
        ev = self.events[-1]
        start = self.events[-2].suppress_to_ms if len(self.events) > 1 else -np.inf
        return sum(1 for t in stroke_times_ms if start < t < ev.suppress_from_ms)

    @property
    def lengths(self)-> int:
        return len(self.events)

