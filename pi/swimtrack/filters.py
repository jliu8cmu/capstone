from collections import deque

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from swimtrack.ublox import AngleReading

import numpy as np

from swimtrack.ublox import AngleReading


@dataclass
class FilterConfig:
    min_rssi: int = -85
    median_len: int = 3
    max_gap_ms: int = 1000

class AngleFilter:
    def __init__(self, cfg: Optional[FilterConfig] = None):
        self.cfg = cfg or FilterConfig()
        self.window = deque(maxlen=self.cfg.median_len)
        self.last_ts: Optional[int] = None
        self.rejected_weak = 0


    def update(self, r: AngleReading)-> Optional[Tuple[float, float]]:
        if r.rssi < self.cfg.min_rssi:
            self.rejected_weak+= 1
            return None
        
        if self.last_ts is not None and r.timestamp_ms - self.last_ts > self.cfg.max_gap_ms:
            self.window.clear()
        self.last_ts = r.timestamp_ms
        self.window.append((r.azimuth, r.elevation))
        az, el = np.median(np.array(self.window), axis = 0)
        return float(az), float(el)