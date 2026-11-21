from dataclasses import dataclass

from typing import List, Optional

import numpy as np
from scipy import signal

SAMPLE_PERIOD_MS = 1000.0 / 104.0

@dataclass
class StrokeConfig:
    fs_hz: float = 104.0
    axis_tau_s: float = 5.0
    cutoff_hz: float = 0.8
    min_peak_dps: float = 80.0
    min_rise_dps: float = 150.0
    min_interval_s: float = 0.8

    rate_window: int = 4
    stale_s: float = 3.0
    max_interval_s: float = 4.0

@dataclass
class StrokeEvent:
    t_ms: float
    peak_dps: float

class StrokeDetector:
    def __init__(self, cfg: Optional[StrokeConfig] = None):
        self.cfg = cfg or StrokeConfig()
        self.sos = signal.butter(2, self.cfg.cutoff_hz, fs=self.cfg.fs_hz, output = 'sos')
        self.zi = None
        self.alpha = 1.0 - np.exp(-1.0/ (self.cfg.fs_hz * self.cfg.axis_tau_s))
        self.mean = np.zeros(3)
        self.cov = np.eye(3)
        self.axis = np.array([1.0,0.0,0.0])
        self.samples = 0
        self.prev2 = self.prev1 = None
        self.prev1_t = None
        self.valley = np.inf
        self.last_t: Optional[float] = None
        self.events: List[StrokeEvent] = []
    def update(self, t_ms, gyro_dps) -> List[StrokeEvent]:
        t_ms = np.asarray(t_ms, dtype = float)
        gyro = np.asarray(gyro_dps, dtype=float).reshape(-1,3)
        if len(gyro) == 0:
            return []

        proj = self._project(gyro)
        if self.zi is None:
            self.zi = signal.sosfilt_zi(self.sos) * proj[0]
        y, self.zi = signal.sosfilt(self.sos, proj, zi=self.zi)

        new = []
        for ti, yi in zip(t_ms, y):
            if self.prev2 is not None and self.prev1 > self.prev2 and self.prev1 >= yi:
                if (self._accept(self.prev1_t, self.prev1)) :
                    ev = StrokeEvent(self.prev1_t, float(self.prev1))
                    new.append(ev)
                    self.events.append(ev)
                    self.last_t = self.prev1_t
                    self.valley = np.inf


            self.valley = min(self.valley, yi)
            self.prev2, self.prev1, self.prev1_t = self.prev1, yi, ti
        return new

    def _project(self, gyro: np.ndarray)-> np.ndarray:
        out = np.empty(len(gyro))
        for k, g in enumerate(gyro):
            self.mean += self.alpha * (g-self.mean)
            d = g-self.mean
            self.cov += self.alpha * (np.outer(d,d) - self.cov)
            self.samples += 1
            if self.samples % 5 == 0:
                axis = np.linalg.eigh(self.cov)[1][:,-1]
                self.axis = axis if axis @ self.axis >= 0 else -axis
            out[k] = d @ self.axis
        return out

    def _accept(self, t, peak) -> bool:
        if (peak < self.cfg.min_peak_dps) or (peak- self. valley < self.cfg.min_rise_dps):
            return False
        if self.last_t is not None and t- self.last_t < self.cfg.min_interval_s * 1000:
            return False
        return True

    def update_packet(self, p) -> List[StrokeEvent]:
        t = p.t0_ms + np.arange(p.n) * SAMPLE_PERIOD_MS
        return self.update(t, p.gyro_dps)

    @property
    def count(self) -> int:
        return len(self.events)


    def discard_between(self, t0_ms:float, t1_ms: float) -> int:
        keep = [e for e in self.events if not t0_ms <= e.t_ms <= t1_ms]
        removed = len(self.events) - len(keep)
        self.events = keep
        return removed
    

    def rate_spm(self, now_ms: float) -> Optional[float]:
        if (len(self.events) < 2) or (now_ms - self.events[-1].t_ms > self.cfg.stale_s * 1000):
            return None
        times = np.array([e.t_ms for e in self.events[-(self.cfg.rate_window + 1):]])
        gaps = np.diff(times)
        gaps = gaps[gaps <= self.cfg.max_interval_s * 1000]
        if len(gaps) == 0:
            return None
        return 60000.0 /  float(np.median(gaps))


