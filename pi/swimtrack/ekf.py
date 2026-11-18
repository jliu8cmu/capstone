from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from swimtrack.geometry import AnchorConfig, angles_to_position, position_to_angles


@dataclass
class EkfConfig:
    accel_std: float=2.0
    angle_std_deg: float = 1.5
    init_pos_std: float = 1.0
    init_vel_std: float = 1.0
    gate: float=13.8
    reset_after_s: float = 5.0
    max_rejects: int = 10

class SwimmerEKF:
    def __init__(self, anchor:AnchorConfig, cfg:Optional[EkfConfig] = None):
        self.anchor = anchor
        self.cfg = cfg or EkfConfig()
        self.x : Optional[np.ndarray] = None
        self.P = np.eye(4)
        self.t_ms = 0.0
        self.R = np.eye(2) * self.cfg.angle_std_deg ** 2
        self.rejected = 0
        self.reject_streak = 0
        self.restarts= 0

    def _start(self, az: float, el: float, t_ms: float) ->bool:
        pos = angles_to_position(az, el, self.anchor)
        if pos is None:
            return False
        self.x = np.array([pos[0], pos[1], 0.0,0.0])
        p, v = self.cfg.init_pos_std **2, self.cfg.init_vel_std **2
        self.P = np.diag([p,p,v,v])
        self.t_ms = t_ms
        return True

    @staticmethod
    def _transition(dt: float) -> np.ndarray:
        F = np.eye(4)
        F[0,2] = F[1,3] = dt
        return F

    def _process_noise(self, dt:float)->np.ndarray:
        q = self.cfg.accel_std ** 2
        a, b, c = dt**4 / 4, dt ** 3 / 2, dt **2
        return q * np.array([[a,0,b,0],
                             [0,a,0,b],
                             [b,0,c,0],
                             [0,b,0,c]])


    def predict(self, t_ms: float) ->None:
        dt = (t_ms - self.t_ms) / 1000.0
        if self.x is None or dt <= 0:
            return
        F = self._transition(dt)
        self.x  = F @ self.x
        self.P = F @ self.P @ F.T + self._process_noise(dt)

        self.t_ms = t_ms

    def _jacobian(self, px: float, py: float, eps: float = 1e-3) -> np.ndarray:
        H = np.zeros((2,4))
        for i, (dx,dy) in enumerate(((eps, 0.0), (0.0, eps))):
            hi = np.array(position_to_angles(px+dx, py+dy, self.anchor))
            lo = np.array(position_to_angles(px-dx, py-dy, self.anchor))
            H[:,i] = (hi-lo) / (2*eps)

        return H


    def update(self, az: float, el:float, t_ms:float) ->bool:
        if self.x is None or (t_ms - self.t_ms) / 1000.0 > self.cfg.reset_after_s:
            return self._start(az, el, t_ms)
        self.predict(t_ms)

        h = np.array(position_to_angles(self.x[0], self.x[1], self.anchor))
        y = np.array([az, el])- h
        H = self._jacobian(self.x[0], self.x[1])
        S = H @self.P @ H.T + self.R
        S_inv = np.linalg.inv(S)

        if y @ S_inv @ y > self.cfg.gate:
            self.rejected += 1
            self.reject_streak += 1
            if self.reject_streak >= self.cfg.max_rejects:
                self.restarts += 1
                self.reject_streak = 0
                return self._start(az, el, t_ms)
            return False

        self.reject_streak = 0

        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ y
        I_KH = np.eye(4) - K @ H
        self.P  = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        return True


    def state_at(self, t_ms:float) -> Optional[np.ndarray]:
        if self.x is None:
            return None
        dt = max(0.0, (t_ms - self.t_ms) / 1000.0)
        return self._transition(dt) @ self.x

    @property
    def position(self) -> Optional[Tuple[float,float]]:
        return None if self.x is None else (float(self.x[0]), float(self.x[1]))

    @property
    def speed(self) -> Optional[float]:
        return None if self.x is None else float(np.hypot(self.x[2], self.x[3]))
