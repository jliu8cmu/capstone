
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from swimtrack import packet as pk

import csv






@dataclass
class AnchorConfig:
    x: float = 0.0
    y: float= 0.0
    height: float = 2.0 #estimate
    yaw_deg: float = 0.0
    tilt_deg: float = 20.0 #rough estimate
    tag_height: float =0.0
    az_sign: float = 1.0
    el_sign: float = 1.0
    az_offset_deg: float = 0.0
    el_offset_deg: float = 0.0
    max_range: float = 60.0

@dataclass
class PoolConfig:
    length: float = 25.0 
    lane_width: float = 2.13 #adjust to pool
    lanes: int = 8

    @property
    def width(self)->float:
        return self.lane_width * self.lanes


def _rotation(cfg: AnchorConfig) -> np.ndarray:
    yaw, tilt = np.radians(cfg.yaw_deg), np.radians(cfg.tilt_deg)
    rz = np.array([[np.cos(yaw), -np.sin(yaw), 0.0],
                     [np.sin(yaw), np.cos(yaw), 0.0],
                     [0.0, 0.0, 1.0]])
    ry = np.array([[np.cos(tilt), 0.0, np.sin(tilt)],
                        [0.0, 1.0, 0.0],
                        [-np.sin(tilt), 0.0, np.cos(tilt)]])
    return rz @ ry

def _anchor_pos(cfg: AnchorConfig)->np.ndarray:
    return np.array([cfg.x, cfg.y, cfg.height])

def angles_to_position(az_deg: float, el_deg: float, cfg:AnchorConfig)-> Optional[Tuple[float, float]]:
    az = np.radians(cfg.az_sign * az_deg + cfg.az_offset_deg)
    el = np.radians(cfg.el_sign * el_deg + cfg.el_offset_deg)


    d_anchor = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    d = _rotation(cfg) @ d_anchor
    if d[2] >= -1e-9:
        return None
    a = _anchor_pos(cfg)
    t = (cfg.tag_height - a[2] / d[2])
    p = a + t*d

    if np.hypot(p[0] - a[0], p[1] - a[1]) > cfg.max_range:
        return None
    return float(p[0]), float(p[1])

def position_to_angles(x: float, y:float , cfg: AnchorConfig) ->Tuple[float, float]:
    d = np.array([x,y,cfg.tag_height]) - _anchor_pos(cfg)
    d_anchor = _rotation(cfg).T @ d

    az = np.degrees(np.arctan2(d_anchor[1], d_anchor[0]))
    el = np.degrees(np.arctan2(d_anchor[2], np.hypot(d_anchor[0], d_anchor[1])))
    return (float((az-cfg.az_offset_deg) * cfg.az_sign), float((el - cfg.el_offset_deg) *  cfg.el_sign))

def lane_of(y: float, pool: PoolConfig)->Optional[int]:
    if not 0.0 <= y < pool.width:
        return None

    return int(y // pool.lane_width) + 1

def in_pool(x: float, y:float, pool:PoolConfig, margin:float = 0.0) ->bool:
    return -margin <= x<= pool.length + margin and -margin <= y <= pool.width + margin

