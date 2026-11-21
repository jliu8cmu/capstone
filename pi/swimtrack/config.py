from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Optional

import yaml

from swimtrack.ekf import EkfConfig
from swimtrack.filters import FilterConfig
from swimtrack.geometry import AnchorConfig, PoolConfig
from swimtrack.strokes import StrokeConfig

from swimtrack.walls import WallConfig

from dataclasses import dataclass, field, fields, replace

def default_anchor() -> AnchorConfig:
    return AnchorConfig(x=0.0, y=PoolConfig().width / 2, height=2.5, yaw_deg=0.0, tilt_deg=15.0)

@dataclass
class Settings:
    pool: PoolConfig = field(default_factory=PoolConfig)
    anchor: AnchorConfig = field(default_factory=default_anchor)
    filters: FilterConfig = field(default_factory=FilterConfig)
    ekf: EkfConfig = field(default_factory=EkfConfig)
    strokes: StrokeConfig = field(default_factory=StrokeConfig)
    uplink_url: Optional[str] = None
    walls: WallConfig = field(default_factory = WallConfig)



SECTIONS = {"pool": PoolConfig, "anchor": AnchorConfig, "filters": FilterConfig,
            "ekf": EkfConfig, "strokes": StrokeConfig, "walls": WallConfig}


def _build(default, values: dict, section: str):
    allowed = {f.name for f in fields(default)}
    unknown = set(values) - allowed
    if unknown:
        raise ValueError(f"[{section}] unknown setting(s): {', '.join(sorted(unknown))}; "
                         f"allowed: {', '.join(sorted(allowed))}")
    return replace(default, **values)

def load_settings(path: Optional[str]) -> Settings:
    if path is None:
        return Settings()
    raw = yaml.safe_load(Path(path).read_text()) or {}
    unknown = set(raw) - set(SECTIONS) - {"uplink_url"}
    if unknown:
        raise ValueError(f"unknown section(s) in {path}: {', '.join(sorted(unknown))}")
    defaults = Settings()
    parts = {name: _build(getattr(defaults, name), raw.get(name) or {}, name) for name in SECTIONS}
    return Settings(**parts, uplink_url=raw.get("uplink_url"))



