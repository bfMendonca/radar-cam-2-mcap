"""Converter configuration: frozen dataclasses with defaults, overridden by a TOML file."""

from __future__ import annotations

import math
import tomllib
from dataclasses import dataclass, field, fields, replace
from pathlib import Path
from typing import Any, Literal, TypeVar, get_args, get_origin, get_type_hints

__all__ = [
    "CameraIntrinsics",
    "Config",
    "ConfigError",
    "FrameTransform",
    "ImageConfig",
    "McapConfig",
    "RadarConfig",
    "TimeConfig",
    "TopicsConfig",
    "load_config",
]


class ConfigError(ValueError):
    """Raised for an unreadable or invalid configuration file."""


@dataclass(frozen=True)
class ImageConfig:
    format: Literal["jpeg", "png"] = "jpeg"
    quality: int = 90
    subsampling: int = 2


@dataclass(frozen=True)
class McapConfig:
    compression: Literal["zstd", "lz4", "none"] = "zstd"
    chunk_size: int = 4 * 1024 * 1024


def _default_k() -> tuple[float, ...]:
    return (1285.7, 0.0, 648.0, 0.0, 1285.7, 486.0, 0.0, 0.0, 1.0)


def _default_p() -> tuple[float, ...]:
    return (1285.7, 0.0, 648.0, 0.0, 0.0, 1285.7, 486.0, 0.0, 0.0, 0.0, 1.0, 0.0)


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int = 1296
    height: int = 972
    k: tuple[float, ...] = field(default_factory=_default_k)
    d: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0)
    r: tuple[float, ...] = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    p: tuple[float, ...] = field(default_factory=_default_p)
    distortion_model: str = "plumb_bob"
    calibrated: bool = False


@dataclass(frozen=True)
class FrameTransform:
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


@dataclass(frozen=True)
class TopicsConfig:
    image: str = "/camera/image/compressed"
    camera_info: str = "/camera/camera_info"
    camera_metadata: str = "/camera/metadata"
    radar_scan: str = "/radar/scan"
    radar_status: str = "/radar/status"
    events: str = "/events"
    tf_static: str = "/tf_static"
    frame_camera: str = "camera_optical_frame"
    frame_radar: str = "radar_link"


@dataclass(frozen=True)
class RadarConfig:
    # Which direction a positive LD2451 angle means is UNVERIFIED (spec section 6).
    angle_sign: int = 1
    # Recover data frames whose count/length overstate the targets present (footer early).
    recover_short_frames: bool = True
    # The LD2451 driver's `approaching` flag (direction byte 00) is empirically inverted on real
    # session s000060; the driver check is unverified. Set False if the driver is fixed.
    invert_direction: bool = True


@dataclass(frozen=True)
class TimeConfig:
    crosscheck_tolerance_ms: float = 50.0
    base: Literal["wall", "boot"] = "wall"


def _default_transforms() -> tuple[FrameTransform, ...]:
    return (
        FrameTransform("base_link", "camera_link", (0.005, 0.0195, 0.042), (0.0, 0.0, 0.0)),
        FrameTransform("base_link", "radar_link", (0.0051, -0.0145, 0.042), (0.0, 0.0, 0.0)),
        FrameTransform(
            "camera_link",
            "camera_optical_frame",
            (0.0, 0.0, 0.0),
            (-math.pi / 2, 0.0, -math.pi / 2),
        ),
    )


@dataclass(frozen=True)
class Config:
    image: ImageConfig = field(default_factory=ImageConfig)
    mcap: McapConfig = field(default_factory=McapConfig)
    camera: CameraIntrinsics = field(default_factory=CameraIntrinsics)
    topics: TopicsConfig = field(default_factory=TopicsConfig)
    radar: RadarConfig = field(default_factory=RadarConfig)
    time: TimeConfig = field(default_factory=TimeConfig)
    transforms: tuple[FrameTransform, ...] = field(default_factory=_default_transforms)


T = TypeVar("T")


def _check_value(where: str, value: Any, hint: Any) -> Any:
    origin = get_origin(hint)
    if origin is Literal:
        if value not in get_args(hint):
            raise ConfigError(f"{where}: {value!r} not one of {list(get_args(hint))}")
        return value
    if origin is tuple:
        if not isinstance(value, list | tuple):
            raise ConfigError(f"{where}: expected a list")
        args = get_args(hint)
        if len(args) == 2 and args[1] is Ellipsis:
            return tuple(_check_value(where, v, args[0]) for v in value)
        if len(args) != len(value):
            raise ConfigError(f"{where}: expected {len(args)} values")
        return tuple(_check_value(where, v, a) for v, a in zip(value, args, strict=True))
    if hint is float:
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise ConfigError(f"{where}: expected a number")
        return float(value)
    if hint is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError(f"{where}: expected an integer")
        return value
    if hint in (str, bool):
        if not isinstance(value, hint):
            raise ConfigError(f"{where}: expected {hint.__name__}")
        return value
    raise ConfigError(f"{where}: unsupported type")  # pragma: no cover


def _override(base: T, table: Any, where: str) -> T:
    """Return `base` (a dataclass) with the keys named in `table` replaced."""
    if not isinstance(table, dict):
        raise ConfigError(f"[{where}] must be a table")
    hints = get_type_hints(type(base))
    names = {f.name for f in fields(base)}  # type: ignore[arg-type]
    unknown = sorted(set(table) - names)
    if unknown:
        raise ConfigError(f"unknown key(s) in [{where}]: {', '.join(unknown)}")
    checked = {k: _check_value(f"{where}.{k}", v, hints[k]) for k, v in table.items()}
    return replace(base, **checked)  # type: ignore[type-var]


def _parse_transforms(items: Any) -> tuple[FrameTransform, ...]:
    if not isinstance(items, list):
        raise ConfigError("'transforms' must be an array of tables ([[transforms]])")
    out: list[FrameTransform] = []
    for i, item in enumerate(items):
        if not isinstance(item, dict):
            raise ConfigError(f"transforms[{i}] must be a table")
        unknown = sorted(set(item) - {"parent", "child", "xyz", "rpy"})
        missing = sorted({"parent", "child", "xyz", "rpy"} - set(item))
        if unknown:
            raise ConfigError(f"unknown key(s) in transforms[{i}]: {', '.join(unknown)}")
        if missing:
            raise ConfigError(f"missing key(s) in transforms[{i}]: {', '.join(missing)}")
        hints = get_type_hints(FrameTransform)
        out.append(
            FrameTransform(
                **{k: _check_value(f"transforms[{i}].{k}", v, hints[k]) for k, v in item.items()}
            )
        )
    return tuple(out)


_SECTIONS = ("image", "mcap", "camera", "topics", "radar", "time")

# Distortion coefficient counts of the ROS camera models; other models are not checked.
_DISTORTION_LEN = {"plumb_bob": 5, "rational_polynomial": 8, "equidistant": 4}
_TOPIC_KEYS = (
    "image",
    "camera_info",
    "camera_metadata",
    "radar_scan",
    "radar_status",
    "events",
    "tf_static",
)


def _validate(cfg: Config) -> None:
    """Range, length and distinctness checks that the type checks cannot express."""

    def need(ok: bool, msg: str) -> None:
        if not ok:
            raise ConfigError(msg)

    img, cam = cfg.image, cfg.camera
    need(1 <= img.quality <= 100, f"image.quality: {img.quality} not in 1..100")
    need(
        img.subsampling in (0, 1, 2),
        f"image.subsampling: {img.subsampling} not 0 (4:4:4), 1 (4:2:2) or 2 (4:2:0)",
    )
    need(cfg.mcap.chunk_size > 0, f"mcap.chunk_size: {cfg.mcap.chunk_size} must be positive")
    need(cfg.radar.angle_sign in (-1, 1), f"radar.angle_sign: {cfg.radar.angle_sign} not -1 or 1")
    need(cam.width > 0, f"camera.width: {cam.width} must be positive")
    need(cam.height > 0, f"camera.height: {cam.height} must be positive")
    for key, n in (("k", 9), ("r", 9), ("p", 12)):
        got = len(getattr(cam, key))
        need(got == n, f"camera.{key}: expected {n} values, got {got}")
    n_d = _DISTORTION_LEN.get(cam.distortion_model)
    need(
        n_d is None or len(cam.d) == n_d,
        f"camera.d: {cam.distortion_model} needs {n_d} values, got {len(cam.d)}",
    )
    tol = cfg.time.crosscheck_tolerance_ms
    need(
        math.isfinite(tol) and tol >= 0,
        f"time.crosscheck_tolerance_ms: {tol} must be a non-negative number",
    )
    by_name: dict[str, list[str]] = {}
    for key in _TOPIC_KEYS:
        by_name.setdefault(getattr(cfg.topics, key), []).append(key)
    for name, keys in by_name.items():
        need(len(keys) == 1, f"topics: {', '.join(keys)} all name {name!r}; topics must differ")


def load_config(path: Path | None) -> Config:
    """Load defaults, or defaults overridden by the keys a TOML file names."""
    cfg = Config()
    if path is None:
        return cfg
    try:
        with open(path, "rb") as fh:
            data = tomllib.load(fh)
    except (OSError, tomllib.TOMLDecodeError) as exc:
        raise ConfigError(f"cannot read config {path}: {exc}") from exc
    unknown = sorted(set(data) - set(_SECTIONS) - {"transforms"})
    if unknown:
        raise ConfigError(f"unknown section(s): {', '.join(unknown)}")
    updates: dict[str, Any] = {
        s: _override(getattr(cfg, s), data[s], s) for s in _SECTIONS if s in data
    }
    if "transforms" in data:
        updates["transforms"] = _parse_transforms(data["transforms"])
    cfg = replace(cfg, **updates)
    _validate(cfg)
    return cfg
