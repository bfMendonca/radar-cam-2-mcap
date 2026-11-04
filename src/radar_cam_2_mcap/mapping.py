"""Pure mappers from decoded session data to ROS 2 message dicts, plus the message schemas.

The dicts use the field names of the ``.msg`` files in ``msgdefs/`` and are written by
``mcap_ros2.writer.Writer``.
"""

from __future__ import annotations

import json
import math
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import ld2451

from radar_cam_2_mcap.config import CameraIntrinsics, FrameTransform, RadarConfig
from radar_cam_2_mcap.video import VideoFrame

__all__ = [
    "MSGDEFS",
    "STAMP",
    "camera_info",
    "camera_metadata",
    "compressed_image",
    "event_message",
    "radar_scan",
    "radar_status",
    "tf_static",
]

_MSGDIR = Path(__file__).parent / "msgdefs"
_LEVEL_INFO = 20
_LEVEL_WARN = 30
_RECORDER = "radar_cam_recorder"
_CONVERTER = "radar_cam_2_mcap"


def STAMP(ns: int) -> dict[str, int]:
    """Split nanoseconds into a ``builtin_interfaces/Time`` dict."""
    sec, nanosec = divmod(int(ns), 1_000_000_000)
    return {"sec": sec, "nanosec": nanosec}


def _header(stamp_ns: int, frame_id: str) -> dict[str, Any]:
    return {"stamp": STAMP(stamp_ns), "frame_id": frame_id}


def compressed_image(frame: VideoFrame, stamp_ns: int, frame_id: str) -> dict[str, Any]:
    """``sensor_msgs/CompressedImage``; ``format`` is ``jpeg`` or ``png``."""
    return {"header": _header(stamp_ns, frame_id), "format": frame.fmt, "data": frame.data}


def camera_info(cfg: CameraIntrinsics, stamp_ns: int, frame_id: str) -> dict[str, Any]:
    """``sensor_msgs/CameraInfo`` from configuration (uncalibrated unless ``cfg.calibrated``)."""
    return {
        "header": _header(stamp_ns, frame_id),
        "height": cfg.height,
        "width": cfg.width,
        "distortion_model": cfg.distortion_model,
        "d": list(cfg.d),
        "k": list(cfg.k),
        "r": list(cfg.r),
        "p": list(cfg.p),
        "binning_x": 0,
        "binning_y": 0,
        "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False},
    }


def _num(fields: dict[str, Any], key: str, default: float = 0.0) -> float:
    value = fields.get(key)
    return float(value) if isinstance(value, int | float) else default


def _nums(fields: dict[str, Any], key: str, n: int) -> list[float]:
    value = fields.get(key)
    if isinstance(value, list) and len(value) == n:
        return [float(v) for v in value]
    return [0.0] * n


def camera_metadata(frame: VideoFrame, stamp_ns: int, boot_ns: int, segment: int) -> dict[str, Any]:
    """``radar_cam_msgs/CameraMetadata`` from the frame's meta.json object.

    Keys absent from meta.json give 0 (or an empty string).
    """
    f = frame.meta.fields
    crop = f.get("ScalerCrop")
    return {
        "header": _header(stamp_ns, ""),
        "exposure_us": int(_num(f, "ExposureTime")),
        "analogue_gain": _num(f, "AnalogueGain"),
        "digital_gain": _num(f, "DigitalGain"),
        "lux": _num(f, "Lux"),
        "colour_temperature": int(_num(f, "ColourTemperature")),
        "colour_gains": _nums(f, "ColourGains", 2),
        "colour_correction_matrix": _nums(f, "ColourCorrectionMatrix", 9),
        "sensor_black_levels": [int(v) for v in _nums(f, "SensorBlackLevels", 4)],
        "focus_fom": int(_num(f, "FocusFoM")),
        "ae_state": int(_num(f, "AeState")),
        "scaler_crop": crop if isinstance(crop, str) else "",
        "frame_wall_clock_us": frame.meta.frame_wall_clock_us or 0,
        "boot_ns": boot_ns,
        "segment": segment,
        "frame_index": frame.meta.index,
    }


def radar_scan(
    report: ld2451.RadarReport, stamp_ns: int, frame_id: str, cfg: RadarConfig
) -> dict[str, Any]:
    """``radar_msgs/RadarScan``; one return per target (an empty frame gives no returns).

    range is metres; azimuth is ``cfg.angle_sign * radians(angle_deg)`` (positive anticlockwise,
    REP-103; the LD2451 angle sign is UNVERIFIED, so the default ``angle_sign = 1`` is an
    assumption); elevation is 0.0 (unknown, the radar is 2D); doppler_velocity is m/s, negative
    for approaching targets (physically falling range) and positive for receding ones. The driver's
    ``approaching`` flag is inverted on real session s000060, so the effective flag is
    ``t.approaching XOR cfg.invert_direction`` (default true; ``RadarStatus`` is not affected);
    amplitude is the raw LD2451 SNR, not
    calibrated dB.
    """
    flip = cfg.invert_direction
    returns = [
        {
            "range": float(t.distance_m),
            "azimuth": cfg.angle_sign * math.radians(t.angle_deg),
            "elevation": 0.0,
            "doppler_velocity": (-1.0 if t.approaching != flip else 1.0) * t.speed_kmh / 3.6,
            "amplitude": float(t.snr),
        }
        for t in report.targets
    ]
    return {"header": _header(stamp_ns, frame_id), "returns": returns}


def radar_status(report: ld2451.RadarReport, stamp_ns: int, boot_ns: int) -> dict[str, Any]:
    """``radar_cam_msgs/RadarStatus``; ``target_count`` may be below ``reported_count``."""
    return {
        "header": _header(stamp_ns, ""),
        "reported_count": report.reported_count,
        "target_count": len(report.targets),
        "approach_alarm": report.approach_alarm,
        "boot_ns": boot_ns,
    }


def _quaternion(roll: float, pitch: float, yaw: float) -> dict[str, float]:
    """Quaternion for fixed-axis roll, pitch, yaw (ROS rpy: R = Rz(yaw) Ry(pitch) Rx(roll))."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return {
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
        "w": cr * cp * cy + sr * sp * sy,
    }


def tf_static(transforms: Sequence[FrameTransform], stamp_ns: int) -> dict[str, Any]:
    """``tf2_msgs/TFMessage`` holding one stamped transform per configured frame."""
    return {
        "transforms": [
            {
                "header": _header(stamp_ns, t.parent),
                "child_frame_id": t.child,
                "transform": {
                    "translation": {"x": t.xyz[0], "y": t.xyz[1], "z": t.xyz[2]},
                    "rotation": _quaternion(*t.rpy),
                },
            }
            for t in transforms
        ]
    }


def event_message(event: dict[str, Any], stamp_ns: int) -> dict[str, Any]:
    """``rcl_interfaces/Log`` from an ``events.jsonl`` dict or a converter finding.

    Recorder incidents are WARN (30) and state changes INFO (20); an event carrying its own
    integer ``level`` (converter findings, ``kind`` ``converter``) keeps it. ``msg`` is the
    reason, followed by the ``detail`` as compact JSON when it is not empty.
    """
    kind = event.get("kind")
    own = event.get("level")
    if kind == "converter":
        name = _CONVERTER
        level = own if isinstance(own, int) else _LEVEL_WARN
    else:
        name = _RECORDER
        level = _LEVEL_WARN if kind == "incident" else _LEVEL_INFO
    text = str(event.get("reason", ""))
    detail = event.get("detail")
    if detail:
        text += " " + json.dumps(detail, separators=(",", ":"), ensure_ascii=False)
    return {
        "stamp": STAMP(stamp_ns),
        "level": level,
        "name": name,
        "msg": text,
        "file": "",
        "function": "",
        "line": 0,
    }


def _read(name: str) -> str:
    return (_MSGDIR / f"{name}.msg").read_text(encoding="utf-8")


def _schema(top: str, deps: Sequence[str]) -> str:
    """ros2msg schema text: the message, then each dependency after a line of 80 ``=``."""
    parts = [_read(top.split("/")[1])]
    for dep in deps:
        parts.append(f"{'=' * 80}\nMSG: {dep}\n{_read(dep.split('/')[1])}")
    return "\n".join(p if p.endswith("\n") else p + "\n" for p in parts)


_HEADER = ("std_msgs/Header", "builtin_interfaces/Time")

MSGDEFS: dict[str, str] = {
    "sensor_msgs/CompressedImage": _schema("sensor_msgs/CompressedImage", _HEADER),
    "sensor_msgs/CameraInfo": _schema(
        "sensor_msgs/CameraInfo", (*_HEADER, "sensor_msgs/RegionOfInterest")
    ),
    "radar_cam_msgs/CameraMetadata": _schema("radar_cam_msgs/CameraMetadata", _HEADER),
    "radar_msgs/RadarScan": _schema("radar_msgs/RadarScan", (*_HEADER, "radar_msgs/RadarReturn")),
    "radar_cam_msgs/RadarStatus": _schema("radar_cam_msgs/RadarStatus", _HEADER),
    "rcl_interfaces/Log": _schema("rcl_interfaces/Log", ("builtin_interfaces/Time",)),
    "tf2_msgs/TFMessage": _schema(
        "tf2_msgs/TFMessage",
        (
            "geometry_msgs/TransformStamped",
            "std_msgs/Header",
            "builtin_interfaces/Time",
            "geometry_msgs/Transform",
            "geometry_msgs/Vector3",
            "geometry_msgs/Quaternion",
        ),
    ),
}
