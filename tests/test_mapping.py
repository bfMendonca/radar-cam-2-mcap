from __future__ import annotations

import io
import math
from dataclasses import replace

import ld2451
import pytest
from mcap_ros2.reader import read_ros2_messages
from mcap_ros2.writer import Writer

from radar_cam_2_mcap import mapping
from radar_cam_2_mcap.config import CameraIntrinsics, Config, RadarConfig
from radar_cam_2_mcap.video import FrameMeta, VideoFrame

CFG = Config()
JPEG = b"\xff\xd8\xff\xe0fakejpeg\xff\xd9"
T0 = 1_700_000_000_123_456_789


def _report(*targets: ld2451.Target, header: bytes | None = None) -> ld2451.RadarReport:
    hdr = bytes([len(targets), 1 if any(t.approaching for t in targets) else 0])
    return ld2451.RadarReport(
        t_rx_ns=5, targets=targets, header_bytes=hdr if header is None else header, raw=b""
    )


def _target(
    angle: int = 10, dist: int = 25, speed: int = 36, approaching: bool = True
) -> ld2451.Target:
    return ld2451.Target(
        angle_deg=angle, distance_m=dist, approaching=approaching, speed_kmh=speed, snr=77
    )


def _frame() -> VideoFrame:
    fields = {
        "ExposureTime": 39891,
        "AnalogueGain": 8.0,
        "DigitalGain": 1.0,
        "Lux": 58.5,
        "ColourTemperature": 4538,
        "ColourGains": [1.5, 1.4],
        "ColourCorrectionMatrix": [float(i) for i in range(9)],
        "SensorBlackLevels": [1024] * 4,
        "FocusFoM": 1954,
        "AeState": 1,
        "ScalerCrop": "(0, 0)/2592x1944",
        "FrameWallClock": 1792638188250940,
        "SensorTimestamp": 644508838000,
    }
    meta = FrameMeta(
        index=3, sensor_ts_ns=644508838000, frame_wall_clock_us=1792638188250940, fields=fields
    )
    return VideoFrame(meta=meta, data=JPEG, fmt="jpeg", width=1296, height=972)


def test_stamp_splits_seconds_and_nanoseconds() -> None:
    assert mapping.STAMP(T0) == {"sec": 1_700_000_000, "nanosec": 123_456_789}


def test_radar_scan_maps_target_units_and_signs() -> None:
    scan = mapping.radar_scan(_report(_target()), T0, "radar_link", RadarConfig())
    (ret,) = scan["returns"]
    assert ret["range"] == 25.0
    assert ret["azimuth"] == pytest.approx(math.radians(10))
    assert ret["elevation"] == 0.0
    assert ret["doppler_velocity"] == 10.0  # driver approaching=True is inverted by default
    assert ret["amplitude"] == 77.0
    assert scan["header"] == {"stamp": mapping.STAMP(T0), "frame_id": "radar_link"}


def test_default_inverts_driver_direction() -> None:
    # Empirical (real session s000060): driver approaching=False means range falling.
    rep = _report(_target(approaching=False), _target(approaching=True))
    a, b = mapping.radar_scan(rep, T0, "radar_link", RadarConfig())["returns"]
    assert a["doppler_velocity"] == -10.0
    assert b["doppler_velocity"] == 10.0


def test_invert_direction_false_keeps_driver_sign() -> None:
    rep = _report(_target(approaching=True), _target(approaching=False))
    cfg = RadarConfig(invert_direction=False)
    a, b = mapping.radar_scan(rep, T0, "radar_link", cfg)["returns"]
    assert a["doppler_velocity"] == -10.0
    assert b["doppler_velocity"] == 10.0


def test_invert_direction_does_not_touch_status() -> None:
    rep = _report(_target(approaching=True))
    assert mapping.radar_status(rep, T0, 7)["approach_alarm"] == rep.approach_alarm


def test_empty_report_gives_empty_returns() -> None:
    scan = mapping.radar_scan(_report(header=b""), T0, "radar_link", RadarConfig())
    assert scan["returns"] == []


def test_angle_sign_flips_azimuth() -> None:
    rep = _report(_target(angle=-20))
    (a,) = mapping.radar_scan(rep, T0, "r", RadarConfig(angle_sign=1))["returns"]
    (b,) = mapping.radar_scan(rep, T0, "r", RadarConfig(angle_sign=-1))["returns"]
    assert a["azimuth"] == pytest.approx(-math.radians(20))
    assert b["azimuth"] == pytest.approx(math.radians(20))


def test_radar_status_shows_short_frame() -> None:
    rep = _report(_target(), header=bytes([2, 1]))
    st = mapping.radar_status(rep, T0, 99)
    assert (st["reported_count"], st["target_count"], st["approach_alarm"], st["boot_ns"]) == (
        2,
        1,
        True,
        99,
    )


def test_camera_info_defaults() -> None:
    info = mapping.camera_info(CameraIntrinsics(), T0, "camera_optical_frame")
    assert tuple(info["k"]) == (1285.7, 0, 648, 0, 1285.7, 486, 0, 0, 1)
    assert all(v == 0 for v in info["d"])
    assert (info["width"], info["height"]) == (1296, 972)
    assert info["distortion_model"] == "plumb_bob"
    assert info["roi"]["do_rectify"] is False


def test_tf_static_contains_three_default_transforms_with_expected_translation() -> None:
    msg = mapping.tf_static(CFG.transforms, T0)
    by_child = {t["child_frame_id"]: t for t in msg["transforms"]}
    assert set(by_child) == {"camera_link", "radar_link", "camera_optical_frame"}
    cam = by_child["camera_link"]
    assert cam["header"]["frame_id"] == "base_link"
    assert cam["transform"]["translation"] == {"x": 0.005, "y": 0.0195, "z": 0.042}
    assert by_child["radar_link"]["transform"]["translation"]["y"] == -0.0145


def test_optical_frame_quaternion() -> None:
    msg = mapping.tf_static(CFG.transforms, T0)
    opt = next(t for t in msg["transforms"] if t["child_frame_id"] == "camera_optical_frame")
    q = opt["transform"]["rotation"]
    for got, want in zip((q["x"], q["y"], q["z"], q["w"]), (-0.5, 0.5, -0.5, 0.5), strict=True):
        assert got == pytest.approx(want, abs=1e-9)


def test_compressed_image_format_field() -> None:
    msg = mapping.compressed_image(_frame(), T0, "camera_optical_frame")
    assert msg["format"] == "jpeg"
    assert len(msg["data"]) == len(JPEG)
    assert msg["header"]["frame_id"] == "camera_optical_frame"


def test_camera_metadata_fields() -> None:
    msg = mapping.camera_metadata(_frame(), T0, 644508838000, 2)
    assert msg["exposure_us"] == 39891
    assert msg["scaler_crop"] == "(0, 0)/2592x1944"
    assert (msg["segment"], msg["frame_index"], msg["boot_ns"]) == (2, 3, 644508838000)
    assert msg["frame_wall_clock_us"] == 1792638188250940


def test_camera_metadata_tolerates_missing_fields() -> None:
    frame = _frame()
    bare = replace(frame, meta=replace(frame.meta, frame_wall_clock_us=None, fields={}))
    msg = mapping.camera_metadata(bare, T0, 1, 0)
    assert msg["exposure_us"] == 0
    assert msg["colour_gains"] == [0.0, 0.0]
    assert len(msg["colour_correction_matrix"]) == 9
    assert msg["frame_wall_clock_us"] == 0


def test_event_message_levels_and_detail() -> None:
    rec = {
        "kind": "incident",
        "state": "recording",
        "reason": "frame drops",
        "boottime_ns": 1,
        "detail": {"segment": 0, "drops": 5},
    }
    m = mapping.event_message(rec, T0)
    assert m["level"] == 30 and m["name"] == "radar_cam_recorder"
    assert m["msg"] == 'frame drops {"segment":0,"drops":5}'
    st = mapping.event_message({"kind": "state", "reason": "recording", "detail": {}}, T0)
    assert st["level"] == 20 and st["msg"] == "recording"
    own = mapping.event_message(
        {"kind": "converter", "level": 40, "reason": "bad", "detail": {}}, T0
    )
    assert own["level"] == 40 and own["name"] == "radar_cam_2_mcap"


def test_msgdefs_parse() -> None:
    assert set(mapping.MSGDEFS) == {
        "sensor_msgs/msg/CompressedImage",
        "sensor_msgs/msg/CameraInfo",
        "radar_cam_msgs/msg/CameraMetadata",
        "radar_msgs/msg/RadarScan",
        "radar_cam_msgs/msg/RadarStatus",
        "rcl_interfaces/msg/Log",
        "tf2_msgs/msg/TFMessage",
    }
    two = _report(_target(), _target(angle=-5, approaching=False))
    msgs = {
        "sensor_msgs/msg/CompressedImage": mapping.compressed_image(_frame(), T0, "cam"),
        "sensor_msgs/msg/CameraInfo": mapping.camera_info(CameraIntrinsics(), T0, "cam"),
        "radar_cam_msgs/msg/CameraMetadata": mapping.camera_metadata(_frame(), T0, 7, 1),
        "radar_msgs/msg/RadarScan": mapping.radar_scan(two, T0, "radar", RadarConfig()),
        "radar_cam_msgs/msg/RadarStatus": mapping.radar_status(two, T0, 7),
        "rcl_interfaces/msg/Log": mapping.event_message(
            {"kind": "incident", "reason": "x", "detail": {"a": 1}}, T0
        ),
        "tf2_msgs/msg/TFMessage": mapping.tf_static(CFG.transforms, T0),
    }
    buf = io.BytesIO()
    w = Writer(buf)
    for i, (dt, text) in enumerate(mapping.MSGDEFS.items()):
        w.write_message(f"/t{i}", w.register_msgdef(dt, text), msgs[dt], log_time=T0 + i)
    w.finish()  # type: ignore[no-untyped-call]
    buf.seek(0)
    got = {m.channel.topic: m.ros_msg for m in read_ros2_messages(buf)}
    assert len(got) == len(mapping.MSGDEFS)
    by_type = {dt: got[f"/t{i}"] for i, dt in enumerate(mapping.MSGDEFS)}
    img = by_type["sensor_msgs/msg/CompressedImage"]
    assert bytes(img.data) == JPEG and img.format == "jpeg"
    assert img.header.stamp.sec == 1_700_000_000
    scan = by_type["radar_msgs/msg/RadarScan"]
    assert len(scan.returns) == 2
    assert scan.returns[0].doppler_velocity == pytest.approx(10.0)
    assert scan.returns[1].doppler_velocity == pytest.approx(-10.0)
    assert by_type["sensor_msgs/msg/CameraInfo"].k[0] == pytest.approx(1285.7)
    assert by_type["radar_cam_msgs/msg/CameraMetadata"].exposure_us == 39891
    assert by_type["radar_cam_msgs/msg/RadarStatus"].target_count == 2
    assert by_type["rcl_interfaces/msg/Log"].level == 30
    assert len(by_type["tf2_msgs/msg/TFMessage"].transforms) == 3
