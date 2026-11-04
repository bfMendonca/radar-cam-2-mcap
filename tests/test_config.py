import math
from pathlib import Path

import pytest

from radar_cam_2_mcap.config import ConfigError, load_config


def test_defaults_match_spec() -> None:
    cfg = load_config(None)
    assert cfg.image.format == "jpeg"
    assert cfg.image.quality == 90
    assert cfg.image.subsampling == 2
    assert cfg.mcap.compression == "zstd"
    assert cfg.camera.width == 1296
    assert cfg.camera.height == 972
    assert cfg.camera.calibrated is False
    by_child = {t.child: t for t in cfg.transforms}
    assert len(cfg.transforms) == 3
    cam = by_child["camera_link"]
    assert (cam.parent, cam.xyz, cam.rpy) == ("base_link", (0.005, 0.0195, 0.042), (0, 0, 0))
    rad = by_child["radar_link"]
    assert (rad.parent, rad.xyz, rad.rpy) == ("base_link", (0.0051, -0.0145, 0.042), (0, 0, 0))
    opt = by_child["camera_optical_frame"]
    assert opt.parent == "camera_link"
    assert opt.xyz == (0, 0, 0)
    assert opt.rpy == (-math.pi / 2, 0, -math.pi / 2)


def test_file_overrides_only_named_keys(tmp_path: Path) -> None:
    path = tmp_path / "c.toml"
    path.write_text("[image]\nquality = 95\n")
    cfg = load_config(path)
    assert cfg.image.quality == 95
    assert cfg.image.format == "jpeg"
    assert cfg.mcap.compression == "zstd"


def test_unknown_key_rejected(tmp_path: Path) -> None:
    path = tmp_path / "c.toml"
    path.write_text("[image]\nbogus = 1\n")
    with pytest.raises(ConfigError):
        load_config(path)
    path.write_text("[nonsense]\nx = 1\n")
    with pytest.raises(ConfigError):
        load_config(path)


def test_default_toml_equals_builtin_defaults() -> None:
    path = Path(__file__).parent.parent / "config" / "default.toml"
    assert load_config(path) == load_config(None)


def test_recover_short_frames_default_and_override(tmp_path: Path) -> None:
    assert load_config(None).radar.recover_short_frames is True
    path = tmp_path / "c.toml"
    path.write_text("[radar]\nrecover_short_frames = false\n")
    cfg = load_config(path)
    assert cfg.radar.recover_short_frames is False
    assert cfg.radar.angle_sign == 1


def test_default_intrinsics_reproduce_vendor_field_of_view() -> None:
    # Vendor-spec-derived and uncalibrated: fx = fy = 3.6 mm / 2.8 um (2x2 binned) = 1285.7 px.
    cam = load_config(None).camera
    fx, cx, fy, cy = cam.k[0], cam.k[2], cam.k[4], cam.k[5]
    assert (fx, fy) == (1285.7, 1285.7)
    hfov = math.degrees(2 * math.atan(cx / fx))
    vfov = math.degrees(2 * math.atan(cy / fy))
    assert hfov == pytest.approx(53.50, abs=0.05)
    assert vfov == pytest.approx(41.41, abs=0.05)
    assert cam.calibrated is False
