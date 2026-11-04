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


def test_invert_direction_default_and_override(tmp_path: Path) -> None:
    assert load_config(None).radar.invert_direction is True
    path = tmp_path / "c.toml"
    path.write_text("[radar]\ninvert_direction = false\n")
    cfg = load_config(path)
    assert cfg.radar.invert_direction is False
    assert cfg.radar.angle_sign == 1


def test_invert_direction_wrong_type_rejected(tmp_path: Path) -> None:
    path = tmp_path / "c.toml"
    for bad in ("1", '"yes"'):
        path.write_text(f"[radar]\ninvert_direction = {bad}\n")
        with pytest.raises(ConfigError):
            load_config(path)


@pytest.mark.parametrize(
    ("toml", "needle"),
    [
        ("[image]\nquality = 0\n", "image.quality"),
        ("[image]\nquality = 101\n", "image.quality"),
        ("[image]\nsubsampling = 5\n", "image.subsampling"),
        ("[image]\nsubsampling = -1\n", "image.subsampling"),
        ("[mcap]\nchunk_size = 0\n", "mcap.chunk_size"),
        ("[radar]\nangle_sign = 2\n", "radar.angle_sign"),
        ("[radar]\nangle_sign = 0\n", "radar.angle_sign"),
        ("[camera]\nk = [1.0, 2.0]\n", "camera.k"),
        ("[camera]\nr = [1.0]\n", "camera.r"),
        ("[camera]\np = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n", "camera.p"),
        ("[camera]\nd = [0.0, 0.0, 0.0]\n", "camera.d"),
        ("[camera]\nwidth = 0\n", "camera.width"),
        ("[camera]\nheight = -4\n", "camera.height"),
        ("[time]\ncrosscheck_tolerance_ms = -1.0\n", "time.crosscheck_tolerance_ms"),
        ('[topics]\ncamera_metadata = "/camera/image/compressed"\n', "topics"),
        ('[topics]\nevents = "/tf_static"\n', "topics"),
    ],
)
def test_out_of_range_values_rejected(tmp_path: Path, toml: str, needle: str) -> None:
    path = tmp_path / "c.toml"
    path.write_text(toml)
    with pytest.raises(ConfigError, match=needle.replace(".", r"\.")):
        load_config(path)


def test_in_range_edge_values_accepted(tmp_path: Path) -> None:
    path = tmp_path / "c.toml"
    path.write_text(
        "[image]\nquality = 1\nsubsampling = 0\n[mcap]\nchunk_size = 1\n"
        "[radar]\nangle_sign = -1\n"
        '[camera]\ndistortion_model = "rational_polynomial"\n'
        "d = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
    )
    cfg = load_config(path)
    assert cfg.radar.angle_sign == -1 and len(cfg.camera.d) == 8


def test_duplicate_topic_error_names_both_keys(tmp_path: Path) -> None:
    path = tmp_path / "c.toml"
    path.write_text('[topics]\nradar_status = "/radar/scan"\n')
    with pytest.raises(ConfigError, match=r"radar_scan.*radar_status"):
        load_config(path)


def test_config_range_error_exits_2(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    from radar_cam_2_mcap.cli import main

    path = tmp_path / "c.toml"
    path.write_text("[radar]\nangle_sign = 2\n")
    fixture = Path(__file__).parent / "fixtures" / "interrupted"
    assert main(["convert", str(fixture), "-o", str(tmp_path / "o"), "--config", str(path)]) == 2
    assert "angle_sign" in capsys.readouterr().err
    assert not (tmp_path / "o").exists()
