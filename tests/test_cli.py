import pytest

from radar_cam_2_mcap import __version__
from radar_cam_2_mcap.cli import main


def test_version(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit) as exc:
        main(["--version"])
    assert exc.value.code == 0
    assert __version__ in capsys.readouterr().out


def test_no_args_prints_help_and_returns_2(capsys: pytest.CaptureFixture[str]) -> None:
    assert main([]) == 2
    assert "usage" in capsys.readouterr().out.lower()
