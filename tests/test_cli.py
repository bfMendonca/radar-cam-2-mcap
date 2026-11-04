import json
import shutil
import subprocess
import sys
from pathlib import Path
from typing import ClassVar

import pytest
from mcap.reader import make_reader

from radar_cam_2_mcap import __version__
from radar_cam_2_mcap.cli import main
from radar_cam_2_mcap.config import Config
from radar_cam_2_mcap.convert import ConvertOptions, convert_session
from radar_cam_2_mcap.video import read_frame_meta


def test_version(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit) as exc:
        main(["--version"])
    assert exc.value.code == 0
    assert __version__ in capsys.readouterr().out


def test_no_args_prints_help_and_returns_2(capsys: pytest.CaptureFixture[str]) -> None:
    assert main([]) == 2
    assert "usage" in capsys.readouterr().out.lower()


FIXTURES = Path(__file__).parent / "fixtures"


def _copy(name: str, tmp: Path, dest: str | None = None) -> Path:
    dst = tmp / (dest or name)
    shutil.copytree(FIXTURES / name, dst)
    return dst


def _break_meta(session: Path) -> None:
    meta = session / "video" / "0001.meta.json"
    meta.write_text(json.dumps([m.fields for m in read_frame_meta(meta)][:-3]))


def _shift_segment1(session: Path, new_gap_ns: int) -> None:
    """Shift segment 1 (frames, radar, anchors, events) so that the gap becomes new_gap_ns."""
    m0 = read_frame_meta(session / "video" / "0000.meta.json")
    m1 = read_frame_meta(session / "video" / "0001.meta.json")
    delta = (m1[0].sensor_ts_ns - m0[-1].sensor_ts_ns) - new_gap_ns
    assert delta > 0
    meta = session / "video" / "0001.meta.json"
    objs = [m.fields for m in m1]
    for o in objs:
        o["SensorTimestamp"] -= delta
        if o.get("FrameWallClock") is not None:
            o["FrameWallClock"] -= delta // 1000
    meta.write_text(json.dumps(objs))
    csv = session / "video" / "0001.frames.csv"
    lines = csv.read_text().splitlines()
    out = [lines[0]]
    for line in lines[1:]:
        f = line.split(",")
        f[1] = str(int(f[1]) - delta)
        if f[2]:
            f[2] = str(int(f[2]) - delta // 1000)
        out.append(",".join(f))
    csv.write_text("\n".join(out) + "\n")
    rad = session / "radar" / "0001.csv"
    lines = rad.read_text().splitlines()
    out = [lines[0]]
    for line in lines[1:]:
        t, rest = line.split(",", 1)
        out.append(f"{int(t) - delta},{rest}")
    rad.write_text("\n".join(out) + "\n")
    sj = session / "session.json"
    data = json.loads(sj.read_text())
    for cs in data["clock_samples"]:
        if cs["boottime_ns"] > m0[-1].sensor_ts_ns:
            cs["boottime_ns"] -= delta
            cs["monotonic_ns"] -= delta
            cs["realtime_ns"] -= delta
    data["segments"][1]["start_boottime_ns"] -= delta
    data["segments"][1]["end_boottime_ns"] -= delta
    sj.write_text(json.dumps(data, indent=1))
    ev = session / "events.jsonl"
    evs = [json.loads(line) for line in ev.read_text().splitlines() if line.strip()]
    for e in evs:
        if e["boottime_ns"] > m0[-1].sensor_ts_ns:
            e["boottime_ns"] -= delta
    ev.write_text("\n".join(json.dumps(e) for e in evs) + "\n")


@pytest.fixture
def clean_session(tmp_path: Path) -> Path:
    """complete_2seg with a realistic 3.5 s segment gap: findings are notes only."""
    src = _copy("complete_2seg", tmp_path, "clean")
    _shift_segment1(src, 3_500_000_000)
    return src


def test_realistic_segment_gap_is_a_note_not_a_warning(clean_session: Path, tmp_path: Path) -> None:
    res = convert_session(clean_session, tmp_path / "o", Config(), ConvertOptions())
    assert any(n == "segment gap 0->1: 3500000000 ns" for n in res.notes), res.notes
    assert not any("segment gap" in w for w in res.warnings)


def test_convert_exit_0_for_clean_session(
    clean_session: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    out = tmp_path / "out"
    assert main(["convert", str(clean_session), "-o", str(out)]) == 0
    cap = capsys.readouterr()
    assert cap.out.splitlines() == [str(out / "s000066.mcap")]
    assert (out / "s000066.mcap").is_file()
    assert "ok:" in cap.err and "note: segment gap 0->1" in cap.err
    assert "1 converted, 0 with warnings, 0 failed" in cap.err


def test_exit_3_when_warnings(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    src = _copy("in_progress_unlisted", tmp_path)
    out = tmp_path / "out"
    assert main(["convert", str(src), "-o", str(out)]) == 3
    cap = capsys.readouterr()
    assert "ok with warnings" in cap.err and "    warning: " in cap.err
    assert len(cap.out.splitlines()) == 1


def test_exit_1_and_no_output_file_on_failure(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    src = _copy("complete_2seg", tmp_path)
    _break_meta(src)
    out = tmp_path / "out"
    assert main(["convert", str(src), "-o", str(out)]) == 1
    cap = capsys.readouterr()
    assert cap.out == ""
    assert "failed:" in cap.err and "--allow-prefix to convert the common prefix" in cap.err
    assert cap.err.count("--allow-prefix") == 1  # hint not appended twice
    assert "1 failed" in cap.err
    assert list(out.iterdir()) == []  # neither final nor .tmp


def test_allow_prefix_converts_broken_session(tmp_path: Path) -> None:
    src = _copy("complete_2seg", tmp_path)
    _break_meta(src)
    assert main(["convert", str(src), "-o", str(tmp_path / "o"), "--allow-prefix"]) == 3


def test_existing_output_refused_without_overwrite(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    src = _copy("complete_2seg", tmp_path)
    out = tmp_path / "out"
    out.mkdir()
    target = out / "s000066.mcap"
    target.write_bytes(b"precious")
    assert main(["convert", str(src), "-o", str(out)]) == 1
    assert target.read_bytes() == b"precious"
    assert "--overwrite" in capsys.readouterr().err


def test_overwrite_replaces(tmp_path: Path) -> None:
    src = _copy("complete_2seg", tmp_path)
    out = tmp_path / "out"
    out.mkdir()
    target = out / "s000066.mcap"
    target.write_bytes(b"old")
    assert main(["convert", str(src), "-o", str(out), "--overwrite"]) == 3  # 299 s gap WARN
    assert target.read_bytes() != b"old"
    assert not list(out.glob("*.tmp"))


def test_directory_of_sessions_batch(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    batch = tmp_path / "batch"
    batch.mkdir()
    _copy("interrupted", batch)
    _copy("complete_2seg", batch)
    (batch / "not-a-session").mkdir()
    out = tmp_path / "out"
    assert main(["convert", str(batch), "-o", str(out)]) == 3
    cap = capsys.readouterr()
    assert len(cap.out.splitlines()) == 2
    assert len(list(out.glob("*.mcap"))) == 2
    # sorted child order: complete_2seg before interrupted
    assert cap.err.index("complete_2seg") < cap.err.index("interrupted")
    assert "2 converted, 2 with warnings, 0 failed" in cap.err


def test_one_failure_does_not_stop_the_batch(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    batch = tmp_path / "batch"
    batch.mkdir()
    bad = _copy("complete_2seg", batch, "a_bad")
    _break_meta(bad)
    _copy("interrupted", batch, "b_good")
    out = tmp_path / "out"
    assert main(["convert", str(batch), "-o", str(out)]) == 1
    cap = capsys.readouterr()
    assert len(cap.out.splitlines()) == 1 and Path(cap.out.strip()).is_file()
    assert "1 converted, 1 with warnings, 1 failed" in cap.err


def test_jobs_2_matches_serial(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    batch = tmp_path / "batch"
    batch.mkdir()
    _copy("complete_2seg", batch)
    _copy("interrupted", batch)
    out1, out2 = tmp_path / "o1", tmp_path / "o2"
    assert main(["convert", str(batch), "-o", str(out1)]) == 3
    serial = capsys.readouterr()
    assert main(["convert", str(batch), "-o", str(out2), "--jobs", "2"]) == 3
    par = capsys.readouterr()
    assert [Path(p).name for p in par.out.splitlines()] == [
        Path(p).name for p in serial.out.splitlines()
    ]
    assert par.err.index("complete_2seg") < par.err.index("interrupted")
    for p in out1.glob("*.mcap"):
        assert (out2 / p.name).is_file()


def test_jobs_2_reports_failures(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    a = _copy("complete_2seg", tmp_path)
    _break_meta(a)
    b = _copy("interrupted", tmp_path)
    assert main(["convert", str(a), str(b), "-o", str(tmp_path / "o"), "--jobs", "2"]) == 1
    assert "failed:" in capsys.readouterr().err


def test_quiet_hides_success_lines_but_keeps_warnings(
    clean_session: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    other = _copy("interrupted", tmp_path)
    assert (
        main(["convert", str(clean_session), str(other), "-o", str(tmp_path / "o"), "--quiet"]) == 3
    )
    err = capsys.readouterr().err
    assert "ok:" not in err and "note:" not in err
    assert "ok with warnings" in err and "warning:" in err


def test_no_args_prints_help_exit_2_unchanged(capsys: pytest.CaptureFixture[str]) -> None:
    assert main([]) == 2
    assert "convert" in capsys.readouterr().out


def test_convert_without_output_dir_exits_2(tmp_path: Path) -> None:
    with pytest.raises(SystemExit) as exc:
        main(["convert", str(FIXTURES / "complete_2seg")])
    assert exc.value.code == 2


def test_bad_config_exits_2_before_converting(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    cfg = tmp_path / "bad.toml"
    cfg.write_text("[nonsense]\nx = 1\n")
    out = tmp_path / "out"
    assert (
        main(["convert", str(FIXTURES / "complete_2seg"), "-o", str(out), "--config", str(cfg)])
        == 2
    )
    assert "unknown section" in capsys.readouterr().err
    assert not out.exists()


def test_nonexistent_path_fails_but_others_continue(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    src = _copy("interrupted", tmp_path)
    out = tmp_path / "out"
    code = main(["convert", str(tmp_path / "nope"), str(src), "-o", str(out)])
    cap = capsys.readouterr()
    assert code == 1
    assert "nope" in cap.err and "failed" in cap.err
    assert len(cap.out.splitlines()) == 1


def test_only_nonexistent_path_is_a_failure(tmp_path: Path) -> None:
    assert main(["convert", str(tmp_path / "nope"), "-o", str(tmp_path / "o")]) == 1


def test_directory_without_sessions_is_usage_error(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    empty = tmp_path / "empty"
    empty.mkdir()
    assert main(["convert", str(empty), "-o", str(tmp_path / "o")]) == 2
    assert "no sessions found" in capsys.readouterr().err


def test_duplicate_arguments_converted_once(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    src = _copy("interrupted", tmp_path)
    code = main(["convert", str(src), str(src / "."), str(src), "-o", str(tmp_path / "o")])
    cap = capsys.readouterr()
    assert code == 3
    assert len(cap.out.splitlines()) == 1
    assert "1 converted" in cap.err


def _first_log_time(path: Path) -> int:
    with open(path, "rb") as fh:
        return min(m.log_time for _s, _c, m in make_reader(fh).iter_messages())


def test_time_base_boot_reaches_output(tmp_path: Path) -> None:
    src = _copy("complete_2seg", tmp_path)
    wall, boot = tmp_path / "wall", tmp_path / "boot"
    main(["convert", str(src), "-o", str(wall), "--time-base", "wall"])
    main(["convert", str(src), "-o", str(boot), "--time-base", "boot"])
    t_wall = _first_log_time(wall / "s000066.mcap")
    t_boot = _first_log_time(boot / "s000066.mcap")
    assert t_wall > 1_000_000_000_000_000_000  # realtime epoch ns
    assert t_boot < 10**13  # boot ns: well under three hours of uptime


def test_per_segment_writes_one_file_per_segment(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    src = _copy("complete_2seg", tmp_path)
    out = tmp_path / "out"
    main(["convert", str(src), "-o", str(out), "--per-segment"])
    lines = capsys.readouterr().out.splitlines()
    assert sorted(Path(p).name for p in lines) == ["s000066-seg0000.mcap", "s000066-seg0001.mcap"]
    assert all(Path(p).is_file() for p in lines)


def test_python_dash_m_version() -> None:
    res = subprocess.run(
        [sys.executable, "-m", "radar_cam_2_mcap", "--version"],
        capture_output=True,
        text=True,
        check=True,
    )
    assert __version__ in res.stdout


def test_worker_crash_is_a_failure_and_keeps_finished_results(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    """A fake executor runs the first session for real and breaks the pool for the second.

    A real os._exit in a worker cannot be injected under the spawn start method (the child
    re-imports the module, so monkeypatches do not reach it).
    """
    from concurrent.futures import Future
    from concurrent.futures.process import BrokenProcessPool

    from radar_cam_2_mcap import cli

    class FakePool:
        def __init__(self, max_workers: int) -> None:
            self.n = 0

        def __enter__(self) -> "FakePool":
            return self

        def __exit__(self, *a: object) -> None:
            return None

        def submit(self, fn, *args):  # type: ignore[no-untyped-def]
            fut: Future[object] = Future()
            self.n += 1
            if self.n == 1:
                fut.set_result(fn(*args))
            else:
                fut.set_exception(BrokenProcessPool("a process in the pool was terminated"))
            return fut

    monkeypatch.setattr(cli, "ProcessPoolExecutor", FakePool)
    a = _copy("complete_2seg", tmp_path)
    b = _copy("interrupted", tmp_path)
    out = tmp_path / "out"
    assert main(["convert", str(a), str(b), "-o", str(out), "--jobs", "2"]) == 1
    cap = capsys.readouterr()
    assert cap.out.splitlines() == [str(out / "s000066.mcap")]
    assert f"failed: {b}: BrokenProcessPool: worker crashed" in cap.err
    assert "Traceback" not in cap.err
    assert "1 converted, 1 with warnings, 1 failed" in cap.err


def test_unknown_flag_exits_2() -> None:
    with pytest.raises(SystemExit) as exc:
        main(["convert", "x", "-o", "y", "--bogus"])
    assert exc.value.code == 2


def test_output_dir_that_is_a_file_fails_cleanly(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    src = _copy("interrupted", tmp_path)
    target = tmp_path / "afile"
    target.write_text("x")
    assert main(["convert", str(src), "-o", str(target)]) == 1
    cap = capsys.readouterr()
    assert cap.out == "" and "Traceback" not in cap.err
    assert "failed:" in cap.err and "afile" in cap.err
    assert "existing file" in cap.err


def test_quiet_success_with_notes_prints_only_paths(
    clean_session: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    out = tmp_path / "o"
    assert main(["convert", str(clean_session), "-o", str(out), "--quiet"]) == 0
    cap = capsys.readouterr()
    assert cap.out.splitlines() == [str(out / "s000066.mcap")]
    assert "ok:" not in cap.err and "note:" not in cap.err


@pytest.mark.parametrize("extra", [[], ["--overwrite"], ["--overwrite", "--jobs", "2"]])
def test_colliding_output_names_refused_before_converting(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], extra: list[str]
) -> None:
    """Two sessions with the same session_id (two Pis, or a copied folder) collide."""
    batch = tmp_path / "batch"
    batch.mkdir()
    a = _copy("complete_2seg", batch, "pi_a")
    b = _copy("complete_2seg", batch, "pi_b")
    _copy("interrupted", batch, "pi_c")
    out = tmp_path / "out"
    assert main(["convert", str(batch), "-o", str(out), *extra]) == 2
    cap = capsys.readouterr()
    assert cap.out == ""
    assert "s000066.mcap" in cap.err and str(a) in cap.err and str(b) in cap.err
    assert not out.exists() or list(out.iterdir()) == []


def test_per_segment_collision_detected(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    a = _copy("complete_2seg", tmp_path, "a")
    b = _copy("complete_2seg", tmp_path, "b")
    out = tmp_path / "out"
    assert main(["convert", str(a), str(b), "-o", str(out), "--per-segment"]) == 2
    assert "s000066-seg0000.mcap" in capsys.readouterr().err


class _ScriptedPool:
    """Stand-in for ProcessPoolExecutor: call ``n`` (1-based) does what ``script[n]`` says.

    "run" runs the job in-process, "crash" fails the future with BrokenProcessPool (after
    ``on_crash``), "raise" makes submit() itself raise OSError. Missing entries run.
    """

    script: ClassVar[dict[int, str]] = {}
    on_crash: ClassVar[object] = None

    def __init__(self, max_workers: int) -> None:
        self.n = 0

    def __enter__(self) -> "_ScriptedPool":
        return self

    def __exit__(self, *a: object) -> None:
        return None

    def submit(self, fn, *args):  # type: ignore[no-untyped-def]
        from concurrent.futures import Future
        from concurrent.futures.process import BrokenProcessPool

        self.n += 1
        action = self.script.get(self.n, "run")
        if action == "raise":
            raise OSError(11, "Resource temporarily unavailable")
        fut: Future[object] = Future()
        if action == "crash":
            if callable(self.on_crash):
                self.on_crash()
            fut.set_exception(BrokenProcessPool("a process in the pool was terminated"))
        else:
            fut.set_result(fn(*args))
        return fut


def _three_sessions(tmp_path: Path) -> list[Path]:
    batch = tmp_path / "batch"
    batch.mkdir()
    a = _copy("complete_2seg", batch, "a")
    b = _copy("interrupted", batch, "b")
    c = _copy("in_progress_unlisted", batch, "c")
    return [a, b, c]


def test_submit_failure_reports_not_run_and_keeps_summary(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    from radar_cam_2_mcap import cli

    monkeypatch.setattr(_ScriptedPool, "script", {2: "raise"})
    monkeypatch.setattr(cli, "ProcessPoolExecutor", _ScriptedPool)
    _a, b, c = _three_sessions(tmp_path)
    out = tmp_path / "out"
    assert main(["convert", str(tmp_path / "batch"), "-o", str(out), "--jobs", "2"]) == 1
    cap = capsys.readouterr()
    assert "Traceback" not in cap.err
    assert cap.out.splitlines() == [str(out / "s000066.mcap")]
    assert f"failed: {b}: NotRun: not run: pool broken (OSError" in cap.err
    assert f"failed: {c}: NotRun: not run: pool broken" in cap.err
    assert "1 converted, 1 with warnings, 2 failed" in cap.err


def test_pool_creation_failure_reports_every_session_not_run(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    from radar_cam_2_mcap import cli

    def boom(max_workers: int) -> None:
        raise OSError(24, "Too many open files")

    monkeypatch.setattr(cli, "ProcessPoolExecutor", boom)
    _three_sessions(tmp_path)
    assert main(["convert", str(tmp_path / "batch"), "-o", str(tmp_path / "o"), "--jobs", "2"]) == 1
    err = capsys.readouterr().err
    assert err.count("NotRun: not run: pool broken (OSError") == 3
    assert "0 converted, 0 with warnings, 3 failed" in err


def test_crashed_worker_temp_files_removed_and_pending_not_run(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    from radar_cam_2_mcap import cli

    out = tmp_path / "out"
    leftover = out / "s000030.mcap.k3j9x.tmp"
    unrelated = out / "notes.tmp"

    def write_leftover() -> None:  # what a SIGKILLed worker leaves behind
        out.mkdir(exist_ok=True)
        leftover.write_bytes(b"partial")
        unrelated.write_bytes(b"keep")

    monkeypatch.setattr(_ScriptedPool, "script", {2: "crash"})
    monkeypatch.setattr(_ScriptedPool, "on_crash", staticmethod(write_leftover))
    monkeypatch.setattr(cli, "ProcessPoolExecutor", _ScriptedPool)
    _a, b, c = _three_sessions(tmp_path)
    assert main(["convert", str(tmp_path / "batch"), "-o", str(out), "--jobs", "2"]) == 1
    cap = capsys.readouterr()
    assert not leftover.exists() and unrelated.exists()
    assert f"failed: {b}: BrokenProcessPool: worker crashed" in cap.err
    assert "removed 1 leftover temp file" in cap.err
    assert f"failed: {c}: NotRun: not run: pool broken" in cap.err
    assert cap.out.splitlines() == [str(out / "s000066.mcap")]
