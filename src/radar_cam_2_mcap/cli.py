"""Command line interface.

Exit codes: 0 every session converted without WARN-level findings; 3 every session converted
but at least one has warnings; 1 at least one session failed; 2 usage or config error (before
any conversion starts). Output paths go to stdout, one per line; the summary goes to stderr.
"""

from __future__ import annotations

import argparse
import logging
import sys
from collections.abc import Sequence
from concurrent.futures import Future, ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

from radar_cam_2_mcap import __version__
from radar_cam_2_mcap.config import Config, ConfigError, load_config
from radar_cam_2_mcap.convert import ConvertOptions, OutputExistsError, convert_session
from radar_cam_2_mcap.video import FrameCountMismatch

log = logging.getLogger(__name__)


@dataclass
class _Outcome:
    """Picklable result of one session (plain data only, so it crosses process boundaries)."""

    session: str
    outputs: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    error_type: str | None = None
    error: str | None = None

    @property
    def failed(self) -> bool:
        return self.error_type is not None


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="radar-cam-2-mcap",
        description="Convert a radar-cam-recorder session folder into a ROS 2 MCAP file.",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    sub = parser.add_subparsers(dest="command")
    conv = sub.add_parser(
        "convert",
        help="convert one or more sessions",
        description=(
            "Convert sessions to MCAP. A path containing session.json is one session; any "
            "other directory is a batch directory whose child folders containing session.json "
            "are converted. Exit codes: 0 ok, 3 ok with warnings, 1 a session failed, "
            "2 usage or config error."
        ),
    )
    conv.add_argument("sessions", nargs="+", metavar="SESSION_OR_DIR", type=Path)
    conv.add_argument("-o", "--output-dir", required=True, type=Path, metavar="OUT_DIR")
    conv.add_argument("--config", type=Path, metavar="FILE", help="TOML config file")
    conv.add_argument("--overwrite", action="store_true", help="replace existing output files")
    conv.add_argument(
        "--per-segment", action="store_true", help="write one MCAP file per recording segment"
    )
    conv.add_argument(
        "--allow-prefix",
        action="store_true",
        help="when video frames and meta disagree in count, convert the common prefix",
    )
    conv.add_argument("--time-base", choices=("wall", "boot"), default=None)
    conv.add_argument("--jobs", type=int, default=1, metavar="N", help="parallel sessions")
    conv.add_argument("--quiet", action="store_true", help="only print failures and warnings")
    return parser


def _run_one(session: str, out_dir: str, cfg: Config, opts: ConvertOptions) -> _Outcome:
    """Convert one session; never raises (module level so it can run in a worker process)."""
    log.info("converting %s", session)
    try:
        res = convert_session(Path(session), Path(out_dir), cfg, opts)
    except Exception as exc:
        msg = str(exc) or type(exc).__name__
        if isinstance(exc, FrameCountMismatch) and not opts.allow_prefix:
            msg += " (--allow-prefix converts the common prefix of the frames)"
        elif isinstance(exc, (FileExistsError, NotADirectoryError)):
            msg += f" (is the output directory {out_dir} an existing file?)"
        elif isinstance(exc, OutputExistsError) and "--overwrite" not in msg:
            msg += " (pass --overwrite to replace existing output)"
        return _Outcome(session, error_type=type(exc).__name__, error=msg)
    return _Outcome(
        session,
        outputs=[str(p) for p in res.outputs],
        warnings=list(res.warnings),
        notes=list(res.notes),
    )


def _await(future: Future[_Outcome], session: str) -> _Outcome:
    """Result of a worker; a crashed worker or pool becomes a failed outcome for its session."""
    try:
        return future.result()
    except Exception as exc:
        return _Outcome(
            session,
            error_type=type(exc).__name__,
            error=f"worker crashed: {exc}" if str(exc) else "worker crashed",
        )


def _collect(
    paths: Sequence[Path],
) -> tuple[list[Path], list[_Outcome]]:
    """Expand arguments to session folders (deduplicated) plus failures for bad arguments."""
    sessions: list[Path] = []
    seen: set[Path] = set()
    bad: list[_Outcome] = []

    def add(p: Path) -> None:
        key = p.resolve()
        if key not in seen:
            seen.add(key)
            sessions.append(p)

    for path in paths:
        if (path / "session.json").is_file():
            add(path)
        elif path.is_dir():
            try:
                children = sorted(c for c in path.iterdir() if (c / "session.json").is_file())
            except OSError as exc:
                bad.append(_Outcome(str(path), error_type=type(exc).__name__, error=str(exc)))
                continue
            for child in children:
                add(child)
        else:
            bad.append(
                _Outcome(
                    str(path),
                    error_type="UsageError",
                    error="not a session folder or a directory of sessions",
                )
            )
    return sessions, bad


def _print_summary(outcomes: Sequence[_Outcome], quiet: bool) -> None:
    def err(text: str) -> None:
        print(text, file=sys.stderr)

    for o in outcomes:
        if o.failed:
            err(f"failed: {o.session}: {o.error_type}: {o.error}")
            continue
        if quiet and not o.warnings:
            continue
        status = "ok with warnings" if o.warnings else "ok"
        err(f"{status}: {o.session} -> {', '.join(o.outputs)}")
        for w in o.warnings:
            err(f"    warning: {w}")
        if not quiet:
            for n in o.notes:
                err(f"    note: {n}")
    failed = sum(o.failed for o in outcomes)
    warned = sum(bool(o.warnings) for o in outcomes if not o.failed)
    converted = len(outcomes) - failed
    err(f"{converted} converted, {warned} with warnings, {failed} failed")


def _convert(args: argparse.Namespace, parser: argparse.ArgumentParser) -> int:
    if args.jobs < 1:
        parser.error("--jobs must be at least 1")
    try:
        cfg = load_config(args.config)
    except ConfigError as exc:
        print(f"radar-cam-2-mcap: config error: {exc}", file=sys.stderr)
        return 2
    sessions, bad = _collect(args.sessions)
    if not sessions and not bad:
        print("radar-cam-2-mcap: no sessions found (no session.json)", file=sys.stderr)
        return 2
    time_base: Literal["wall", "boot"] | None = args.time_base
    opts = ConvertOptions(
        overwrite=args.overwrite,
        per_segment=args.per_segment,
        allow_prefix=args.allow_prefix,
        time_base=time_base,
    )
    out_dir = str(args.output_dir)
    results: list[_Outcome]
    if args.jobs > 1 and len(sessions) > 1:
        with ProcessPoolExecutor(max_workers=args.jobs) as pool:
            futures = [pool.submit(_run_one, str(s), out_dir, cfg, opts) for s in sessions]
            results = [_await(f, str(s)) for f, s in zip(futures, sessions, strict=True)]
    else:
        results = [_run_one(str(s), out_dir, cfg, opts) for s in sessions]
    outcomes = [*bad, *results]
    for o in results:
        for path in o.outputs:
            print(path)
    _print_summary(outcomes, args.quiet)
    if any(o.failed for o in outcomes):
        return 1
    if any(o.warnings for o in outcomes):
        return 3
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s: %(message)s")
    parser = _build_parser()
    args = parser.parse_args(argv)  # argparse exits 2 on usage errors
    if args.command == "convert":
        return _convert(args, parser)
    parser.print_help()
    return 2
