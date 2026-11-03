"""Command line interface."""

from __future__ import annotations

import argparse
from collections.abc import Sequence

from radar_cam_2_mcap import __version__


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="radar-cam-2-mcap",
        description="Convert a radar-cam-recorder session folder into a ROS 2 MCAP file.",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    parser.parse_args(argv)
    parser.print_help()
    return 2
