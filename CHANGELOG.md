# Changelog

All notable changes to this project are documented in this file.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [0.1.0]

First release (alpha).

### Added

- `radar-cam-2-mcap convert`: session folder (or a directory of sessions) to ROS 2 MCAP, with
  `--per-segment`, `--allow-prefix`, `--time-base`, `--jobs`, `--overwrite`, `--quiet`, `--config`.
- Output contract 0.1 (draft): camera images, camera info, camera metadata, radar scans, radar
  status, events and static transforms; file-level metadata with `time_quality` and warnings.
- Exit codes 0 (ok), 3 (ok with warnings), 1 (failure), 2 (usage or config error).
- Config keys including `radar.angle_sign` (unverified) and `radar.invert_direction` (default
  true, derived from real session s000060).
- Dockerfile (multi-stage, uv, non-root); not yet built or run.
- `scripts/verify_output.py` and [docs/verification.md](docs/verification.md).
