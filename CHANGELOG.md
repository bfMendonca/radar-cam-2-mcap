# Changelog

All notable changes to this project are documented in this file.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Fixed

- MCAP schema names are now ROS 2 style (`sensor_msgs/msg/CompressedImage`, not
  `sensor_msgs/CompressedImage`), so `ros2 bag` and av-perception-lab accept the files. Field
  types and `MSG:` dependency lines inside the schema text keep the short `pkg/Type` form, as in
  rosbag2. `contract_version` is now 0.2 (schema names are part of the contract).
- Sessions with the same `session_id` in one run (two recorders, or a copied folder) are refused
  with exit code 2 before anything is converted, instead of corrupting or replacing each other's
  output. Temp files now have unique names (`<name>.mcap.<random>.tmp`).
- A frame the decoder drops no longer shifts the timestamps of the frames after it: frames are
  paired with `meta.json` by packet index, and a skipped frame is reported as a warning.
- `--jobs`: a failing `submit()`, pool start or broken pool no longer ends in a traceback.
  Sessions that never started are reported `not run: pool broken`, the summary is always
  printed, and temp files left by a killed worker are removed.
- Config values with a valid type but an invalid range, length or duplicate topic name are a
  config error (exit code 2) instead of a late failure or silently wrong output.
- `--per-segment` on a session without convertible segments fails with a clear error.
- The `--allow-prefix` hint is no longer printed twice.

### Changed

- The frame counts of all segments are checked before conversion starts, so a
  `FrameCountMismatch` is reported in seconds (skipped with `--allow-prefix`).
- A warning is recorded when the decoded frame size differs from `[camera] width`/`height`.
- `ld2451` is a direct `git+https` reference pinned to commit c0ea789, so `pip` and `pipx`
  installs get the same driver as `uv`. Lower bounds for `av`, `mcap` and `mcap-ros2-support`;
  package metadata now has the README and project URLs.
- CI runs Python 3.11 and 3.12, and checks `ruff format`.

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
