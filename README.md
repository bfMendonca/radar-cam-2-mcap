<div align="center">

# radar-cam-2-mcap

**Convert a radar-cam-recorder session folder into a ROS 2 MCAP file**

[![CI](https://github.com/bfMendonca/radar-cam-2-mcap/actions/workflows/ci.yml/badge.svg)](https://github.com/bfMendonca/radar-cam-2-mcap/actions/workflows/ci.yml)
[![Python 3.11 | 3.12](https://img.shields.io/badge/python-3.11%20%7C%203.12-blue)](pyproject.toml)
[![License: MIT](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)
[![mypy: strict](https://img.shields.io/badge/mypy-strict-blue)](pyproject.toml)
[![uv](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/uv/main/assets/badge/v0.json)](https://github.com/astral-sh/uv)
<br>
[![MCAP + ROS 2 CDR](https://img.shields.io/badge/MCAP-ROS%202%20CDR-22314E)](https://mcap.dev)
[![Raspberry Pi Zero](https://img.shields.io/badge/Raspberry%20Pi-Zero%20camera%20v1.3-C51A4A)](https://github.com/bfMendonca/radar-cam-recorder)
[![HLK-LD2451 radar](https://img.shields.io/badge/radar-HLK--LD2451-lightgrey)](https://github.com/bfMendonca/ld2451)
<br>
[![GitHub stars](https://img.shields.io/github/stars/bfMendonca/radar-cam-2-mcap?style=flat&logo=github)](https://github.com/bfMendonca/radar-cam-2-mcap/stargazers)
[![Forks](https://img.shields.io/github/forks/bfMendonca/radar-cam-2-mcap?style=flat&logo=github)](https://github.com/bfMendonca/radar-cam-2-mcap/forks)
[![Last commit](https://img.shields.io/github/last-commit/bfMendonca/radar-cam-2-mcap/main?style=flat&logo=github)](https://github.com/bfMendonca/radar-cam-2-mcap/commits/main)

[Features](#features) ·
[How it works](#how-it-works) ·
[Install](#install) ·
[Quick start](#quick-start) ·
[Usage](#usage) ·
[Output](#output) ·
[Known limitations](#known-limitations) ·
[Contributing](CONTRIBUTING.md)

<br>

<img src="docs/images/pipeline.svg" alt="Flow diagram: a session folder and a TOML config are decoded, mapped to ROS 2 messages, merged by log time and written to an MCAP file through a temporary file." width="90%">

<sub>The conversion pipeline, drawn from the stages in <code>src/radar_cam_2_mcap/convert.py</code>.</sub>

</div>

---

A Raspberry Pi Zero speed trap records plain files: raw H.264 video, per-frame camera metadata and
raw radar bytes. This tool runs on a laptop, workstation or container and turns one recorded
session into one MCAP file with standard ROS 2 messages and an explicit timestamp on every frame
and radar reading. It never modifies the source folder.

Status: version 0.1.0, alpha. Topic and metadata names are a draft (`contract_version` 0.2). The
catalog and ingestion pipeline that will consume the files are a separate project and are out of
scope here.

## Features

- **One file per session.** The 5-minute segments are joined; `--per-segment` writes one file per
  segment instead.
- **Images frame by frame.** `sensor_msgs/msg/CompressedImage`, JPEG quality 90 with 4:2:0
  subsampling by default, PNG optional (`[image]`).
- **Capture timing.** `log_time` equals `header.stamp` on every image and radar message. Boot
  time is mapped to wall time through the recorder's clock anchors, and `time_quality` records
  how trustworthy that clock was ([time model](docs/time-model.md)).
- **Radar frame recovery.** Radar bytes are reassembled with the
  [ld2451](https://github.com/bfMendonca/ld2451) driver; data frames whose count or length
  overstate the targets present are kept (`recover_short_frames`). Messages are
  `radar_msgs/msg/RadarScan`.
- **Sensor placement and camera info.** `/tf_static` comes from the OpenSCAD design of the mount;
  `CameraInfo` comes from the config, with uncalibrated Pi camera v1.3 defaults.
- **Damaged sessions.** Interrupted and in-progress sessions convert with a warning; frames that
  fail to decode are skipped without shifting later timestamps ([usage notes](docs/usage.md)).
- **Safe output.** Each file is written to a temp file and renamed only when the session
  converts; `--jobs N` converts N sessions in parallel.

## How it works

The diagram above is the flow `convert_session` follows for each session.

1. Read `session.json` and discover the segments, then count the frames of every segment
   (H.264 slice NAL units against `meta.json` entries) before decoding anything.
2. Build a `Timeline` from the clock anchors in `session.json`.
3. Stream three generators: video (PyAV decode, Pillow encode), radar (`ld2451` decoder with
   recovery) and `events.jsonl`, plus the static `CameraInfo` and `/tf_static` messages.
4. Merge the streams by `log_time` (`heapq.merge`), so memory stays near one frame per stream.
5. Write MCAP chunks (zstd by default) to a temp file with file-level metadata records, then
   rename it to `<session_id>.mcap`.

## Install

You need Python 3.11 or newer (`python3 --version`) and, because the `ld2451` driver is installed
from GitHub, `git` and network access. Get [uv](https://docs.astral.sh/uv/) if you do not have it.
CI runs Python 3.11 and 3.12 on Ubuntu 24.04 and macOS.

With uv (recommended: `uv.lock` pins every dependency). From any directory:

```sh
git clone https://github.com/bfMendonca/radar-cam-2-mcap.git
cd radar-cam-2-mcap
uv sync
uv run radar-cam-2-mcap --version
```

The last command prints `radar-cam-2-mcap 0.1.0`. `uv sync` took about 3 s on a warm cache.

With pip or pipx (versions resolved from the ranges in `pyproject.toml`, not from the lock):

```sh
pipx install git+https://github.com/bfMendonca/radar-cam-2-mcap.git
```

The converter subclasses internals of the `ld2451` commit pinned in `pyproject.toml` (a direct
`git+https` reference). Do not replace it with another `ld2451`.

## Quick start

From the repository root, convert the bundled two-segment test session. It needs no hardware:

```sh
uv run radar-cam-2-mcap convert tests/fixtures/complete_2seg -o out
```

It prints the output path on stdout and a summary on stderr, and exits with code 3 because the
fixture's two segments are far apart in time:

```
ok with warnings: tests/fixtures/complete_2seg -> out/s000066.mcap
    warning: segment gap 0->1: 299632291000 ns
    note: radar decoding problems: trailing_bytes=12
    note: recovered 14 short radar data frames: reported count exceeds targets present
1 converted, 1 with warnings, 0 failed
out/s000066.mcap
```

`out/s000066.mcap` is now a 7.3 MB file. Check it (ordering, stamps, message counts):

```sh
uv run python scripts/verify_output.py out/s000066.mcap --session tests/fixtures/complete_2seg
```

It ends with `RESULT: OK (0 failed checks)`. To convert your own recordings, copy a session folder
(`sNNNNNN`, containing `session.json`) from the Pi, then run
`uv run radar-cam-2-mcap convert path/to/sNNNNNN -o out`.

Playing the file in Foxglove or ROS tools has not been tested ([Known limitations](#known-limitations)).

## Usage

```
radar-cam-2-mcap convert [-h] -o OUT_DIR [--config FILE] [--overwrite] [--per-segment]
                         [--allow-prefix] [--time-base {wall,boot}] [--jobs N] [--quiet]
                         SESSION_OR_DIR [SESSION_OR_DIR ...]
```

A folder containing `session.json` is one session; any other directory is a batch whose child
folders containing `session.json` are converted.

| Option | Effect |
| --- | --- |
| `-o`, `--output-dir` | Output directory (created if missing). Required. |
| `--config FILE` | TOML file that overrides only the keys it names ([configuration](docs/configuration.md)). |
| `--overwrite` | Replace existing output. Without it, an existing output file is an error. |
| `--per-segment` | One `sNNNNNN-segNNNN.mcap` per segment instead of one file per session. |
| `--allow-prefix` | When H.264 and `meta.json` disagree in frame count, convert the common prefix with a warning. |
| `--time-base {wall,boot}` | Timestamp base for this run; overrides `[time] base`. |
| `--jobs N` | Convert N sessions in parallel, one process each. Default 1. |
| `--quiet` | Print only failures and warnings. |

| Exit code | Meaning |
| --- | --- |
| 0 | Every session converted with no warnings. |
| 3 | Every session converted, at least one with warnings. |
| 1 | At least one session failed. |
| 2 | Usage or config error, no session found, or two sessions would write the same output file. |

Warnings include an unsynced clock, an interrupted or in-progress session, segment gaps of 10 s
or more, unrecoverable radar framing errors and a clock cross-check beyond tolerance. Gaps under
10 s, recovered radar frames and trailing bytes alone are notes. Details on damaged sessions, temp
files, name collisions and `--jobs` failures are in [docs/usage.md](docs/usage.md).

## Output

One MCAP per session, ROS 2 CDR with `ros2msg` schemas embedded in the file. Topic names are
configurable.

| Topic | Type |
| --- | --- |
| `/camera/image/compressed` | `sensor_msgs/msg/CompressedImage`, one per decoded frame |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo`, once per file |
| `/camera/metadata` | `radar_cam_msgs/msg/CameraMetadata`, per frame (exposure, gains, lux, ...) |
| `/radar/scan` | `radar_msgs/msg/RadarScan`, one per radar data frame |
| `/radar/status` | `radar_cam_msgs/msg/RadarStatus`, per radar data frame |
| `/events` | `rcl_interfaces/msg/Log`, recorder events plus the converter's findings |
| `/tf_static` | `tf2_msgs/msg/TFMessage`, once per file |

Each file also carries metadata records (`radar_cam_2_mcap`, `source_session`, `segments`,
`radar_stats`) with the contract version, time quality, warnings and counts. Fields, radar
mapping and sensor placement are in [docs/output-format.md](docs/output-format.md);
[docs/verification.md](docs/verification.md) lists the checks run on six real recordings.

## Docker

```sh
docker build -t radar-cam-2-mcap .
docker run --rm -v /path/to/sessions:/in:ro -v /path/to/out:/out radar-cam-2-mcap convert /in -o /out
```

The container runs as UID 1000, so the output directory must be writable by it. This image has
not been built or run: Docker was not available when it was written.

## Known limitations

- Playback in Foxglove or any real-time player has not been tested; only image spacing in
  `log_time` order was checked by script.
- Camera intrinsics are uncalibrated vendor-spec defaults. Sensor placement comes from CAD, not
  from measurement on the hardware.
- `radar.angle_sign` is unverified, and `radar.invert_direction` (default true) was derived from
  one session ([configuration](docs/configuration.md#caveats-on-the-defaults)).
- `amplitude` in `RadarReturn` is the raw SNR value, not dB.
- `contract_version` 0.2 is a draft; topic and metadata names may change.
- Output is about 4 to 7 times the size of the H.264 input, because frames are stored
  individually.
- `--per-segment`, `--jobs`, custom transforms, PNG output and cross-check failures have not been
  run on real data.
- It does not calibrate the camera, detect vehicles, compute speeds, or run on the Pi.

## Development

```sh
uv sync --all-groups
uv run pytest -q
uv run ruff check .
uv run ruff format --check .
uv run mypy --strict src
```

The test suite takes a few minutes. See [CONTRIBUTING.md](CONTRIBUTING.md),
[CHANGELOG.md](CHANGELOG.md) and [SECURITY.md](SECURITY.md).

## License

[MIT](LICENSE)

## Star history

<a href="https://star-history.com/#bfMendonca/radar-cam-2-mcap&Date">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=bfMendonca/radar-cam-2-mcap&type=Date&theme=dark">
    <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=bfMendonca/radar-cam-2-mcap&type=Date">
    <img alt="Star history chart" src="https://api.star-history.com/svg?repos=bfMendonca/radar-cam-2-mcap&type=Date">
  </picture>
</a>
