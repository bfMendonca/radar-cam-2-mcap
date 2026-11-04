# radar-cam-2-mcap

Convert a session folder recorded by [radar-cam-recorder](https://github.com/bfMendonca/radar-cam-recorder)
(a Raspberry Pi Zero speed trap with a camera and an HLK-LD2451 radar) into one ROS 2 MCAP file.

The Pi records plain files: raw H.264 video, per-frame camera metadata and raw radar bytes. This
tool runs on a laptop, workstation or container, reassembles the radar frames with the
[ld2451](https://github.com/bfMendonca/ld2451) driver, decodes the video frame by frame and writes
standard ROS 2 messages (CDR) with explicit, documented timestamps. It never modifies the source
folder.

Status: version 0.1.0, alpha. The topic and metadata names are a draft (`contract_version` 0.1).
The catalog and ingestion pipeline that will consume the files are a separate project and are out
of scope here.

## Install

You need Python 3.11 or newer. CI runs Python 3.11 and 3.12 on Ubuntu 24.04 and macOS.

With [uv](https://docs.astral.sh/uv/) (recommended: `uv.lock` pins every dependency):

```sh
git clone https://github.com/bfMendonca/radar-cam-2-mcap.git
cd radar-cam-2-mcap
uv sync
uv run radar-cam-2-mcap --version
```

With pip or pipx (dependencies resolved from the version ranges in `pyproject.toml`, not the lock):

```sh
pipx install git+https://github.com/bfMendonca/radar-cam-2-mcap.git
# or, in a virtual environment:
pip install git+https://github.com/bfMendonca/radar-cam-2-mcap.git
```

Either way the `ld2451` driver is installed from GitHub at the commit pinned in `pyproject.toml`
(a direct `git+https` reference), so installing needs network access and `git`. The converter
relies on internals of that exact driver commit; do not replace it with another `ld2451`.

## Quick start

Copy a session folder (`sNNNNNN`, containing `session.json`) from the Pi, then:

```sh
uv run radar-cam-2-mcap convert path/to/s000066 -o out
```

This writes `out/s000066.mcap` and prints its path on stdout. A summary goes to stderr, for example:

```
ok with warnings: tests/fixtures/complete_2seg -> out/s000066.mcap
    warning: segment gap 0->1: 299632291000 ns
    note: radar decoding problems: trailing_bytes=12
    note: recovered 14 short radar data frames: reported count exceeds targets present
1 converted, 1 with warnings, 0 failed
```

(That output is from the committed test fixture `tests/fixtures/complete_2seg`.)

Check a result with the bundled script, which verifies ordering, stamps and message counts:

```sh
uv run python scripts/verify_output.py out/s000066.mcap --session path/to/s000066
```

Playing the file in Foxglove or ROS tools has not been tested (see [Known limitations](#known-limitations)).

## Usage

```
radar-cam-2-mcap convert [-h] -o OUT_DIR [--config FILE] [--overwrite] [--per-segment]
                         [--allow-prefix] [--time-base {wall,boot}] [--jobs N] [--quiet]
                         SESSION_OR_DIR [SESSION_OR_DIR ...]
```

| Option | Effect |
| --- | --- |
| `SESSION_OR_DIR` | A folder containing `session.json` is one session. Any other directory is a batch: its child folders that contain `session.json` are converted. |
| `-o`, `--output-dir` | Output directory (created if missing). Required. |
| `--config FILE` | TOML file; overrides only the keys it names. See [Configuration](#configuration). |
| `--overwrite` | Replace existing output. Without it, an existing output file is an error. |
| `--per-segment` | Write `sNNNNNN-segNNNN.mcap` per recording segment instead of one file per session. |
| `--allow-prefix` | When the H.264 stream and `meta.json` disagree in frame count, convert the common prefix and record a warning. |
| `--time-base {wall,boot}` | Timestamp base for this run; overrides `[time] base`. |
| `--jobs N` | Convert N sessions in parallel (one process each). Default 1. |
| `--quiet` | Print only failures and warnings. |

Each output is written to a uniquely named temp file in the output directory
(`<name>.mcap.<random>.tmp`) and renamed to its final name only when the session converts. A
session that fails, including a `--jobs` worker that is killed, leaves no file under the final
name; temp files of a killed worker are removed when the pool finishes. Only if the main
`radar-cam-2-mcap` process itself is killed can a `*.mcap.*.tmp` file stay behind.

The output name comes from `session_id` in `session.json`, and every recorder numbers its sessions
from `s000001`, so sessions from two devices can share a name. If two sessions of one run would
write the same output file, nothing is converted and the exit code is 2; convert them into
separate output directories.

With `--per-segment` the files of one session are renamed one after another, not as a set: if a
later rename fails, the earlier files already have their final names (with `--overwrite`, the
previous versions of those files are gone) although the session is reported failed.

With `--jobs N`, at most N sessions run at a time. If a worker dies (for example killed by the
out-of-memory killer), the sessions that were running are reported failed and the ones that had
not started are reported `not run: pool broken`; the summary is always printed.

### Exit codes

| Code | Meaning |
| --- | --- |
| 0 | Every session converted with no warnings. |
| 3 | Every session converted, at least one with warnings. |
| 1 | At least one session failed. |
| 2 | Usage or config error, or no session found. |

Warnings include an unsynced clock, an interrupted or in-progress session, segment gaps of 10 s or
more, unrecoverable radar framing errors and a clock cross-check beyond tolerance. Gaps under 10 s,
recovered radar frames and trailing bytes alone are notes, not warnings.

### Damaged sessions

Sessions in state `complete`, `interrupted` or `in_progress` are accepted; the last two produce a
warning. Segments are discovered from the files as well as from `session.json`. `meta.json` is the
authority for per-frame timestamps. Before any decoding, the frames of every segment are counted
(H.264 slice NAL units against `meta.json` entries); if they differ, the session fails at once with
a `FrameCountMismatch` error unless you pass `--allow-prefix`, which converts the common prefix of
each segment with a warning. An unknown recorder `format_version` is refused.

Frames are paired with `meta.json` by their position in the stream, not by the order the decoder
returns them. If the decoder cannot decode a frame (damaged data), that frame and its `meta.json`
entry are skipped and a warning names the frame indexes; the frames after it keep their own
timestamps.

## Output

One MCAP per session. Messages use ROS 2 CDR with `ros2msg` schemas, which are embedded in the
file. The message definitions are in [`src/radar_cam_2_mcap/msgdefs`](src/radar_cam_2_mcap/msgdefs);
the upstream ones are vendored unchanged and `radar_cam_msgs/*` are written for this project.

`contract_version` **0.1 is a draft proposal** for topic and metadata names, written before the
consuming pipeline exists. Topics and frame IDs are configurable; every file records its
`contract_version`.

| Topic | Type | Content |
| --- | --- | --- |
| `/camera/image/compressed` | `sensor_msgs/CompressedImage` | One per decoded frame, JPEG by default (PNG optional). |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | From config, written once per file. |
| `/camera/metadata` | `radar_cam_msgs/CameraMetadata` | Per frame: exposure, gains, lux, colour temperature and gains, colour correction matrix, black levels, focus FoM, AE state, scaler crop, recorder wall clock, boot-clock timestamp, segment and frame index. |
| `/radar/scan` | `radar_msgs/RadarScan` | One per decoded data frame; an empty frame gives a scan with no returns. |
| `/radar/status` | `radar_cam_msgs/RadarStatus` | Per data frame: reported target count, targets actually present, approach alarm, boot-clock receive time. |
| `/events` | `rcl_interfaces/Log` | Every `events.jsonl` entry (recorder incidents as WARN, state changes as INFO) plus the converter's own findings. |
| `/tf_static` | `tf2_msgs/TFMessage` | Sensor placement, written once per file. |

File-level metadata records:

* `radar_cam_2_mcap`: `contract_version`, `converter_version`, `time_quality`, `time_base`,
  `warnings`, `notes`, image and MCAP settings, `crosscheck_max_diff_ms`, `crosscheck_samples`.
* `source_session`: session id, boot id, state, stop reason, `ntp_synced`, `wall_clock_source`,
  clock anchors, recorder config, devices.
* `segments`: per-segment frame and radar counts and boot-clock spans.
* `radar_stats`: radar decoder counters (reads, frames, bytes discarded, framing errors, ...).

With `--per-segment`, the `warnings` and `notes` of each file are cumulative: file k holds the
session-level warnings plus the findings of files 0 to k, not those of later files. The summary
on stderr and the exit code cover the whole session. In the `/events` topic, each file has only
the findings that concern it.

### Radar mapping

Each LD2451 target becomes one `RadarReturn`:

| Field | Value |
| --- | --- |
| `range` | Distance in metres. |
| `azimuth` | `angle_sign` times the angle in radians (positive anticlockwise, REP-103). |
| `elevation` | 0.0 (unknown; the radar is 2D). |
| `doppler_velocity` | Speed in m/s. Negative for approaching targets, positive for receding ones. |
| `amplitude` | The raw LD2451 SNR value. It is not calibrated dB, although the upstream field is documented as dB. |

`header.stamp` is the radar receive time (`t_rx_ns`) mapped to the output time base; no latency
correction is applied, so it can carry up to one serial read of delay.

### Sensor placement (tf)

```
base_link (sensor plate: top face, centred, on the front face)
├── camera_link ── camera_optical_frame
└── radar_link
```

ROS convention (x forward, y left, z up), metres. Defaults from `config/default.toml`:

| Parent | Child | xyz (m) | rpy (rad) |
| --- | --- | --- | --- |
| `base_link` | `camera_link` | 0.005, 0.0195, 0.042 | 0, 0, 0 |
| `base_link` | `radar_link` | 0.0051, -0.0145, 0.042 | 0, 0, 0 |
| `camera_link` | `camera_optical_frame` | 0, 0, 0 | -pi/2, 0, -pi/2 |

The numbers were derived from the dimensions of the OpenSCAD design of the printed mount (the
`pizero-radar-cam-testplate` project), **not measured on hardware**. Both sensors are assumed to
point straight forward. Update `[[transforms]]` in your config if the mount differs; providing any
`[[transforms]]` replaces the whole default list.

## Time model

* Every device timestamp from the Pi is on `CLOCK_BOOTTIME` (radar `t_rx_ns`, camera
  `SensorTimestamp`).
* `session.json` holds clock anchors that pair boot time with wall-clock time (at start, at each
  segment and at stop). The converter maps boot time to wall time by linear interpolation
  between the surrounding anchors. Outside the anchor range it uses the nearest anchor's offset
  and records a warning.
* `log_time` and `header.stamp` are equal on every image, scan, status and camera-metadata
  message. With `--time-base boot` (or `[time] base = "boot"`) both are the boot-clock value
  instead of wall time. The metadata key `time_base` records which was used.
* `FrameWallClock` from the recorder is only a cross-check. It is compared on sampled frames and a
  difference above `[time] crosscheck_tolerance_ms` becomes a warning; the anchors win.
* `time_quality` in the file metadata is `rtc` or `ntp` when the recorder says the wall clock came
  from there, `ntp` when `ntp_synced` is true, otherwise `unsynced`. An unsynced session is still
  converted, with a warning, and its wall-clock timestamps are only as good as the Pi's clock was.
  Use `--time-base boot` if you do not trust that clock.
* Playback in `log_time` order reproduces the capture timing, including the recorder's frame drops
  and the camera restart gap (2.9 to 4.5 s in the verified sessions) at each segment boundary.

## Configuration

`config/default.toml` lists every key with its default. Pass a file with `--config`; it overrides
only the keys it names. Unknown keys, wrong types and out-of-range values (for example
`quality` outside 1 to 100, `subsampling` other than 0 to 2, `angle_sign` other than -1 or 1,
`k`/`r` without 9 values, `p` without 12, `d` of the wrong length for `plumb_bob`, or two topics
with the same name) are rejected with exit code 2 before anything is converted.

| Section | Key | Default | Meaning |
| --- | --- | --- | --- |
| `[image]` | `format` | `"jpeg"` | `"jpeg"` or `"png"`. |
| | `quality` | 90 | JPEG quality. |
| | `subsampling` | 2 | Pillow JPEG subsampling: 0 = 4:4:4, 1 = 4:2:2, 2 = 4:2:0. |
| `[mcap]` | `compression` | `"zstd"` | `"zstd"`, `"lz4"` or `"none"`. |
| | `chunk_size` | 4194304 | Chunk size in bytes. |
| `[camera]` | `width`, `height` | 1296, 972 | Image size for `CameraInfo`. A warning is recorded if the decoded frames have another size. |
| | `k`, `d`, `r`, `p` | see file | Intrinsics, distortion, rectification and projection matrices. |
| | `distortion_model` | `"plumb_bob"` | |
| | `calibrated` | `false` | Accepted in the config; not written to the output file. |
| `[topics]` | `image`, `camera_info`, `camera_metadata`, `radar_scan`, `radar_status`, `events`, `tf_static` | see the topic table | Topic names. |
| | `frame_camera`, `frame_radar` | `"camera_optical_frame"`, `"radar_link"` | Frame IDs of the camera and radar messages. |
| `[radar]` | `angle_sign` | 1 | Sign applied to the angle. Unverified, see below. |
| | `recover_short_frames` | `true` | Keep data frames whose count or length overstate the targets present. |
| | `invert_direction` | `true` | Invert the driver's `approaching` flag when mapping `doppler_velocity`. See below. |
| `[time]` | `crosscheck_tolerance_ms` | 50.0 | Cross-check threshold. |
| | `base` | `"wall"` | `"wall"` or `"boot"`. |
| `[[transforms]]` | `parent`, `child`, `xyz`, `rpy` | three entries | See [Sensor placement](#sensor-placement-tf). |

### Caveats on the defaults

* **`radar.angle_sign` is unverified.** Which direction a positive LD2451 angle means is not
  stated in the driver docs. One vehicle pass in session s000060 supports `+1` (positive azimuth
  on the left), with moderate confidence; see
  [docs/verification.md](docs/verification.md#d-radar-to-camera-alignment-and-angle-sign-s000060-best-effort).
  If you find it is the other way round, set `angle_sign = -1`.
* **`radar.invert_direction = true` was derived from data, not from the driver.** In session
  s000060 the driver's `approaching` flag was false for targets whose range was falling, so the
  converter flips it by default; the result is that `doppler_velocity` is negative for approaching
  targets. The cause (driver or firmware) was not investigated. Set it to `false` if the driver is
  fixed. It affects `/radar/scan` only: `approach_alarm` in `/radar/status` is the driver's value
  unchanged.
* **Camera intrinsics are uncalibrated vendor-spec defaults** for the OV5647 (v1.3) at 1296x972:
  fx = fy = 1285.7 px, principal point (648, 486), zero distortion. They come from datasheet
  figures that were not independently verified. Supply a real calibration through `[camera]`.

## Docker

The `Dockerfile` is multi-stage, uses uv, runs as a non-root user and has the entrypoint
`radar-cam-2-mcap`.

```sh
docker build -t radar-cam-2-mcap .
docker run --rm -v /path/to/sessions:/in:ro -v /path/to/out:/out radar-cam-2-mcap convert /in -o /out
```

The container user has UID 1000, so the output directory must be writable by it. This image has
not been built or run yet (Docker was not available when it was written).

## Verification

The converter was run on six real recordings, including multi-segment, interrupted and
in-progress sessions. The results, the checks performed (frame pairing, timestamps, segment
boundaries, radar recovery) and what was not verified are in [docs/verification.md](docs/verification.md).
The design is in [docs/superpowers/specs](docs/superpowers/specs/2026-11-01-radar-cam-2-mcap-design.md).

## Known limitations

* Playback in Foxglove or any real-time player has not been tested. Only a script-level check of
  image spacing in `log_time` order was done.
* Camera intrinsics are uncalibrated and the sensor placement is derived from CAD, not measured.
* `angle_sign` is unverified and `invert_direction` is empirical (see above).
* `amplitude` is the raw SNR value, not dB.
* `contract_version` 0.1 is a draft; topic and metadata names may change.
* Output is roughly 4 to 7 times the size of the H.264 input, because frames are stored
  individually as JPEG.
* `--per-segment`, `--jobs`, custom transforms, PNG output and cross-check failures have not been
  exercised on real data.
* The converter does not calibrate the camera, detect vehicles, compute speeds, or run on the Pi.
  The catalog and ingestion pipeline are out of scope.

## Development

```sh
uv sync --all-groups
uv run pytest
uv run ruff check .
uv run ruff format --check .
uv run mypy
```

See [CONTRIBUTING.md](CONTRIBUTING.md), [CHANGELOG.md](CHANGELOG.md) and [SECURITY.md](SECURITY.md).

## License

[MIT](LICENSE)
