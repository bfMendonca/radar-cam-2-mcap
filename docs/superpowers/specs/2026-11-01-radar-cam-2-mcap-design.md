# radar-cam-2-mcap: design

Status: DRAFT for approval. Nothing is built yet.

## 1. Intent

`radar-cam-recorder` (the Raspberry Pi Zero speed-trap recorder) cannot write MCAP on
the Pi: the Pi Zero is 32-bit ARMv6 and cannot run ROS 2. It records plain files instead:
raw H.264 video, per-frame camera metadata and raw radar bytes in CSV.

`radar-cam-2-mcap` is an offline converter that runs on a laptop, workstation or container.
Given a session folder `sNNNNNN` copied from the Pi, it writes one professional MCAP file
that looks like any other robot's recording, so the separate ingestion/catalog pipeline can
treat it the same. Playing the MCAP back must reproduce video frames and radar readings at
the times they were captured.

Why it matters: the data will train machine-learning models later, so fidelity, correct
timestamps and honest metadata matter more than file size.

## 2. Goals, non-goals, anti-goals

Goals
- One MCAP per session, segments joined; a flag to produce one MCAP per segment instead.
- Correct, explicit timestamps for every message, with a clear statement of time quality.
- Standard ROS 2 message types (CDR), readable by Foxglove and ROS tooling.
- All fixed parameters (camera info, sensor placement, image format, compression, topic
  names) come from config files with sensible defaults for this hardware.
- Tolerates real-world damage: interrupted sessions, truncated files, missing files, old
  recorder versions.
- Simple to use: one command; open-source quality (README, tests, CI, typing, uv).

Non-goals
- The catalog, ingestion pipeline, FiftyOne, Qwen/Ollama and the docker-compose stack.
  They belong to a separate project (see section 10).
- Camera calibration (the converter accepts calibration; it does not compute it).
- Running on the Pi, or modifying the Pi's recordings.
- Detecting vehicles or computing speeds.

Anti-goals (a failure even if it "works")
- Silently misaligning video and radar timestamps.
- Silently dropping data, or "fixing" damaged sessions without saying so.
- Presenting unsynced wall-clock time as trustworthy.

## 3. What the real data looks like

Studied on `workstation:~/workspace/hlk-ld2451-speed-trap/data` (21 sessions, 2.5 GB) and
the recorder source in `../radar-cam-recorder`. Sessions are written by recorder
`format_version: 1`.

Layout of `sNNNNNN/`: `session.json`, `events.jsonl`, `video/NNNN.{h264,meta.json,
pts.txt,frames.csv,log}`, `radar/NNNN.csv`.

Observations that drive the design:
- States seen: 18 `complete`, 2 `interrupted` (s000030, s000032), 1 `in_progress`
  (s000033, which has a fourth video segment that is not in `session.json` and has no
  `frames.csv`). Only s000066 (5 segments) and s000071 (6 segments) are multi-segment and
  complete; they are the main test inputs.
- Video is Annex B H.264, High profile, 1296x972, no B-frames, SPS/PPS inline. The camera
  is an OV5647 (Pi camera v1.3) in its 2x2 binned full-field mode.
- Frame rate is variable. Target is 25 fps; about 21.6 fps is delivered, with roughly 420
  to 470 drops per 5-minute segment and gaps up to 2.7 s. Every frame needs its own
  timestamp.
- About 3.3 to 3.6 s of video is lost at each segment boundary (camera restart); radar
  capture continues across it. `events.jsonl` records these as `segment gap`.
- Radar `radar/NNNN.csv` is `t_rx_ns,hex` per serial read. Reads are fragmented: command
  ACK frames (`fdfcfbfa...`) and data frames (`f4f3f2f1...f8f7f6`) can span reads. Frames
  must be reassembled with the `ld2451` driver's framing/decoding. `t_rx_ns` is the
  receive time of the read carrying the frame's first byte. At session start the file
  contains command ACKs from radar configuration.
- All device timestamps are `CLOCK_BOOTTIME`: radar `t_rx_ns`, camera `SensorTimestamp`.
  `session.json["clock_samples"]` pairs boot, monotonic and wall-clock readings at start,
  at each segment and at stop. Each frame also has `FrameWallClock` (microseconds).
- Wall-clock quality varies: `ntp_synced` is true in 8 of 21 sessions, and no existing
  session has the `wall_clock_source` or `rtc` keys that the recorder's README describes
  (older recorder). New recordings come from a Pi with an RTC fitted.
- `setup.geometry` is empty and `setup.notes` is the same text in every session, so sensor
  placement cannot come from the Pi.
- Per-frame camera metadata includes exposure, analogue/digital gain, lux, colour
  temperature and gains, colour correction matrix, black levels, focus FoM, AE state and
  `ScalerCrop`.

## 4. Decisions made with the user

1. One MCAP per session by default (segments joined); per-segment output as an option.
2. Video is stored frame by frame as `sensor_msgs/CompressedImage`, not as an H.264 stream.
3. Default image encoding: JPEG, quality 90, 4:2:0 chroma, configurable (format `jpeg` or
   `png`, and quality). Chosen from a measured test on s000066 segment 0 (250 frames):
   q90 gives 113 KiB/frame, 48.0 dB PSNR, SSIM 0.993, about 5.8x the H.264 size (the
   existing 2.5 GB becomes about 14 GB). PNG is about 31x. The user accepted this size.
4. MCAP chunk compression is configurable (`zstd`, `lz4`, `none`) along with chunk size.
5. `log_time` and `header.stamp` use wall-clock time derived from the recorded clock
   anchors. The boot-clock value is also kept. Each file records `time_quality`
   (`rtc`, `ntp` or `unsynced`). Sessions without sync are converted with a warning and
   `time_quality: unsynced`.
6. Sensor placement comes from the OpenSCAD design
   `~/openscad-projects/pizero-radar-cam-testplate/src/main.scad` (the printed mount; the
   README photo is out of date). Fixed parameters live in config files.
7. Python, `uv`, README via the readme skill, open-source structure like the sibling
   projects. A `Dockerfile` so a compose stack can run it as a job.

## 5. Output contract

All messages are ROS 2 messages with CDR encoding (`ros2msg` schemas). Topic names and
frame IDs are configurable; defaults below are the converter's call.

| Topic | Type | Notes |
|---|---|---|
| `/camera/image/compressed` | `sensor_msgs/CompressedImage` | JPEG by default; one per frame; `header.stamp` from frame time |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | from config; published per frame or at a fixed rate (builder's call, document it) |
| `/camera/metadata` | custom type | exposure, gains, lux, colour temperature, CCM, black levels, focus FoM, AE state, `ScalerCrop`, boot-clock timestamp, source segment and frame index |
| `/radar/scan` | `radar_msgs/RadarScan` | one per decoded data frame; target fields per section 6 |
| `/radar/status` | custom type | reported target count, approach alarm; firmware and detection parameters are file metadata |
| `/events` | `diagnostic_msgs/DiagnosticArray` or `rcl_interfaces/Log` | from `events.jsonl`, plus converter findings (segment gaps, drops, repairs) |
| `/tf_static` | `tf2_msgs/TFMessage` | sensor placement, section 7 |

File-level MCAP metadata records: converter name and version; source session id, boot id,
state, stop reason; `time_quality` and the clock anchors used; recorder config; camera and
radar device info (firmware, detection parameters); image encoding settings; per-segment
statistics (frames, drops, max gap); and a `warnings` list for anything repaired or
unusual. Each channel carries its encoding settings in channel metadata.

Reproduce-in-time requirement: playback in `log_time` order reproduces capture timing.

## 6. Radar message mapping

Standard type is `radar_msgs/RadarScan` (header plus an array of `RadarReturn`). Searches
confirmed the structure but not the exact upstream `RadarReturn` field list; the builder
must read the upstream `.msg` for the ROS 2 distribution in use before finalising.

LD2451 targets (up to 5 per frame): angle (degrees, `value - 0x80`), distance (m), speed
(km/h), approaching flag, SNR. The mapping: range = distance; azimuth = angle in radians;
elevation unknown (NaN or 0, builder's call, documented); Doppler velocity = signed radial
speed in m/s (sign convention per ROS: positive means moving away, so approaching targets
are negative); SNR/amplitude fields filled if the type has them. An empty frame produces
a scan with no returns, not no message.

Open item: which direction a positive LD2451 angle means (left or right) is not stated in
the driver docs. Mark it unverified in the config and docs until checked against a
recording or the vendor PDF in `driver-hlk-ld2451/docs/vendor/`.

## 7. Sensor placement (tf)

From the OpenSCAD testplate (CAD: X right seen from the front, Y backwards, Z up; sensors
look toward -Y). Derived from the dimensions in the source, not measured on the hardware:

- Camera lens centre and radar antenna centre are both 42 mm above the plate top.
- The radar is about 34 mm to the right of the camera seen from the front, so in ROS
  terms (x forward, y left, z up) `radar` is at y = -0.034 m relative to `camera`.
- Radar antenna face and camera board face are both about 5 mm in front of the plate: the
  depth offset is about zero (the lens barrel sticks about 4.5 mm further; the optical
  centre is approximated at the board face).
- Both boresights point straight forward; the radar is mounted vertically so its angle
  output is horizontal.

Frames (builder to finalise): a base frame for the sensor plate, `camera_link`,
`camera_optical_frame` (REP-103 optical convention: z forward, x right, y down),
`radar_link`. Defaults go in the config file with pitch/roll of the whole rig set to zero
and overridable per run. If the CAD changes, the config is the single place to update.

## 8. Camera info

Uncalibrated v1.3 camera (OV5647), 1296x972 which is the full 2592x1944 field binned 2x2.
Defaults derived from vendor specs (focal length 3.6 mm, pixel 1.4 um, so 2.8 um binned):
fx = fy of about 1286 px, cx = 648, cy = 486, zero distortion, and `calibrated = false`
recorded in the file. These figures are from memory of the vendor datasheet and must be
verified by the builder. The config accepts a real calibration (K, D, R, P, distortion
model) which replaces the defaults, and the resolution recorded in each session is
checked against the config.

## 9. Behaviour and errors

Time: for each segment use the nearest `clock_samples` anchors to convert boot-clock to
wall-clock; cross-check against `FrameWallClock`. Report disagreement beyond a threshold
as a warning. `time_quality` is derived by the converter (the recorder's
`wall_clock_source` is used if present, otherwise `ntp_synced`, otherwise `unsynced`).

Video: decode sequentially and pair frames with `frames.csv` rows (rebuilt from
`meta.json` if missing or stale, tolerating a truncated tail). If the decoded frame count
and the row count disagree, stop with a clear error by default; an explicit flag may
allow converting the common prefix and records the mismatch as a warning. Never realign
silently.

Radar: reassemble frames across reads with the `ld2451` framing code; count and report
discarded bytes and invalid frames; ignore command ACKs for the scan topic (they may feed
file metadata). Tolerate a partial last line.

Sessions: accept `complete`, `interrupted` and `in_progress`, warning for the latter two.
Segments are discovered from files as well as `session.json`, because s000033 has a
segment that is not listed. Unknown future `format_version` is refused with a clear
message. Missing optional keys never crash the converter.

Output: write to a temporary file and rename on success; never leave a truncated MCAP
under the final name. Refuse to overwrite unless asked. Exit codes distinguish success,
success with warnings, and failure.

## 10. Relationship to the platform

The platform (FiftyOne, Qwen via Ollama or similar, storage, compose) is a separate
project with its own architecture review. This project's job there is only to honour the
MCAP contract above and to ship a `Dockerfile` so that a compose file can run
`radar-cam-2-mcap` as a one-shot job over a mounted folder. Nothing in this repo depends
on FiftyOne or any model.

## 11. Constraints

- Converter runs on a laptop/workstation/container, not on the Pi. Python version to be
  chosen by the builder (the workspace uses 3.9 to 3.13 for the Pi side).
- `uv` for dependencies; ruff, mypy (strict) and pytest as in `radar-cam-recorder`.
- Depends on the `ld2451` driver (git dependency in the sibling project) for radar
  decoding, and on a video decoder (PyAV is the candidate; the test used it) and the
  `mcap` Python library.
- Source folders are read-only; the converter never modifies them.

## 12. Left to the builder

Package and CLI layout; exact CLI flags beyond those named here; config file schema and
format (TOML preferred, to match the recorder); custom message definitions for
`/camera/metadata` and `/radar/status`; whether to publish `CameraInfo` per frame;
directory-of-sessions batch mode; test fixtures (use truncated slices of real sessions,
small enough to commit); CI; README structure (use the readme skill); the radar message
mapping details once the upstream definition is read.

## 13. Open items needing verification

- Upstream `radar_msgs/RadarReturn` field list.
- Sign convention of the LD2451 angle.
- Camera intrinsic defaults.
- JPEG size estimate for scenes other than the one tested.

Resolved by checking all 21 sessions (35 video segments; slice NAL units counted in the
H.264 stream, compared with `frames.csv`, `pts.txt` and `meta.json`): 32 segments agree
exactly on all four. The 3 that do not are the last segment of each damaged session:
s000030/0001 (stream 2245, frames.csv 2244, pts 1659, meta 2245), s000032/0001 (2286,
2271, 1659, 2272) and s000033/0003 (2319, no frames.csv, 2069, 2194). So `pts.txt` is
buffered and unreliable at a truncated tail, and counts can differ by a few frames there.
`meta.json` is the authority for per-frame timestamps; strict mode therefore only fails
on damaged sessions, which is what the explicit prefix option is for.

## 14. Review outcomes and calls made by the author (user can overturn)

A fresh reviewer read this document; its questions were sorted as follows.

Answered by the conversation or the data
- Unsynced sessions: the user chose to convert them with `time_quality: unsynced`; the
  wall clock is still used for `log_time`. Author's call on top: a `--time-base boot`
  option writes boot-clock time instead, for anyone who distrusts an unsynced wall clock.
- Frame-count agreement: resolved above.
- Radar decoding offline: the `ld2451` driver has `FrameDecoder` (framing) and
  `decode_report(payload, t_rx_ns)`; the recorder's own `reader.py` already replays stored
  sessions with them. Reuse the driver; pin it to a tagged commit in `pyproject.toml`.

Minor, decided by the author
- Time authority: convert boot-clock to wall-clock per segment, linearly interpolating
  between the `clock_samples` anchors that bracket it (start, each segment, stop); the
  last segment of a session without a stop anchor uses its start anchor. `FrameWallClock`
  is a cross-check only: disagreement above 50 ms (configurable) is a warning in the
  file's `warnings`, anchors win, exit status is "success with warnings".
- Segment repair: a segment is converted if it has `meta.json` plus either video or radar.
  `frames.csv` and `pts.txt` are never trusted over `meta.json`. If the stream has more
  frames than `meta.json`, extra trailing frames are dropped; if fewer, the frames beyond
  the stream are dropped. Both cases record a warning. A segment with no usable data is
  skipped with a warning.
- Custom message types: ROS package namespace `radar_cam_msgs`, with the `.msg` files
  shipped in the repository and embedded as schemas in the output.
- Radar timestamp: use `t_rx_ns` as is; document that it carries up to one serial read of
  latency and no correction is applied.
- Scale: a single process, streaming frame by frame with bounded memory; batch mode runs
  sessions one after another unless `--jobs N` is given.

Answered by the user
- The ingestion pipeline has no requirements yet. The topic names, frame IDs, message
  types and file-level metadata keys in section 5 are therefore a proposal to be reviewed
  together with the platform project. To make that safe, every file records a
  `contract_version` (starting at `0.1`, marked draft) in its file-level metadata, and the
  topic table is the single place to change when the review concludes.
