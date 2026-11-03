# radar-cam-2-mcap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A `uv`-managed Python CLI that converts a `radar-cam-recorder` session folder (`sNNNNNN`) into one ROS 2 (CDR) MCAP file with per-frame images, decoded radar scans, tf, camera info, events and honest time metadata.

**Architecture:** A small pipeline of focused modules: tolerant session reader -> time module (boot clock to wall clock) -> per-stream generators (video frames, radar reports) -> message mappers -> one MCAP sink. The orchestrator merges the streams by `log_time` and writes to a temp file that is renamed on success. Config (TOML) carries every fixed parameter.

**Tech Stack:** Python >= 3.10, `uv`, `mcap` + `mcap-ros2-support`, PyAV (`av`), Pillow, NumPy, the `ld2451` driver (git dependency, pinned to a tag or commit), pytest, ruff, mypy `--strict`.

**Spec:** `docs/superpowers/specs/2026-11-01-radar-cam-2-mcap-design.md` (section numbers below refer to it).

## Global Constraints

- One MCAP per session, segments joined (spec 4.1); `--per-segment` writes one file per segment.
- Messages are ROS 2 CDR with `ros2msg` schemas (spec 5). Default topics: `/camera/image/compressed`, `/camera/camera_info`, `/camera/metadata`, `/radar/scan`, `/radar/status`, `/events`, `/tf_static`.
- Image default: JPEG, quality 90, 4:2:0 chroma, configurable (`jpeg` or `png`, quality) (spec 4.3).
- MCAP chunk compression configurable: `zstd` (default), `lz4`, `none`, plus chunk size (spec 4.4).
- `log_time` and `header.stamp` are wall-clock ns derived from clock anchors; the boot-clock value is kept on camera and radar messages; every file records `time_quality` in {`rtc`,`ntp`,`unsynced`}, `contract_version` = `0.1` (draft), converter version, and a `warnings` list (spec 4.5, 5, 14).
- Unsynced sessions convert with a warning; `--time-base boot` writes boot-clock time instead (spec 14).
- Source session folders are never modified; output is written to a temp file and renamed only on success; existing output is refused unless `--overwrite`.
- `meta.json` is the authority for per-frame timestamps; `frames.csv` and `pts.txt` are never trusted over it (spec 13, 14).
- `FrameWallClock` cross-check tolerance 50 ms, configurable; anchors win (spec 14).
- Exit codes: 0 success, 3 success with warnings, 1 failure (2 is argparse usage). This is an author's call, written in the README.
- Tooling matches `../radar-cam-recorder`: ruff, mypy strict, pytest, GitHub Actions CI, MIT licence.
- Radar angle sign is unverified: config key `radar.angle_sign` (default `1`, documented as unverified).
- Camera intrinsic defaults (1296x972, fx = fy = 1285.7, cx 648, cy 486, zero distortion, `calibrated = false`) come from vendor specs from memory and must be checked in Task 6.

## Review Focus

1. A radar data frame split across two serial reads, with a stray byte spilling into the next read, still decodes to one scan with the right `t_rx_ns` (Task 5).
2. A session shaped like s000033 (state `in_progress`, a video segment not listed in `session.json`, no `frames.csv`) converts with warnings and does not crash (Tasks 2, 4, 7).
3. A radar frame with zero targets produces a `RadarScan` with an empty `returns` list, not a missing message (Task 5).
4. A conversion that fails halfway leaves no file at the final output path, and an existing output is not overwritten without `--overwrite` (Task 9).
5. A session with no `stop` anchor, or messages whose boot time falls before the first anchor, still gets a finite wall-clock time and a warning (Task 3).

---

## File Structure

```
pyproject.toml, uv.lock, LICENSE, README.md, CHANGELOG.md, CONTRIBUTING.md, Dockerfile, .github/workflows/ci.yml
config/default.toml                      # documented defaults, copy-and-edit
src/radar_cam_2_mcap/__init__.py         # __version__, CONTRACT_VERSION
src/radar_cam_2_mcap/config.py           # TOML -> frozen dataclasses, defaults
src/radar_cam_2_mcap/session.py          # tolerant session reader
src/radar_cam_2_mcap/timebase.py         # anchors, Timeline, time_quality, cross-check
src/radar_cam_2_mcap/video.py            # meta.json parser, frame counting, decode, encode
src/radar_cam_2_mcap/radar.py            # radar CSV -> RadarReport via ld2451
src/radar_cam_2_mcap/mapping.py          # pure functions: domain objects -> message dicts
src/radar_cam_2_mcap/msgdefs/            # vendored .msg texts (radar_msgs, radar_cam_msgs, others)
src/radar_cam_2_mcap/sink.py             # McapSink around mcap-ros2-support
src/radar_cam_2_mcap/convert.py          # orchestration, stream merge, atomic output
src/radar_cam_2_mcap/cli.py              # argparse CLI, exit codes
tests/ (one file per module), tests/fixtures/ (small real-session slices), scripts/make_fixture.py, scripts/verify_output.py
```

---

### Task 1: Project scaffold, config loader and CLI skeleton

**Files:**
- Create: `pyproject.toml`, `LICENSE`, `.gitignore`, `.python-version`, `config/default.toml`, `src/radar_cam_2_mcap/{__init__,config,cli}.py`, `tests/test_config.py`, `tests/test_cli.py`, `.github/workflows/ci.yml`

**Interfaces:**
- Produces (`config.py`): frozen dataclasses `ImageConfig(format: Literal["jpeg","png"]="jpeg", quality: int=90, subsampling: int=2)`, `McapConfig(compression: Literal["zstd","lz4","none"]="zstd", chunk_size: int=4*1024*1024)`, `CameraIntrinsics(width:int, height:int, k:tuple[float,...], d:tuple[float,...], r:tuple[float,...], p:tuple[float,...], distortion_model:str="plumb_bob", calibrated:bool=False)`, `FrameTransform(parent:str, child:str, xyz:tuple[float,float,float], rpy:tuple[float,float,float])`, `TopicsConfig` (the seven topic names, plus `frame_camera`, `frame_radar`), `RadarConfig(angle_sign:int=1)`, `TimeConfig(crosscheck_tolerance_ms: float=50.0, base: Literal["wall","boot"]="wall")`, and `Config` aggregating all of them plus `transforms: tuple[FrameTransform, ...]`. `load_config(path: Path | None) -> Config` (None gives defaults; a file overrides only the keys it names; unknown keys raise `ConfigError`).
- Produces (`cli.py`): `main(argv: Sequence[str] | None = None) -> int` with `--version`; the `convert` subcommand is added in Task 9.
- Default transforms (derived from the OpenSCAD testplate, spec 7), ROS convention x forward, y left, z up: `base_link -> camera_link` xyz (0.005, 0.0195, 0.042) rpy (0,0,0); `base_link -> radar_link` xyz (0.0051, -0.0145, 0.042) rpy (0,0,0); `camera_link -> camera_optical_frame` xyz (0,0,0) rpy (-pi/2, 0, -pi/2). `base_link` is the sensor plate: origin at the plate top, centred, on the front face.

- [ ] **Step 1: Failing tests.** `test_config.py`: `test_defaults_match_spec` asserts jpeg/90/subsampling 2, zstd, width 1296, height 972, `calibrated is False`, and the three default transforms above; `test_file_overrides_only_named_keys` writes a TOML with `[image] quality = 95` and asserts quality 95 with format still `jpeg`; `test_unknown_key_rejected` expects `ConfigError`. `test_cli.py`: `test_version` runs `main(["--version"])` and expects a SystemExit with 0 and the version on stdout.
- [ ] **Step 2: Run** `uv run pytest -q` and see them fail (no package).
- [ ] **Step 3: Scaffold and implement.** `uv init --package` layout (src layout, `radar_cam_2_mcap`), `requires-python = ">=3.10"`, console script `radar-cam-2-mcap = radar_cam_2_mcap.cli:main`, dependency groups mirroring `../radar-cam-recorder/pyproject.toml` (test, dev). Add runtime dependencies `mcap`, `mcap-ros2-support`, `av`, `pillow`, `numpy`, and `ld2451` as a git source pinned to a commit (`[tool.uv.sources]`). Implement `config.py` with `tomllib`. Write `config/default.toml` as the documented defaults. `ci.yml` runs ruff, mypy strict and pytest on Linux and macOS. Run `git init` first: the folder is not a repository yet.
- [ ] **Step 4: Run** `uv run pytest -q && uv run ruff check . && uv run mypy --strict src` and expect all green.
- [ ] **Step 5: Commit** `feat: project scaffold, config loader, cli skeleton`.

---

### Task 2: Tolerant session reader

**Files:**
- Create: `src/radar_cam_2_mcap/session.py`, `tests/test_session.py`, `scripts/make_fixture.py`, `tests/fixtures/` (see below)
- Test: `tests/test_session.py`

**Interfaces:**
- Produces: `class SessionError(Exception)`; `@dataclass(frozen=True) class ClockAnchor(label: str, boottime_ns: int, realtime_ns: int)`; `@dataclass(frozen=True) class SegmentFiles(index: int, h264: Path | None, meta: Path | None, radar: Path | None)`; `@dataclass class SessionInfo(path: Path, session_id: str, state: str, stop_reason: str | None, boot_id: str | None, ntp_synced: bool, wall_clock_source: str | None, anchors: list[ClockAnchor], recorder_config: dict, devices: dict, segments: list[SegmentFiles], listed_segment_indexes: list[int], warnings: list[str])`; `load_session(path: Path) -> SessionInfo`; `load_events(path: Path) -> list[dict]`.
- `scripts/make_fixture.py <session_dir> <out_dir> --frames N` builds a fixture by copying `session.json`, `events.jsonl`, the first N frames' worth of `meta.json` entries, the matching leading bytes of the `.h264` (it starts with SPS/PPS and an IDR), and the radar CSV lines inside that time window.

- [ ] **Step 1: Make fixtures** from real sessions pulled from `workstation:~/workspace/hlk-ld2451-speed-trap/data` (copy first, never edit there): `tests/fixtures/complete_2seg` (from s000066, first 2 segments, 40 frames each), `tests/fixtures/in_progress_unlisted` (from s000033: state `in_progress`, segment 3 present on disk but not in `session.json`, no `frames.csv`), `tests/fixtures/interrupted` (from s000030). Keep each fixture under about 1.5 MB.
- [ ] **Step 2: Failing tests.** `test_complete_session_lists_segments` (two `SegmentFiles` with h264, meta and radar present; no warnings); `test_unlisted_segment_is_found_with_warning` (segment 3 present, a warning mentions it, `state == "in_progress"`); `test_missing_optional_keys_do_not_crash` (a `session.json` without `wall_clock_source`, `rtc`, `devices` loads); `test_unknown_format_version_refused` (`format_version: 2` raises `SessionError` naming the version); `test_events_tolerate_partial_last_line`.
- [ ] **Step 3: Run** `uv run pytest tests/test_session.py -v`, expect FAIL.
- [ ] **Step 4: Implement** the signatures above. Segments come from `video/NNNN.*` and `radar/NNNN.*` file names as well as `session.json["segments"]`; a segment is kept if it has `meta.json` plus h264 or a radar CSV (spec 14); otherwise it is dropped with a warning. `format_version` other than 1 raises.
- [ ] **Step 5: Run** the tests, expect PASS. **Commit** `feat: tolerant session reader with real-session fixtures`.

---

### Task 3: Time module

**Files:**
- Create: `src/radar_cam_2_mcap/timebase.py`, `tests/test_timebase.py`

**Interfaces:**
- Consumes: `ClockAnchor`, `SessionInfo` (Task 2), `TimeConfig` (Task 1).
- Produces: `class Timeline`: `__init__(self, anchors: Sequence[ClockAnchor], base: Literal["wall","boot"]="wall")`, `to_log_ns(self, boot_ns: int) -> int`, `warnings: list[str]`. `time_quality(session: SessionInfo) -> Literal["rtc","ntp","unsynced"]`. `crosscheck(timeline: Timeline, samples: Sequence[tuple[int, int]], tolerance_ms: float) -> list[str]` where each sample is `(boottime_ns, frame_wall_clock_us)`.

- [ ] **Step 1: Failing tests.** `test_interpolates_between_anchors` (anchors at boot 0 and 300 s with a 1 ms wall drift gives the linear value at 150 s); `test_before_first_anchor_uses_first_offset_and_warns`; `test_after_last_anchor_uses_last_offset` (no stop anchor); `test_boot_base_returns_boot_ns`; `test_time_quality_prefers_recorder_value_then_ntp_then_unsynced` (`wall_clock_source="rtc"` gives `rtc`, absent plus `ntp_synced=True` gives `ntp`, otherwise `unsynced`); `test_crosscheck_flags_disagreement_above_tolerance` (60 ms off gives one warning, 40 ms off gives none, with tolerance 50).
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement.** Piecewise-linear offset `realtime - boottime` between consecutive anchors sorted by boot time; outside the anchors use the nearest anchor's constant offset. A single anchor is valid. Zero anchors raises `SessionError`.
- [ ] **Step 4: Run, expect PASS.** **Commit** `feat: boot-to-wall clock timeline and time quality`.

---

### Task 4: Video frames

**Files:**
- Create: `src/radar_cam_2_mcap/video.py`, `tests/test_video.py`

**Interfaces:**
- Consumes: `SegmentFiles` (Task 2), `ImageConfig` (Task 1).
- Produces: `@dataclass(frozen=True) class FrameMeta(index: int, sensor_ts_ns: int, frame_wall_clock_us: int | None, fields: dict[str, Any])`; `read_frame_meta(meta: Path) -> list[FrameMeta]` (tolerates a truncated tail and a missing closing bracket); `count_h264_frames(path: Path) -> int` (counts slice NAL units, types 1 and 5, streaming in large chunks); `class FrameCountMismatch(Exception)`; `@dataclass(frozen=True) class VideoFrame(meta: FrameMeta, data: bytes, fmt: Literal["jpeg","png"], width: int, height: int)`; `iter_video_frames(seg: SegmentFiles, image: ImageConfig, *, allow_prefix: bool, warnings: list[str]) -> Iterator[VideoFrame]`.
- Rules (spec 9, 14): `meta.json` is the authority for timestamps. If the stream has more frames than metadata, extra trailing frames are dropped; if fewer, metadata beyond the stream is dropped; both append a warning. Without `allow_prefix` any difference raises `FrameCountMismatch` before decoding starts. A segment with no h264 yields nothing and warns.

- [ ] **Step 1: Failing tests** using the fixtures: `test_read_frame_meta_counts_and_first_timestamp`; `test_read_frame_meta_truncated_tail` (cut the file mid-object, earlier frames survive); `test_count_h264_frames_matches_meta_on_complete_fixture`; `test_decoded_frames_pair_with_meta_in_order` (indexes 0..N-1, `sensor_ts_ns` strictly increasing, JPEG bytes start with `ff d8`, width 1296 and height 972); `test_mismatch_raises_without_allow_prefix` (use a fixture whose h264 is cut so counts differ); `test_allow_prefix_converts_common_prefix_with_warning`; `test_png_format_roundtrip` (PNG signature, `fmt == "png"`); `test_jpeg_uses_420_subsampling` (open with Pillow, check the sampling factors of the JPEG).
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement.** Decode with PyAV (`av.open(path, format="h264")`, demux and decode). Convert to RGB and encode with Pillow (`subsampling=2` for 4:2:0, `quality` from config).
- [ ] **Step 4: Run, expect PASS.** **Commit** `feat: video frame decoding with meta.json as timestamp authority`.

---

### Task 5: Radar decoding

**Files:**
- Create: `src/radar_cam_2_mcap/radar.py`, `tests/test_radar.py`

**Interfaces:**
- Consumes: the `ld2451` driver (`FrameDecoder`, `decode_report`, `RadarReport`, `Target`) and the recorder's CSV format (`t_rx_ns,hex`; header line; partial last line). Follow the replay pattern in `../radar-cam-recorder/radar_cam_recorder/reader.py` and read `../driver-hlk-ld2451/ld2451/framing.py` for the decoder API before writing code.
- Produces: `@dataclass class RadarStats(reads: int, data_frames: int, ack_frames: int, bytes_discarded: int, invalid_frames: int)`; `iter_radar_reports(path: Path, stats: RadarStats) -> Iterator[RadarReport]`.

- [ ] **Step 1: Failing tests.** `test_fragmented_data_frame_decodes_once_with_first_byte_time` (build a CSV where a `f4f3f2f1...f8f7f6` frame is split across three reads and a stray `f5` leads the next read; one report, `t_rx_ns` equals the first read's time); `test_command_ack_frames_are_counted_not_reported` (the `fdfcfbfa` ACKs at the start of a real fixture are in `ack_frames`, not yielded); `test_empty_frame_yields_report_with_no_targets`; `test_partial_last_line_ignored`; `test_real_fixture_decodes_targets` (the s000060-derived fixture yields at least one report with a target whose angle, distance and speed are in the driver's ranges).
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement** by feeding each read's bytes to the driver's frame decoder with its `t_rx_ns`, and `decode_report` for data frames.
- [ ] **Step 4: Run, expect PASS.** **Commit** `feat: radar CSV decoding through the ld2451 driver`.

---

### Task 6: Message definitions, mapping, camera info and tf

**Files:**
- Create: `src/radar_cam_2_mcap/mapping.py`, `src/radar_cam_2_mcap/msgdefs/*.msg`, `tests/test_mapping.py`

**Interfaces:**
- Consumes: `VideoFrame` (Task 4), `RadarReport`/`Target` (Task 5), `CameraIntrinsics`, `FrameTransform`, `RadarConfig`, `TopicsConfig` (Task 1).
- Produces: `STAMP(ns: int) -> dict` (`{"sec", "nanosec"}`); `compressed_image(frame: VideoFrame, stamp_ns: int, frame_id: str) -> dict`; `camera_info(cfg: CameraIntrinsics, stamp_ns: int, frame_id: str) -> dict`; `camera_metadata(frame: VideoFrame, stamp_ns: int, boot_ns: int, segment: int) -> dict` (type `radar_cam_msgs/CameraMetadata`); `radar_scan(report: RadarReport, stamp_ns: int, frame_id: str, cfg: RadarConfig) -> dict`; `radar_status(report: RadarReport, stamp_ns: int, boot_ns: int) -> dict` (type `radar_cam_msgs/RadarStatus`); `tf_static(transforms: Sequence[FrameTransform], stamp_ns: int) -> dict`; `event_message(event: dict, stamp_ns: int) -> dict`; `MSGDEFS: dict[str, str]` mapping each datatype name to its full `ros2msg` text including dependencies.
- Radar mapping (spec 6): range = distance in m; azimuth = `angle_sign * radians(angle_deg)`; elevation = 0.0 (documented as unknown); Doppler velocity in m/s = speed / 3.6, negative for approaching targets, positive for receding; SNR goes into the amplitude or SNR field the upstream type provides.
- **Verification steps in this task (spec 13):** (a) fetch the upstream `radar_msgs` `RadarScan.msg` and `RadarReturn.msg` for ROS 2 Jazzy and vendor them exactly into `msgdefs/`, then fix the mapping field names to match; (b) confirm the camera defaults (focal length 3.6 mm, 1.4 um pixel, so 2.8 um binned) from the Raspberry Pi camera documentation, and correct `config/default.toml` if they differ; (c) find the LD2451 angle sign in `../driver-hlk-ld2451/docs/vendor/HLK-LD2451_Serial_Protocol_V1.03_EN.pdf`, and if it is not stated keep `angle_sign = 1` and leave the "unverified" note in the README.

- [ ] **Step 1: Failing tests.** `test_radar_scan_maps_target_units_and_signs` (angle 10, distance 25, speed 36 km/h approaching gives range 25.0, azimuth radians(10), Doppler -10.0); `test_receding_target_has_positive_doppler`; `test_empty_report_gives_empty_returns`; `test_angle_sign_flips_azimuth`; `test_camera_info_defaults` (`k == (fx,0,cx,0,fy,cy,0,0,1)` with fx 1285.7, cx 648, cy 486, `d` all zero, width 1296, height 972); `test_tf_static_contains_three_default_transforms_with_expected_translation`; `test_compressed_image_format_field` (`format == "jpeg"`, data length matches); `test_msgdefs_parse` (every datatype in `MSGDEFS` is accepted by `mcap_ros2`'s schema parser).
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement** the mapper functions as pure functions returning dicts; write `CameraMetadata.msg` (exposure_us, analogue_gain, digital_gain, lux, colour_temperature, colour_gains[2], colour_correction_matrix[9], sensor_black_levels[4], focus_fom, ae_state, scaler_crop string, frame_wall_clock_us, boot_ns, segment, frame_index) and `RadarStatus.msg` (header, reported_count, approach_alarm, boot_ns) under the `radar_cam_msgs` namespace.
- [ ] **Step 4: Run, expect PASS.** **Commit** `feat: message mapping, vendored schemas, camera info and tf`.

---

### Task 7: MCAP sink and single-session conversion

**Files:**
- Create: `src/radar_cam_2_mcap/sink.py`, `src/radar_cam_2_mcap/convert.py`, `tests/test_sink.py`, `tests/test_convert.py`

**Interfaces:**
- Consumes: Tasks 1 to 6.
- Produces: `class McapSink`: `__init__(self, path: Path, cfg: McapConfig)`, `register(self, topic: str, datatype: str) -> None` (uses `MSGDEFS`), `write(self, topic: str, message: dict, log_ns: int, publish_ns: int | None = None) -> None`, `add_metadata(self, name: str, values: Mapping[str, str]) -> None`, `close(self) -> None`, and context-manager support. `@dataclass class ConvertOptions(overwrite: bool=False, per_segment: bool=False, allow_prefix: bool=False, time_base: Literal["wall","boot"] | None=None)`; `@dataclass class ConvertResult(outputs: list[Path], warnings: list[str])`; `convert_session(session_dir: Path, out_dir: Path, cfg: Config, opts: ConvertOptions) -> ConvertResult`.
- Behaviour: build a `Timeline` from the anchors; create generators for video frames, radar reports and events per segment; merge all messages by `log_ns` with `heapq.merge` so memory stays bounded; write `/tf_static` and `/camera/camera_info` once at the first message time (CameraInfo also repeated with each frame is NOT done; document it); file-level metadata records `contract_version`, converter version, session id, boot id, state, stop reason, `time_quality`, anchors, image settings, device info, recorder config, per-segment statistics and `warnings`. Output name `<session_id>.mcap`.

- [ ] **Step 1: Failing tests.** `test_sink_roundtrip` (write one message per type, read back with `mcap_ros2.reader`, check values); `test_sink_compression_option` (`zstd`, `lz4`, `none` each produce a file whose chunks use that compression); `test_convert_complete_fixture` (topics present with expected counts, `log_time` non-decreasing across the file, metadata has `time_quality` and `contract_version == "0.1"`); `test_unsynced_fixture_is_flagged` (`time_quality == "unsynced"` and a warning); `test_in_progress_unlisted_segment_converts_with_warnings`; `test_per_segment_option_writes_one_file_per_segment`; `test_time_base_boot_uses_boot_clock`; `test_source_folder_unchanged` (hash the fixture tree before and after).
- [ ] **Step 2: Run, expect FAIL.**
- [ ] **Step 3: Implement** `sink.py` on `mcap_ros2.writer.Writer` and `convert.py` as described.
- [ ] **Step 4: Run, expect PASS.** **Commit** `feat: mcap sink and session conversion`.

---

### Task 8: Events, warnings and cross-check wiring

**Files:**
- Modify: `src/radar_cam_2_mcap/convert.py`
- Test: `tests/test_convert.py`

**Interfaces:**
- Consumes: `load_events` (Task 2), `event_message` (Task 6), `crosscheck` (Task 3).
- Produces: `/events` messages for every `events.jsonl` entry (incident or state change) plus converter findings (segment gaps with their duration, frame drops per segment, repaired or dropped frames, radar discarded bytes, time cross-check failures); the same findings appended to the file-level `warnings` metadata.

- [ ] **Step 1: Failing tests.** `test_events_topic_has_recorder_events_and_converter_findings` (an `in_progress_unlisted` conversion yields events for `camera exited` and `segment gap`, and a converter finding about the unlisted segment); `test_crosscheck_warning_reaches_events_and_metadata` (shift the fixture's `FrameWallClock` by 80 ms); `test_segment_gap_event_carries_gap_ns`.
- [ ] **Step 2: Run, expect FAIL.** **Step 3: Implement.** **Step 4: Run, expect PASS.** **Commit** `feat: events topic and warning reporting`.

---

### Task 9: CLI, batch mode and atomic output

**Files:**
- Modify: `src/radar_cam_2_mcap/cli.py`, `src/radar_cam_2_mcap/convert.py`
- Test: `tests/test_cli.py`

**Interfaces:**
- Consumes: `convert_session`, `ConvertOptions`, `load_config`.
- Produces: `radar-cam-2-mcap convert SESSION_OR_DIR... -o OUT_DIR [--config FILE] [--overwrite] [--per-segment] [--allow-prefix] [--time-base {wall,boot}] [--jobs N] [--quiet]`. A path that contains `session.json` is one session; otherwise its children that do are converted in order. Exit codes: 0 success, 3 success with warnings, 1 any failure; the summary on stderr lists each session, output path, warnings and failures. Output is written to `<name>.mcap.tmp` in the destination directory and `os.replace`d on success; the temp file is deleted on failure.

- [ ] **Step 1: Failing tests.** `test_convert_exit_0_for_clean_session`; `test_exit_3_when_warnings`; `test_exit_1_and_no_output_file_on_failure` (force a `FrameCountMismatch`; assert neither the final path nor a `.tmp` file exists); `test_existing_output_refused_without_overwrite` (exit 1, file unchanged) and `test_overwrite_replaces`; `test_directory_of_sessions_batch`; `test_one_failure_does_not_stop_the_batch` (other sessions are converted, exit 1).
- [ ] **Step 2: Run, expect FAIL.** **Step 3: Implement** (`--jobs` via `concurrent.futures.ProcessPoolExecutor`, default 1). **Step 4: Run, expect PASS.** **Commit** `feat: cli with batch mode, exit codes and atomic output`.

---

### Task 10: Verification on the real sessions

**Files:**
- Create: `scripts/verify_output.py`, `docs/verification.md`

**Interfaces:**
- `verify_output.py OUTPUT.mcap` prints per-topic message counts, first and last `log_time`, checks `log_time` is non-decreasing, checks `header.stamp` equals `log_time` on stamped messages, and prints the file-level metadata. Exit 1 on any failed check.

- [ ] **Step 1: Copy** s000066, s000071, s000030, s000033 and one single-segment `ntp_synced` session (s000055) from the workstation into a scratch directory outside the repository.
- [ ] **Step 2: Convert** each with default settings, and s000066 again with `--time-base boot`, `lz4` and `none` compression; run `verify_output.py` on every result. Expect: s000066 and s000071 give one file each with all segments; s000030 and s000033 convert with warnings (s000033 needs `--allow-prefix` if its counts differ); s000055 reports `time_quality` `ntp`.
- [ ] **Step 3: Measure and record** in `docs/verification.md`: output size per session and per compression setting against the 14 GB estimate, conversion time per session, the gap durations found at segment boundaries, and any warnings. Compare the camera `SensorTimestamp`-derived time with `FrameWallClock` and note the largest difference observed.
- [ ] **Step 4: Open one converted file in Foxglove** (or play it back with a script that sleeps between messages) and confirm the image and radar timelines match the recording; record what was seen. If playback tooling is unavailable, say so in the document.
- [ ] **Step 5: Commit** `docs: verification on real sessions`.

---

### Task 11: Docker, README and release docs

**Files:**
- Create: `Dockerfile`, `.dockerignore`, `README.md`, `CHANGELOG.md`, `CONTRIBUTING.md`, `SECURITY.md`
- Test: manual `docker build` and run

**Interfaces:**
- `Dockerfile`: multi-stage, `uv`-based, non-root user, entrypoint `radar-cam-2-mcap`; usage `docker run --rm -v SESSIONS:/in:ro -v OUT:/out IMAGE convert /in -o /out`.

- [ ] **Step 1: Write the README with the `readme` skill.** Every statement is checked against the code: install, quick start, config reference (all keys from `config/default.toml`), output contract table (topics, types, `contract_version 0.1` as a draft), time model and `time_quality`, exit codes, tf diagram and its provenance (the OpenSCAD testplate), the angle-sign and intrinsics caveats, docker usage, and links to the recorder and driver projects. State clearly that the catalog and ingestion are out of scope.
- [ ] **Step 2: Build and run the image** against the `complete_2seg` fixture; check the output with `verify_output.py`.
- [ ] **Step 3: Update `CHANGELOG.md`** (`0.1.0`), run the full suite (`uv run pytest`, ruff, mypy strict), then commit `docs: readme, docker and release files`.
