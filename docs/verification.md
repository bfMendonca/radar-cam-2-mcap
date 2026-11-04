# Verification on real sessions

Converter 0.1.0 (branch `feat/converter`, HEAD 61367cc) run on real recordings from the
workstation (copied read-only with `rsync -a` into a scratch directory; nothing written on the
workstation). Tooling: `scripts/verify_output.py` (committed); the ad-hoc analysis scripts used
for checks a to e lived in the scratch directory and are described under "Commands".

## Not verified

* **Foxglove playback** (or any real-time playback): not possible in this environment. Only a
  script-level playback check (spacing of image messages in log-time order, check e) was done.
* Wall-clock **conversion times** for s000066, s000071 and the s000060 compression variants are
  not trustworthy (see "Timing"); no clean timing comparison between compression settings exists.
* `--per-segment`, `--jobs`, `--config` with custom transforms, PNG output and `[time]
  crosscheck_tolerance_ms` failures were not exercised on real data.
* s000070 (in the controller's survey) was not converted.
* The LD2451 angle sign is only supported by one vehicle pass (check d), not proven.

## Sessions

Default settings (`uv run radar-cam-2-mcap convert SESSION -o OUT --overwrite`), zstd.
"x" = output size / total h264 bytes.

| Session | Segments | State / ntp | Exit | h264 | MCAP | x | Images | Scans | time_quality | crosscheck max (ms) / samples | verify_output |
|---|---|---|---|---|---|---|---|---|---|---|---|
| s000055 | 1 | complete / ntp true | 0 | 10.2 MB | 53.5 MB | 5.2 | 511 | 19 | ntp | 0.033 / 22 | OK |
| s000060 | 1 | complete / ntp false | 3 | 89.3 MB | 367.0 MB | 4.1 | 4514 | 1211 | unsynced | 0.043 / 182 | OK |
| s000066 | 5 | complete / ntp true | 3 | 563.5 MB | 2.771 GB | 4.9 | 34385 | 7157 | ntp | 0.070 / 1384 | OK |
| s000071 | 6 | complete / ntp false | 3 | 760.6 MB | 3.905 GB | 5.1 | 38044 | 7763 | unsynced | 0.090 / 1532 | OK |
| s000030 | 2 | interrupted | 1 without, 3 with `--allow-prefix` | 165.4 MB | 865.3 MB | 5.2 | 8883 | 371 | unsynced | 0.029 / 359 | OK |
| s000033 | 4 (4th unlisted) | in_progress | 1 without, 3 with `--allow-prefix` | 453.1 MB | 3.059 GB | 6.8 | 22527 | 749 | unsynced | 0.069 / 909 | OK |

Every output: `log_time` non-decreasing in file order and file order equals
`iter_messages(log_time_order=True)` order; `header.stamp == log_time` on all image, scan, status
and camera-metadata messages (0 differences, 0 ns). Image count equals the sum of `meta.json`
objects in every case (for s000030/s000033 the difference with `--allow-prefix` is 0 because the
converter's prefix equals the meta.json count: the extra h264 frames, 1 and 125, are the ones
dropped).

Output is about 4 to 7 times the h264 size. All six sessions together (about 2.0 GB of h264) gave
about 11.0 GB of MCAP; the 14 GB estimate is therefore plausible for the whole survey but was not
measured over all sessions.

### Damaged sessions without `--allow-prefix`

Both fail with exit 1, no output file left behind (the `.tmp` is removed):

* s000030: `FrameCountMismatch: segment 0001: h264 has 2245 frames but meta.json has 2244; use
  --allow-prefix to convert the common prefix (--allow-prefix converts the common prefix of the
  frames)` after 123 s (segment 0 fully converted first).
* s000033: `FrameCountMismatch: segment 0003: h264 has 2319 frames but meta.json has 2194; ...`
  after 378 s (three segments fully converted first).

Observation: the count check happens when the segment is reached, so the failure comes after
minutes of wasted work. A pre-pass counting frames of all segments before streaming would fail
fast. The doubled "(--allow-prefix converts the common prefix of the frames)" text also repeats
the hint.

### Warnings and notes per session

| Session | Warnings (WARN) | Notes (INFO) |
|---|---|---|
| s000055 | none | none |
| s000060 | unsynced; `radar decoding problems: bytes_discarded=19` | recovered 236 short radar frames |
| s000066 | `radar decoding problems: framing_errors=3, bytes_discarded=130` | recovered 861 short radar frames; gaps 0->1 3.711 s, 1->2 3.029 s, 2->3 3.853 s, 3->4 2.863 s |
| s000071 | unsynced; `framing_errors=2, bad_payloads=1, bytes_discarded=74` | recovered 1305 short frames; gaps 3.673, 3.041, 3.444, 3.428, 3.261 s |
| s000030 | interrupted; segment 1 not listed; boot time before first anchor; boot time after last anchor; h264 longer by 1 frame (seg 1); unsynced; `bytes_discarded=19` | gap 0->1 4.452 s |
| s000033 | in progress; segment 3 not listed; boot time after last anchor; h264 longer by 125 frames (seg 3); unsynced; `partial_lines=1, bytes_discarded=19` | gaps 3.578, 3.349, 3.232 s |

All segment gaps (2.9 to 4.5 s) are INFO notes; no gap was a WARN (threshold 10 s). s000055 has
`time_quality` ntp; s000066 ntp (matches metadata `ntp_synced` true); s000071 unsynced (matches
false). The unsynced warning appears on every session with `ntp_synced` false (s000060, 071,
030, 033).

### Radar stats (metadata record `radar_stats`)

| Session | reads | data_frames | ack | bytes_discarded | framing_errors | bad_payloads | partial_lines | garbled | recovered | trailing |
|---|---|---|---|---|---|---|---|---|---|---|
| s000055 | 48 | 19 | 6 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| s000060 | 2495 | 1211 | 6 | 19 | 0 | 0 | 0 | 0 | 236 | 0 |
| s000066 | 14535 | 7157 | 6 | 130 | 3 | 0 | 0 | 0 | 861 | 0 |
| s000071 | 15718 | 7763 | 6 | 74 | 2 | 1 | 0 | 0 | 1305 | 0 |
| s000030 | n/a | 371 | n/a | 19 | 0 | 0 | 0 | 0 | 0 | 0 |
| s000033 | n/a | 749 | n/a | 19 | 0 | 0 | 1 | 0 | 0 | 0 |

(n/a: not copied into this report; present in the output metadata.)

## Compression comparison (s000060, default zstd vs lz4 vs none, plus boot time base)

| Variant | Size (bytes) | vs zstd | verify_output | Image JPEG bytes (SHA-256 over all 4514 messages, first 16 hex) |
|---|---|---|---|---|
| zstd (default) | 366,978,807 | 1.000 | OK | b14ca45df28428f1 |
| lz4 (`[mcap] compression = "lz4"`) | 376,158,509 | 1.025 | OK | b14ca45df28428f1 |
| none | 377,501,788 | 1.029 | OK | b14ca45df28428f1 |
| `--time-base boot` (zstd) | 366,964,190 | 1.000 | OK | b14ca45df28428f1 |

JPEG payloads are identical across all variants. JPEG data is already compressed, so the
compression setting changes the size by only about 3 percent. With `--time-base boot` the first
and last image `log_time` are the boot-clock sensor timestamps (66486602000 and 263141706000 ns),
`time_base = boot` in the metadata, and the stored cross-check is unchanged.

### Timing

First runs (early in the session): s000055 10.0 s, s000060 82.2 s, s000030 164 s,
s000033 840 s. Later runs were much slower with no change in code or data: s000066 7498 s,
s000071 13892 s, and the s000060 variants 1720 to 1951 s each (including one re-run of the default
settings, 1733 s, versus 82 s earlier), while s000055 stayed at 10 to 14 s. The host showed
about 3.8 GB of 5 GB swap in use at the end; the cause was not investigated. The later times are
not meaningful as converter throughput and no compression-versus-time conclusion is drawn.

## Data checks

### a. Pixel-to-frame alignment (s000066, segments 0, 2, 4; s000071 segment 3)

The original `.h264` is decoded with PyAV in order and compared (PSNR on RGB) with the JPEG
stored in the message whose camera-metadata (`segment`, `frame_index`) matches; neighbours
k-1 and k+1 are compared against the same decoded frame.

| Segment | Frame | PSNR own JPEG | vs k-1 | vs k+1 |
|---|---|---|---|---|
| s000066 seg 0 | 0 | 49.68 dB | n/a | 25.80 dB |
| | 3390 | 49.03 | 38.08 | 37.61 |
| | 6779 | 48.60 | 37.03 | n/a |
| s000066 seg 2 | 0 | 49.36 | n/a | 32.91 |
| | 3480 | 50.19 | 35.87 | 32.88 |
| | 6960 | 50.57 | 36.19 | n/a |
| s000066 seg 4 | 0 | 50.23 | n/a | 35.08 |
| | 3377 | 47.84 | 24.64 | 24.84 |
| | 6753 | 47.78 | 36.98 | n/a |
| s000071 seg 3 | 0 | 48.37 | n/a | 35.42 |
| | 3417 | 47.84 | 28.95 | 28.50 |
| | 6833 | 47.83 | 27.47 | n/a |

Result: PASS. Every own-frame PSNR is above 47 dB and every neighbour is at least 9.8 dB worse
(a frame offset would be caught). The pairing of decoded frames with `meta.json` entries is
correct at start, middle and end of each checked segment. This closes the open Task 4 item.

### b. Counts and radar recovery

* Image count equals the sum of `meta.json` objects: s000066 34385 = 6780+6960+6961+6930+6754;
  s000071 38044 = 7066+6928+6828+6834+6869+3519.
* Radar scans equal the reports decoded independently with `radar_cam_2_mcap.radar` helpers over
  the session's CSVs with one shared decoder (s000060 1211, s000066 7157, s000071 7763: identical
  to the MCAP counts). Note this uses the same decoder code as the converter, so it checks
  stream plumbing (segment order, no loss in merging), not decoder correctness.
* Malformed frames, plain decoder (recovery off) vs the converter:

| Session | Controller survey | Plain decoder: framing errors | Recovered by converter | Left (framing_errors + bad_payloads) |
|---|---|---|---|---|
| s000060 | 236 of 1211 | 236 (975 reports) | 236 | 0 |
| s000066 | 864 of 7160 | 864 (6296 reports) | 861 | 3 |
| s000071 | 1306 of 7766 | 1307 (6458 reports) | 1305 | 2 + 1 bad payload |

So the converter recovers about 99.6 percent of the malformed frames; three (s000066) and three
(s000071) remain lost and are reported as WARN. Reports in the MCAP (7157 and 7763) are 3 below
the survey's 7160 and 3 below 7766.

### c. Timestamps

* First image `log_time` versus first frame `FrameWallClock`: s000066 1793353171516043406 ns vs
  1793353171516005000 (difference +0.038 ms); s000060 +0.043 ms; s000071 +0.033 ms.
* Largest absolute difference over all frames (own computation, `log_time` against
  `FrameWallClock * 1000`): s000060 0.044 ms (4514 frames); s000066 0.085 ms (34385 frames);
  s000071 0.262 ms (38044 frames, worst in segment 1). Stored cross-check values (sampled)
  0.043, 0.070 and 0.090 ms agree in magnitude. All are far below the 50 ms tolerance.
  Within a segment `log_time` gaps equal sensor-timestamp gaps exactly (0.000 ms maximum
  difference over all consecutive pairs).
* Segment boundaries (anchor interpolation continuity), last image of segment N to first of N+1,
  log_time gap versus sensor-timestamp gap (and `FrameWallClock` gap):

| Session | Boundary | log gap (s) | sensor gap (s) | difference |
|---|---|---|---|---|
| s000066 | 0->1 | 3.710794 | 3.710794 | 0.000 ms |
| | 1->2 | 3.029403 | 3.029403 | 0.000 |
| | 2->3 | 3.852747 | 3.852747 | 0.000 |
| | 3->4 | 2.862992 | 2.862992 | 0.000 |
| s000071 | 0->1 | 3.673186 | 3.673186 | 0.000 |
| | 1->2 | 3.040876 | 3.040876 | 0.000 |
| | 2->3 | 3.444352 | 3.444352 | 0.000 |
| | 3->4 | 3.427963 | 3.427963 | 0.000 |
| | 4->5 | 3.260949 | 3.260949 | 0.000 |

The wall-clock gaps from `FrameWallClock` agree to the microsecond. Continuity PASS. (The
interpolation anchors in these sessions are per-segment, so every boundary falls on an anchor
and the continuity is expected to be exact; a boundary with a clock step would show here.)

### d. Radar-to-camera alignment and angle sign (s000060, best effort)

The scene is a vehicle-mounted camera looking through a windscreen (dashboard in the lower third
of the image) on a highway in rain: the radar is moving, so stationary scenery also returns as
targets (many returns at 19 to 32 km/h with SNR 255, ranges falling steadily). Frames extracted
from the MCAP: nearest image within 7.3 ms of the scan.

* Scan at t0 + 17.43 s: target r=26 m, azimuth +8 deg, 88 km/h; t0+17.71: r=19, +10 deg;
  t0+17.85: r=16, +14 deg; t0+17.99: r=13, +18 deg; t0+18.13: r=10, +20 deg (SNR 156 to 255).
  Lateral offset r*sin(az) is a constant +3.4 to +3.6 m on the positive-azimuth side while the
  range closes at about 23 m/s.
* The frame at t0+18.13 shows a silver car on the LEFT of the image, in the opposite carriageway
  beyond the central barrier, at the position expected for a closing vehicle 10 to 15 m ahead
  and 3.5 m to the left. The frame at t0+17.85 shows no clear vehicle there yet (it is further
  away and small). The radar's positive azimuth therefore matches LEFT of the sensor, as the
  converter assumes (`angle_sign = 1`, ROS convention).

Confidence: moderate. It rests on one pass; the geometry (constant lateral offset, closing
oncoming vehicle, car visible left of the barrier in the next frame) is consistent, but only one
event was checked and projection to pixels was not computed. Recommend keeping `angle_sign = 1`
and re-checking with a clearer pass (a vehicle crossing the radar's field from right to left).

**Direction flag appears inverted (see defects).** In the same data the `approaching` flag is
false for targets whose range is falling.

### e. Playback spacing (log-time order)

Image-to-image spacing in `log_time` order against `FrameDuration` (39.978 ms, constant):

| Session | Images | Gaps > 60 ms | Largest gaps |
|---|---|---|---|
| s000060 | 4514 | 365 (equals `session.json` drops of 365) | 519.7, 199.9, 159.9 (x6), 120.0 ms |
| s000066 | 34385 | 2518 | 3852.7, 3710.8, 3029.4, 2863.0 (segment boundaries), 1279.3 ms (seg 0, frame 767), 439.8, 359.8 ms |
| s000071 | 38044 | 2892 | 3673.2, 3444.4, 3428.0, 3260.9, 3040.9 (boundaries), 1359.3, 1279.3, 1239.3 ms |

Every image gap equals the sensor-timestamp gap, so gaps are real recorder drops or segment
changes, not conversion artefacts. s000060: the gap count of 365 equals the drop count in
`session.json`. Playback would show the freezes at the same times as the recording.
Real-time playback was not done.

## Defects and observations

1. **Possible inverted direction / sign of `doppler_velocity` (needs controller decision).**
   In s000060, over consecutive scans, matched targets (same speed within 3 km/h, same sign,
   range change under 4 m): targets with `doppler_velocity > 0` (that is, `approaching` false)
   had falling range in 754 of 769 pairs; targets with `doppler_velocity < 0` (`approaching`
   true) had rising range in 26 of 29 pairs. The radar's own `direction` config is 2 (both
   directions). Physically a target whose range falls is approaching, so the flag and the
   mapped sign look reversed. Suspected cause: `ld2451.protocol` maps direction byte 0x00 to
   `approaching=True`; either the library or its reading of the datasheet is inverted for this
   firmware (V1.13.25070114), or the range field is reported differently. Not investigated
   further per the task rules. Command: see "Commands" (`dirsign.py`). Effect: Foxglove users
   and any downstream consumer get the wrong sign convention in `doppler_velocity` and
   `RadarStatus.approach_alarm` would be affected as well.
   Addressed in `radar_scan` by config `radar.invert_direction` (default true; `RadarStatus` untouched).
2. **Frame-count check happens late** for damaged sessions (minutes of conversion before the
   `FrameCountMismatch` failure; see above). Optional improvement: count all segments first.
3. **Timing anomaly**: 20x slower conversions after the first runs on the same machine and data
   (unexplained, possibly host swap pressure or suspend); no code defect shown.
4. s000066 and s000071 each lose 3 radar frames to unrecoverable framing errors (WARN, by
   design). Not a converter defect; recorded for completeness.
5. Minor: the `FrameCountMismatch` message repeats the `--allow-prefix` hint in two forms.

No defect was found in time alignment, frame pairing, ordering, stamps, counts or compression.

## Commands

```
# scratch: /private/tmp/claude-503/-Users-bmendonca-workspace-radar-cam-2-mcap/495955f3-bea1-4166-b296-bc1ef529bc7c/scratchpad/real
rsync -a -e "ssh -o BatchMode=yes" workstation:workspace/hlk-ld2451-speed-trap/data/sNNNNNN in/

uv run radar-cam-2-mcap convert in/s000066 -o out/s000066 --overwrite       # also s000071 s000055 s000060
uv run radar-cam-2-mcap convert in/s000030 -o out/s000030 --overwrite       # fails, exit 1
uv run radar-cam-2-mcap convert in/s000030 -o out/s000030 --overwrite --allow-prefix   # same for s000033
uv run radar-cam-2-mcap convert in/s000060 -o out/c-lz4 --overwrite --config lz4.toml  # [mcap] compression = "lz4"; none likewise
uv run radar-cam-2-mcap convert in/s000060 -o out/c-boot --overwrite --time-base boot

uv run python scripts/verify_output.py out/s000066/s000066.mcap --session in/s000066
uv run python scripts/verify_output.py out/s000030/s000030.mcap --session in/s000030 --allow-prefix
```

Scratch-only analysis scripts (not committed): `analyze.py` (radar re-decode with
`radar_cam_2_mcap.radar`, `FrameWallClock` differences, segment boundaries, playback spacing),
`psnr.py` (check a), `cmp.py` (JPEG hashes), `findveh.py`, `tracks.py`, `getframe.py` and
`dirsign.py` (check d).
