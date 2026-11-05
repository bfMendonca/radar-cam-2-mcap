# Configuration

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
| `[[transforms]]` | `parent`, `child`, `xyz`, `rpy` | three entries | See [Sensor placement](output-format.md#sensor-placement-tf). |

## Caveats on the defaults

* **`radar.angle_sign` is unverified.** Which direction a positive LD2451 angle means is not
  stated in the driver docs. One vehicle pass in session s000060 supports `+1` (positive azimuth
  on the left), with moderate confidence; see
  [verification.md](verification.md#d-radar-to-camera-alignment-and-angle-sign-s000060-best-effort).
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

