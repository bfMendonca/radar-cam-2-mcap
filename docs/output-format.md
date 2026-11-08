# Output format

One MCAP per session. Messages use ROS 2 CDR with `ros2msg` schemas, which are embedded in the
file. The message definitions are in [`src/radar_cam_2_mcap/msgdefs`](../src/radar_cam_2_mcap/msgdefs);
the upstream ones are vendored unchanged and `radar_cam_msgs/*` are written for this project.

Top-level schema names use the ROS 2 form `pkg/msg/Type`, as rosbag2 writes them. Inside the schema
text, field types and `MSG:` dependency lines use the short `pkg/Type` form.

`contract_version` **0.2 is a draft proposal** for topic and metadata names, written before the
consuming pipeline exists. Topics and frame IDs are configurable; every file records its
`contract_version`.

| Topic | Type | Content |
| --- | --- | --- |
| `/camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | One per decoded frame, JPEG by default (PNG optional). |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | From config, written once per file. |
| `/camera/metadata` | `radar_cam_msgs/msg/CameraMetadata` | Per frame: exposure, gains, lux, colour temperature and gains, colour correction matrix, black levels, focus FoM, AE state, scaler crop, recorder wall clock, boot-clock timestamp, segment and frame index. |
| `/radar/scan` | `radar_msgs/msg/RadarScan` | One per decoded data frame; an empty frame gives a scan with no returns. |
| `/radar/status` | `radar_cam_msgs/msg/RadarStatus` | Per data frame: reported target count, targets actually present, approach alarm, boot-clock receive time. |
| `/events` | `rcl_interfaces/msg/Log` | Every `events.jsonl` entry (recorder incidents as WARN, state changes as INFO) plus the converter's own findings. |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Sensor placement, written once per file. |

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

## Radar mapping

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

## Sensor placement (tf)

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

