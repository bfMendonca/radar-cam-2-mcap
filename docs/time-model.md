# Time model

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

