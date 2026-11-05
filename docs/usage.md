# Usage notes

Details behind the options in the [README](../README.md#usage).

## Output files and failures

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


## Damaged sessions

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

