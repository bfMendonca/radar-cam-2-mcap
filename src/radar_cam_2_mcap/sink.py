"""MCAP output on top of ``mcap_ros2.writer.Writer``."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Mapping
from pathlib import Path
from types import TracebackType
from typing import IO, Any

from mcap.writer import CompressionType
from mcap_ros2.writer import Writer

from radar_cam_2_mcap.config import McapConfig
from radar_cam_2_mcap.mapping import MSGDEFS

__all__ = ["McapSink"]

_COMPRESSION = {
    "zstd": CompressionType.ZSTD,
    "lz4": CompressionType.LZ4,
    "none": CompressionType.NONE,
}


class McapSink:
    """Writes ROS 2 messages (CDR) to one MCAP file; a per-topic sequence number is added."""

    def __init__(self, path: Path, cfg: McapConfig) -> None:
        self.path = Path(path)
        self._fh: IO[bytes] | None = self.path.open("wb")
        try:
            self._writer = Writer(
                self._fh,
                chunk_size=cfg.chunk_size,
                compression=_COMPRESSION[cfg.compression],
                enable_crcs=True,
            )
        except BaseException:
            self._fh.close()
            self._fh = None
            raise
        self._schemas: dict[str, Any] = {}
        self._seq: defaultdict[str, int] = defaultdict(int)

    def register(self, topic: str, datatype: str) -> None:
        self._schemas[topic] = self._writer.register_msgdef(datatype, MSGDEFS[datatype])

    def write(
        self,
        topic: str,
        message: dict[str, Any],
        log_ns: int,
        publish_ns: int | None = None,
    ) -> None:
        schema = self._schemas[topic]  # KeyError: topic was not registered
        self._writer.write_message(
            topic,
            schema,
            message,
            log_time=log_ns,
            publish_time=log_ns if publish_ns is None else publish_ns,
            sequence=self._seq[topic],
        )
        self._seq[topic] += 1

    def add_metadata(self, name: str, values: Mapping[str, str]) -> None:
        self._writer.add_metadata(name, dict(values))

    def close(self) -> None:
        """Finish the MCAP (summary, footer) and close the file."""
        if self._fh is None:
            return
        try:
            self._writer.finish()  # type: ignore[no-untyped-call]
        finally:
            self._fh.close()
            self._fh = None

    def abort(self) -> None:
        """Close the file without finishing it (the caller discards it)."""
        if self._fh is not None:
            self._fh.close()
            self._fh = None

    def __enter__(self) -> McapSink:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        if exc_type is None:
            self.close()
        else:
            self.abort()
