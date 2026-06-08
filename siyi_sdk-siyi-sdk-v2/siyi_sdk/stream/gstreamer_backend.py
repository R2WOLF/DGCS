# Copyright (c) 2026 Mohamed Abdelkader <mohamedashraf123@gmail.com>
# All rights reserved.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""GStreamer RTSP backend using appsink — lowest latency, hardware acceleration.

The GStreamer pipeline decodes H.264 or H.265, converts to BGR, and feeds
frames into an appsink. A GLib.MainLoop runs in a daemon thread; the
new-sample signal handler posts frames to the asyncio event loop.
"""

from __future__ import annotations

import asyncio
import threading
import time
from collections import deque
from collections.abc import AsyncGenerator
from typing import Final, Literal, cast

import structlog

try:
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GLib", "2.0")
    from gi.repository import GLib, Gst

    Gst.init(None)
    _GST_AVAILABLE = True
except Exception:  # gi not installed or version unavailable
    _GST_AVAILABLE = False

from .base import AbstractStreamBackend
from .models import StreamConfig, StreamFrame

_log: Final = structlog.get_logger(__name__)

_RECONNECT_DELAY_CAP: Final[float] = 30.0

# Single pipeline for all codecs — decodebin negotiates H.264/H.265 automatically
# from the RTSP SDP, so no codec hint is needed.
_AUTO_PIPELINE = (
    "rtspsrc location={url} protocols={proto} latency={latency} buffer-mode=slave "
    "! decodebin "
    "! videoconvert "
    "! video/x-raw,format=BGR "
    "! queue max-size-buffers=1 leaky=downstream "
    "! appsink name=sink emit-signals=true max-buffers=1 drop=true"
)


class GStreamerBackend(AbstractStreamBackend):
    """GStreamer + appsink RTSP backend.

    Uses GLib.MainLoop in a daemon thread. Frames are extracted in the
    new-sample signal handler and dispatched to the asyncio queue.

    Args:
        config: Stream configuration.
        codec: Codec hint; "h264" or "h265".

    Raises:
        ImportError: If PyGObject / GStreamer is not installed.
    """

    BACKEND_NAME: Final = "gstreamer"

    def __init__(
        self,
        config: StreamConfig,
        codec: Literal["h264", "h265"] = "h264",
    ) -> None:
        """Initialise the GStreamer backend.

        Args:
            config: Stream configuration.
            codec: Codec pipeline to use; "h264" or "h265".

        Raises:
            ImportError: If PyGObject is not available.
        """
        if not _GST_AVAILABLE:
            raise ImportError(
                "PyGObject and GStreamer are required for GStreamerBackend. "
                "Install system packages: gstreamer1.0-plugins-good gstreamer1.0-plugins-bad "
                "python3-gi, then: pip install PyGObject"
            )
        super().__init__(config)
        self._codec = codec
        self._loop: asyncio.AbstractEventLoop | None = None
        self._queue: asyncio.Queue[StreamFrame] | None = None
        self._latest: deque[StreamFrame] = deque(maxlen=1)
        self._pipeline: Gst.Pipeline | None = None
        self._glib_loop: GLib.MainLoop | None = None
        self._glib_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    def _build_pipeline_str(self) -> str:
        """Build the GStreamer pipeline string from configuration.

        Returns:
            Pipeline description string suitable for gst_parse_launch.
        """
        proto = "tcp" if self._config.transport == "tcp" else "udp"
        return _AUTO_PIPELINE.format(
            url=self._config.rtsp_url,
            proto=proto,
            latency=self._config.latency_ms,
        )

    async def connect(self) -> None:
        """Build the GStreamer pipeline and start the GLib main loop thread."""
        self._stop_event.clear()
        self._loop = asyncio.get_event_loop()
        self._queue = asyncio.Queue(maxsize=1)

        pipeline_str = self._build_pipeline_str()
        _log.info("gstreamer_pipeline", pipeline=pipeline_str)

        self._pipeline = Gst.parse_launch(pipeline_str)
        sink = self._pipeline.get_by_name("sink")
        sink.connect("new-sample", self._on_sample)

        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        self._glib_loop = GLib.MainLoop()
        self._glib_thread = threading.Thread(
            target=self._glib_loop.run,
            name="siyi-gst-mainloop",
            daemon=True,
        )
        self._glib_thread.start()

        self._pipeline.set_state(Gst.State.PLAYING)
        _log.info("gstreamer_backend_connected", url=self._config.rtsp_url)

    async def disconnect(self) -> None:
        """Stop the pipeline and quit the GLib main loop."""
        self._stop_event.set()
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None
        if self._glib_loop is not None:
            self._glib_loop.quit()
            self._glib_loop = None
        if self._glib_thread is not None:
            self._glib_thread.join(timeout=5.0)
            self._glib_thread = None
        _log.info("gstreamer_backend_disconnected")

    def frame_available(self) -> bool:
        """Return True if a frame is buffered.

        Returns:
            True when the latest deque contains a frame.
        """
        return bool(self._latest)

    def read_frame_nowait(self) -> StreamFrame | None:
        """Return the most recently decoded frame without blocking.

        Returns:
            Most recent StreamFrame, or None if none available.
        """
        return self._latest[-1] if self._latest else None

    async def frame_generator(self) -> AsyncGenerator[StreamFrame, None]:
        """Yield frames posted by the GStreamer appsink callback.

        Yields:
            StreamFrame objects in arrival order.
        """
        if self._queue is None:
            return
        while not self._stop_event.is_set():
            try:
                frame = await asyncio.wait_for(self._queue.get(), timeout=0.1)
                yield frame
            except asyncio.TimeoutError:
                continue

    def _on_sample(self, sink: object) -> object:
        """GStreamer appsink new-sample signal handler.

        Extracts the video buffer, converts to numpy BGR array, and dispatches
        to the asyncio queue via call_soon_threadsafe.

        Args:
            sink: The appsink element that emitted the signal.

        Returns:
            GLib flow return constant.
        """
        import numpy as np  # deferred so module loads without numpy at import time

        # Cast sink from object to GStreamer appsink type (Gst is Any via ignore_missing_imports).
        gst_sink = cast("Gst.Element", sink)
        try:
            sample = gst_sink.emit("pull-sample")
            buf = sample.get_buffer()
            caps = sample.get_caps()
            structure = caps.get_structure(0)
            width: int = structure.get_value("width")
            height: int = structure.get_value("height")

            ok, map_info = buf.map(Gst.MapFlags.READ)
            if not ok:
                return Gst.FlowReturn.OK

            try:
                img = np.frombuffer(map_info.data, dtype=np.uint8).reshape(height, width, 3).copy()
            finally:
                buf.unmap(map_info)

            sf = StreamFrame(
                frame=img,
                timestamp=time.monotonic(),
                width=width,
                height=height,
                backend=self.BACKEND_NAME,
            )
            self._latest.append(sf)

            _log.debug(
                "gst_frame_decoded",
                backend=self.BACKEND_NAME,
                width=width,
                height=height,
                timestamp=sf.timestamp,
            )

            loop = self._loop
            queue = self._queue
            if loop is not None and queue is not None:
                def _put(q: asyncio.Queue[StreamFrame] = queue, f: StreamFrame = sf) -> None:
                    try:
                        q.put_nowait(f)
                    except asyncio.QueueFull:
                        pass

                loop.call_soon_threadsafe(_put)

        except Exception as exc:
            _log.error("gst_sample_error", exc=type(exc).__name__, msg=str(exc))

        return Gst.FlowReturn.OK

    def _on_bus_message(self, bus: object, message: object) -> None:
        """GStreamer bus message handler for error and EOS events.

        Args:
            bus: The GStreamer bus (unused).
            message: The GStreamer message object.
        """
        gst_message = cast("Gst.Message", message)
        msg_type = gst_message.type
        if msg_type == Gst.MessageType.ERROR:
            err, _debug = gst_message.parse_error()
            _log.error("gst_pipeline_error", error=str(err))
            self._stop_event.set()
        elif msg_type == Gst.MessageType.EOS:
            _log.warning("gst_pipeline_eos")
            self._stop_event.set()
