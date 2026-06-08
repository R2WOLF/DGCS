"""SIYI single-stream hub with a minimal control UI.

The UI keeps only camera switching and gimbal centering. Joystick input
arrives over UART from either an ESP32 or STM Nucleo board and is used to 
drive gimbal pitch from the Y axis and gimbal yaw from the X axis on the 
front A8 mini. The rear A2 mini is treated as tilt-only while the debug 
panel shows raw X/Y values.

Supported Joystick Boards:
- ESP32-C3 (original)
- STM32L476RG Nucleo (new)
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import struct
import subprocess
import socket
import queue
import serial
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from enum import Enum
from urllib.parse import urlparse
from typing import Optional
from collections import deque

# Optional HM30 UDP poller (built alongside this repo)
try:
    from hm30_udp_poller import HM30UDPPoller
except Exception:
    HM30UDPPoller = None

try:
    import tkinter as tk
    from tkinter import ttk
except Exception:
    tk = None
    ttk = None

try:
    os.environ.setdefault(
        "OPENCV_FFMPEG_CAPTURE_OPTIONS",
        "rtsp_transport;tcp|stimeout;5000000|buffer_size;1048576",
    )
    import cv2
    from PIL import Image, ImageDraw, ImageTk, ImageFont
except Exception:
    cv2 = None
    Image = None
    ImageDraw = None
    ImageTk = None

try:
    from gimbal_sdk import GimbalSDK
except Exception:
    GimbalSDK = None


DEFAULT_CAMERA_1_IP = "192.168.144.25"
DEFAULT_CAMERA_2_IP = "192.168.144.26"
DEFAULT_GIMBAL_PORT = 37260
DEFAULT_RTSP_PORT = 8554
DEFAULT_RTSP_PATH = "main.264"
FRONT_CAMERA_MODEL = "A8 mini"
REAR_CAMERA_MODEL = "A2 mini"
HM30_GROUND_UNIT_IP = "192.168.144.12"

# ==========================================
# JOYSTICK BOARD CONFIGURATION
# ==========================================
# Select which board sends joystick data: "esp32" or "stm32_nucleo"
JOYSTICK_BOARD_TYPE = os.environ.get("JOYSTICK_BOARD_TYPE", "stm32_nucleo").strip().lower()

# UART port fixed to ACM0 (both boards)
UART_PORT = "/dev/ttyACM0"
UART_BAUD = 115200
UART_START_BYTE = 0xAA
UART_END_BYTE = 0x55
JOYSTICK_DEADZONE = 100  # ADC dead zone (0-4095 range)
JOYSTICK_CENTER = 2048   # ADC center value (12-bit: 0-4095, middle of range)
UART_READ_CHUNK = 256

# ==========================================
# LEGACY UART JOYSTICK CONFIG (ESP32 -> RPI)
# ==========================================

VIDEO_RENDER_SIZE = (1280, 720)
# Video fill mode options:
# - 'contain' (default): preserve aspect ratio and fit the whole frame inside the canvas (letterbox)
# - 'stretch': scale the frame to exactly the canvas size (may distort aspect)
# Default to 'contain' to avoid cropping or distortion.
VIDEO_FILL_MODE = os.environ.get("SIYI_VIDEO_FILL_MODE", "contain").strip().lower()
# Allow upscaling when the camera frame is smaller than the canvas. Set to 'false'
# to avoid upscaling artifacts on low-res streams.
VIDEO_ALLOW_UPSCALE = os.environ.get("SIYI_VIDEO_ALLOW_UPSCALE", "true").strip().lower() == "true"
JOYSTICK_ACTIVE_POLL_DELAY = max(0.001, float(os.environ.get("SIYI_JOYSTICK_ACTIVE_POLL_DELAY", "0.002")))
JOYSTICK_IDLE_POLL_DELAY = max(0.002, float(os.environ.get("SIYI_JOYSTICK_IDLE_POLL_DELAY", "0.005")))
JOYSTICK_ACTIVE_FAST_POLLS = max(1, int(os.environ.get("SIYI_JOYSTICK_ACTIVE_FAST_POLLS", "4")))
CONTROL_SEND_HZ = max(10.0, float(os.environ.get("SIYI_CONTROL_SEND_HZ", "35")))
CONTROL_SEND_INTERVAL = 1.0 / CONTROL_SEND_HZ

# ==========================================
# PERFORMANCE OPTIMIZATION SETTINGS
# ==========================================
FRAME_RENDER_POOL_SIZE = 2  
ADAPTIVE_FRAME_SKIP = True  
FRAME_SKIP_THRESHOLD = 0.8  
LAZY_HEARTBEAT = True  
HEARTBEAT_IDLE_TIME = 2.0 
USE_BILINEAR_RESAMPLE = True 
BUFFER_LOG_LIMIT = 1000
# LOW LATENCY MODE: Aggressive optimizations for minimal video/control lag
LOW_LATENCY_MODE = os.environ.get("SIYI_LOW_LATENCY", "false").strip().lower() == "true"
UI_POLL_MS = max(10, int(os.environ.get("SIYI_UI_POLL_MS", "16" if LOW_LATENCY_MODE else "33")))  # 60fps if low-latency
RENDER_FUTURE_POLL_MS = max(1, int(os.environ.get("SIYI_RENDER_FUTURE_POLL_MS", "2" if LOW_LATENCY_MODE else "12")))
STREAM_FRAME_DRAIN_COUNT = max(0, int(os.environ.get("SIYI_STREAM_DRAIN_COUNT", "0" if LOW_LATENCY_MODE else "1")))
STREAM_STALL_TIMEOUT_SEC = max(1.5, float(os.environ.get("SIYI_STREAM_STALL_TIMEOUT_SEC", "2.0" if LOW_LATENCY_MODE else "3.5")))
STREAM_RESTART_COOLDOWN_SEC = max(1.0, float(os.environ.get("SIYI_STREAM_RESTART_COOLDOWN_SEC", "3.0" if LOW_LATENCY_MODE else "5.0")))
HM30_STATUS_UDP_PORT = max(0, int(os.environ.get("SIYI_HM30_STATUS_UDP_PORT", "19856")))
HM30_STATUS_POLL_SEC = max(0.05, float(os.environ.get("SIYI_HM30_STATUS_POLL_SEC", "0.1")))
HM30_STATUS_STALE_SEC = max(1.0, float(os.environ.get("SIYI_HM30_STATUS_STALE_SEC", "2.5")))
HM30_BATTERY_LOW_VOLTAGE = max(0.1, float(os.environ.get("SIYI_HM30_BATTERY_LOW_VOLTAGE", "49.0")))
HM30_SIGNAL_LOW_PERCENT = max(1, min(100, int(os.environ.get("SIYI_HM30_SIGNAL_LOW_PERCENT", "30"))))
HM30_GROUND_UNIT_IP = "192.168.144.12"
AIR_UNIT_IP = os.environ.get("SIYI_AIR_UNIT_IP", "192.168.144.11").strip()
AIR_UNIT_PING_SEC = max(0.5, float(os.environ.get("SIYI_AIR_UNIT_PING_SEC", "1.5")))
CONTROL_SEND_HZ_LOW_LATENCY = max(30.0, float(os.environ.get("SIYI_CONTROL_SEND_HZ_LOW_LATENCY", "75")))
CONTROL_SEND_HZ_NORMAL = max(10.0, float(os.environ.get("SIYI_CONTROL_SEND_HZ", "50")))
CONTROL_SEND_HZ = CONTROL_SEND_HZ_LOW_LATENCY if LOW_LATENCY_MODE else CONTROL_SEND_HZ_NORMAL
CONTROL_SEND_INTERVAL = 1.0 / CONTROL_SEND_HZ

# ==========================================
# PROFESSIONAL OSD THEME CONFIGURATION
# ==========================================
# Theme options: 'dark' (default), 'light', 'pro_blue', 'thermal'
OSD_THEME = os.environ.get("SIYI_OSD_THEME", "dark").strip().lower()
# Show center reticle for aiming reference
OSD_SHOW_RETICLE = os.environ.get("SIYI_OSD_SHOW_RETICLE", "true").strip().lower() == "true"
# OSD transparency (0-255, where 255 is opaque)
OSD_BOX_ALPHA = max(100, min(255, int(os.environ.get("SIYI_OSD_BOX_ALPHA", "255"))))


def build_rtsp_url(ip: str, port: int = DEFAULT_RTSP_PORT, path: str = DEFAULT_RTSP_PATH) -> str:
    return f"rtsp://{ip}:{port}/{path}"


def _tlog(msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


# ===== HM30 UDP Protocol: CRC16 & Packet Handling =====

# CRC16 lookup table — polynomial G(X) = X^16 + X^12 + X^5 + 1
CRC16_TABLE = [
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
    0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
    0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
    0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
    0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
    0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
    0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
    0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
    0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
    0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
    0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
    0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0,
]

def calculate_crc16(data: bytes) -> int:
    """Calculate CRC16 checksum using XMODEM standard."""
    crc = 0
    for byte in data:
        temp = (crc >> 8) & 0xff
        crc = ((crc << 8) ^ CRC16_TABLE[byte ^ temp]) & 0xffff
    return crc

def parse_hm30_packet(raw_data: bytes) -> Optional[dict]:
    """
    Parse a received SIYI HM30 UDP packet.
    Returns dict with 'cmd_id', 'data' on success, or None on failure.
    """
    if len(raw_data) < 11:
        return None
    if raw_data[0:2] != b'\x55\x66':
        return None
    try:
        import struct
        ctrl = raw_data[2]
        data_len = struct.unpack('<H', raw_data[3:5])[0]
        seq = struct.unpack('<H', raw_data[5:7])[0]
        cmd_id = raw_data[7]
        data = raw_data[8:8 + data_len]
        crc16 = struct.unpack('<H', raw_data[8 + data_len:10 + data_len])[0]
        
        # Verify CRC16
        crc_data = raw_data[2:8 + data_len]
        calculated_crc = calculate_crc16(crc_data)
        if calculated_crc != crc16:
            # Log CRC mismatch with details
            try:
                import queue
                # Try to get the log queue from somewhere if available
                pass
            except:
                pass
            return None
        
        return {"cmd_id": cmd_id, "seq": seq, "data": data}
    except (struct.error, IndexError) as e:
        return None


# ===== Professional OSD Renderer with Font Caching & Optimization =====

class OSDRenderer:
    """Professional DJI-style OSD rendering with cached fonts and optimized operations."""
    
    # OSD Color Palette (RGBA) - Default Dark Theme
    COLOR_TEXT_PRIMARY = (230, 238, 246, 255)      # Light blue-gray
    COLOR_TEXT_SECONDARY = (170, 180, 195, 255)    # Muted blue-gray
    COLOR_BOX_BG = (18, 24, 34, 180)               # Dark blue-gray with transparency
    COLOR_SUCCESS = (34, 197, 94, 255)             # Green
    COLOR_WARNING = (245, 158, 11, 255)            # Amber/Orange
    COLOR_DANGER = (239, 68, 68, 255)              # Red
    COLOR_INFO = (59, 130, 246, 255)               # Blue
    COLOR_TEXT_INVERSE = (255, 255, 255, 255)      # White
    
    # OSD Layout Constants
    PADDING = 12
    BOX_HEIGHT = 34
    BOX_CORNER_RADIUS = 10
    RETICLE_SIZE = 40
    RETICLE_COLOR = (100, 200, 100, 100)           # Soft green
    
    _font_cache = {}
    _theme_initialized = False
    
    @classmethod
    def initialize_theme(cls, theme: str = "dark", box_alpha: int = 180):
        """Initialize OSD colors based on selected theme."""
        if cls._theme_initialized:
            return
        
        box_alpha = max(100, min(255, box_alpha))  # Clamp alpha
        
        if theme == "pro_blue":
            # Professional blue theme (DJI-like)
            cls.COLOR_BOX_BG = (10, 30, 60, box_alpha)
            cls.COLOR_TEXT_PRIMARY = (100, 200, 255, 255)
            cls.COLOR_INFO = (80, 180, 255, 255)
        elif theme == "light":
            # Light theme for outdoor high-brightness
            cls.COLOR_BOX_BG = (220, 220, 220, box_alpha)
            cls.COLOR_TEXT_PRIMARY = (30, 40, 50, 255)
            cls.COLOR_TEXT_SECONDARY = (80, 90, 100, 255)
        elif theme == "thermal":
            # Thermal imaging theme (high contrast)
            cls.COLOR_BOX_BG = (0, 0, 0, box_alpha)
            cls.COLOR_TEXT_PRIMARY = (255, 255, 200, 255)
        # else: use default dark theme
        
        cls._theme_initialized = True
    
    @classmethod
    def get_font(cls, size: int = 12, bold: bool = False, mono: bool = False) -> object:
        """Get cached PIL font with fallback to default."""
        cache_key = (size, bold, mono)
        if cache_key not in cls._font_cache:
            try:
                font_name = "DejaVuSansMono" if mono else ("DejaVuSans-Bold" if bold else "DejaVuSans")
                # Try to load from system font directories
                font = ImageDraw.ImageFont.truetype(font_name, size)
            except Exception:
                try:
                    # Fallback to default
                    font = ImageDraw.ImageFont.load_default()
                except Exception:
                    font = None
            cls._font_cache[cache_key] = font
        return cls._font_cache[cache_key]
    
    @staticmethod
    def draw_rounded_box(draw: object, box: list, radius: int = 10, fill=None, outline=None):
        """Draw rounded rectangle with better performance."""
        if draw is None:
            return
        try:
            draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline)
        except Exception:
            draw.rectangle(box, fill=fill, outline=outline)
    
    @staticmethod
    def draw_signal_bars(draw: object, x: int, y: int, signal_percent: int, color: tuple):
        """Draw 4-bar signal strength indicator (professional style)."""
        if draw is None or signal_percent < 0:
            return
        
        bar_width = 3
        bar_spacing = 1
        bar_heights = [4, 8, 12, 16]  # Height of each bar
        
        for i in range(4):
            bar_x = x + i * (bar_width + bar_spacing)
            bar_y = y + 16 - bar_heights[i]
            threshold = (i + 1) * 25  # 25%, 50%, 75%, 100%
            
            if signal_percent >= threshold:
                draw.rectangle([bar_x, bar_y, bar_x + bar_width, y + 16], fill=color)
            else:
                draw.rectangle([bar_x, bar_y, bar_x + bar_width, y + 16], fill=(*color[:3], 50))
    
    @staticmethod
    def draw_center_reticle(draw: object, w: int, h: int, size: int = 40, color: tuple = (100, 200, 100, 100)):
        """Draw center crosshair reticle (DJI style)."""
        if draw is None:
            return
        
        cx, cy = w // 2, h // 2
        cross_size = size // 2
        
        # Horizontal line
        draw.line([(cx - cross_size, cy), (cx - cross_size // 2, cy)], fill=color, width=1)
        draw.line([(cx + cross_size // 2, cy), (cx + cross_size, cy)], fill=color, width=1)
        
        # Vertical line
        draw.line([(cx, cy - cross_size), (cx, cy - cross_size // 2)], fill=color, width=1)
        draw.line([(cx, cy + cross_size // 2), (cx, cy + cross_size)], fill=color, width=1)
        
        # Center dot
        draw.ellipse([cx - 2, cy - 2, cx + 2, cy + 2], fill=color, outline=color)


@dataclass

class HM30Status:
    battery_voltage: Optional[float] = None
    signal_quality: Optional[float] = None
    link_status: str = "Unknown"
    source: str = ""
    air_unit_reachable: bool = False
    camera_temp_c: Optional[float] = None
    updated_ts: float = 0.0

    def is_fresh(self) -> bool:
        if self.source in {"", "disabled"}:
            return True
        return bool(self.updated_ts) and (time.time() - self.updated_ts) <= HM30_STATUS_STALE_SEC

    def battery_low(self) -> bool:
        return self.battery_voltage is not None and self.battery_voltage <= HM30_BATTERY_LOW_VOLTAGE

    def signal_percent(self) -> int:
        if self.signal_quality is None:
            return -1
        return max(0, min(100, int(round(self.signal_quality))))


class CameraName(str, Enum):
    SIYI_1 = "FRONT"
    SIYI_2 = "REAR"


class CameraStream:
    def __init__(self, name: str, url: str, log_fn=None):
        self.name = name
        self.url = url
        self.frame = None
        self._frame_seq = 0
        self.connected = False
        self.fps = 0.0
        self.frame_count = 0  # Phase 2: Track frames for skipping logic
        self.drop_count = 0   # Phase 2: Track dropped frames
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._log_fn = log_fn or (lambda msg: print(msg))
        self._skip_frame = False  # Phase 2: Signal to skip next frame

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name=f"Stream-{self.name}")
        self._thread.start()
        _tlog(f"[{self.name}] stream thread starting — URL: {self.url}")

    def stop(self) -> None:
        self._stop.set()
        self.connected = False
        _tlog(f"[{self.name}] stop requested")

    def is_alive(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def is_reachable(self, timeout: float = 0.5) -> bool:
        """Quick presence check before starting a capture thread.

        Uses a short TCP connect probe to the RTSP port so we do not spin up
        OpenCV capture until the camera is actually reachable on the network.
        """
        try:
            parsed = urlparse(self.url)
            host = parsed.hostname
            port = parsed.port
            if not host or port is None:
                return False
        except Exception:
            return False

        try:
            with socket.create_connection((host, port), timeout=timeout):
                return True
        except Exception:
            return False

    def can_open_stream(self, timeout_ms: int = 1000) -> bool:
        """Strict readiness check: confirm RTSP opens before starting the stream thread.

        This runs only during startup or camera switching, so it does not affect
        live-feed latency once streaming is already running.
        """
        if cv2 is None:
            return self.is_reachable()

        backends = [
            getattr(cv2, "CAP_FFMPEG", None),
            None,
            getattr(cv2, "CAP_GSTREAMER", None),
        ]
        for backend in backends:
            try:
                cap = cv2.VideoCapture(self.url, backend) if backend is not None else cv2.VideoCapture(self.url)
            except Exception:
                continue

            try:
                for prop in (
                    getattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC", None),
                    getattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC", None),
                ):
                    try:
                        if prop is not None:
                            cap.set(prop, timeout_ms)
                    except Exception:
                        pass

                if cap.isOpened():
                    return True
            finally:
                try:
                    cap.release()
                except Exception:
                    pass
        return False

    def wait_until_reachable(self, stop_event: threading.Event, probe_interval: float = 1.0) -> bool:
        while not stop_event.is_set():
            if self.is_reachable() and self.can_open_stream():
                return True
            time.sleep(probe_interval)
        return False

    def _open_capture(self):
        if cv2 is None:
            return None

        backends = [
            ("FFMPEG", getattr(cv2, "CAP_FFMPEG", None)),
            ("DEFAULT", None),
            ("GSTREAMER", getattr(cv2, "CAP_GSTREAMER", None)),
        ]
        for label, backend in backends:
            if backend is None and label != "DEFAULT":
                continue
            try:
                cap = cv2.VideoCapture(self.url, backend) if backend is not None else cv2.VideoCapture(self.url)
            except Exception as exc:
                self._log_fn(f"[{self.name}] backend={label} VideoCapture() failed: {exc}")
                continue

            # If capture not opened, release and continue
            if not cap.isOpened():
                try:
                    cap.release()
                except Exception:
                    pass
                self._log_fn(f"[{self.name}] backend={label} could not open URL {self.url}")
                continue

            # set reasonable timeouts when available
            for prop in (
                getattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC", None),
                getattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC", None),
            ):
                try:
                    if prop is not None:
                        cap.set(prop, 5000)
                except Exception:
                    pass

            self._log_fn(f"[{self.name}] backend={label} opened successfully: {self.url}")
            return (cap, label)

        return None

    def _run(self) -> None:
        if cv2 is None:
            _tlog(f"[{self.name}] opencv-python is not installed")
            return

        while not self._stop.is_set():
            cap_info = self._open_capture()
            if cap_info is None:
                self.connected = False
                time.sleep(2.0)
                continue

            cap, backend_label = cap_info
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            self.connected = True
            frames = 0
            last_fps_ts = time.time()
            last_frame_ts = time.time()

            while not self._stop.is_set():
                try:
                    if STREAM_FRAME_DRAIN_COUNT > 0:
                        for _ in range(STREAM_FRAME_DRAIN_COUNT):
                            if not cap.grab():
                                break
                        ok, frame = cap.retrieve()
                        if not ok or frame is None:
                            ok, frame = cap.read()
                    else:
                        ok, frame = cap.read()
                except Exception as exc:
                    self.connected = False
                    self._log_fn(f"[{self.name}] backend={backend_label} read exception: {exc}")
                    break
                now = time.time()
                if not ok or frame is None:
                    self.connected = False
                    # log more context for diagnostics
                    try:
                        self._log_fn(f"[{self.name}] backend={backend_label} read failed (ok={ok}, frame={type(frame)}) — reconnecting")
                    except Exception:
                        _tlog(f"[{self.name}] stream dropped; reconnecting")
                    break

                last_frame_ts = now
                self.connected = True
                with self._lock:
                    self.frame = frame
                    self._frame_seq += 1

                frames += 1
                if now - last_fps_ts >= 5.0:
                    self.fps = frames / (now - last_fps_ts)
                    frames = 0
                    last_fps_ts = now

            cap.release()
            time.sleep(1.0)

    def get_frame(self):
        with self._lock:
            # Return the latest immutable frame reference to avoid copy overhead.
            return self.frame

    def get_frame_and_seq(self):
        with self._lock:
            return self._frame_seq, self.frame


class SiyiController:
    supports_yaw = True
    supports_light = False
    supports_zoom = False

    def __init__(self, ip: str, port: int = DEFAULT_GIMBAL_PORT, log_fn=None, supports_attitude: bool = True):
        self.ip = ip
        self.port = port
        self.connected = False
        self.camera_angle_deg = 90.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.roll_deg = 0.0
        self._log_fn = log_fn or (lambda msg: print(msg))
        self._sdk = None
        self.inverted = False
        self._lock = threading.Lock()
        self.supports_attitude = bool(supports_attitude)

    def connect(self) -> bool:
        if GimbalSDK is None:
            self._log_fn(f"SIYI SDK unavailable for {self.ip}:{self.port}")
            return False
        try:
            self._sdk = GimbalSDK(self.ip, self.port)
            self.connected = bool(self._sdk.connect())
            # detect mounting orientation and invert controls if camera is upside-down
            try:
                cfg = None
                if hasattr(self._sdk, "get_config"):
                    cfg = self._sdk.get_config()
                if cfg and isinstance(cfg, dict) and cfg.get("mounting") == "Upside-Down":
                    self.inverted = True
                    self._log_fn(f"{self.ip}: mounting=Upside-Down — inverting pitch controls")
                else:
                    self.inverted = False
            except Exception:
                # non-fatal: keep default orientation
                self.inverted = False
            return self.connected
        except Exception as exc:
            self._log_fn(f"SIYI connect failed {self.ip}:{self.port}: {exc}")
            self.connected = False
            return False

    def disconnect(self) -> None:
        if self._sdk:
            try:
                self._sdk.disconnect()
            except Exception:
                pass
        self.connected = False

    def heartbeat(self) -> None:
        if not self._sdk:
            return
        try:
            if hasattr(self._sdk, "heartbeat"):
                self._sdk.heartbeat()
            if hasattr(self._sdk, "get_attitude"):
                self._sdk.get_attitude()
                self.pitch_deg = float(getattr(self._sdk, "pitch_deg", self.pitch_deg))
                self.yaw_deg = float(getattr(self._sdk, "yaw_deg", self.yaw_deg))
                self.roll_deg = float(getattr(self._sdk, "roll_deg", self.roll_deg))
            self.connected = True
        except Exception:
            self.connected = False

    def get_thermal_temp(self) -> Optional[float]:
        """Request camera thermal temperature (center region max temp)."""
        if not self._sdk or not hasattr(self._sdk, "request_local_temp"):
            return None
        try:
            # Request center region temperature (640x480 center area)
            temp_data = self._sdk.request_local_temp(640, 480, 100, 100, 0)
            if temp_data and hasattr(temp_data, "max_c"):
                return float(temp_data.max_c)
        except Exception:
            pass
        return None

    def pitch_speed(self, speed: int) -> None:
        """CRITICAL: Send gimbal pitch command immediately with no buffering."""
        if self._sdk:
            s = int(speed)
            if self.inverted:
                s = -s
            clamped_speed = max(-100, min(100, s))
            try:
                # Send command immediately - no delays
                self._sdk.rotate(0, clamped_speed)
            except Exception as exc:
                # Log but don't block on errors
                pass

    def set_pitch_angle(self, angle_deg: float) -> None:
        """Set the logical camera angle without UI clamping."""
        self.camera_angle_deg = float(angle_deg)
        if not self.supports_attitude:
            return
        if self._sdk and hasattr(self._sdk, "set_angle"):
            pitch_deg = -90.0 + (self.camera_angle_deg / 180.0) * 115.0
            if self.inverted:
                pitch_deg = -pitch_deg
            self._sdk.set_angle(pitch_deg)

    def adjust_pitch_angle(self, delta_deg: float) -> None:
        self.set_pitch_angle(self.camera_angle_deg + delta_deg)

    def yaw_speed(self, speed: int) -> None:
        """CRITICAL: Send gimbal yaw command immediately with no buffering."""
        if self._sdk:
            clamped_speed = max(-100, min(100, int(speed)))
            try:
                # Send command immediately - no delays
                self._sdk.rotate(clamped_speed, 0)
            except Exception as exc:
                # Log but don't block on errors
                pass

    def rotate(self, yaw_speed: int, pitch_speed: int) -> None:
        """Send a combined yaw/pitch command in a single SDK call."""
        if self._sdk:
            yaw_clamped = max(-100, min(100, int(yaw_speed)))
            pitch_clamped = max(-100, min(100, int(pitch_speed)))
            try:
                self._sdk.rotate(yaw_clamped, pitch_clamped)
            except Exception:
                pass

    def center(self) -> None:
        if self.supports_attitude:
            self.set_pitch_angle(90.0)
        if self._sdk and hasattr(self._sdk, "center"):
            self._sdk.center()

    def stop(self) -> None:
        if self._sdk and hasattr(self._sdk, "stop"):
            self._sdk.stop()

    def record(self, enabled: bool = True) -> None:
        if self._sdk and hasattr(self._sdk, "record"):
            self._sdk.record(enabled)


class ControlHub:
    def __init__(self, cam1_ip: str, cam2_ip: str):
        self.log_queue: queue.Queue[str] = queue.Queue(maxsize=BUFFER_LOG_LIMIT)  # Phase 1: Bounded queue
        self.cameras = {
            CameraName.SIYI_1: {
                "ip": cam1_ip,
                "model": FRONT_CAMERA_MODEL,
                "yaw_supported": True,
                "stream": CameraStream(CameraName.SIYI_1.value, build_rtsp_url(cam1_ip), log_fn=self.log_queue.put),
                "gimbal": SiyiController(cam1_ip, log_fn=self.log_queue.put, supports_attitude=True),
            },
            CameraName.SIYI_2: {
                "ip": cam2_ip,
                "model": REAR_CAMERA_MODEL,
                "yaw_supported": False,
                "stream": CameraStream(CameraName.SIYI_2.value, build_rtsp_url(cam2_ip), log_fn=self.log_queue.put),
                "gimbal": SiyiController(cam2_ip, log_fn=self.log_queue.put, supports_attitude=False),
            },
        }
        self.active_camera = CameraName.SIYI_1
        self.joystick_enabled = True
        self.stop_event = threading.Event()
        self._poll_threads: list[threading.Thread] = []
        self._last_pitch_speed = 0
        self._last_yaw_speed = 0
        self._last_control_send_ts = 0.0
        self._uart_buffer = bytearray(512)  # Phase 1: Pre-allocated buffer
        self._uart_buffer_pos = 0  # Phase 1: Track buffer position
        self._command_queue: queue.Queue = queue.Queue(maxsize=32)
        self._command_worker: Optional[threading.Thread] = None
        self._transition_lock = threading.Lock()
        self._transition_target: Optional[CameraName] = None
        self._transition_state: str = "Ready"
        # Phase 1 & 6: Queue-based joystick (lock-free)
        self._joystick_queue = queue.Queue(maxsize=1)  # Keep only latest
        self.joystick_x_raw = 0  # Raw ADC value (0-4095)
        self.joystick_y_raw = 0  # Raw ADC value (0-4095)
        # Phase 2: Lazy heartbeat tracking
        self._last_command_time = {}  # Track last command per camera
        self._last_stream_frame_seq = {}
        self._last_stream_progress_ts = {}
        self._last_stream_restart_ts = {}
        self._hm30_status = HM30Status(link_status="Awaiting HM30", source="disabled", updated_ts=time.time())
        self._hm30_status_lock = threading.Lock()
        self._hm30_stop = threading.Event()
        self._hm30_thread: Optional[threading.Thread] = None
        # ESP telemetry (JSON over serial) - populated by _joystick_loop when JSON lines are received
        self._esp_telemetry: dict = {}
        self._esp_telemetry_lock = threading.Lock()
        self._esp_telemetry_ts: float = 0.0
        # Camera control disable flag (set by App when button 3 pressed)
        self.camera_control_disabled = False
        # Internal UDP poller thread (uses integrated packet parsing)
        self._hm30_poller_thread: Optional[threading.Thread] = None
        # HM30 network connectivity check (ground unit)
        self.hm30_network_connected = False
        self._hm30_network_lock = threading.Lock()
        self._hm30_network_thread: Optional[threading.Thread] = None
        # Air unit connectivity check (robot ping)
        self._hm30_air_unit_thread: Optional[threading.Thread] = None
        now = time.time()
        for cam in self.cameras:
            self._last_command_time[cam] = now
            self._last_stream_frame_seq[cam] = -1
            self._last_stream_progress_ts[cam] = now
            self._last_stream_restart_ts[cam] = 0.0

    def _set_transition_state(self, target: Optional[CameraName], state: str) -> None:
        with self._transition_lock:
            self._transition_target = target
            self._transition_state = state

    def get_transition_state(self) -> tuple[Optional[CameraName], str]:
        with self._transition_lock:
            return self._transition_target, self._transition_state

    def _start_command_worker(self) -> None:
        if self._command_worker and self._command_worker.is_alive():
            return

        def _worker() -> None:
            while not self.stop_event.is_set():
                try:
                    name, handler = self._command_queue.get(timeout=0.2)
                except queue.Empty:
                    continue
                try:
                    handler()
                except Exception as exc:
                    self.log_queue.put(f"{name} failed: {exc}")
                finally:
                    self._command_queue.task_done()

        self._command_worker = threading.Thread(target=_worker, daemon=True, name="SiyiCommandQueue")
        self._command_worker.start()

    def _submit_command(self, name: str, handler) -> bool:
        if self.stop_event.is_set():
            return False
        self._start_command_worker()
        try:
            self._command_queue.put_nowait((name, handler))
            return True
        except queue.Full:
            self.log_queue.put(f"{name} dropped: command queue full")
            return False

    def _should_start_hm30_bridge(self) -> bool:
        return HM30_STATUS_UDP_PORT > 0

    def is_hm30_network_connected(self) -> bool:
        """Check if Pi is connected to HM30 network (via ground unit ping)."""
        with self._hm30_network_lock:
            return self.hm30_network_connected

    def _start_hm30_bridge(self) -> None:
        if self._hm30_thread and self._hm30_thread.is_alive():
            return
        if not self._should_start_hm30_bridge():
            return
        self._hm30_stop.clear()
        self._hm30_thread = threading.Thread(target=self._hm30_status_loop, daemon=True, name="HM30Status")
        self._hm30_thread.start()
        # Start ground unit connectivity checker
        if not self._hm30_network_thread or not self._hm30_network_thread.is_alive():
            self._hm30_network_thread = threading.Thread(target=self._hm30_network_check_loop, daemon=True, name="HM30NetCheck")
            self._hm30_network_thread.start()
        # Start air unit (robot) connectivity checker
        if AIR_UNIT_IP and (not self._hm30_air_unit_thread or not self._hm30_air_unit_thread.is_alive()):
            self._hm30_air_unit_thread = threading.Thread(target=self._air_unit_ping_loop, daemon=True, name="AirUnitPing")
            self._hm30_air_unit_thread.start()

    def _update_hm30_status(self, **values) -> None:
        with self._hm30_status_lock:
            if "battery_voltage" in values and values["battery_voltage"] is not None:
                self._hm30_status.battery_voltage = float(values["battery_voltage"])
            if "signal_quality" in values and values["signal_quality"] is not None:
                self._hm30_status.signal_quality = float(values["signal_quality"])
            if "link_status" in values and values["link_status"] is not None:
                self._hm30_status.link_status = str(values["link_status"])
            if "source" in values and values["source"] is not None:
                self._hm30_status.source = str(values["source"])
            if "air_unit_reachable" in values and values["air_unit_reachable"] is not None:
                self._hm30_status.air_unit_reachable = bool(values["air_unit_reachable"])
            if "camera_temp_c" in values and values["camera_temp_c"] is not None:
                self._hm30_status.camera_temp_c = float(values["camera_temp_c"])
            self._hm30_status.updated_ts = time.time()

    def get_hm30_status_snapshot(self) -> HM30Status:
        with self._hm30_status_lock:
            return HM30Status(
                battery_voltage=self._hm30_status.battery_voltage,
                signal_quality=self._hm30_status.signal_quality,
                link_status=self._hm30_status.link_status,
                source=self._hm30_status.source,
                air_unit_reachable=self._hm30_status.air_unit_reachable,
                camera_temp_c=self._hm30_status.camera_temp_c,
                updated_ts=self._hm30_status.updated_ts,
            )

    def _ping_host(self, host: str, timeout_ms: int = 800) -> bool:
        if not host:
            return False
        is_windows = platform.system().lower().startswith("win")
        count_arg = "-n" if is_windows else "-c"
        timeout_arg = "-w" if is_windows else "-W"
        timeout_value = str(max(1, int(timeout_ms if is_windows else max(1, timeout_ms // 1000))))
        try:
            result = subprocess.run(
                ["ping", count_arg, "1", timeout_arg, timeout_value, host],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            return result.returncode == 0
        except Exception:
            return False

    def _parse_hm30_payload(self, payload: dict) -> None:
        # Be tolerant: payloads may nest data under keys like 'data', 'result', 'params'
        if not isinstance(payload, dict):
            return

        # Unwrap common nesting
        for wrapper in ("data", "result", "params", "payload"):
            if wrapper in payload and isinstance(payload[wrapper], dict):
                payload = payload[wrapper]
                break

        battery_voltage = payload.get("battery_voltage")
        if battery_voltage is None:
            for key in ("rc_bat", "rc_bat_v", "rc_bat_voltage", "ground_unit_battery_voltage"):
                if key in payload:
                    battery_voltage = payload.get(key)
                    break
        # Additional battery keys seen in some bridges
        if battery_voltage is None:
            for key in ("battery", "bat_v", "voltage", "voltage_mv", "bat_mv"):
                if key in payload:
                    battery_voltage = payload.get(key)
                    break
        if isinstance(battery_voltage, str):
            try:
                battery_voltage = float(battery_voltage)
            except Exception:
                battery_voltage = None
        if isinstance(battery_voltage, (int, float)) and battery_voltage > 40:
            battery_voltage = float(battery_voltage) / 10.0

        signal_quality = payload.get("signal_quality")
        if signal_quality is None:
            for key in ("signal", "link_quality", "image_signal_quality"):
                if key in payload:
                    signal_quality = payload.get(key)
                    break
        # Accept RSSI (dBm) by converting to approximate percent
        if signal_quality is None:
            for key in ("rssi", "rssidbm", "signal_dbm"):
                if key in payload:
                    try:
                        rssi = float(payload.get(key))
                        # map [-100, -30] dBm -> [0,100]
                        def _rssi_to_percent(v: float) -> int:
                            try:
                                v = float(v)
                            except Exception:
                                return -1
                            if v <= -100:
                                return 0
                            if v >= -30:
                                return 100
                            return int(round(((v + 100.0) / 70.0) * 100.0))

                        signal_quality = _rssi_to_percent(rssi)
                    except Exception:
                        signal_quality = None
                    break
        if isinstance(signal_quality, str):
            try:
                signal_quality = float(signal_quality)
            except Exception:
                signal_quality = None

        link_status = payload.get("link_status")
        if link_status is None:
            link_status = "OK" if signal_quality is not None and float(signal_quality) > 0 else "No Link"
        # Prefer an explicit source field if present
        source = payload.get("source") or payload.get("src") or payload.get("origin") or "hm30"

        # Log raw payload keys for debugging when enabled
        try:
            self.log_queue.put(f"HM30: parsed payload keys={list(payload.keys())} src={source}")
        except Exception:
            pass

        self._update_hm30_status(
            battery_voltage=battery_voltage,
            signal_quality=signal_quality,
            link_status=link_status,
            source=source,
        )

    def _update_esp_telemetry(self, payload: dict) -> None:
        """Update the latest ESP telemetry (thread-safe). Accepts the JSON object emitted by the ESP/STM32.

        Expected structure from esp_telemetry.ino / nucleo_joystick.ino: 
        {"type":"telemetry","bat":11.4,"imu":{...},"joy":{"x":..,"y":..,"btn":[btn0,btn1,btn2,btn3]},"extra_data":value}
        
        This will update self._esp_telemetry and optionally override joystick values if provided.
        Supports battery voltage, 4 button states, IMU data, and extra sensor data.
        """
        if not isinstance(payload, dict):
            return
        with self._esp_telemetry_lock:
            self._esp_telemetry = payload
            self._esp_telemetry_ts = time.time()

        # If the payload contains joystick values, use them as the source of truth for joystick positions
        try:
            joy = payload.get("joy") or {}
            if isinstance(joy, dict):
                x = joy.get("x")
                y = joy.get("y")
                if isinstance(x, (int, float)) and isinstance(y, (int, float)):
                    # Cast to int and clamp to expected ADC range
                    self.joystick_x_raw = int(max(0, min(4095, int(x))))
                    self.joystick_y_raw = int(max(0, min(4095, int(y))))
        except Exception:
            pass

    def _hm30_status_loop(self) -> None:
        """Listen for HM30 status updates on UDP port (sent by ground unit or bridge)."""
        if HM30_STATUS_UDP_PORT <= 0:
            return

        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", HM30_STATUS_UDP_PORT))
            sock.settimeout(HM30_STATUS_POLL_SEC)
            
            self.log_queue.put(f"HM30: Listening for status updates on UDP port {HM30_STATUS_UDP_PORT}")
            
            while not self._hm30_stop.is_set():
                try:
                    data, addr = sock.recvfrom(4096)
                    text = data.decode("utf-8", errors="replace").strip()
                    
                    # Log incoming data
                    try:
                        self.log_queue.put(f"HM30: Received {len(text)}B from {addr[0]}:{addr[1]}")
                    except Exception:
                        pass
                    
                    # Parse JSON payload
                    if text and text[0] == '{':
                        try:
                            payload = json.loads(text)
                            if isinstance(payload, dict):
                                self._parse_hm30_payload(payload)
                        except json.JSONDecodeError as e:
                            try:
                                self.log_queue.put(f"HM30: JSON parse error - {e}")
                            except Exception:
                                pass
                        except Exception as e:
                            try:
                                self.log_queue.put(f"HM30: Payload parse error - {e}")
                            except Exception:
                                pass
                except socket.timeout:
                    # Normal timeout, just continue listening
                    pass
                except Exception as e:
                    self.log_queue.put(f"HM30: Socket error - {e}")
                    break
        except Exception as e:
            self.log_queue.put(f"HM30: Failed to bind UDP port {HM30_STATUS_UDP_PORT} - {e}")
        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
                pass
            time.sleep(HM30_STATUS_POLL_SEC)

    def _hm30_network_check_loop(self) -> None:
        """Continuously check if Pi is connected to HM30 network via ground unit."""
        last_state = False
        check_interval = 1.0  # Check every 1 second
        while not self._hm30_stop.is_set():
            try:
                connected = self._ping_host(HM30_GROUND_UNIT_IP, timeout_ms=800)
                if connected != last_state:
                    with self._hm30_network_lock:
                        self.hm30_network_connected = connected
                    last_state = connected
                    status_str = "✓ HM30 Network" if connected else "✗ HM30 Network"
                    self.log_queue.put(f"{status_str} (ground unit {HM30_GROUND_UNIT_IP})")
            except Exception:
                pass
            time.sleep(check_interval)

    def _air_unit_ping_loop(self) -> None:
        """Continuously check if GCS is connected to robot (air unit) via ping."""
        last_state = False
        check_interval = AIR_UNIT_PING_SEC
        while not self._hm30_stop.is_set():
            try:
                connected = self._ping_host(AIR_UNIT_IP, timeout_ms=800)
                if connected != last_state:
                    # Update air_unit_reachable in HM30 status snapshot
                    self._update_hm30_status(air_unit_reachable=connected)
                    last_state = connected
                    status_str = "✓ GCS ↔ Robot" if connected else "✗ GCS ↔ Robot"
                    self.log_queue.put(f"{status_str} (air unit {AIR_UNIT_IP})")
            except Exception:
                pass
            time.sleep(check_interval)

    @property
    def streams(self):
        return {cam: cfg["stream"] for cam, cfg in self.cameras.items()}

    def active_controller(self) -> SiyiController:
        return self.cameras[self.active_camera]["gimbal"]

    def active_stream(self) -> CameraStream:
        return self.cameras[self.active_camera]["stream"]

    def active_camera_supports_yaw(self) -> bool:
        return bool(self.cameras[self.active_camera].get("yaw_supported", True))

    def start(self) -> None:
        self._start_command_worker()
        self.active_controller().connect()
        self._start_active_stream_when_ready(self.active_camera)
        self._start_hm30_bridge()
        self._poll_threads = [
            threading.Thread(target=self._poll_loop, args=(cam,), daemon=True, name=f"Poll-{cam.value}")
            for cam in self.cameras
        ]
        self._poll_threads.append(
            threading.Thread(target=self._joystick_loop, daemon=True, name="Joystick")
        )
        for thread in self._poll_threads:
            thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self._hm30_stop.set()
        # Threads will exit on stop_event signal
        for cfg in self.cameras.values():
            cfg["stream"].stop()
            cfg["gimbal"].disconnect()

    def set_active_camera(self, camera: CameraName) -> None:
        if camera == self.active_camera:
            return

        self._set_transition_state(camera, "Queued")

        def _switch_camera() -> None:
            self._set_transition_state(camera, "Connecting")
            old = self.active_camera
            target_controller = self.cameras[camera]["gimbal"]

            try:
                if not target_controller.connect():
                    self._set_transition_state(None, "Ready")
                    self.log_queue.put(f"Active camera switch failed: {camera.value}")
                    return

                if old != camera:
                    self.cameras[old]["stream"].stop()
                self.active_camera = camera
                self._start_active_stream_when_ready(camera)
                self._last_stream_frame_seq[camera] = -1
                self._last_stream_progress_ts[camera] = time.time()
                model = self.cameras[camera].get("model", camera.value)
                self.log_queue.put(f"Active camera: {camera.value} ({model})")
            except Exception as exc:
                self.log_queue.put(f"Active camera switch failed: {camera.value}: {exc}")
            finally:
                self._set_transition_state(None, "Ready")

        if not self._submit_command(f"camera-switch:{camera.value}", _switch_camera):
            self._set_transition_state(None, "Ready")

    def request_center_active(self) -> bool:
        def _center() -> None:
            self.active_controller().center()

        return self._submit_command(f"center:{self.active_camera.value}", _center)

    def _start_active_stream_when_ready(self, camera: CameraName) -> None:
        stream: CameraStream = self.cameras[camera]["stream"]

        def _probe_and_start() -> None:
            if not stream.wait_until_reachable(self.stop_event):
                return
            if self.stop_event.is_set():
                return
            self.log_queue.put(f"{camera.value} camera reachable on network; starting stream")
            stream.start()

        threading.Thread(target=_probe_and_start, daemon=True, name=f"Probe-{camera.value}").start()

    def _restart_stream(self, camera: CameraName, reason: str) -> None:
        now = time.time()
        last_restart = self._last_stream_restart_ts.get(camera, 0.0)
        if (now - last_restart) < STREAM_RESTART_COOLDOWN_SEC:
            return

        stream: CameraStream = self.cameras[camera]["stream"]
        self._last_stream_restart_ts[camera] = now
        self.log_queue.put(
            f"{camera.value} stream restart requested: {reason}"
        )

        stream.stop()

        # Give the previous capture loop a moment to unwind before restart.
        deadline = time.time() + 1.0
        while stream.is_alive() and time.time() < deadline:
            time.sleep(0.05)

        if stream.is_alive():
            self.log_queue.put(
                f"{camera.value} stream thread still busy; restart deferred"
            )
            return

        stream.start()
        self._last_stream_frame_seq[camera] = -1
        self._last_stream_progress_ts[camera] = time.time()

    def move_pitch(self, speed: int) -> None:
        self.active_controller().pitch_speed(speed)
        self._last_command_time[self.active_camera] = time.time()  # Phase 2: Track command time

    def move_yaw(self, speed: int) -> None:
        self.active_controller().yaw_speed(speed)
        self._last_command_time[self.active_camera] = time.time()  # Phase 2: Track command time

    def stop_motion(self) -> None:
        self.active_controller().stop()

    def nudge_pitch(self, speed: int, duration: float = 0.25) -> None:
        self.move_pitch(speed)
        threading.Timer(duration, lambda: self.move_pitch(0)).start()

    def nudge_yaw(self, speed: int, duration: float = 0.25) -> None:
        self.move_yaw(speed)
        threading.Timer(duration, lambda: self.move_yaw(0)).start()

    def _poll_loop(self, camera: CameraName) -> None:
        gimbal = self.cameras[camera]["gimbal"]
        stream: CameraStream = self.cameras[camera]["stream"]
        last_temp_poll = 0.0
        while not self.stop_event.is_set():
            try:
                now = time.time()

                # Active-stream watchdog: restart on frame stall so timeout/decoder
                # stalls recover without manual intervention.
                if camera == self.active_camera:
                    frame_seq, _ = stream.get_frame_and_seq()
                    if frame_seq != self._last_stream_frame_seq[camera]:
                        self._last_stream_frame_seq[camera] = frame_seq
                        self._last_stream_progress_ts[camera] = now
                    elif stream.is_alive() and (now - self._last_stream_progress_ts[camera] >= STREAM_STALL_TIMEOUT_SEC):
                        age = now - self._last_stream_progress_ts[camera]
                        self._restart_stream(camera, f"no new frame for {age:.1f}s")

                if LAZY_HEARTBEAT:
                    time_since_command = now - self._last_command_time[camera]
                    if time_since_command > HEARTBEAT_IDLE_TIME:
                        gimbal.heartbeat()
                        time.sleep(0.5)
                    else:
                        time.sleep(0.1) 
                else:
                    gimbal.heartbeat()
                    time.sleep(0.5)

                # Poll thermal temperature periodically (every 2 seconds) for active camera
                if (now - last_temp_poll) >= 2.0 and camera == self.active_camera:
                    temp = gimbal.get_thermal_temp()
                    if temp is not None:
                        self._update_hm30_status(camera_temp_c=temp, source="camera_thermal")
                    last_temp_poll = now
            except Exception as exc:
                self.log_queue.put(f"{camera.value} poll error: {exc}")
                time.sleep(0.5)

    def _joystick_loop(self) -> None:
        """Read joystick data from UART (binary protocol compatible with ESP32/STM32 Nucleo)."""
        

        ser = None
        poll_count = 0  # Track polls for adaptive sleep

        while not self.stop_event.is_set():
            if ser is None:
                try:
                    # CRITICAL FIX: Use timeout=0 (non-blocking)
                    ser = serial.Serial(UART_PORT, UART_BAUD, timeout=0)
                    ser.reset_input_buffer()
                    self.log_queue.put(f"Joystick: UART connected to ({UART_PORT} @ {UART_BAUD})")
                except Exception as exc:
                    self.log_queue.put(f"Joystick: UART failed to connect - {exc}")
                    time.sleep(2.0)
                    continue

            try:
                # CRITICAL: Non-blocking read - check in_waiting first
                if hasattr(ser, "in_waiting") and ser.in_waiting > 0:
                    # Data available - read immediately
                    raw = ser.read(min(ser.in_waiting, UART_READ_CHUNK))
                    if not raw:
                        continue

                    # Heuristic: if the incoming bytes contain ASCII JSON (newline-delimited), prefer parsing lines
                    handled_as_text = False
                    try:
                        if b'{' in raw or b'\n' in raw:
                            text = raw.decode('utf-8', errors='ignore')
                            # Split into lines and attempt to parse any JSON objects
                            lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
                            for ln in lines:
                                if not ln:
                                    continue
                                if ln[0] == '{' and ln[-1] == '}':
                                    try:
                                        payload = json.loads(ln)
                                        # Only treat as telemetry if it contains expected keys
                                        if isinstance(payload, dict) and payload.get('type') in (None, 'telemetry', 'status'):
                                            self._update_esp_telemetry(payload)
                                            handled_as_text = True
                                            # continue to next line
                                            continue
                                    except Exception:
                                        # Not valid JSON - fall through to binary processing for this chunk
                                        handled_as_text = False
                                        break
                            # If all non-empty lines were valid JSON and handled, we can skip binary processing
                            if handled_as_text:
                                poll_count = 0
                    except Exception:
                        handled_as_text = False

                    # Fallback: treat as binary joystick packet(s)
                    try:
                        self._process_joystick_bytes(raw)
                        poll_count = 0  # Reset sleep counter when data arrives
                    except Exception:
                        # If parsing failed, ignore this chunk
                        pass
                
                # CRITICAL: Apply gimbal control IMMEDIATELY with every poll
                self._apply_joystick_control()
                
                # Adaptive sleep: Poll faster when active, slower when idle
                if poll_count < JOYSTICK_ACTIVE_FAST_POLLS:
                    # Fast polling while fresh joystick data is arriving.
                    time.sleep(JOYSTICK_ACTIVE_POLL_DELAY)
                    poll_count += 1
                else:
                    # Slower idle polling to reduce CPU when the stick is still.
                    time.sleep(JOYSTICK_IDLE_POLL_DELAY)
                    poll_count += 1

            except Exception as exc:
                self.log_queue.put(f"Joystick: UART read error - {exc}")
                if ser:
                    try:
                        ser.close()
                    except Exception:
                        pass
                ser = None
                time.sleep(1.0)

        # Cleanup
        if ser:
            try:
                ser.close()
            except Exception:
                pass

    def _process_joystick_bytes(self, raw: bytes) -> None:
        """Phase 1: Optimized packet parsing with pre-allocated buffer."""
        # Copy new data into buffer
        for byte in raw:
            if self._uart_buffer_pos < len(self._uart_buffer):
                self._uart_buffer[self._uart_buffer_pos] = byte
                self._uart_buffer_pos += 1

        # Search for complete packets
        while self._uart_buffer_pos >= 7:
            # Look for start byte
            start_idx = -1
            for i in range(self._uart_buffer_pos - 6):
                if self._uart_buffer[i] == UART_START_BYTE:
                    start_idx = i
                    break
            
            if start_idx < 0:
                self._uart_buffer_pos = 0
                return
            
            if start_idx > 0:
                # Shift buffer to remove junk data
                self._uart_buffer[0:self._uart_buffer_pos - start_idx] = self._uart_buffer[start_idx:self._uart_buffer_pos]
                self._uart_buffer_pos -= start_idx
            
            # Check end byte
            if self._uart_buffer[6] != UART_END_BYTE:
                self._uart_buffer_pos -= 1
                if self._uart_buffer_pos < 7:
                    self._uart_buffer_pos = 0
                continue
            
            # Validate checksum
            x_high = self._uart_buffer[1]
            x_low = self._uart_buffer[2]
            y_high = self._uart_buffer[3]
            y_low = self._uart_buffer[4]
            received_checksum = self._uart_buffer[5]
            
            expected_checksum = x_high ^ x_low ^ y_high ^ y_low
            if expected_checksum != received_checksum:
                self._uart_buffer_pos -= 1
                continue
            
            # Valid packet - update joystick values (lock-free via queue)
            joystick_x = (y_high << 8) | y_low
            joystick_y = (x_high << 8) | x_low
            self.joystick_x_raw = joystick_x
            self.joystick_y_raw = joystick_y
            
            # Remove processed packet
            self._uart_buffer_pos -= 7
            if self._uart_buffer_pos > 0:
                self._uart_buffer[0:self._uart_buffer_pos] = self._uart_buffer[7:7 + self._uart_buffer_pos]

    def _apply_joystick_control(self) -> None:
        """CRITICAL: Send gimbal commands with minimum latency.
        
        Sends commands more aggressively to prevent gimbal response lag.
        Only skips when value is exactly zero and was already zero.
        
        Disabled if camera_control_disabled flag is set (e.g., by button 3).
        """
        if not self.joystick_enabled or self.camera_control_disabled:
            return

        # Direct atomic read - no locks needed
        x_value = self.joystick_x_raw
        y_value = self.joystick_y_raw

        x_offset = x_value - JOYSTICK_CENTER
        y_offset = y_value - JOYSTICK_CENTER
        yaw_supported = self.active_camera_supports_yaw()
        if (not yaw_supported) or abs(x_offset) < JOYSTICK_DEADZONE:
            yaw_speed = 0
        else:
            yaw_speed = int((x_offset / 2048.0) * 127)
            yaw_speed = max(-127, min(127, yaw_speed))

        if abs(y_offset) < JOYSTICK_DEADZONE:
            pitch_speed = 0
        else:
            pitch_speed = int((y_offset / 2048.0) * 127)
            pitch_speed = max(-127, min(127, pitch_speed))

        # Immediate send on value change; while holding the stick, resend at a
        # bounded rate to keep motion smooth without saturating CPU/network.
        now = time.time()
        speed_changed = (
            (yaw_speed != self._last_yaw_speed)
            or (pitch_speed != self._last_pitch_speed)
        )
        has_motion = (yaw_speed != 0) or (pitch_speed != 0)
        resend_due = (now - self._last_control_send_ts) >= CONTROL_SEND_INTERVAL

        if speed_changed or (has_motion and resend_due):
            self.active_controller().rotate(yaw_speed, pitch_speed)
            self._last_yaw_speed = yaw_speed
            self._last_pitch_speed = pitch_speed
            self._last_control_send_ts = now


# Pygame renderer removed — revert to Tk/PIL ImageTk pipeline for embedded view


class App:
    C = {
        "bg": "#1e1e1e",
        "surface": "#2a2a2a",
        "panel": "#333333",
        "border": "#404040",
        "accent": "#3b82f6",
        "success": "#22c55e",
        "warn": "#f59e0b",
        "danger": "#ef4444",
        "text": "#e2e8f0",
        "muted": "#64748b",
        "dim": "#334155",
    }

    def __init__(
        self,
        root,
        hub: ControlHub,
        *,
        fullscreen: bool = False,
        hide_cursor: bool = False,
        show_debug: bool = False,
        theme: str = "dark",
        osd_alpha: int = 180,
        show_reticle: bool = True,
        low_latency: bool = False,
    ):
        self.root = root
        self.hub = hub
        self._show_debug = show_debug
        self._show_reticle = show_reticle and not low_latency  # Disable reticle in low-latency
        self._low_latency = low_latency
        
        # Initialize professional OSD theme with CLI parameters (skip if low-latency)
        if not low_latency:
            OSDRenderer.initialize_theme(theme, osd_alpha)
        
        self.root.title("GCS")
        self._fullscreen = fullscreen
        self._is_linux = platform.system().lower() == "linux"
        self.root.configure(bg=self.C["bg"])
        self.root.geometry("1280x780")
        self.root.minsize(900, 600)
        if hide_cursor:
            self.root.option_add("*Cursor", "none")
            self.root.configure(cursor="none")
        if fullscreen:
            self._set_fullscreen(True)
            self.root.bind("<Escape>", self._exit_fullscreen)
            self.root.bind("<F11>", self._toggle_fullscreen)
        
        # Phase 1: Frame rendering cache
        self._last_canvas_size = None
        self._last_photo = None
        self._last_rendered_token = (None, -1)
        self._canvas_w = VIDEO_RENDER_SIZE[0]
        self._canvas_h = VIDEO_RENDER_SIZE[1]
        
        # Phase 2: Thread pool for image processing
        self._render_pool = ThreadPoolExecutor(max_workers=FRAME_RENDER_POOL_SIZE, thread_name_prefix="RenderWorker")
        self._pending_render = False
        
        # Phase 2: Change detection for debug info
        self._last_debug_values = None
        self._last_overlay_refresh_ts = 0.0
        self._overlay_refresh_interval = max(0.1, HM30_STATUS_POLL_SEC)

        # Button state tracking for emergency stop and level-based camera controls
        self._emergency_stop_active = False
        self._emergency_stop_last_ts = 0.0
        self._camera_control_disabled = False

        self._build_ui()
        self._poll_ui()

    def _build_ui(self) -> None:
        try:
            ttk.Style(self.root).theme_use("clam")
        except Exception:
            pass

        # Header: Control Buttons (top-right only)
        top = tk.Frame(self.root, bg=self.C["bg"])
        top.pack(fill="x", padx=4, pady=4)
        top.columnconfigure(0, weight=0)
        top.columnconfigure(1, weight=1)
        top.columnconfigure(2, weight=0)

        # GCS branding with logo on the left
        logo_frame = tk.Frame(top, bg=self.C["bg"])
        logo_frame.grid(row=0, column=0, sticky="w", padx=(0, 8))
        
        # Load and display logo
        try:
            logo_img = Image.open("logo.png")
            logo_img.thumbnail((35, 35), Image.Resampling.LANCZOS)
            self._logo_photo = ImageTk.PhotoImage(logo_img)
            logo_widget = tk.Label(logo_frame, image=self._logo_photo, bg=self.C["bg"])
            logo_widget.pack(side="left", padx=(0, 4))
        except Exception:
            pass
        
        gcs_label = tk.Label(logo_frame, text="GCS", font=("Segoe UI", 14, "bold"), bg=self.C["bg"], fg="#ffffff")
        gcs_label.pack(side="left")

        # CENTER: UART Data Display (3 info items from ESP32)
        uart_center = tk.Frame(top, bg=self.C["bg"])
        uart_center.grid(row=0, column=1, sticky="ew", padx=8)
        uart_center.columnconfigure(0, weight=1, uniform="uart")
        uart_center.columnconfigure(1, weight=1, uniform="uart")
        uart_center.columnconfigure(2, weight=1, uniform="uart")
        
        # ===== CUSTOMIZE THESE 3 LABELS TO SHOW YOUR UART DATA =====
        # Update the text and variables in _update_uart_display() method below
        # self.uart_label_1 = tk.Label(uart_center, text="Joystick X: --", 
        #                               bg=self.C["bg"], fg="#87CEEB", font=("Segoe UI", 10))
        # self.uart_label_1.grid(row=0, column=0, padx=4)
        
        # self.uart_label_2 = tk.Label(uart_center, text="Joystick Y: --", 
        #                               bg=self.C["bg"], fg="#87CEEB", font=("Segoe UI", 10))
        # self.uart_label_2.grid(row=0, column=1, padx=4)
        
        # self.uart_label_3 = tk.Label(uart_center, text="Status: Waiting", 
        #                               bg=self.C["bg"], fg="#87CEEB", font=("Segoe UI", 10))
        # self.uart_label_3.grid(row=0, column=2, padx=4)
        # ===== END CUSTOMIZATION AREA =====

        # Right: Control Buttons (top-right only)
        hdr_controls = tk.Frame(top, bg=self.C["bg"])
        hdr_controls.grid(row=0, column=2, sticky="e")
        hdr_controls.columnconfigure(0, weight=1, uniform="hdr")
        hdr_controls.columnconfigure(1, weight=1, uniform="hdr")
        self.active_var = tk.StringVar(value=self.hub.active_camera.value)
        self.status_var = tk.StringVar(value="Ready")
        tk.Button(hdr_controls, text="⇄ Switch Camera", command=self._switch_camera, 
                  bg="#333333", fg="#ffffff", activebackground="#404040", activeforeground="#ffffff",
                  relief="flat", highlightthickness=0, font=("Segoe UI", 9)).grid(row=0, column=0, sticky="ew", padx=2)
        tk.Button(hdr_controls, text="Center", command=self._center_active,
                  bg="#333333", fg="#ffffff", activebackground="#404040", activeforeground="#ffffff",
                  relief="flat", highlightthickness=0, font=("Segoe UI", 9)).grid(row=0, column=1, sticky="ew", padx=2)

        # Body: Single column layout with full-width video
        body = tk.Frame(self.root, bg=self.C["bg"])
        body.pack(fill="both", expand=True, padx=4, pady=4)
        body.columnconfigure(0, weight=1)  # Single column, full width
        body.rowconfigure(0, weight=1)     # Video takes available space
        if self._show_debug:
            body.rowconfigure(1, weight=0)     # Debug bar at bottom

        # Video frame - spans full width (with camera label above)
        self.video_lf = tk.Frame(body, bg=self.C["bg"])
        self.video_lf.grid(row=0, column=0, sticky="nsew")
        self.video_lf.rowconfigure(0, weight=0)     # Camera label row
        self.video_lf.rowconfigure(1, weight=1)     # Video canvas row
        self.video_lf.columnconfigure(0, weight=1)
        
        # Camera label above video
        self.camera_label = tk.Label(self.video_lf, text=f"▶ {self.hub.active_camera.value}", 
                                      bg=self.C["bg"], fg="#ffffff", font=("Segoe UI", 12, "bold"))
        self.camera_label.grid(row=0, column=0, sticky="w", padx=4, pady=2)
        
        self.video_canvas = tk.Canvas(self.video_lf, bg=self.C["bg"], highlightthickness=0)
        self.video_canvas.grid(row=1, column=0, sticky="nsew")
        self.video_canvas.bind("<Configure>", self._on_canvas_resize)
        self.video_canvas_text = self.video_canvas.create_text(
            10,
            10,
            anchor="nw",
            text="Waiting for frame…",
            fill="#aaaaaa",
            font=("Segoe UI", 12),
        )
        self.video_canvas_image = None

        self.debug_label = None
        if self._show_debug:
            # Debug info bar - horizontal layout below video
            debug_bar = tk.Frame(body, bg=self.C["bg"])
            debug_bar.grid(row=1, column=0, sticky="ew", padx=4, pady=(4, 0))
            debug_bar.columnconfigure(0, weight=1)
            self.debug_label = ttk.Label(
                debug_bar,
                text="Initializing…",
                font=("Segoe UI", 9),
                anchor="w"
            )
            self.debug_label.grid(row=0, column=0, sticky="ew")

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _set_debug_info(self, text: str) -> None:
        """Update the horizontal debug info bar below video."""
        if self.debug_label is not None:
            self.debug_label.config(text=text)

    def _on_canvas_resize(self, event) -> None:
        self._canvas_w = max(int(getattr(event, "width", self._canvas_w)), 1)
        self._canvas_h = max(int(getattr(event, "height", self._canvas_h)), 1)

    def _update_uart_display(self) -> None:
        """
        Update the UART data labels in the top center area showing board telemetry.
        
        Available variables from board (ESP32/STM32):
        - self.hub.joystick_x_raw: Joystick X value (0-4095)
        - self.hub.joystick_y_raw: Joystick Y value (0-4095)
        - Board battery voltage
        - 4 Button states
        - Extra sensor data (future use)
        """
        try:
            # Prefer board telemetry (JSON) when available
            board_telem = None
            try:
                with self.hub._esp_telemetry_lock:
                    board_telem = dict(self.hub._esp_telemetry) if self.hub._esp_telemetry else None
            except Exception:
                board_telem = None

            # if board_telem:
            #     # LABEL 1: Board Battery Voltage
            #     board_bat = board_telem.get('bat') or board_telem.get('battery') or board_telem.get('voltage')
            #     if isinstance(board_bat, (int, float)):
            #         label_1_text = f"🔋 Board: {float(board_bat):.2f}V"
            #     else:
            #         label_1_text = f"🔋 Board: --.--V"
            #     self.uart_label_1.config(text=label_1_text)

            #     # LABEL 2: Button States and IMU or Joystick
            #     btns = None
            #     try:
            #         btns = board_telem.get('joy', {}).get('btn') if isinstance(board_telem.get('joy'), dict) else None
            #     except Exception:
            #         btns = None
                
            #     if isinstance(btns, (list, tuple)):
            #         btn_states = "".join(["🟢" if b else "⚪" for b in btns[:4]])
            #         label_2_text = f"Buttons: {btn_states}"
            #     else:
            #         # Fallback to IMU if available
            #         imu = board_telem.get('imu') or {}
            #         if isinstance(imu, dict) and ('ax' in imu or 'ay' in imu):
            #             ax = imu.get('ax', 0.0)
            #             ay = imu.get('ay', 0.0)
            #             label_2_text = f"IMU ax={ax:.2f} ay={ay:.2f}"
            #         else:
            #             joy = board_telem.get('joy') or {}
            #             jx = joy.get('x') if isinstance(joy, dict) else None
            #             jy = joy.get('y') if isinstance(joy, dict) else None
            #             if jx is not None and jy is not None:
            #                 label_2_text = f"JOY x={int(jx):4d} y={int(jy):4d}"
            #             else:
            #                 label_2_text = f"JOY x={self.hub.joystick_x_raw:4d} y={self.hub.joystick_y_raw:4d}"
            #     self.uart_label_2.config(text=label_2_text)

                # LABEL 3: Extra Data or Telemetry Status
                extra = board_telem.get('extra_data')
                if extra is not None:
                    if isinstance(extra, (int, float)):
                        label_3_text = f"Extra: {extra:.2f}"
                    elif isinstance(extra, str):
                        label_3_text = f"Data: {extra[:20]}"
                    else:
                        label_3_text = f"Data: {str(extra)[:20]}"
                else:
                    # Show freshness
                    age = None
                    try:
                        age = time.time() - self.hub._esp_telemetry_ts
                    except Exception:
                        age = None
                    if age is None:
                        status = "Waiting"
                    elif age < 1.0:
                        status = "Active"
                    elif age < 5.0:
                        status = "Idle"
                    else:
                        status = "Stale"
                    label_3_text = f"Status: {status}"
                self.uart_label_3.config(text=label_3_text)
            # else:
                # # Fallback: Show raw joystick values
                # label_1_text = f"🔋 Board: Waiting..."
                # self.uart_label_1.config(text=label_1_text)

                # label_2_text = f"Buttons: ⚪⚪⚪⚪"
                # self.uart_label_2.config(text=label_2_text)

                # label_3_text = f"Status: {'Active' if (self.hub.joystick_x_raw != 0 or self.hub.joystick_y_raw != 0) else 'Idle'}"
                # self.uart_label_3.config(text=label_3_text)
        except Exception:
            pass

    def _toggle_fullscreen(self, event=None) -> None:
        self._fullscreen = not self._fullscreen
        self._set_fullscreen(self._fullscreen)

    def _exit_fullscreen(self, event=None) -> None:
        self._fullscreen = False
        self._set_fullscreen(False)

    def _set_fullscreen(self, enable: bool) -> None:
        """Set fullscreen mode with platform-aware handling for Raspberry Pi."""
        if self._is_linux:
            # On Linux (especially RPi), use overrideredirect + geometry for better compatibility
            if enable:
                self.root.overrideredirect(True)
                self.root.geometry(f"{self.root.winfo_screenwidth()}x{self.root.winfo_screenheight()}+0+0")
                self.root.lift()
                self.root.focus_set()
            else:
                self.root.overrideredirect(False)
                self.root.geometry("1280x780")
        else:
            # On Windows/macOS, use the native -fullscreen attribute
            self.root.attributes("-fullscreen", enable)

    def _render_frame(self, frame, frame_token=None, force_overlay_refresh: bool = False) -> None:
        """Phase 1 & 2: Optimized frame rendering with caching and async processing."""
        if cv2 is None or Image is None or ImageTk is None:
            return

        # If no new frame arrived and overlay refresh not needed, skip render.
        if not force_overlay_refresh and frame_token is not None and frame_token == self._last_rendered_token:
            return
        
        try:
            current_size = (self._canvas_w, self._canvas_h)
            if current_size[0] < 2 or current_size[1] < 2:
                current_size = VIDEO_RENDER_SIZE
            # Phase 1: Always submit a processing task when idle; avoid submitting if one pending
            self._last_canvas_size = current_size
            if not self._pending_render:
                # Phase 2: Use thread pool for image processing
                self._pending_render = True
                if frame_token is not None:
                    self._last_rendered_token = frame_token
                future = self._render_pool.submit(self._process_frame_async, frame, current_size)
                self.root.after(RENDER_FUTURE_POLL_MS, lambda: self._on_frame_ready(future))
        except Exception:
            pass

    def _build_placeholder_image(self, size: tuple[int, int]):
        if Image is None:
            return None
        width = max(1, int(size[0]))
        height = max(1, int(size[1]))
        img = Image.new("RGB", (width, height), (13, 15, 20))
        if ImageDraw is not None:
            draw = ImageDraw.Draw(img)
            draw.text((20, 20), "HM30 HUD", fill=(180, 190, 205))
            draw.text((20, 42), "Waiting for camera frame...", fill=(120, 130, 145))
        return img

    def _draw_signal_bars(self, draw, origin_x: int, origin_y: int, percent: int, fill: tuple[int, int, int, int]) -> None:
        bar_width = 6
        gap = 3
        heights = [6, 10, 14, 18, 22]
        active = 0 if percent < 0 else max(0, min(5, int(round(percent / 20.0))))
        for index, height in enumerate(heights, start=1):
            x0 = origin_x + (index - 1) * (bar_width + gap)
            y0 = origin_y + (22 - height)
            color = fill if index <= active else (120, 130, 145, 180)
            draw.rounded_rectangle([x0, y0, x0 + bar_width, origin_y + 22], radius=2, fill=color)

    def _draw_hm30_overlay(self, img):
        """Professional OSD overlay with board telemetry, buttons, and dual battery display."""
        if ImageDraw is None or img is None:
            return img

        status = self.hub.get_hm30_status_snapshot()
        overlay = img.convert("RGBA")
        draw = ImageDraw.Draw(overlay)
        w, h = overlay.size
        
        # Get board telemetry data (from ESP/STM32)
        board_telem = None
        try:
            with self.hub._esp_telemetry_lock:
                board_telem = dict(self.hub._esp_telemetry) if self.hub._esp_telemetry else None
        except Exception:
            board_telem = None
        
        board_battery = None
        robot_battery = None
        board_buttons = [False, False, False, False]
        extra_data = None
        
        if board_telem:
            # Extract GCS battery voltage (controller board)
            board_battery = board_telem.get('bat_gcs') or board_telem.get('bat') or board_telem.get('battery')
            try:
                if isinstance(board_battery, (int, float)):
                    board_battery = float(board_battery)
                else:
                    board_battery = None
            except Exception:
                board_battery = None
            
            # Extract robot battery voltage (gimbal/system being controlled)
            robot_battery = board_telem.get('bat_robot')
            try:
                if isinstance(robot_battery, (int, float)):
                    robot_battery = float(robot_battery)
                else:
                    robot_battery = None
            except Exception:
                robot_battery = None
            
            # Extract 4 button states
            try:
                btns = board_telem.get('joy', {}).get('btn') if isinstance(board_telem.get('joy'), dict) else None
                if isinstance(btns, (list, tuple)) and len(btns) >= 4:
                    board_buttons = [bool(btns[i]) for i in range(4)]
                elif isinstance(btns, (list, tuple)):
                    board_buttons = [bool(b) for b in btns] + [False] * (4 - len(btns))
            except Exception:
                pass
            
            # ===== BUTTON STATE HANDLING =====
            # Button 1 (index 0): Emergency Stop - follow the live button state so it clears on release
            self._emergency_stop_active = bool(board_buttons[0])
            self._emergency_stop_last_ts = time.time()
            
            # Button 2 (index 1): Camera Select - pressed means rear camera, released means front camera
            target_camera = CameraName.SIYI_2 if board_buttons[1] else CameraName.SIYI_1
            if target_camera != self.hub.active_camera:
                self.hub.set_active_camera(target_camera)

            # Button 3 (index 2): Camera Control Disable - pressed means disabled, released means enabled
            self._camera_control_disabled = bool(board_buttons[2])
            self.hub.camera_control_disabled = self._camera_control_disabled
            
            # Extract extra data for future use
            extra_data = board_telem.get('extra_data') or board_telem.get('data')
        
        # Prepare status values
        fresh = status.is_fresh()
        hm30_battery = status.battery_voltage
        signal_percent = status.signal_percent()
        
        # ===== TOP-LEFT: BOARD BATTERY VOLTAGE =====
        if board_battery is not None:
            board_bat_text = f"🔋 {board_battery:.2f}V"
            if board_battery < 11.2:
                board_bat_color = OSDRenderer.COLOR_DANGER
            elif board_battery < 11.4:
                board_bat_color = OSDRenderer.COLOR_WARNING
            else:
                board_bat_color = OSDRenderer.COLOR_SUCCESS
        else:
            board_bat_text = "🔋 --.--V"
            board_bat_color = OSDRenderer.COLOR_TEXT_SECONDARY
        
        top_left_box = [OSDRenderer.PADDING, OSDRenderer.PADDING, 
                        OSDRenderer.PADDING + 180, OSDRenderer.PADDING + OSDRenderer.BOX_HEIGHT]
        OSDRenderer.draw_rounded_box(draw, top_left_box, OSDRenderer.BOX_CORNER_RADIUS, 
                                     fill=OSDRenderer.COLOR_BOX_BG)
        draw.text((top_left_box[0] + 10, top_left_box[1] + 7), board_bat_text, fill=board_bat_color)
        
        # ===== TOP-RIGHT: 4 BUTTONS INDICATOR =====
        button_box_size = 20
        button_spacing = 8
        buttons_start_x = w - OSDRenderer.PADDING - (button_box_size * 4 + button_spacing * 3) - 10
        buttons_y = OSDRenderer.PADDING + 7
        
        # button_labels = ["B1", "B2", "B3", "B4"]
        # for idx, (is_pressed, label) in enumerate(zip(board_buttons, button_labels)):
        #     button_x = buttons_start_x + idx * (button_box_size + button_spacing)
        #     button_fill = OSDRenderer.COLOR_SUCCESS if is_pressed else (100, 110, 120, 200)
        #     button_outline = OSDRenderer.COLOR_SUCCESS if is_pressed else (100, 110, 120, 255)
            
        #     # Draw button box
        #     draw.rectangle([button_x, buttons_y, button_x + button_box_size, buttons_y + button_box_size],
        #                   fill=button_fill, outline=button_outline, width=2)
        #     # Draw button label
        #     text_color = (0, 0, 0, 255) if is_pressed else (200, 200, 200, 255)
        #     draw.text((button_x + 3, buttons_y + 2), label, fill=text_color)
        
        # ===== BOTTOM-LEFT: ROBOT BATTERY (GIMBAL/SYSTEM BEING CONTROLLED) =====
        robot_bat_text = ""
        robot_bat_color = OSDRenderer.COLOR_TEXT_SECONDARY
        
        # Prefer robot battery from telemetry, fallback to HM30
        displayed_robot_bat = robot_battery if robot_battery is not None else hm30_battery
        
        if displayed_robot_bat is not None:
            low_battery = displayed_robot_bat <= HM30_BATTERY_LOW_VOLTAGE
            if robot_battery is not None:
                robot_bat_text = f"🤖 {displayed_robot_bat:.2f}V"
            else:
                robot_bat_text = f"HM30 {displayed_robot_bat:.1f}V"
            robot_bat_color = OSDRenderer.COLOR_DANGER if low_battery else OSDRenderer.COLOR_SUCCESS
        else:
            robot_bat_text = "🤖 --.--V"
        
        if not fresh and robot_battery is None:  # Only warn if using HM30 data
            robot_bat_text += " ⚠"
            robot_bat_color = OSDRenderer.COLOR_WARNING
        
        bottom_left_box = [OSDRenderer.PADDING, h - OSDRenderer.PADDING - OSDRenderer.BOX_HEIGHT,
                           OSDRenderer.PADDING + 200, h - OSDRenderer.PADDING]
        OSDRenderer.draw_rounded_box(draw, bottom_left_box, OSDRenderer.BOX_CORNER_RADIUS,
                                     fill=OSDRenderer.COLOR_BOX_BG)
        draw.text((bottom_left_box[0] + 10, bottom_left_box[1] + 7), robot_bat_text, fill=robot_bat_color)
        
        # ===== BOTTOM-RIGHT: EXTRA DATA PLACEHOLDER =====
        if extra_data is not None:
            extra_text = ""
            if isinstance(extra_data, (int, float)):
                extra_text = f"Extra: {extra_data:.2f}"
            elif isinstance(extra_data, str):
                extra_text = f"Data: {extra_data[:15]}"
            elif isinstance(extra_data, dict):
                # If extra data is a dict, try to extract first key-value
                for k, v in extra_data.items():
                    if isinstance(v, (int, float)):
                        extra_text = f"{k}: {v:.2f}"
                    else:
                        extra_text = f"{k}: {str(v)[:12]}"
                    break
            
            if extra_text:
                bottom_right_box = [w - OSDRenderer.PADDING - 200, h - OSDRenderer.PADDING - OSDRenderer.BOX_HEIGHT,
                                    w - OSDRenderer.PADDING, h - OSDRenderer.PADDING]
                OSDRenderer.draw_rounded_box(draw, bottom_right_box, OSDRenderer.BOX_CORNER_RADIUS,
                                             fill=OSDRenderer.COLOR_BOX_BG)
                draw.text((bottom_right_box[0] + 10, bottom_right_box[1] + 7), extra_text, 
                         fill=OSDRenderer.COLOR_INFO)
        
        # ===== EMERGENCY STOP OVERLAY =====
        estop_visible = self._emergency_stop_active
        if estop_visible:
            try:
                estop_visible = (time.time() - self._emergency_stop_last_ts) <= 0.5
            except Exception:
                estop_visible = False

        if estop_visible:
            # Draw red semi-transparent overlay
            emergency_overlay = Image.new("RGBA", (w, h), (255, 0, 0, 40))
            overlay.paste(emergency_overlay, (0, 0), emergency_overlay)
            
            # Draw centered emergency stop text
            try:
                emergency_font = ImageFont.truetype("arial.ttf", 80)
            except:
                emergency_font = ImageFont.load_default()
            
            emergency_text = "EMERGENCY STOP"
            bbox = draw.textbbox((0, 0), emergency_text, font=emergency_font)
            text_width = bbox[2] - bbox[0]
            text_height = bbox[3] - bbox[1]
            text_x = (w - text_width) // 2
            text_y = (h - text_height) // 2
            
            # Draw text with dark background
            padding = 20
            draw.rectangle([text_x - padding, text_y - padding, 
                          text_x + text_width + padding, text_y + text_height + padding],
                         fill=(200, 0, 0, 220), outline=(255, 0, 0, 255), width=3)
            draw.text((text_x, text_y), emergency_text, fill=(255, 255, 255, 255), font=emergency_font)
            
            
            # Draw "Press Button to Clear" underneath
            try:
                small_font = ImageFont.truetype("arial.ttf", 24)
            except:
                small_font = ImageFont.load_default()
            
            clear_text = "Press Button to Clear"
            draw.text(((w - draw.textbbox((0, 0), clear_text, font=small_font)[2]) // 2, 
                      text_y + text_height + 30), clear_text, fill=(255, 200, 200, 255), font=small_font)
        
        # ===== CENTER RETICLE =====
        if self._show_reticle:
            OSDRenderer.draw_center_reticle(draw, w, h, size=50, color=OSDRenderer.RETICLE_COLOR)
        
        # ===== COMPOSITE: Blend RGBA overlay back to image =====
        return overlay
    
    def _process_frame_async(self, frame, target_size):
        """Phase 2: Async frame processing in thread pool."""
        try:
            # If not connected to HM30 network, show black screen
            if not self.hub.is_hm30_network_connected():
                black_img = Image.new("RGB", (max(1, int(target_size[0])), max(1, int(target_size[1]))), (0, 0, 0))
                if ImageDraw is not None:
                    draw = ImageDraw.Draw(black_img)
                    draw.text((20, 20), "HM30 Network Disconnected", fill=(255, 0, 0))
                return black_img
            
            if frame is None:
                img = self._build_placeholder_image(target_size)
                if img is None:
                    return None
            else:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(rgb)
            
            src_w, src_h = img.size
            target_w, target_h = target_size

            # Choose scaling behavior based on VIDEO_FILL_MODE
            if VIDEO_FILL_MODE == "stretch":
                # Directly resize to exact target dimensions (may change aspect ratio)
                new_size = (max(1, int(target_w)), max(1, int(target_h)))
                if USE_BILINEAR_RESAMPLE:
                    try:
                        resample = Image.Resampling.BILINEAR
                    except Exception:
                        resample = Image.BILINEAR
                else:
                    try:
                        resample = Image.Resampling.LANCZOS
                    except Exception:
                        resample = Image.LANCZOS
                img = img.resize(new_size, resample)
            else:
                # 'contain' behavior: scale to fit while preserving aspect ratio
                desired_scale = min(target_w / float(src_w), target_h / float(src_h))
                if not VIDEO_ALLOW_UPSCALE:
                    scale = min(desired_scale, 1.0)
                else:
                    scale = desired_scale
                if scale <= 0:
                    scale = 1.0
                if scale != 1.0:
                    new_size = (max(1, int(round(src_w * scale))), max(1, int(round(src_h * scale))))
                    if USE_BILINEAR_RESAMPLE:
                        try:
                            resample = Image.Resampling.BILINEAR
                        except Exception:
                            resample = Image.BILINEAR
                    else:
                        try:
                            resample = Image.Resampling.LANCZOS
                        except Exception:
                            resample = Image.LANCZOS
                    img = img.resize(new_size, resample)

            img = self._draw_hm30_overlay(img) if not self._low_latency else img
            
            # Return the processed PIL Image; PhotoImage must be created on main thread
            return img
        except Exception as e:
            self.hub.log_queue.put(f"Frame processing error: {e}")
            return None
    
    def _on_frame_ready(self, future):
        """Phase 2: Called when async frame processing completes."""
        # If the future isn't done yet, reschedule to check again shortly.
        if not future.done():
            self.root.after(RENDER_FUTURE_POLL_MS, lambda: self._on_frame_ready(future))
            return

        self._pending_render = False
        try:
            img = future.result()
        except Exception as exc:
            try:
                self.hub.log_queue.put(f"Frame future error: {exc}")
            except Exception:
                pass
            return

        if img is None:
            return

        # Create PhotoImage on main thread from PIL Image
        try:
            photo = ImageTk.PhotoImage(image=img)
        except Exception as exc:
            try:
                self.hub.log_queue.put(f"PhotoImage create error: {exc}")
            except Exception:
                pass
            return

        try:
            self.video_canvas.delete(self.video_canvas_text)
            canvas_w = self._canvas_w
            canvas_h = self._canvas_h
            img_w, img_h = photo.width(), photo.height()
            x = max((canvas_w - img_w) // 2, 0)
            y = max((canvas_h - img_h) // 2, 0)
            if self.video_canvas_image is None:
                self.video_canvas_image = self.video_canvas.create_image(x, y, anchor="nw", image=photo, tags=("frame",))
            else:
                self.video_canvas.coords(self.video_canvas_image, x, y)
                self.video_canvas.itemconfig(self.video_canvas_image, image=photo)
            self.video_canvas.image = photo  # Keep reference
            self._last_photo = photo  # Cache for cleanup
        except Exception:
            pass

    def _poll_ui(self) -> None:
  
        latest_message = None      
        
        while True:
            try:
                latest_message = self.hub.log_queue.get_nowait()
            except queue.Empty:
                break

        transition_target, transition_state = self.hub.get_transition_state()
        if transition_target is not None:
            active_text = f"{self.hub.active_camera.value} → {transition_target.value}"
            self.active_var.set(active_text)
            self.status_var.set(f"{transition_state}: switching to {transition_target.value}")
            self.camera_label.config(text=f"▶ {active_text}")
        else:
            self.active_var.set(self.hub.active_camera.value)
            self.camera_label.config(text=f"▶ {self.hub.active_camera.value}")
            if latest_message:
                self.status_var.set(latest_message)

        # Update UART data display in top center
        self._update_uart_display()

        frame_seq, frame = self.hub.active_stream().get_frame_and_seq()
        frame_token = (self.hub.active_camera, frame_seq)

        # Check if overlay refresh timer has elapsed
        now = time.time()
        force_overlay_refresh = False
        if (now - self._last_overlay_refresh_ts) >= self._overlay_refresh_interval:
            self._last_overlay_refresh_ts = now
            force_overlay_refresh = True
            # Invalidate frame token to force redraw with latest overlay
            frame_token = (self.hub.active_camera, frame_seq, now)

        self._render_frame(frame, frame_token, force_overlay_refresh=force_overlay_refresh)

       
        if self._show_debug:
            joy_x = self.hub.joystick_x_raw
            joy_y = self.hub.joystick_y_raw
            current_values = (self.hub.active_camera.value, joy_x, joy_y, latest_message)
            
            # Only update if values changed
            if current_values != self._last_debug_values:
                status_text = f"🎥 {self.hub.active_camera.value}  |  Joystick: X={joy_x:4d} Y={joy_y:4d}  |  Status: {latest_message if latest_message else 'Ready'}"
                self._set_debug_info(status_text)
                self._last_debug_values = current_values
        else:
            current_values = (self.hub.active_camera.value, latest_message)
            if current_values != self._last_debug_values:
                status_text = f"🎥 {self.hub.active_camera.value}  |  Status: {latest_message if latest_message else 'Ready'}"
                self._set_debug_info(status_text)
                self._last_debug_values = current_values

        self.root.after(UI_POLL_MS, self._poll_ui)

    def _switch_camera(self) -> None:
        new = CameraName.SIYI_2 if self.hub.active_camera == CameraName.SIYI_1 else CameraName.SIYI_1
        self.hub.set_active_camera(new)

    def _center_active(self) -> None:
        # Queue the blocking SDK command so the UI thread stays responsive.
        if not self.hub.request_center_active():
            try:
                self.hub.log_queue.put("Center command dropped")
            except Exception:
                pass

    def _on_close(self) -> None:
        self.hub.stop()
        # Phase 2: Cleanup thread pool
        self._render_pool.shutdown(wait=False)
        self.root.destroy()


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SIYI camera hub with professional OSD")
    parser.add_argument("--cam1-ip", default=DEFAULT_CAMERA_1_IP, help="camera 1 IP address")
    parser.add_argument("--cam2-ip", default=DEFAULT_CAMERA_2_IP, help="camera 2 IP address")
    parser.add_argument("--headless", action="store_true", help="run without GUI")
    parser.add_argument("--fullscreen", action="store_true", help="start the GCS window fullscreen")
    parser.add_argument("--hide-cursor", action="store_true", help="hide the mouse cursor in the GCS window")
    parser.add_argument("--kiosk", action="store_true", help="start fullscreen with the cursor hidden")
    parser.add_argument("--show-debug", action="store_true", help="show the developer debug bar")
    parser.add_argument("--dev-ui", action="store_true", help="show the developer debug bar")
    parser.add_argument("--theme", choices=["dark", "pro_blue", "light", "thermal"], default="dark", help="OSD theme (dark, pro_blue, light, thermal)")
    parser.add_argument("--osd-alpha", type=int, default=180, help="OSD box transparency (100-255)")
    parser.add_argument("--no-reticle", action="store_true", help="disable center crosshair reticle")
    parser.add_argument("--low-latency", action="store_true", help="optimize for minimal video/control latency (disables OSD)")
    return parser


def main() -> None:
    args = build_argument_parser().parse_args()
    hub = ControlHub(args.cam1_ip, args.cam2_ip)
    hub.start()

    if args.headless or tk is None:
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            hub.stop()
        return

    root = tk.Tk()
    app = App(
        root,
        hub,
        fullscreen=bool(args.fullscreen or args.kiosk),
        hide_cursor=bool(args.hide_cursor or args.kiosk),
        show_debug=bool(args.show_debug or args.dev_ui),
        theme=args.theme,
        osd_alpha=args.osd_alpha,
        show_reticle=not args.no_reticle,
        low_latency=args.low_latency,
    )
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        hub.stop()


if __name__ == "__main__":
    main()