#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════╗
║   SIYI A2 Mini  —  Gimbal Control Station v1.0      ║
║   UDP SDK Protocol + RTSP Video Feed                 ║
║   Compatible: WSL Ubuntu 24.04 / Linux / Windows    ║
╚══════════════════════════════════════════════════════╝

Dependencies:
    pip3 install opencv-python pillow numpy
    sudo apt install python3-tk  (WSL / Ubuntu)
"""

import argparse
import socket
import struct
import threading
import time
import queue
import tkinter as tk
from tkinter import font as tkfont
try:
    import cv2
    HAVE_CV2 = True
except Exception:
    cv2 = None
    HAVE_CV2 = False
from PIL import Image, ImageTk, ImageDraw
import numpy as np

# ═══════════════════════════════════════════════════════════════════
# CONSTANTS
# ═══════════════════════════════════════════════════════════════════

DEFAULT_IP   = "192.168.144.25"
DEFAULT_PORT = 37260
DEFAULT_RTSP1 = "rtsp://192.168.144.25:8554/main.264"
DEFAULT_RTSP2 = "rtsp://192.168.144.108:554/stream=1"

DEMO_FRAME_SIZE = (720, 1280)

STX_BYTES   = b'\x55\x66'
# Heartbeat packet (pre-computed from manual)
HEARTBEAT_PKT = bytes([0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x59,0x8B])

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


# ═══════════════════════════════════════════════════════════════════
# CRC16 & PACKET BUILDER
# ═══════════════════════════════════════════════════════════════════

def crc16_compute(data: bytes) -> int:
    """CRC16 using the SIYI polynomial table."""
    crc = 0
    for byte in data:
        idx = byte ^ ((crc >> 8) & 0xFF)
        crc = CRC16_TABLE[idx] ^ ((crc << 8) & 0xFFFF)
    return crc


def build_packet(cmd_id: int, payload: bytes = b'', seq: int = 0,
                 need_ack: bool = True) -> bytes:
    """
    Frame:
      STX(2) | CTRL(1) | data_len(2,LE) | SEQ(2,LE) | CMD_ID(1) | DATA | CRC16(2,LE)
    """
    ctrl     = 0x01 if need_ack else 0x00
    data_len = len(payload)
    header   = (STX_BYTES
                + struct.pack('B',  ctrl)
                + struct.pack('<H', data_len)
                + struct.pack('<H', seq)
                + struct.pack('B',  cmd_id))
    body     = header + payload
    crc      = crc16_compute(body)
    return body + struct.pack('<H', crc)


def parse_packet(raw: bytes) -> dict | None:
    """Parse a received packet into fields. Returns None on error."""
    if len(raw) < 11:
        return None
    if raw[0:2] != STX_BYTES:
        return None
    ctrl     = raw[2]
    data_len = struct.unpack_from('<H', raw, 3)[0]
    seq      = struct.unpack_from('<H', raw, 5)[0]
    cmd_id   = raw[7]
    if len(raw) < 8 + data_len + 2:
        return None
    data  = raw[8 : 8 + data_len]
    crc_r = struct.unpack_from('<H', raw, 8 + data_len)[0]
    crc_c = crc16_compute(raw[:8 + data_len])
    return {
        'ctrl': ctrl, 'seq': seq, 'cmd_id': cmd_id,
        'data': data, 'crc_ok': (crc_r == crc_c),
    }


def _make_demo_frame(title: str, subtitle: str, accent: tuple[int, int, int],
                     tick: int, size: tuple[int, int] = DEMO_FRAME_SIZE) -> np.ndarray:
    """Create a synthetic RGB frame for laptop-only testing."""
    height, width = size
    frame = np.zeros((height, width, 3), dtype=np.uint8)

    x_ramp = np.linspace(20, 55, width, dtype=np.uint8)
    y_ramp = np.linspace(0, 35, height, dtype=np.uint8)[:, None]
    frame[..., 0] = np.clip(18 + x_ramp + y_ramp, 0, 255)
    frame[..., 1] = np.clip(22 + (x_ramp // 2) + y_ramp, 0, 255)
    frame[..., 2] = np.clip(30 + (x_ramp // 3) + y_ramp, 0, 255)

    bar_x = (tick * 12) % max(1, width - 160)
    bar_y = height // 2
    frame[max(0, bar_y - 30):min(height, bar_y + 30), bar_x:bar_x + 160] = accent
    frame[40:80, 40:width - 40] = (15, 18, 26)
    frame[height - 110:height - 60, 40:width - 40] = (15, 18, 26)

    image = Image.fromarray(frame, mode='RGB')
    draw = ImageDraw.Draw(image)
    draw.text((60, 48), title, fill=(236, 242, 255))
    draw.text((60, 92), subtitle, fill=(181, 192, 214))
    draw.text((60, height - 98), "Laptop demo mode: synthetic frames + mock gimbal SDK", fill=(116, 128, 155))
    draw.text((width - 260, height - 98), time.strftime('%H:%M:%S'), fill=(181, 192, 214))
    return np.array(image, dtype=np.uint8)


# ═══════════════════════════════════════════════════════════════════
# GIMBAL SDK  (UDP)
# ═══════════════════════════════════════════════════════════════════

class GimbalSDK:
    """
    Wraps all UDP communication with the SIYI A2 mini.
    All send() calls are thread-safe; responses arrive via internal queue.
    """

    def __init__(self, ip: str = DEFAULT_IP, port: int = DEFAULT_PORT, mock: bool = False):
        self.ip   = ip
        self.port = port
        self.mock = mock
        self._seq = 0
        self._sock: socket.socket | None = None
        self._running  = False
        self._rx_thread: threading.Thread | None = None
        self._pending: dict[int, queue.Queue] = {}  # seq -> response queue
        self._lock = threading.Lock()
        self._mock_lock = threading.Lock()

        # Live telemetry (updated by _on_packet)
        self.pitch_deg: float = 0.0
        self.yaw_deg:   float = 0.0
        self.roll_deg:  float = 0.0
        self.pitch_vel: float = 0.0
        self.zoom_value: float = 1.0
        self.max_zoom: float = 30.0
        self._mock_last_update = time.time()
        self._mock_config = {
            'hdr_on': False,
            'recording': 0,
            'motion_mode': 'Follow',
            'mounting': 'Normal',
        }

    # ── Lifecycle ─────────────────────────────────────────────────

    def connect(self) -> bool:
        if self.mock:
            self._running = True
            print(f"[SDK] mock connect -> {self.ip}:{self.port}")
            return True
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.settimeout(0.5)
            self._running  = True
            self._rx_thread = threading.Thread(target=self._rx_loop,
                                               daemon=True, name='SIYIRx')
            self._rx_thread.start()
            return True
        except OSError as exc:
            print(f"[SDK] connect error: {exc}")
            return False

    def disconnect(self):
        self._running = False
        if self._sock:
            self._sock.close()
            self._sock = None

    # ── Internal ──────────────────────────────────────────────────

    def _next_seq(self) -> int:
        with self._lock:
            s = self._seq
            self._seq = (self._seq + 1) & 0xFFFF
            return s

    def _rx_loop(self):
        while self._running:
            try:
                raw, _ = self._sock.recvfrom(256)
                pkt = parse_packet(raw)
                if pkt:
                    self._on_packet(pkt)
                    # Wake any waiter for this seq
                    q = self._pending.get(pkt['seq'])
                    if q:
                        q.put(pkt)
            except socket.timeout:
                continue
            except OSError:
                break

    def _on_packet(self, pkt: dict):
        """Update live telemetry from incoming packets."""
        if pkt['cmd_id'] == 0x0D:          # Acquire Gimbal Attitude
            d = pkt['data']
            if len(d) >= 12:
                self.yaw_deg   = struct.unpack_from('<h', d, 0)[0] / 10.0
                self.pitch_deg = struct.unpack_from('<h', d, 2)[0] / 10.0
                self.roll_deg  = struct.unpack_from('<h', d, 4)[0] / 10.0
                self.pitch_vel = struct.unpack_from('<h', d, 8)[0] / 10.0

    def _mock_packet(self, cmd_id: int, payload: bytes = b'', need_ack: bool = True) -> dict:
        seq = self._next_seq()
        return {
            'ctrl': 0x01 if need_ack else 0x00,
            'seq': seq,
            'cmd_id': cmd_id,
            'data': payload,
            'crc_ok': True,
        }

    def _mock_send(self, cmd_id: int, payload: bytes = b'', need_ack: bool = True) -> dict:
        with self._mock_lock:
            now = time.time()
            delta = max(0.0, now - self._mock_last_update)
            self._mock_last_update = now
            if delta:
                decay = max(0.0, 1.0 - min(1.0, delta * 2.5))
                self.pitch_vel *= decay
                self.pitch_deg = float(max(-90.0, min(25.0, self.pitch_deg + self.pitch_vel * delta * 0.2)))

            if cmd_id == 0x01:
                payload = struct.pack('<II', 0x010203, 0x040506)
            elif cmd_id == 0x02:
                payload = b'MOCK-SIYI\x00\x00\x00\x00'
            elif cmd_id == 0x08:
                self.pitch_deg = 0.0
                self.pitch_vel = 0.0
                payload = b'\x01'
            elif cmd_id == 0x04:
                payload = b'\x01'
            elif cmd_id == 0x06:
                payload = b'\x01'
            elif cmd_id == 0x0C:
                if len(payload) >= 1:
                    mode = int(payload[0])
                    if mode == 0x00:
                        self._mock_config['recording'] = 0
                    elif mode == 0x02:
                        self._mock_config['recording'] = 1
                    elif mode == 0x03:
                        self._mock_config['motion_mode'] = 'Lock'
                    elif mode == 0x04:
                        self._mock_config['motion_mode'] = 'Follow'
                    elif mode == 0x05:
                        self._mock_config['motion_mode'] = 'FPV'
                payload = b'\x01'
            elif cmd_id == 0x0A:
                payload = bytes([
                    0,
                    1 if self._mock_config['hdr_on'] else 0,
                    0,
                    self._mock_config['recording'],
                    1,
                    1,
                    0,
                ])
            elif cmd_id == 0x0D:
                payload = struct.pack('<hhhhhh',
                                      int(self.yaw_deg * 10),
                                      int(self.pitch_deg * 10),
                                      int(self.roll_deg * 10),
                                      0,
                                      int(self.pitch_vel * 10),
                                      0)
            elif cmd_id == 0x0E:
                if len(payload) >= 4:
                    self.pitch_deg = max(-90.0, min(25.0, struct.unpack_from('<h', payload, 2)[0] / 10.0))
                payload = b'\x01'
            elif cmd_id == 0x07:
                if len(payload) >= 2:
                    self.pitch_vel = float(struct.unpack_from('b', payload, 1)[0])
                payload = b''
            elif cmd_id == 0x05:
                if len(payload) >= 1:
                    zoom_dir = int(struct.unpack_from('b', payload, 0)[0])
                    if zoom_dir > 0:
                        self.zoom_value = min(self.max_zoom, self.zoom_value + 1.0)
                    elif zoom_dir < 0:
                        self.zoom_value = max(1.0, self.zoom_value - 1.0)
                payload = b'\x01'
            elif cmd_id == 0x0F:
                if len(payload) >= 2:
                    integer = int(payload[0])
                    fraction = int(payload[1])
                    self.zoom_value = max(1.0, min(self.max_zoom, integer + (fraction / 10.0)))
                payload = b'\x01'
            elif cmd_id == 0x16:
                integer = int(self.max_zoom)
                fraction = int(round((self.max_zoom - integer) * 10.0))
                payload = bytes([integer & 0xFF, fraction & 0xFF])

            pkt = self._mock_packet(cmd_id, payload, need_ack=need_ack)
            self._on_packet(pkt)
            return pkt

    # ── Core send ─────────────────────────────────────────────────

    def send(self, cmd_id: int, payload: bytes = b'',
             timeout: float = 1.0) -> dict | None:
        """Send a command and wait for ACK. Returns parsed packet or None."""
        if self.mock:
            return self._mock_send(cmd_id, payload, need_ack=True)
        if not self._running or not self._sock:
            return None
        seq  = self._next_seq()
        pkt  = build_packet(cmd_id, payload, seq)
        resp_q: queue.Queue = queue.Queue()
        self._pending[seq] = resp_q
        try:
            self._sock.sendto(pkt, (self.ip, self.port))
            return resp_q.get(timeout=timeout)
        except queue.Empty:
            return None
        finally:
            self._pending.pop(seq, None)

    def send_no_ack(self, cmd_id: int, payload: bytes = b''):
        """Fire-and-forget (e.g. heartbeat, rotation commands)."""
        if self.mock:
            self._mock_send(cmd_id, payload, need_ack=False)
            return
        if not self._running or not self._sock:
            return
        seq = self._next_seq()
        pkt = build_packet(cmd_id, payload, seq, need_ack=False)
        try:
            self._sock.sendto(pkt, (self.ip, self.port))
        except OSError:
            pass

    # ── Commands ──────────────────────────────────────────────────

    def heartbeat(self):
        """Send pre-built heartbeat to keep TCP-mode connections alive."""
        if self.mock:
            return
        if self._sock and self._running:
            try:
                self._sock.sendto(HEARTBEAT_PKT, (self.ip, self.port))
            except OSError:
                pass

    def get_firmware_version(self) -> dict | None:
        """CMD 0x01 — Returns {camera_fw, gimbal_fw} or None."""
        resp = self.send(0x01)
        if resp and resp['crc_ok'] and len(resp['data']) >= 8:
            d = resp['data']
            cam_raw    = struct.unpack_from('<I', d, 0)[0]
            gimbal_raw = struct.unpack_from('<I', d, 4)[0]
            def _fmt(v):
                return f"v{(v>>16)&0xFF}.{(v>>8)&0xFF}.{v&0xFF}"
            return {'camera_fw': _fmt(cam_raw), 'gimbal_fw': _fmt(gimbal_raw)}
        return None

    def get_hardware_id(self) -> str | None:
        """CMD 0x02 — Returns hardware ID string or None."""
        resp = self.send(0x02)
        if resp and resp['crc_ok']:
            raw = resp['data'][:12]
            return ''.join(chr(b) for b in raw if 32 <= b < 127).strip('\x00')
        return None

    def rotate(self, yaw_speed: int = 0, pitch_speed: int = 0):
        """
        CMD 0x07 — Velocity control.
        yaw_speed, pitch_speed: -100 to +100.
        NOTE: A2 mini only supports pitch rotation (yaw ignored by hardware).
        Send 0,0 to stop.
        """
        yaw_s   = max(-100, min(100, yaw_speed))
        pitch_s = max(-100, min(100, pitch_speed))
        if self.mock:
            self.pitch_vel = float(pitch_s)
            self.pitch_deg = float(max(-90.0, min(25.0, self.pitch_deg + pitch_s * 0.05)))
            return
        payload = struct.pack('bb', yaw_s, pitch_s)
        self.send_no_ack(0x07, payload)

    def stop(self):
        """Send zero-velocity to halt rotation."""
        self.rotate(0, 0)

    def center(self) -> bool:
        """CMD 0x08 — Return gimbal to center (0° pitch)."""
        if self.mock:
            self.pitch_deg = 0.0
            self.pitch_vel = 0.0
            return True
        resp = self.send(0x08, b'\x01')
        return bool(resp and resp['crc_ok']
                    and resp['data'] and resp['data'][0] == 1)

    def get_attitude(self):
        """CMD 0x0D — Poll attitude; updates self.pitch/yaw/roll_deg."""
        self.send(0x0D)

    def set_angle(self, pitch_deg: float) -> bool:
        """
        CMD 0x0E — Absolute pitch control.
        pitch_deg: -90.0 to +25.0 (A2 mini; yaw not available).
        """
        pitch_val = int(max(-900, min(250, pitch_deg * 10)))
        if self.mock:
            self.pitch_deg = pitch_val / 10.0
            self.pitch_vel = 0.0
            return True
        payload = struct.pack('<hh', 0, pitch_val)   # yaw=0 (ignored)
        resp = self.send(0x0E, payload)
        return bool(resp and resp['crc_ok'])

    def zoom(self, direction: int) -> bool:
        """
        CMD 0x05 — Manual zoom direction.
        direction: +1 (zoom in), -1 (zoom out), 0 (stop).
        """
        zoom_dir = max(-1, min(1, int(direction)))
        if self.mock:
            if zoom_dir > 0:
                self.zoom_value = min(self.max_zoom, self.zoom_value + 1.0)
            elif zoom_dir < 0:
                self.zoom_value = max(1.0, self.zoom_value - 1.0)
            return True
        resp = self.send(0x05, struct.pack('b', zoom_dir))
        if resp and resp['crc_ok']:
            if zoom_dir > 0:
                self.zoom_value = min(self.max_zoom, self.zoom_value + 1.0)
            elif zoom_dir < 0:
                self.zoom_value = max(1.0, self.zoom_value - 1.0)
            return True
        return False

    def set_zoom(self, zoom_value: float) -> bool:
        """
        CMD 0x0F — Absolute zoom.
        Payload is [integer, fractional_tenths], e.g. 4.5x -> [0x04, 0x05].
        """
        self.zoom_value = max(1.0, min(self.max_zoom, float(zoom_value)))
        integer = int(self.zoom_value)
        fraction = int(round((self.zoom_value - integer) * 10.0))
        if fraction >= 10:
            integer = min(int(self.max_zoom), integer + 1)
            fraction = 0
        payload = struct.pack('BB', integer & 0xFF, fraction & 0xFF)
        if self.mock:
            return True
        resp = self.send(0x0F, payload)
        return bool(resp and resp['crc_ok'])

    def get_max_zoom(self) -> float | None:
        """CMD 0x16 — Query max zoom value."""
        if self.mock:
            return self.max_zoom
        resp = self.send(0x16)
        if not (resp and resp['crc_ok'] and resp['data']):
            return None
        data = resp['data']
        if len(data) >= 2:
            value = float(data[0]) + (float(data[1]) / 10.0)
        else:
            value = float(data[0])
        self.max_zoom = max(1.0, min(60.0, value))
        if self.zoom_value > self.max_zoom:
            self.zoom_value = self.max_zoom
        return self.max_zoom

    def manual_focus(self, direction: int) -> bool:
        """CMD 0x06 — Manual focus step (+1 / -1)."""
        focus_dir = max(-1, min(1, int(direction)))
        if self.mock:
            return True
        resp = self.send(0x06, struct.pack('b', focus_dir))
        return bool(resp and resp['crc_ok'])

    def auto_focus(self) -> bool:
        """CMD 0x04 — Autofocus trigger."""
        if self.mock:
            return True
        resp = self.send(0x04, b'\x01')
        return bool(resp and resp['crc_ok'])

    def record(self, enabled: bool = True) -> bool:
        """CMD 0x0C — Recording control."""
        payload = bytes([0x02 if enabled else 0x00])
        if self.mock:
            self._mock_config['recording'] = 1 if enabled else 0
            return True
        resp = self.send(0x0C, payload)
        return bool(resp and resp['crc_ok'])

    def lock_mode(self) -> bool:
        """CMD 0x0C — Lock mode."""
        if self.mock:
            self._mock_config['motion_mode'] = 'Lock'
            return True
        resp = self.send(0x0C, b'\x03')
        return bool(resp and resp['crc_ok'])

    def follow_mode(self) -> bool:
        """CMD 0x0C — Follow mode."""
        if self.mock:
            self._mock_config['motion_mode'] = 'Follow'
            return True
        resp = self.send(0x0C, b'\x04')
        return bool(resp and resp['crc_ok'])

    def fpv_mode(self) -> bool:
        """CMD 0x0C — FPV mode."""
        if self.mock:
            self._mock_config['motion_mode'] = 'FPV'
            return True
        resp = self.send(0x0C, b'\x05')
        return bool(resp and resp['crc_ok'])

    def get_config(self) -> dict | None:
        """CMD 0x0A — Acquire gimbal configuration (mounting direction etc.)."""
        if self.mock:
            return dict(self._mock_config)
        resp = self.send(0x0A)
        if resp and resp['crc_ok'] and len(resp['data']) >= 7:
            d = resp['data']
            mount_map = {0: 'Reserved', 1: 'Normal', 2: 'Upside-Down'}
            return {
                'hdr_on':    bool(d[1]),
                'recording': d[3],
                'motion_mode': {0:'Lock',1:'Follow',2:'FPV'}.get(d[4],'?'),
                'mounting':  mount_map.get(d[5], 'Unknown'),
            }
        return None


# ═══════════════════════════════════════════════════════════════════
# RTSP VIDEO THREAD
# ═══════════════════════════════════════════════════════════════════

class RTSPStream:
    """Background thread capturing RTSP frames via OpenCV."""

    def __init__(self, url: str):
        self.url     = url
        self.frame   = None
        self.running = False
        self._thread: threading.Thread | None = None
        self._lock   = threading.Lock()
        self.connected = False
        self.fps_actual = 0.0
        self.color_space = 'bgr'

    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._loop,
                                        daemon=True, name='RTSPStream')
        self._thread.start()

    def stop(self):
        self.running = False
        self.connected = False

    def _loop(self):
        if not HAVE_CV2:
            self.connected = False
            while self.running:
                time.sleep(0.5)
            return

        cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        t_last = time.time()
        frames = 0
        while self.running:
            ret, frame = cap.read()
            if ret:
                self.connected = True
                with self._lock:
                    self.frame = frame
                frames += 1
                now = time.time()
                if now - t_last >= 1.0:
                    self.fps_actual = frames / (now - t_last)
                    frames  = 0
                    t_last  = now
            else:
                self.connected = False
                time.sleep(0.1)
        cap.release()
        self.connected = False

    def get_frame(self) -> np.ndarray | None:
        with self._lock:
            return self.frame.copy() if self.frame is not None else None


class DemoStream:
    """Synthetic video source for laptop testing without a camera."""

    def __init__(self, title: str, subtitle: str, accent: tuple[int, int, int]):
        self.title = title
        self.subtitle = subtitle
        self.accent = accent
        self.frame = None
        self.running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self.connected = False
        self.fps_actual = 0.0
        self.color_space = 'rgb'
        self._tick = 0

    def start(self):
        self.running = True
        self.connected = True
        self._thread = threading.Thread(target=self._loop,
                                        daemon=True, name=f'{self.title}DemoStream')
        self._thread.start()

    def stop(self):
        self.running = False
        self.connected = False

    def _loop(self):
        last = time.time()
        frames = 0
        while self.running:
            frame = _make_demo_frame(self.title, self.subtitle, self.accent, self._tick)
            self._tick += 1
            with self._lock:
                self.frame = frame
            frames += 1
            now = time.time()
            if now - last >= 1.0:
                self.fps_actual = frames / (now - last)
                frames = 0
                last = now
            time.sleep(1.0 / 30.0)

    def get_frame(self) -> np.ndarray | None:
        with self._lock:
            return self.frame.copy() if self.frame is not None else None


# ═══════════════════════════════════════════════════════════════════
# MAIN GUI
# ═══════════════════════════════════════════════════════════════════

class App:
    # ── Palette ───────────────────────────────────────────────────
    C = {
        'bg':      '#0d0f14',
        'surface': '#141720',
        'panel':   '#1c2030',
        'border':  '#252b3a',
        'accent':  '#3b82f6',    # blue
        'success': '#22c55e',    # green
        'warn':    '#f59e0b',    # amber
        'danger':  '#ef4444',    # red
        'text':    '#e2e8f0',
        'muted':   '#64748b',
        'dim':     '#334155',
        'teal':    '#14b8a6',
    }

    def __init__(self, root: tk.Tk, *, ip: str = DEFAULT_IP, port: int = DEFAULT_PORT,
                 rtsp1: str = DEFAULT_RTSP1, rtsp2: str = DEFAULT_RTSP2,
                 mock: bool = False, demo_streams: bool = False):
        self.root = root
        self.root.title("SIYI A2 mini — Gimbal Control Station")
        self.root.configure(bg=self.C['bg'])
        self.root.geometry("1400x720")
        self.root.minsize(1100, 600)

        self._mock_mode = mock
        self._demo_streams = demo_streams or mock or not HAVE_CV2
        self._rtsp_default1 = rtsp1
        self._rtsp_default2 = rtsp2

        self.sdk     = GimbalSDK(ip, port, mock=mock)
        self.stream1 = self._make_stream(rtsp1, "CAM 1")
        self.stream2 = self._make_stream(rtsp2, "CAM 2")

        self._connected     = False
        self._streaming1    = False
        self._streaming2    = False
        self._hb_job        = None
        self._att_job       = None
        self._pitch_active  = False
        self._pitch_dir     = 0

        self._build()
        # Force geometry calculation before any rendering happens
        self.root.update_idletasks()
        # Lock video frame sizes after the window has actually mapped.
        self.root.after(100, self._lock_video_frame_sizes)
        self._tick_video()
        self._tick_status()

    # ═══════════════════════════════════════════════════════════════
    # BUILD UI
    # ═══════════════════════════════════════════════════════════════

    def _build(self):
        C = self.C

        # ── Title bar ────────────────────────────────────────────
        bar = tk.Frame(self.root, bg=C['surface'], height=46)
        bar.pack(fill='x')
        bar.pack_propagate(False)

        tk.Label(bar, text="SIYI A2 mini",
                 bg=C['surface'], fg=C['accent'],
                 font=('Courier New', 15, 'bold')).pack(side='left', padx=16, pady=8)
        tk.Label(bar, text="GIMBAL CONTROL STATION",
                 bg=C['surface'], fg=C['muted'],
                 font=('Courier New', 9)).pack(side='left')

        self._lbl_conn_dot = tk.Label(bar, text="●",
                                       bg=C['surface'], fg=C['danger'],
                                       font=('Courier New', 14))
        self._lbl_conn_dot.pack(side='right', padx=6)
        self._lbl_conn_txt = tk.Label(bar, text="OFFLINE",
                                       bg=C['surface'], fg=C['muted'],
                                       font=('Courier New', 9, 'bold'))
        self._lbl_conn_txt.pack(side='right')

        # ── Body ─────────────────────────────────────────────────
        body = tk.Frame(self.root, bg=C['bg'])
        body.pack(fill='both', expand=False, padx=8, pady=6)
        
        # Use grid for better control of layout proportions
        body.grid_rowconfigure(0, weight=3)  # Video area gets 3x weight
        body.grid_columnconfigure(0, weight=1)  # Video area expands horizontally
        body.grid_columnconfigure(1, weight=0)  # Sidebar fixed width

        # Video area (left, controlled expansion) — split into two cameras
        self._video_frame = tk.Frame(body, bg='#000', bd=1,
                                      highlightbackground=C['border'],
                                      highlightthickness=1)
        self._video_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        self._video_frame.grid_propagate(False)  # Lock size to grid allocation
        
        # Configure grid for equal-width columns
        self._video_frame.grid_columnconfigure(0, weight=1)  # CAM 1 column
        self._video_frame.grid_columnconfigure(1, weight=1)  # CAM 2 column
        self._video_frame.grid_rowconfigure(0, weight=1)     # Video rows expand

        # Camera 1 (left half)
        self._video_frame1 = tk.Frame(self._video_frame, bg='#000', bd=1,
                                       highlightbackground=C['border'],
                                       highlightthickness=1)
        self._video_frame1.grid(row=0, column=0, sticky='nsew', padx=(0, 2))
        self._video_frame1.grid_propagate(False)  # Lock size, don't resize based on children

        self._video_lbl1 = tk.Label(self._video_frame1, bg='#000',
                                     fg=C['muted'],
                                     text="[ CAM 1 ]\nStart stream",
                                     font=('Courier New', 10),
                                     justify='center')
        self._video_lbl1.place(relx=0, rely=0, relwidth=1, relheight=1)
        self._video_lbl1.config(wraplength=300)

        self._hud1 = tk.Label(self._video_frame1,
                               fg=C['teal'],
                               font=('Courier New', 8),
                               text="", anchor='nw', justify='left')
        self._hud1.place(x=4, y=4)

        self._fps_lbl1 = tk.Label(self._video_frame1,
                                   fg=C['muted'],
                                   font=('Courier New', 7), text="")
        self._fps_lbl1.place(relx=1.0, rely=1.0, anchor='se', x=-3, y=-2)

        # Camera 2 (right half)
        self._video_frame2 = tk.Frame(self._video_frame, bg='#000', bd=1,
                                       highlightbackground=C['border'],
                                       highlightthickness=1)
        self._video_frame2.grid(row=0, column=1, sticky='nsew', padx=(2, 0))
        self._video_frame2.grid_propagate(False)  # Lock size, don't resize based on children

        self._video_lbl2 = tk.Label(self._video_frame2, bg='#000',
                                     fg=C['muted'],
                                     text="[ CAM 2 ]\nStart stream",
                                     font=('Courier New', 10),
                                     justify='center')
        self._video_lbl2.place(relx=0, rely=0, relwidth=1, relheight=1)
        self._video_lbl2.config(wraplength=300)

        self._hud2 = tk.Label(self._video_frame2,
                               fg=C['teal'],
                               font=('Courier New', 8),
                               text="", anchor='nw', justify='left')
        self._hud2.place(x=4, y=4)

        self._fps_lbl2 = tk.Label(self._video_frame2,
                                   fg=C['muted'],
                                   font=('Courier New', 7), text="")
        self._fps_lbl2.place(relx=1.0, rely=1.0, anchor='se', x=-3, y=-2)

        # Right sidebar (fixed width)
        side = tk.Frame(body, bg=C['bg'], width=298)
        side.grid(row=0, column=1, sticky='nsew', padx=(8, 0))
        side.grid_propagate(False)  # Enforce fixed width

        self._build_connection(side)
        self._build_telemetry(side)
        self._build_controls(side)
        self._build_video_panel(side)
        self._build_info_panel(side)

        # ── Status bar ───────────────────────────────────────────
        self._status_var = tk.StringVar(value="Ready — connect to the gimbal to begin.")
        tk.Label(self.root, textvariable=self._status_var,
                 bg=C['surface'], fg=C['muted'],
                 font=('Courier New', 8), anchor='w', padx=10, pady=3
                 ).pack(fill='x', side='bottom')

        # Key bindings
        self.root.bind('<Up>',        self._kb_up_press)
        self.root.bind('<KeyRelease-Up>',   self._kb_up_rel)
        self.root.bind('<Down>',      self._kb_dn_press)
        self.root.bind('<KeyRelease-Down>', self._kb_dn_rel)
        self.root.bind('<space>',     lambda _: self._cmd_center())
        self.root.bind('<Return>',    lambda _: self._cmd_poll_att())
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _make_stream(self, url: str, title: str):
        if self._demo_streams:
            accent = (56, 189, 248) if title == "CAM 1" else (245, 158, 11)
            subtitle = f"{title}  •  {url}"
            return DemoStream(title, subtitle, accent)
        return RTSPStream(url)

    def _lock_video_frame_sizes(self):
        """Lock video frame sizes after geometry calculation to prevent resizing."""
        # Lock the parent container frame
        wv = self._video_frame.winfo_width()
        hv = self._video_frame.winfo_height()
        w1 = self._video_frame1.winfo_width()
        h1 = self._video_frame1.winfo_height()
        w2 = self._video_frame2.winfo_width()
        h2 = self._video_frame2.winfo_height()

        if min(wv, hv, w1, h1, w2, h2) <= 10:
            self.root.after(50, self._lock_video_frame_sizes)
            return

        if wv > 10 and hv > 10:
            self._video_frame.config(width=wv, height=hv)

        # Set explicit sizes to lock frame dimensions
        # With grid_propagate(False), this prevents frames from expanding
        if w1 > 10 and h1 > 10:
            self._video_frame1.config(width=w1, height=h1)
        if w2 > 10 and h2 > 10:
            self._video_frame2.config(width=w2, height=h2)

        # Ensure the child labels stay confined to their frames.
        self._video_lbl1.place(relx=0, rely=0, relwidth=1, relheight=1)
        self._video_lbl2.place(relx=0, rely=0, relwidth=1, relheight=1)

    # ── Section builders ──────────────────────────────────────────

    def _section(self, parent, title):
        C = self.C
        frame = tk.Frame(parent, bg=C['panel'],
                         highlightbackground=C['border'],
                         highlightthickness=1)
        frame.pack(fill='x', pady=(0, 7))
        hdr = tk.Frame(frame, bg=C['dim'])
        hdr.pack(fill='x')
        tk.Label(hdr, text=f"  {title}",
                 bg=C['dim'], fg=C['muted'],
                 font=('Courier New', 8, 'bold'),
                 anchor='w', pady=3).pack(fill='x')
        inner = tk.Frame(frame, bg=C['panel'])
        inner.pack(fill='x', padx=8, pady=6)
        return inner

    def _btn(self, parent, text, cmd, color=None, fg=None, **kw):
        C = self.C
        bg = color or C['accent']
        fg = fg or C['text']
        b = tk.Button(parent, text=text, command=cmd,
                      bg=bg, fg=fg, activebackground=C['dim'],
                      activeforeground=C['text'],
                      font=('Courier New', 9, 'bold'),
                      relief='flat', bd=0, cursor='hand2',
                      padx=6, pady=5, **kw)
        return b

    def _build_connection(self, parent):
        C = self.C
        inner = self._section(parent, "CONNECTION")

        row1 = tk.Frame(inner, bg=C['panel'])
        row1.pack(fill='x', pady=(0, 4))
        tk.Label(row1, text="IP", bg=C['panel'], fg=C['muted'],
                 font=('Courier New', 8), width=5, anchor='w').pack(side='left')
        self._ip_var = tk.StringVar(value=DEFAULT_IP)
        tk.Entry(row1, textvariable=self._ip_var,
                 bg=C['border'], fg=C['text'], insertbackground=C['text'],
                 font=('Courier New', 9), relief='flat', bd=4, width=18
                 ).pack(side='left', padx=(0, 2))

        row2 = tk.Frame(inner, bg=C['panel'])
        row2.pack(fill='x', pady=(0, 6))
        tk.Label(row2, text="Port", bg=C['panel'], fg=C['muted'],
                 font=('Courier New', 8), width=5, anchor='w').pack(side='left')
        self._port_var = tk.StringVar(value=str(DEFAULT_PORT))
        tk.Entry(row2, textvariable=self._port_var,
                 bg=C['border'], fg=C['text'], insertbackground=C['text'],
                 font=('Courier New', 9), relief='flat', bd=4, width=8
                 ).pack(side='left')

        self._conn_btn = self._btn(inner, "▶  CONNECT", self._toggle_connect,
                                    color=C['success'], fg='#000')
        self._conn_btn.pack(fill='x')

    def _build_telemetry(self, parent):
        C = self.C
        inner = self._section(parent, "ATTITUDE  (°)")

        grid = tk.Frame(inner, bg=C['panel'])
        grid.pack(fill='x')
        for i, (axis, attr) in enumerate([("PITCH", '_att_pitch'),
                                           ("YAW",   '_att_yaw'),
                                           ("ROLL",  '_att_roll')]):
            tk.Label(grid, text=axis, bg=C['panel'], fg=C['muted'],
                     font=('Courier New', 8), width=5, anchor='w'
                     ).grid(row=i, column=0, sticky='w', pady=1)
            lbl = tk.Label(grid, text=" ---°",
                           bg=C['panel'], fg=C['teal'],
                           font=('Courier New', 12, 'bold'), anchor='w', width=9)
            lbl.grid(row=i, column=1, sticky='w')
            setattr(self, attr, lbl)

        self._btn(inner, "⟳  POLL ATTITUDE", self._cmd_poll_att,
                  color=C['dim']).pack(fill='x', pady=(6, 0))

    def _build_controls(self, parent):
        C = self.C
        inner = self._section(parent, "GIMBAL CONTROL")

        # Speed slider
        spd_row = tk.Frame(inner, bg=C['panel'])
        spd_row.pack(fill='x', pady=(0, 6))
        tk.Label(spd_row, text="Speed", bg=C['panel'], fg=C['muted'],
                 font=('Courier New', 8)).pack(side='left')
        self._speed_var = tk.IntVar(value=60)
        self._speed_lbl = tk.Label(spd_row, text=" 60%",
                                    bg=C['panel'], fg=C['warn'],
                                    font=('Courier New', 9, 'bold'))
        self._speed_lbl.pack(side='right')
        tk.Scale(spd_row, from_=10, to=100, orient='horizontal',
                 variable=self._speed_var,
                 command=lambda v: self._speed_lbl.config(text=f" {v}%"),
                 bg=C['panel'], fg=C['muted'],
                 troughcolor=C['dim'], highlightthickness=0,
                 showvalue=False, length=120).pack(side='left', padx=4)

        # Tilt buttons
        self._up_btn = self._btn(inner, "▲   TILT  UP   (↑)",
                                  None, color=C['accent'])
        self._up_btn.pack(fill='x', pady=2)
        self._up_btn.bind('<ButtonPress-1>',   lambda _: self._start_pitch(1))
        self._up_btn.bind('<ButtonRelease-1>', lambda _: self._stop_pitch())

        self._dn_btn = self._btn(inner, "▼   TILT DOWN  (↓)",
                                  None, color=C['accent'])
        self._dn_btn.pack(fill='x', pady=2)
        self._dn_btn.bind('<ButtonPress-1>',   lambda _: self._start_pitch(-1))
        self._dn_btn.bind('<ButtonRelease-1>', lambda _: self._stop_pitch())

        self._btn(inner, "⊙   CENTER / HOME  (Space)",
                  self._cmd_center, color=C['danger']
                  ).pack(fill='x', pady=(4, 0))

        # Absolute angle
        ang_inner = self._section(parent, "ABSOLUTE PITCH  (°)")
        self._angle_var = tk.DoubleVar(value=0.0)
        self._angle_lbl = tk.Label(ang_inner, text="0.0°",
                                    bg=C['panel'], fg=C['warn'],
                                    font=('Courier New', 11, 'bold'))
        self._angle_lbl.pack()

        def _upd_angle(v):
            self._angle_lbl.config(text=f"{float(v):.1f}°")

        tk.Scale(ang_inner, from_=-90, to=25, resolution=0.5,
                 orient='horizontal', variable=self._angle_var,
                 command=_upd_angle,
                 bg=C['panel'], fg=C['muted'],
                 troughcolor=C['dim'], highlightthickness=0,
                 showvalue=False, length=270).pack(fill='x', pady=(0, 4))

        self._btn(ang_inner, "→  SEND ANGLE",
                  self._cmd_set_angle, color=C['dim']).pack(fill='x')

        zoom_inner = self._section(parent, "ZOOM / FOCUS")
        row1 = tk.Frame(zoom_inner, bg=C['panel'])
        row1.pack(fill='x')
        self._btn(row1, "ZOOM +", self._cmd_zoom_in, color=C['teal'], fg='#000').pack(side='left', expand=True, fill='x', padx=(0, 3))
        self._btn(row1, "ZOOM -", self._cmd_zoom_out, color=C['warn'], fg='#000').pack(side='left', expand=True, fill='x', padx=(3, 0))

        row2 = tk.Frame(zoom_inner, bg=C['panel'])
        row2.pack(fill='x', pady=(4, 0))
        self._btn(row2, "AF", self._cmd_auto_focus, color=C['accent']).pack(side='left', expand=True, fill='x', padx=(0, 3))
        self._btn(row2, "MF +", self._cmd_focus_far, color=C['dim']).pack(side='left', expand=True, fill='x', padx=3)
        self._btn(row2, "MF -", self._cmd_focus_near, color=C['dim']).pack(side='left', expand=True, fill='x', padx=(3, 0))

        row3 = tk.Frame(zoom_inner, bg=C['panel'])
        row3.pack(fill='x', pady=(4, 0))
        self._btn(row3, "LOCK", self._cmd_lock_mode, color=C['danger']).pack(side='left', expand=True, fill='x', padx=(0, 3))
        self._btn(row3, "FOLLOW", self._cmd_follow_mode, color=C['success'], fg='#000').pack(side='left', expand=True, fill='x', padx=3)
        self._btn(row3, "FPV", self._cmd_fpv_mode, color=C['warn'], fg='#000').pack(side='left', expand=True, fill='x', padx=(3, 0))

        row4 = tk.Frame(zoom_inner, bg=C['panel'])
        row4.pack(fill='x', pady=(4, 0))
        self._btn(row4, "REC", self._cmd_record, color=C['danger']).pack(side='left', expand=True, fill='x')

    def _build_video_panel(self, parent):
        C = self.C
        inner = self._section(parent, "VIDEO STREAMS")

        # Camera 1
        tk.Label(inner, text="CAM 1 URL:", bg=C['panel'], fg=C['muted'],
                 font=('Courier New', 7, 'bold')).pack(fill='x', pady=(0, 2))
        self._rtsp_var1 = tk.StringVar(value=DEFAULT_RTSP1)
        tk.Entry(inner, textvariable=self._rtsp_var1,
                 bg=C['border'], fg=C['muted'], insertbackground=C['text'],
                 font=('Courier New', 6), relief='flat', bd=4
                 ).pack(fill='x', pady=(0, 3))

        self._vid_btn1 = self._btn(inner, "CAM 1: START",
                                    self._toggle_stream1, color=C['teal'], fg='#000')
        self._vid_btn1.pack(fill='x', pady=(0, 6))

        # Camera 2
        tk.Label(inner, text="CAM 2 URL:", bg=C['panel'], fg=C['muted'],
                 font=('Courier New', 7, 'bold')).pack(fill='x', pady=(0, 2))
        self._rtsp_var2 = tk.StringVar(value=DEFAULT_RTSP2)
        tk.Entry(inner, textvariable=self._rtsp_var2,
                 bg=C['border'], fg=C['muted'], insertbackground=C['text'],
                 font=('Courier New', 6), relief='flat', bd=4
                 ).pack(fill='x', pady=(0, 3))

        self._vid_btn2 = self._btn(inner, "CAM 2: START",
                                    self._toggle_stream2, color=C['warn'], fg='#000')
        self._vid_btn2.pack(fill='x')

    def _build_info_panel(self, parent):
        C = self.C
        inner = self._section(parent, "DEVICE INFO")

        btn_row = tk.Frame(inner, bg=C['panel'])
        btn_row.pack(fill='x', pady=(0, 5))
        for text, cmd in [("FW VER", self._cmd_fw_ver),
                           ("HW ID",  self._cmd_hw_id),
                           ("CONFIG", self._cmd_config)]:
            self._btn(btn_row, text, cmd, color=C['dim']
                      ).pack(side='left', padx=2, expand=False, fill='x')

        self._info_txt = tk.Text(inner, height=5,
                                  bg=C['border'], fg=C['teal'],
                                  font=('Courier New', 8),
                                  relief='flat', bd=4, state='disabled',
                                  wrap='word')
        self._info_txt.pack(fill='x')

    # ═══════════════════════════════════════════════════════════════
    # CONNECTION
    # ═══════════════════════════════════════════════════════════════

    def _toggle_connect(self):
        C = self.C
        if not self._connected:
            ip   = self._ip_var.get().strip()
            port = int(self._port_var.get().strip())
            self.sdk = GimbalSDK(ip, port)
            if self.sdk.connect():
                self._connected = True
                self._conn_btn.config(text="■  DISCONNECT",
                                       bg=C['danger'], fg=C['text'])
                self._lbl_conn_dot.config(fg=C['success'])
                self._lbl_conn_txt.config(text="ONLINE", fg=C['success'])
                self._set_status(f"Connected  →  {ip}:{port}")
                self._hb_job  = self.root.after(5000, self._heartbeat_loop)
                self._att_job = self.root.after(400,  self._att_loop)
            else:
                self._set_status("Connection failed — check IP/port and network.")
        else:
            self.sdk.disconnect()
            self._connected = False
            self._conn_btn.config(text="▶  CONNECT",
                                   bg=C['success'], fg='#000')
            self._lbl_conn_dot.config(fg=C['danger'])
            self._lbl_conn_txt.config(text="OFFLINE", fg=C['muted'])
            self._set_status("Disconnected.")
            if self._hb_job:
                self.root.after_cancel(self._hb_job)
            if self._att_job:
                self.root.after_cancel(self._att_job)

    def _heartbeat_loop(self):
        if self._connected:
            threading.Thread(target=self.sdk.heartbeat,
                             daemon=True).start()
            self._hb_job = self.root.after(5000, self._heartbeat_loop)

    def _att_loop(self):
        if self._connected:
            threading.Thread(target=self._fetch_att, daemon=True).start()
            self._att_job = self.root.after(400, self._att_loop)

    def _fetch_att(self):
        self.sdk.get_attitude()
        self.root.after(0, self._refresh_att_labels)

    def _refresh_att_labels(self):
        p = self.sdk.pitch_deg
        y = self.sdk.yaw_deg
        r = self.sdk.roll_deg
        self._att_pitch.config(text=f"{p:+7.1f}°")
        self._att_yaw.config(text=f"{y:+7.1f}°")
        self._att_roll.config(text=f"{r:+7.1f}°")
        self._hud1.config(
            text=f"  PITCH  {p:+.1f}°   YAW  {y:+.1f}°   ROLL  {r:+.1f}°  "
        )

    # ═══════════════════════════════════════════════════════════════
    # GIMBAL COMMANDS
    # ═══════════════════════════════════════════════════════════════

    def _require_conn(self) -> bool:
        if not self._connected:
            self._set_status("Not connected. Connect to the gimbal first.")
            return False
        return True

    def _start_pitch(self, direction: int):
        if not self._require_conn():
            return
        self._pitch_dir = direction
        speed = self._speed_var.get() * direction
        threading.Thread(target=self.sdk.rotate,
                         args=(0, speed), daemon=True).start()
        # Highlight active button
        C = self.C
        if direction > 0:
            self._up_btn.config(bg=C['warn'])
        else:
            self._dn_btn.config(bg=C['warn'])

    def _stop_pitch(self):
        C = self.C
        self._up_btn.config(bg=C['accent'])
        self._dn_btn.config(bg=C['accent'])
        if self._connected:
            threading.Thread(target=self.sdk.stop, daemon=True).start()

    def _kb_up_press(self, _=None):  self._start_pitch(1)
    def _kb_up_rel(self,   _=None):  self._stop_pitch()
    def _kb_dn_press(self, _=None):  self._start_pitch(-1)
    def _kb_dn_rel(self,   _=None):  self._stop_pitch()

    def _cmd_center(self):
        if not self._require_conn():
            return
        self._set_status("Centering gimbal…")
        threading.Thread(target=self._do_center, daemon=True).start()

    def _do_center(self):
        ok = self.sdk.center()
        self.root.after(0, self._set_status,
                        "Centered." if ok else "Center command failed.")

    def _cmd_set_angle(self):
        if not self._require_conn():
            return
        ang = self._angle_var.get()
        self._set_status(f"Setting pitch → {ang:.1f}°")
        threading.Thread(target=self._do_set_angle,
                         args=(ang,), daemon=True).start()

    def _do_set_angle(self, ang: float):
        ok = self.sdk.set_angle(ang)
        self.root.after(0, self._set_status,
                        f"Pitch set to {ang:.1f}°" if ok else "Angle command failed.")

    def _cmd_poll_att(self):
        if not self._require_conn():
            return
        threading.Thread(target=self._fetch_att, daemon=True).start()

    def _cmd_zoom_in(self):
        if not self._require_conn():
            return
        self._set_status("Zooming in…")
        threading.Thread(target=self._do_zoom, args=(1,), daemon=True).start()

    def _cmd_zoom_out(self):
        if not self._require_conn():
            return
        self._set_status("Zooming out…")
        threading.Thread(target=self._do_zoom, args=(-1,), daemon=True).start()

    def _do_zoom(self, direction: int):
        ok = self.sdk.zoom(direction)
        self.root.after(0, self._set_status,
                        "Zoom command sent." if ok else "Zoom command failed.")

    def _cmd_auto_focus(self):
        if not self._require_conn():
            return
        self._set_status("Autofocus…")
        threading.Thread(target=self._do_auto_focus, daemon=True).start()

    def _do_auto_focus(self):
        ok = self.sdk.auto_focus()
        self.root.after(0, self._set_status,
                        "Autofocus sent." if ok else "Autofocus failed.")

    def _cmd_focus_near(self):
        if not self._require_conn():
            return
        self._set_status("Manual focus near…")
        threading.Thread(target=self._do_focus, args=(-1,), daemon=True).start()

    def _cmd_focus_far(self):
        if not self._require_conn():
            return
        self._set_status("Manual focus far…")
        threading.Thread(target=self._do_focus, args=(1,), daemon=True).start()

    def _do_focus(self, direction: int):
        ok = self.sdk.manual_focus(direction)
        self.root.after(0, self._set_status,
                        "Focus command sent." if ok else "Focus command failed.")

    def _cmd_record(self):
        if not self._require_conn():
            return
        self._set_status("Toggling record…")
        threading.Thread(target=self._do_record, daemon=True).start()

    def _do_record(self):
        ok = self.sdk.record(True)
        self.root.after(0, self._set_status,
                        "Record command sent." if ok else "Record command failed.")

    def _cmd_lock_mode(self):
        if not self._require_conn():
            return
        self._set_status("Switching to lock mode…")
        threading.Thread(target=self._do_mode, args=("lock",), daemon=True).start()

    def _cmd_follow_mode(self):
        if not self._require_conn():
            return
        self._set_status("Switching to follow mode…")
        threading.Thread(target=self._do_mode, args=("follow",), daemon=True).start()

    def _cmd_fpv_mode(self):
        if not self._require_conn():
            return
        self._set_status("Switching to FPV mode…")
        threading.Thread(target=self._do_mode, args=("fpv",), daemon=True).start()

    def _do_mode(self, mode: str):
        if mode == "lock":
            ok = self.sdk.lock_mode()
        elif mode == "follow":
            ok = self.sdk.follow_mode()
        else:
            ok = self.sdk.fpv_mode()
        self.root.after(0, self._set_status,
                        f"{mode.upper()} mode sent." if ok else f"{mode.upper()} mode failed.")

    # ═══════════════════════════════════════════════════════════════
    # DEVICE INFO COMMANDS
    # ═══════════════════════════════════════════════════════════════

    def _log(self, text: str):
        self._info_txt.config(state='normal')
        self._info_txt.delete('1.0', 'end')
        self._info_txt.insert('end', text)
        self._info_txt.config(state='disabled')

    def _cmd_fw_ver(self):
        if not self._require_conn():
            return
        self._log("Querying…")
        def _q():
            r = self.sdk.get_firmware_version()
            if r:
                msg = (f"Camera FW : {r['camera_fw']}\n"
                       f"Gimbal FW : {r['gimbal_fw']}")
            else:
                msg = "No response from gimbal."
            self.root.after(0, self._log, msg)
        threading.Thread(target=_q, daemon=True).start()

    def _cmd_hw_id(self):
        if not self._require_conn():
            return
        self._log("Querying…")
        def _q():
            hw = self.sdk.get_hardware_id()
            self.root.after(0, self._log,
                            f"HW ID: {hw}" if hw else "No HW ID response.")
        threading.Thread(target=_q, daemon=True).start()

    def _cmd_config(self):
        if not self._require_conn():
            return
        self._log("Querying…")
        def _q():
            cfg = self.sdk.get_config()
            if cfg:
                msg = (f"Mounting : {cfg['mounting']}\n"
                       f"Mode     : {cfg['motion_mode']}\n"
                       f"HDR      : {'ON' if cfg['hdr_on'] else 'OFF'}")
            else:
                msg = "No config response."
            self.root.after(0, self._log, msg)
        threading.Thread(target=_q, daemon=True).start()

    # ═══════════════════════════════════════════════════════════════
    # VIDEO STREAM
    # ═══════════════════════════════════════════════════════════════

    def _toggle_stream1(self):
        """Toggle Camera 1 stream."""
        C = self.C
        if not self._streaming1:
            url = self._rtsp_var1.get().strip()
            if not HAVE_CV2:
                self._set_status("OpenCV not installed — run: pip install opencv-python")
                return
            self.stream1 = self._make_stream(url, "CAM 1")
            self.stream1.start()
            self._streaming1 = True
            self._vid_btn1.config(text="CAM 1: STOP",
                                   bg=C['danger'], fg=C['text'])
            self._set_status(f"CAM 1 connecting to  {url}")
        else:
            self.stream1.stop()
            self._streaming1 = False
            self._vid_btn1.config(text="CAM 1: START",
                                   bg=C['teal'], fg='#000')
            self._video_lbl1.config(image='',
                                     text="[ CAM 1 ]\nStart stream")
            self._video_lbl1.image = None
            self._fps_lbl1.config(text="")
            self._set_status("CAM 1 stopped.")

    def _toggle_stream2(self):
        """Toggle Camera 2 stream."""
        C = self.C
        if not self._streaming2:
            url = self._rtsp_var2.get().strip()
            if not HAVE_CV2:
                self._set_status("OpenCV not installed — run: pip install opencv-python")
                return
            self.stream2 = self._make_stream(url, "CAM 2")
            self.stream2.start()
            self._streaming2 = True
            self._vid_btn2.config(text="CAM 2: STOP",
                                   bg=C['danger'], fg=C['text'])
            self._set_status(f"CAM 2 connecting to  {url}")
        else:
            self.stream2.stop()
            self._streaming2 = False
            self._vid_btn2.config(text="CAM 2: START",
                                   bg=C['warn'], fg='#000')
            self._video_lbl2.config(image='',
                                     text="[ CAM 2 ]\nStart stream")
            self._video_lbl2.image = None
            self._fps_lbl2.config(text="")
            self._set_status("CAM 2 stopped.")

    def _tick_video(self):
        """Called every ~33 ms to refresh both video labels."""
        def _render_frame(frame, stream, label_widget, fps_widget, hud_widget, frame_box, hud_text):
            if frame is None:
                return
            lw = frame_box.winfo_width()
            lh = frame_box.winfo_height()
            if lw > 10 and lh > 10:
                if HAVE_CV2 and getattr(stream, 'color_space', 'bgr') == 'bgr':
                    h, w = frame.shape[:2]
                    scale = min(lw / w, lh / h)
                    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
                    frame = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
                    image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                else:
                    image = Image.fromarray(frame)
                    image.thumbnail((lw, lh))
            else:
                if HAVE_CV2 and getattr(stream, 'color_space', 'bgr') == 'bgr':
                    image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                else:
                    image = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=image)
            label_widget.config(image=imgtk, text='', bg='#000')
            label_widget.image = imgtk
            fps_widget.config(text=f"{stream.fps_actual:.0f} fps")
            hud_widget.config(text=hud_text)

        # Render Camera 1
        if self._streaming1 and self.stream1.running:
            frame = self.stream1.get_frame()
            _render_frame(frame, self.stream1, self._video_lbl1, self._fps_lbl1,
                          self._hud1, self._video_frame1, "  CAM 1")

        # Render Camera 2
        if self._streaming2 and self.stream2.running:
            frame = self.stream2.get_frame()
            _render_frame(frame, self.stream2, self._video_lbl2, self._fps_lbl2,
                          self._hud2, self._video_frame2, "  CAM 2")

        self.root.after(33, self._tick_video)

    # ═══════════════════════════════════════════════════════════════
    # STATUS & MISC
    # ═══════════════════════════════════════════════════════════════

    def _tick_status(self):
        """Periodic status ping (connectivity indicator in title)."""
        if self._connected:
            pass  # Attitude loop already running
        self.root.after(2000, self._tick_status)

    def _set_status(self, msg: str):
        ts = time.strftime('%H:%M:%S')
        self._status_var.set(f"[{ts}]  {msg}")

    def _on_close(self):
        self.stream1.stop()
        self.stream2.stop()
        if self._connected:
            self.sdk.stop()
            self.sdk.disconnect()
        self.root.destroy()


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SIYI A2 mini gimbal control station")
    parser.add_argument("--ip", default=DEFAULT_IP, help="gimbal IP address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="gimbal UDP port")
    parser.add_argument("--rtsp1", default=DEFAULT_RTSP1, help="camera 1 RTSP URL")
    parser.add_argument("--rtsp2", default=DEFAULT_RTSP2, help="camera 2 RTSP URL")
    parser.add_argument("--mock", action="store_true", help="run without real gimbal hardware")
    parser.add_argument("--demo-streams", action="store_true", help="use generated video frames instead of RTSP")
    parser.add_argument("--headless", action="store_true", help="run without opening the GUI")
    return parser


def run_headless_smoke_test(args: argparse.Namespace) -> int:
    sdk = GimbalSDK(args.ip, args.port, mock=args.mock)
    if not sdk.connect():
        print("connect failed")
        return 1
    print(f"connected to {args.ip}:{args.port} mock={args.mock}")
    print("firmware:", sdk.get_firmware_version())
    print("hardware:", sdk.get_hardware_id())
    print("config:", sdk.get_config())
    print("attitude before:", sdk.pitch_deg, sdk.yaw_deg, sdk.roll_deg)
    sdk.rotate(0, 25)
    time.sleep(0.1)
    sdk.get_attitude()
    print("attitude after rotate:", sdk.pitch_deg, sdk.yaw_deg, sdk.roll_deg, sdk.pitch_vel)
    sdk.center()
    print("attitude after center:", sdk.pitch_deg, sdk.yaw_deg, sdk.roll_deg)
    sdk.disconnect()
    return 0


# ═══════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    args = build_argument_parser().parse_args()
    if args.headless:
        raise SystemExit(run_headless_smoke_test(args))

    root = tk.Tk()
    app  = App(root, ip=args.ip, port=args.port,
               rtsp1=args.rtsp1, rtsp2=args.rtsp2,
               mock=args.mock, demo_streams=args.demo_streams)
    root.mainloop()