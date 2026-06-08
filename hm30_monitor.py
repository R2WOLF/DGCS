#!/usr/bin/env python3
"""
HM30 / SIYI UDP telemetry monitor.

What it does
- Sends SIYI Datalink SDK request frames over UDP.
- Reads:
  - Ground unit battery voltage from CMD 0x16
  - Image transmission signal strength from CMD 0x44
- Verifies SIYI CRC16 exactly as documented (CRC-16/CCITT, init 0x0000).
- Exposes a reusable Python API and a terminal CLI.

Official protocol points used here:
- Frame format: STX(0x5566), CTRL, Data_len, SEQ, CMD_ID, DATA, CRC16, low byte first.
- CMD 0x16: Request Remote Controller System Settings -> includes Rc_bat (Ground Unit Battery Level x 10V).
- CMD 0x44: Request Image Transmission Link Status -> includes signal percentage and RSSI.
- UDP datalink on HM30 ground unit is configured through LAN/Wi‑Fi; the manual's QGC example uses port 19856 and server address 192.168.144.12.

This file is intentionally self-contained so it can be imported by other programs.
"""

from __future__ import annotations

import argparse
import dataclasses
import socket
import struct
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple


STX = 0x5566
CTRL_NEED_ACK = 0x01
CTRL_ACK_PACK = 0x02

CMD_REMOTE_CONTROLLER_SYSTEM_SETTINGS = 0x16  # Rc_bat is here
CMD_IMAGE_TRANSMISSION_LINK_STATUS = 0x44     # signal / rssi here

DEFAULT_UDP_PORT = 19856
DEFAULT_TIMEOUT_S = 0.5
DEFAULT_RETRIES = 3


class SiyiProtocolError(Exception):
    """Raised when a frame is malformed or fails CRC verification."""


def crc16_siyi(data: bytes) -> int:
    """
    SIYI CRC16 implementation.

    This matches the code shown in the manual:
        crc = 0
        temp = (crc >> 8) & 0xff
        oldcrc16 = crc16_tab[*ptr ^ temp]
        crc = (crc << 8) ^ oldcrc16

    Equivalent to CRC-16/CCITT with polynomial 0x1021 and init 0x0000.
    """
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(cmd_id: int, payload: bytes = b"", seq: int = 0, ctrl: int = CTRL_NEED_ACK) -> bytes:
    """
    Build a SIYI frame.
    """
    if not 0 <= cmd_id <= 0xFF:
        raise ValueError("cmd_id must fit in one byte")
    if not 0 <= seq <= 0xFFFF:
        raise ValueError("seq must fit in 16 bits")
    if not 0 <= ctrl <= 0xFF:
        raise ValueError("ctrl must fit in one byte")
    if len(payload) > 0xFFFF:
        raise ValueError("payload too large")

    header = struct.pack(
        "<H B H H B",
        STX,
        ctrl,
        len(payload),
        seq,
        cmd_id,
    )
    crc = crc16_siyi(header + payload)
    return header + payload + struct.pack("<H", crc)


@dataclass(frozen=True)
class SystemSettings:
    """Parsed response to CMD 0x16."""
    bind_state: int
    baud_type: int
    joy_type: int
    rc_bat_x10v: int

    @property
    def battery_voltage_v(self) -> float:
        return self.rc_bat_x10v / 10.0


@dataclass(frozen=True)
class LinkStatus:
    """Parsed response to CMD 0x44."""
    signal_percent: int
    inactive_time: int
    upstream_bps: int
    downstream_bps: int
    tx_bandwidth_mbps: float
    rx_bandwidth_mbps: float
    rssi_dbm: int
    frequency_mhz: int
    channel: int


@dataclass(frozen=True)
class PollResult:
    battery: Optional[SystemSettings] = None
    link: Optional[LinkStatus] = None


class SiyiUdpClient:
    """
    Reusable UDP client for SIYI HM30 datalink telemetry.
    """

    def __init__(
        self,
        host: str,
        port: int = DEFAULT_UDP_PORT,
        local_bind: Tuple[str, int] = ("0.0.0.0", 0),
        timeout: float = DEFAULT_TIMEOUT_S,
        retries: int = DEFAULT_RETRIES,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self.retries = retries
        self.seq = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(local_bind)
        self.sock.settimeout(timeout)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    def __enter__(self) -> "SiyiUdpClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _next_seq(self) -> int:
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFFFF
        return seq

    def _send_request(self, cmd_id: int, payload: bytes = b"") -> None:
        frame = build_frame(cmd_id, payload, seq=self._next_seq(), ctrl=CTRL_NEED_ACK)
        self.sock.sendto(frame, (self.host, self.port))

    @staticmethod
    def parse_frame(frame: bytes) -> Tuple[int, int, int, int, bytes]:
        """
        Returns: (ctrl, data_len, seq, cmd_id, payload)
        """
        if len(frame) < 10:
            raise SiyiProtocolError(f"Frame too short: {len(frame)} bytes")

        stx, ctrl, data_len, seq, cmd_id = struct.unpack_from("<H B H H B", frame, 0)
        if stx != STX:
            raise SiyiProtocolError(f"Bad STX: 0x{stx:04X}")

        expected_len = 2 + 1 + 2 + 2 + 1 + data_len + 2
        if len(frame) < expected_len:
            raise SiyiProtocolError(
                f"Truncated frame: got {len(frame)} bytes, expected {expected_len}"
            )

        payload = frame[8 : 8 + data_len]
        recv_crc = struct.unpack_from("<H", frame, 8 + data_len)[0]
        calc_crc = crc16_siyi(frame[: 8 + data_len])
        if recv_crc != calc_crc:
            raise SiyiProtocolError(
                f"CRC mismatch for cmd 0x{cmd_id:02X}: recv=0x{recv_crc:04X}, calc=0x{calc_crc:04X}"
            )

        return ctrl, data_len, seq, cmd_id, payload

    def _recv_matching(self, expected_cmd: int) -> bytes:
        """
        Read packets until the expected command response is received.
        """
        deadline = time.monotonic() + self.timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"Timeout waiting for response cmd 0x{expected_cmd:02X}")

            self.sock.settimeout(remaining)
            frame, _addr = self.sock.recvfrom(2048)
            ctrl, _data_len, _seq, cmd_id, payload = self.parse_frame(frame)

            # Ack packets carry CTRL bit 1 set; response CMD_ID should match request.
            if cmd_id == expected_cmd and (ctrl & CTRL_ACK_PACK):
                return payload

    def request_system_settings(self) -> SystemSettings:
        """
        CMD 0x16: request remote controller system settings.
        """
        self._send_request(CMD_REMOTE_CONTROLLER_SYSTEM_SETTINGS)
        payload = self._recv_matching(CMD_REMOTE_CONTROLLER_SYSTEM_SETTINGS)

        if len(payload) < 4:
            raise SiyiProtocolError(f"CMD 0x16 payload too short: {len(payload)}")

        bind_state, baud_type, joy_type, rc_bat_x10v = struct.unpack_from("<BBBB", payload, 0)
        return SystemSettings(
            bind_state=bind_state,
            baud_type=baud_type,
            joy_type=joy_type,
            rc_bat_x10v=rc_bat_x10v,
        )

    def request_link_status(self) -> LinkStatus:
        """
        CMD 0x44: request image transmission link status.
        """
        self._send_request(CMD_IMAGE_TRANSMISSION_LINK_STATUS)
        payload = self._recv_matching(CMD_IMAGE_TRANSMISSION_LINK_STATUS)

        if len(payload) < 32:
            raise SiyiProtocolError(f"CMD 0x44 payload too short: {len(payload)}")

        # 8 x int32 little-endian per manual:
        # signal, inactive_time, upstream, downstream, txbandwidth, rxbandwidth, rssi, freq, channel
        vals = struct.unpack_from("<iiiiiiii", payload, 0)
        signal_percent, inactive_time, upstream_bps, downstream_bps, txbw_raw, rxbw_raw, rssi_dbm, freq_mhz = vals
        # The manual shows channel too; some payloads may include it after these eight fields.
        channel = 0
        if len(payload) >= 36:
            channel = struct.unpack_from("<i", payload, 32)[0]

        return LinkStatus(
            signal_percent=signal_percent,
            inactive_time=inactive_time,
            upstream_bps=upstream_bps,
            downstream_bps=downstream_bps,
            tx_bandwidth_mbps=txbw_raw / 1000.0,
            rx_bandwidth_mbps=rxbw_raw / 1000.0,
            rssi_dbm=rssi_dbm,
            frequency_mhz=freq_mhz,
            channel=channel,
        )

    def poll_once(self) -> PollResult:
        return PollResult(
            battery=self.request_system_settings(),
            link=self.request_link_status(),
        )

    def watch(
        self,
        interval_s: float = 1.0,
        on_update: Optional[Callable[[PollResult], None]] = None,
    ) -> None:
        """
        Continuous polling loop.
        """
        while True:
            result = self.poll_once()
            if on_update is not None:
                on_update(result)
            else:
                print(format_result(result))
            time.sleep(interval_s)


def format_result(result: PollResult) -> str:
    parts = []
    if result.battery is not None:
        parts.append(
            f"Battery: {result.battery.battery_voltage_v:.1f} V (raw {result.battery.rc_bat_x10v})"
        )
        parts.append(
            f"Bind: {result.battery.bind_state}, Baud: {result.battery.baud_type}, Joy: {result.battery.joy_type}"
        )
    if result.link is not None:
        parts.append(
            f"Signal: {result.link.signal_percent}% | RSSI: {result.link.rssi_dbm} dBm | "
            f"Freq: {result.link.frequency_mhz} MHz | Channel: {result.link.channel}"
        )
        parts.append(
            f"Up: {result.link.upstream_bps} B/s | Down: {result.link.downstream_bps} B/s | "
            f"Tx: {result.link.tx_bandwidth_mbps:.3f} Mbps | Rx: {result.link.rx_bandwidth_mbps:.3f} Mbps"
        )
    return "\n".join(parts)


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="HM30 / SIYI UDP battery + signal monitor")
    p.add_argument("--ip", default="192.168.144.12", help="HM30 ground unit IP address")
    p.add_argument("--port", type=int, default=DEFAULT_UDP_PORT, help="UDP port (manual example uses 19856)")
    p.add_argument("--interval", type=float, default=1.0, help="Polling interval in seconds")
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="Socket timeout in seconds")
    p.add_argument("--retries", type=int, default=DEFAULT_RETRIES, help="Retries for each request")
    p.add_argument("--once", action="store_true", help="Poll once and exit")
    return p


def main() -> int:
    args = _build_arg_parser().parse_args()

    with SiyiUdpClient(
        host=args.ip,
        port=args.port,
        timeout=args.timeout,
        retries=args.retries,
    ) as client:
        if args.once:
            result = client.poll_once()
            print(format_result(result))
            return 0

        def printer(result: PollResult) -> None:
            # Clear, single-screen output without depending on curses.
            print("\033[2J\033[H", end="")
            print(format_result(result))
            print()
            print("Ctrl+C to stop")

        try:
            client.watch(interval_s=args.interval, on_update=printer)
        except KeyboardInterrupt:
            return 0
        except Exception as e:
            print(f"Error: {e}")
            return 1


if __name__ == "__main__":
    raise SystemExit(main())
