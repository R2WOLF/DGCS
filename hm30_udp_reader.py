#!/usr/bin/env python3
"""
SIYI HM30 — Battery Voltage & Signal Quality Monitor
Reads from HM30 Datalink SDK over UDP at 10 Hz.

Displays:
  • Ground unit battery voltage (CMD 0x16)
  • Signal quality: RSSI (dBm), signal strength (%), packet loss (%)

Hardware: PC Ethernet → HM30 LAN Port (RJ45)
PC IP: 192.168.144.30 / Mask 255.255.255.0
HM30 must have Datalink Mode = UDP in settings.

Reference: HM30 User Manual v1.7 §4.6 (SIYI Datalink SDK)
"""

import socket, struct, threading, time, sys
from dataclasses import dataclass, field
from typing import Optional

# ─────────────────────────────────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────────────────────────────────
HM30_IP   = "192.168.144.12"
HM30_PORT = 37260
LOCAL_PORT = 37261
READ_HZ = 10

# Protocol
# STX is defined in the spec as 0x6655 (low byte first on the wire -> 0x55 0x66)
STX = 0x6655
CMD_SYS_SETTINGS = 0x16   # Battery voltage
CMD_LINK_STATUS  = 0x43   # Packet loss
CMD_IMG_STATUS   = 0x44   # Signal strength, RSSI

# CRC16 table (HM30 Manual §4.8.4)
_CRC16_TAB = [
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

def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ _CRC16_TAB[(crc >> 8) ^ b]) & 0xFFFF
    return crc

_seq = 0
def build_packet(cmd_id: int, data: bytes = b"", seq: Optional[int] = None) -> bytes:
    """Build a protocol packet. If `seq` is None the internal counter `_seq` is used and incremented.
    For deterministic self-tests pass `seq=0` explicitly to avoid incremental sequence changes.
    """
    global _seq
    ctrl = 0x01
    if seq is None:
        seq = _seq & 0xFFFF
        _seq = (_seq + 1) & 0xFFFF

    # Pack: STX(2) handled by caller when needed; here we pack header fields (CTRL, DATA_LEN, SEQ, CMD)
    header = struct.pack("<BHHB", ctrl, len(data), seq & 0xFFFF, cmd_id)
    payload = header + data
    # Prepend STX in wire order (little-endian of 0x6655 produces bytes 0x55 0x66)
    packet = struct.pack("<H", STX) + payload
    return packet + struct.pack("<H", crc16(packet))

@dataclass
class Pkt:
    cmd_id: int
    data: bytes
    crc_ok: bool

def parse_pkt(raw: bytes) -> Optional[Pkt]:
    if len(raw) < 10:
        return None
    stx, ctrl, dlen, seq, cmd = struct.unpack_from("<HBHHB", raw, 0)
    # Minimum total length = STX(2)+CTRL(1)+DATA_LEN(2)+SEQ(2)+CMD(1)+DATA(dlen-1)+CRC(2)
    if stx != STX or len(raw) < (9 + dlen):
        return None
    data = raw[8:8 + (dlen - 1)] if dlen > 1 else b""
    crc_rx, = struct.unpack_from("<H", raw, 8 + (dlen - 1))
    # CRC is computed over CTRL..DATA (exclude STX), i.e. bytes[2: 8 + (dlen-1)]
    crc_ok = crc_rx == crc16(raw[2:8 + (dlen - 1)])
    return Pkt(cmd, data, crc_ok)

@dataclass
class State:
    bat_v:    float = 0.0      # Volts (from CMD 0x16, rc_bat × 0.1V)
    rssi:     int = 0          # dBm (from CMD 0x44)
    signal:   int = 0          # % (from CMD 0x44)
    loss:     int = 0          # % (from CMD 0x43)
    rx_cnt:   int = 0
    crc_err:  int = 0
    last_t:   float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)

state = State()

def dispatch(pkt: Pkt):
    with state.lock:
        state.rx_cnt += 1
        if not pkt.crc_ok:
            state.crc_err += 1
            return
        state.last_t = time.time()
        
        if pkt.cmd_id == CMD_SYS_SETTINGS and len(pkt.data) >= 4:
            # Byte 3 is rc_bat (battery level × 0.1V)
            bat_raw = pkt.data[3]
            state.bat_v = bat_raw * 0.1
        
        elif pkt.cmd_id == CMD_LINK_STATUS and len(pkt.data) >= 3:
            # Byte 2 is packet loss rate (%)
            state.loss = pkt.data[2]
        
        elif pkt.cmd_id == CMD_IMG_STATUS and len(pkt.data) >= 36:
            # Unpack 9 × int32: signal(0), inactive(1), up(2), down(3),
            # txbw(4), rxbw(5), rssi(6), freq(7), ch(8)
            vals = struct.unpack_from("<9i", pkt.data, 0)
            state.signal = vals[0]
            state.rssi = vals[6]

def rx_thread(sock: socket.socket):
    while True:
        try:
            raw, _ = sock.recvfrom(4096)
            pkt = parse_pkt(raw)
            if pkt:
                dispatch(pkt)
        except socket.timeout:
            continue
        except OSError:
            break

def tx_thread(sock: socket.socket, dest):
    while True:
        t0 = time.time()
        try:
            sock.sendto(build_packet(CMD_SYS_SETTINGS), dest)
            sock.sendto(build_packet(CMD_LINK_STATUS),  dest)
            sock.sendto(build_packet(CMD_IMG_STATUS),   dest)
        except OSError:
            break
        time.sleep(max(0, 1.0/READ_HZ - (time.time() - t0)))

# Colors
B = "\033[1m"
R = "\033[0m"
G = "\033[92m"
Y = "\033[93m"
RD = "\033[91m"
C = "\033[96m"

def _health(val: int, warn: int, crit: int) -> str:
    """Color code: green if OK, yellow if warning, red if critical."""
    if val >= warn:
        return G
    elif val >= crit:
        return Y
    else:
        return RD

def _bat_health(v: float) -> str:
    """Battery: red <7.0V, yellow <7.5V, green ok."""
    if v < 7.0:
        return RD
    elif v < 7.5:
        return Y
    else:
        return G

def _bar(val: int, lo: int = -100, hi: int = 0) -> str:
    """Bar for RSSI (lo=-100 dBm, hi=0)."""
    frac = max(0, min(1, (val - lo) / (hi - lo)))
    return "█" * int(frac * 20) + "░" * int((1-frac) * 20)

def print_thread():
    interval = 1.0 / READ_HZ
    while True:
        t0 = time.time()
        with state.lock:
            bat_v = state.bat_v
            rssi = state.rssi
            sig = state.signal
            loss = state.loss
            age = time.time() - state.last_t if state.last_t else 999
            rx = state.rx_cnt
            err = state.crc_err
        
        # Age color
        age_col = G if age < 0.5 else (Y if age < 2 else RD)
        
        out = []
        out.append("\033[2J\033[H")  # clear
        out.append(f"{B}{C}┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓{R}")
        out.append(f"{B}{C}┃  SIYI HM30 — Battery & Signal @ {READ_HZ} Hz     ┃{R}")
        out.append(f"{B}{C}┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛{R}")
        out.append("")
        
        # Battery
        bat_col = _bat_health(bat_v)
        out.append(f"  {B}BATTERY{R}")
        out.append(f"    Voltage:  {bat_col}{bat_v:.2f} V{R}  [target: 7.4V–12.6V]")
        if bat_v == 0:
            out.append(f"    Status:   {Y}waiting for data…{R}")
        elif bat_v < 6.5:
            out.append(f"    Status:   {RD}CRITICALLY LOW — Land immediately{R}")
        elif bat_v < 7.2:
            out.append(f"    Status:   {Y}Low — return to base{R}")
        elif bat_v >= 7.2:
            out.append(f"    Status:   {G}OK{R}")
        
        out.append("")
        out.append(f"  {B}SIGNAL QUALITY{R}")
        
        # Signal strength
        sig_col = _health(sig, 70, 40)
        out.append(f"    Strength:  {sig_col}{sig:3d} %{R}  {sig_col}[{_bar(sig, 0, 100)}]{R}")
        
        # RSSI
        rssi_col = _health(rssi, -60, -80)
        out.append(f"    RSSI:      {rssi_col}{rssi:4d} dBm{R}  {rssi_col}[{_bar(rssi)}]{R}")
        
        # Packet loss
        loss_col = _health(100 - loss, 95, 80)
        out.append(f"    Loss:      {loss_col}{loss:3d} %{R}   [{loss_col}{'█'*int(loss/5)}{'░'*int((100-loss)/5)}{R}]")
        
        out.append("")
        out.append(f"  {B}LINK STATUS{R}")
        out.append(f"    Data age:  {age_col}{age*1000:6.0f} ms{R}")
        out.append(f"    RX pkts:   {rx}   |   CRC errors: {err}")
        
        out.append("")
        out.append(f"  Target: {HM30_IP}:{HM30_PORT}   " + 
                   f"{Y}Press Ctrl+C to quit{R}")
        
        print("\n".join(out), end="", flush=True)
        time.sleep(max(0, interval - (time.time() - t0)))

def main():
    print(f"Connecting to {HM30_IP}:{HM30_PORT} …")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", LOCAL_PORT))
    sock.settimeout(1.0)
    
    dest = (HM30_IP, HM30_PORT)
    
    threading.Thread(target=rx_thread, args=(sock,), daemon=True).start()
    threading.Thread(target=tx_thread, args=(sock, dest), daemon=True).start()
    threading.Thread(target=print_thread, daemon=True).start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nStopped. Bye.\n")
    finally:
        sock.close()
        sys.exit(0)

if __name__ == "__main__":
    main()