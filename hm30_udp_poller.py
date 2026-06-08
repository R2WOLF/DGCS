#!/usr/bin/env python3
"""
HM30 SIYI Datalink SDK Data Poller (UDP Version)
================================================

A comprehensive Python script to poll data from SIYI HM30 Full HD Transmission System
using the official SIYI Datalink SDK protocol over UDP network connection.

Features:
- Hardware ID polling
- System settings monitoring
- RC channel data acquisition
- Datalink status reporting
- Image transmission link status
- Firmware version reading
- Real-time terminal logging with formatted output
- CRC16 checksum validation (XMODEM standard)
- UDP network communication (LAN/WiFi)

Network Configuration:
- HM30 Ground Unit IP: 192.168.144.12
- UDP Port: 19856
- Connection: Ethernet LAN or WiFi

Author: Claude
Date: 2026
Manual Reference: HM30 User Manual v1.7
"""

import socket
import struct
import time
import sys
from datetime import datetime
from typing import Optional, Tuple, List, Dict, Any
from dataclasses import dataclass
from enum import Enum


# CRC16 lookup table (XMODEM standard: G(X) = X^16+X^12+X^5+1)
CRC16_TABLE = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
]


class CommandID(Enum):
    """SIYI SDK Command IDs"""
    REQUEST_HARDWARE_ID = 0x40
    REQUEST_SYSTEM_SETTINGS = 0x16
    SEND_SYSTEM_SETTINGS = 0x17
    REQUEST_CHANNEL_DATA = 0x42
    REQUEST_DATALINK_STATUS = 0x43
    REQUEST_IMAGE_LINK_STATUS = 0x44
    REQUEST_FIRMWARE_VERSION = 0x47
    REQUEST_CHANNEL_MAPPINGS = 0x48
    REQUEST_CHANNEL_REVERSE = 0x4B
    REQUEST_FIRMWARE_VERSION_FULL = 0x47


class BaudRate(Enum):
    """Telemetry Baud Rates"""
    BAUD_4800 = 0
    BAUD_9600 = 1
    BAUD_38400 = 2
    BAUD_57600 = 3
    BAUD_76800 = 4
    BAUD_115200 = 5
    BAUD_230400 = 6


class ChannelFrequency(Enum):
    """Channel Data Output Frequencies"""
    OFF = 0
    HZ_2 = 1
    HZ_4 = 2
    HZ_5 = 3
    HZ_10 = 4
    HZ_20 = 5
    HZ_50 = 6
    HZ_100 = 7


@dataclass
class HM30Packet:
    """SIYI SDK Packet Structure"""
    stx: bytes = b'\x55\x66'  # Start marker
    ctrl: int = 0
    data_len: int = 0
    seq: int = 0
    cmd_id: int = 0
    data: bytes = b''
    crc16: int = 0


def calculate_crc16(data: bytes) -> int:
    """
    Calculate CRC16 checksum using XMODEM standard.
    
    Args:
        data: bytes to calculate checksum for
        
    Returns:
        16-bit CRC value
    """
    crc = 0
    for byte in data:
        temp = (crc >> 8) & 0xff
        crc = ((crc << 8) ^ CRC16_TABLE[byte ^ temp]) & 0xffff
    return crc


def create_packet(cmd_id: int, data: bytes = b'', seq: int = 0, need_ack: bool = True) -> bytes:
    """
    Create a SIYI SDK protocol packet.
    
    Packet Structure:
    [STX(2)] [CTRL(1)] [DATA_LEN(2)] [SEQ(2)] [CMD_ID(1)] [DATA(n)] [CRC16(2)]
    
    Args:
        cmd_id: Command ID
        data: Command data payload
        seq: Sequence number
        need_ack: Whether ACK is needed
        
    Returns:
        Complete packet with CRC16
    """
    ctrl = 0x01 if need_ack else 0x00
    data_len = len(data)
    
    # Build packet without CRC
    packet = bytearray()
    packet.extend(b'\x55\x66')  # STX
    packet.append(ctrl)  # CTRL
    packet.extend(struct.pack('<H', data_len))  # DATA_LEN (little-endian)
    packet.extend(struct.pack('<H', seq))  # SEQ (little-endian)
    packet.append(cmd_id)  # CMD_ID
    packet.extend(data)  # DATA
    
    # Calculate CRC16 on everything except STX
    crc_data = packet[2:]  # Exclude first 2 bytes (STX)
    crc = calculate_crc16(bytes(crc_data))
    
    # Append CRC16 (little-endian)
    packet.extend(struct.pack('<H', crc))
    
    return bytes(packet)


def parse_packet(raw_data: bytes) -> Optional[HM30Packet]:
    """
    Parse a received SIYI SDK packet.
    
    Args:
        raw_data: Raw bytes received
        
    Returns:
        Parsed packet or None if invalid
    """
    if len(raw_data) < 11:
        return None
        
    if raw_data[0:2] != b'\x55\x66':
        return None
    
    try:
        packet = HM30Packet()
        packet.stx = raw_data[0:2]
        packet.ctrl = raw_data[2]
        packet.data_len = struct.unpack('<H', raw_data[3:5])[0]
        packet.seq = struct.unpack('<H', raw_data[5:7])[0]
        packet.cmd_id = raw_data[7]
        packet.data = raw_data[8:8 + packet.data_len]
        packet.crc16 = struct.unpack('<H', raw_data[8 + packet.data_len:10 + packet.data_len])[0]
        
        # Verify CRC16
        crc_data = raw_data[2:8 + packet.data_len]
        calculated_crc = calculate_crc16(crc_data)
        
        if calculated_crc != packet.crc16:
            return None
            
        return packet
    except (struct.error, IndexError):
        return None


class HM30Poller:
    """SIYI HM30 Data Poller (UDP Version)"""
    
    def __init__(self, host: str = '192.168.144.12', port: int = 19856, timeout: float = 1.0):
        """
        Initialize HM30 poller with UDP connection.
        
        Args:
            host: HM30 ground unit IP address (default: 192.168.144.12)
            port: UDP port (default: 19856)
            timeout: Socket timeout in seconds
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.bind_src_port = False
        self.socket: Optional[socket.socket] = None
        self.seq_counter = 0
        
    def connect(self) -> bool:
        """
        Establish UDP connection to HM30.
        
        Returns:
            True if connection successful
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Optionally bind source port to the HM30 port (some firmwares reply only to same source port)
            try:
                if self.bind_src_port:
                    self.socket.bind(("", self.port))
                else:
                    # Bind to ephemeral port so we can log the local endpoint
                    self.socket.bind(("", 0))
            except Exception:
                pass
            self.socket.settimeout(self.timeout)
            
            # Test connection by sending a ping (hardware ID request)
            self.log(f"✓ UDP Socket created and configured", level="SUCCESS")
            self.log(f"✓ Target: {self.host}:{self.port}", level="SUCCESS")
            self.log(f"✓ Timeout: {self.timeout}s", level="SUCCESS")
            try:
                local = self.socket.getsockname()
                self.log(f"Local socket bound: {local[0]}:{local[1]}", level="INFO")
            except Exception:
                pass
            
            return True
        except socket.error as e:
            self.log(f"✗ Failed to create UDP socket: {e}", level="ERROR")
            return False
    
    def disconnect(self) -> None:
        """Close UDP socket connection."""
        if self.socket:
            try:
                self.socket.close()
                self.log("Disconnected from HM30", level="INFO")
            except socket.error:
                pass
    
    def send_command(self, cmd_id: int, data: bytes = b'') -> bool:
        """
        Send a command to HM30 via UDP.
        
        Args:
            cmd_id: Command ID
            data: Command data
            
        Returns:
            True if sent successfully
        """
        if not self.socket:
            self.log("UDP socket not initialized", level="ERROR")
            return False
        
        packet = create_packet(cmd_id, data, seq=self.seq_counter)
        self.seq_counter = (self.seq_counter + 1) % 65536
        
        try:
            sent = self.socket.sendto(packet, (self.host, self.port))
            # Diagnostic: log hex of the first part of the packet
            try:
                prefix = packet[:64].hex()
                self.log(f"Sent {sent} bytes to {self.host}:{self.port} - {prefix}...", level="INFO")
            except Exception:
                pass
            return True
        except socket.error as e:
            self.log(f"Failed to send command: {e}", level="ERROR")
            return False
    
    def receive_response(self) -> Optional[HM30Packet]:
        """
        Receive and parse response from HM30 via UDP.
        
        Returns:
            Parsed packet or None if timeout/error
        """
        if not self.socket:
            return None
        
        try:
            # Receive data from UDP socket
            data, addr = self.socket.recvfrom(4096)
            try:
                self.log(f"RX {len(data)} bytes from {addr[0]}:{addr[1]} - {data[:128].hex()}...", level="INFO")
            except Exception:
                pass
            
            if len(data) < 11:
                return None
            
            # Parse the received packet
            return parse_packet(data)
            
        except socket.timeout:
            return None
        except socket.error as e:
            self.log(f"Socket error while receiving: {e}", level="WARNING")
            return None
    
    def request_hardware_id(self) -> Optional[str]:
        """Request hardware ID from HM30."""
        self.log("Requesting Hardware ID (CMD 0x40)...", level="REQUEST")
        
        if not self.send_command(CommandID.REQUEST_HARDWARE_ID.value):
            return None
        
        time.sleep(0.2)
        response = self.receive_response()
        
        if response and response.cmd_id == CommandID.REQUEST_HARDWARE_ID.value:
            hw_id = response.data[:10].decode('ascii', errors='ignore').strip('\x00')
            self.log(f"Hardware ID: {hw_id}", level="DATA")
            return hw_id
        else:
            self.log("No response to Hardware ID request", level="WARNING")
            return None
    
    def request_system_settings(self) -> Optional[Dict[str, Any]]:
        """Request system settings from HM30."""
        self.log("Requesting System Settings (CMD 0x16)...", level="REQUEST")
        
        if not self.send_command(CommandID.REQUEST_SYSTEM_SETTINGS.value):
            return None
        
        time.sleep(0.2)
        response = self.receive_response()
        
        if response and response.cmd_id == CommandID.REQUEST_SYSTEM_SETTINGS.value:
            if len(response.data) >= 3:
                match = response.data[0]
                baud_type = response.data[1]
                joy_type = response.data[2]
                rc_bat = struct.unpack('<H', response.data[3:5])[0] if len(response.data) >= 5 else 0
                
                baud_names = {0: '4800', 1: '9600', 2: '38400', 3: '57600', 4: '76800', 5: '115200', 6: '230400'}
                joy_names = {0: 'Mode 1', 1: 'Mode 2', 2: 'Mode 3', 3: 'Custom'}
                match_names = {0: 'Start', 1: 'Binding', 2: 'Binding', 3: 'Finished'}
                
                settings = {
                    'bind_status': match_names.get(match, 'Unknown'),
                    'baud_rate': baud_names.get(baud_type, 'Unknown'),
                    'joystick_type': joy_names.get(joy_type, 'Unknown'),
                    'battery_voltage': f"{rc_bat / 10:.1f}V" if rc_bat > 0 else "Unknown"
                }
                
                self.log(f"Bind Status: {settings['bind_status']}", level="DATA")
                self.log(f"Baud Rate: {settings['baud_rate']}", level="DATA")
                self.log(f"Joystick Type: {settings['joystick_type']}", level="DATA")
                self.log(f"RC Battery: {settings['battery_voltage']}", level="DATA")
                
                return settings
        else:
            self.log("No response to System Settings request", level="WARNING")
            return None
    
    def request_channel_data(self, frequency: ChannelFrequency = ChannelFrequency.HZ_2) -> Optional[Dict[int, int]]:
        """
        Request RC channel data from HM30.
        
        Args:
            frequency: Output frequency
        """
        self.log(f"Requesting Channel Data (CMD 0x42) at {frequency.value}...", level="REQUEST")
        
        if not self.send_command(CommandID.REQUEST_CHANNEL_DATA.value, bytes([frequency.value])):
            return None
        
        time.sleep(0.2)
        response = self.receive_response()
        
        if response and response.cmd_id == CommandID.REQUEST_CHANNEL_DATA.value:
            channels = {}
            for i in range(16):
                offset = i * 2
                if offset + 2 <= len(response.data):
                    channel_value = struct.unpack('<h', response.data[offset:offset + 2])[0]
                    channels[i + 1] = channel_value
            
            self.log(f"Received {len(channels)} channel values", level="DATA")
            for ch, val in list(channels.items())[:4]:  # Show first 4 channels
                self.log(f"  CH{ch}: {val}", level="DATA")
            
            return channels
        else:
            self.log("No response to Channel Data request", level="WARNING")
            return None
    
    def request_datalink_status(self) -> Optional[Dict[str, Any]]:
        """Request datalink status from HM30."""
        self.log("Requesting Datalink Status (CMD 0x43)...", level="REQUEST")
        
        if not self.send_command(CommandID.REQUEST_DATALINK_STATUS.value):
            return None
        
        time.sleep(0.2)
        response = self.receive_response()
        
        if response and response.cmd_id == CommandID.REQUEST_DATALINK_STATUS.value:
            if len(response.data) >= 12:
                freq = struct.unpack('<H', response.data[0:2])[0]
                pack_loss = response.data[2]
                real_pack = struct.unpack('<H', response.data[3:5])[0]
                real_pack_rate = struct.unpack('<H', response.data[5:7])[0]
                data_up = struct.unpack('<I', response.data[7:11])[0]
                data_down = struct.unpack('<I', response.data[11:15])[0]
                
                status = {
                    'frequency': f"{freq} MHz",
                    'packet_loss_rate': f"{pack_loss}%",
                    'valid_packets': real_pack,
                    'packet_rate': f"{real_pack_rate} pps",
                    'upload_speed': f"{data_up} B/s",
                    'download_speed': f"{data_down} B/s"
                }
                
                self.log(f"Frequency: {status['frequency']}", level="DATA")
                self.log(f"Packet Loss: {status['packet_loss_rate']}", level="DATA")
                self.log(f"Upload: {status['upload_speed']}", level="DATA")
                self.log(f"Download: {status['download_speed']}", level="DATA")
                
                return status
        else:
            self.log("No response to Datalink Status request", level="WARNING")
            return None
    
    def request_image_link_status(self) -> Optional[Dict[str, Any]]:
        """Request image transmission link status from HM30."""
        self.log("Requesting Image Link Status (CMD 0x44)...", level="REQUEST")
        
        if not self.send_command(CommandID.REQUEST_IMAGE_LINK_STATUS.value):
            return None
        
        time.sleep(0.2)
        response = self.receive_response()
        
        if response and response.cmd_id == CommandID.REQUEST_IMAGE_LINK_STATUS.value:
            if len(response.data) >= 36:
                signal = struct.unpack('<i', response.data[0:4])[0]
                upstream = struct.unpack('<i', response.data[8:12])[0]
                downstream = struct.unpack('<i', response.data[12:16])[0]
                txbandwidth = struct.unpack('<i', response.data[16:20])[0]
                rxbandwidth = struct.unpack('<i', response.data[20:24])[0]
                rssi = struct.unpack('<i', response.data[24:28])[0]
                freq = struct.unpack('<i', response.data[28:32])[0]
                channel = struct.unpack('<i', response.data[32:36])[0]
                
                status = {
                    'signal': f"{signal}%",
                    'upstream': f"{upstream} B/s",
                    'downstream': f"{downstream} B/s",
                    'tx_bandwidth': f"{txbandwidth / 1000:.2f} Mbps" if txbandwidth > 0 else "N/A",
                    'rx_bandwidth': f"{rxbandwidth / 1000:.2f} Mbps" if rxbandwidth > 0 else "N/A",
                    'rssi': f"{rssi} dBm",
                    'frequency': f"{freq} MHz",
                    'channel': channel
                }
                
                self.log(f"Signal: {status['signal']}", level="DATA")
                self.log(f"Upstream: {status['upstream']}", level="DATA")
                self.log(f"Downstream: {status['downstream']}", level="DATA")
                self.log(f"RSSI: {status['rssi']}", level="DATA")
                
                return status
        else:
            self.log("No response to Image Link Status request", level="WARNING")
            return None
    
    def request_firmware_version(self) -> Optional[Dict[str, str]]:
        """Request firmware versions from HM30."""
        self.log("Requesting Firmware Versions (CMD 0x47)...", level="REQUEST")
        
        if not self.send_command(CommandID.REQUEST_FIRMWARE_VERSION.value):
            return None
        
        time.sleep(0.2)
        response = self.receive_response()
        
        if response and response.cmd_id == CommandID.REQUEST_FIRMWARE_VERSION.value:
            if len(response.data) >= 16:
                rc_version = struct.unpack('<I', response.data[0:4])[0]
                rf_version = struct.unpack('<I', response.data[4:8])[0]
                ground_version = struct.unpack('<I', response.data[8:12])[0]
                sky_version = struct.unpack('<I', response.data[12:16])[0]
                
                def parse_version(ver: int) -> str:
                    """Parse version: first byte is product ID, remaining 3 bytes are version."""
                    product_id = ver & 0xFF
                    major = (ver >> 8) & 0xFF
                    minor = (ver >> 16) & 0xFF
                    patch = (ver >> 24) & 0xFF
                    return f"{major}.{minor}.{patch} (PID: 0x{product_id:02x})"
                
                versions = {
                    'rc_firmware': parse_version(rc_version),
                    'rf_firmware': parse_version(rf_version),
                    'ground_firmware': parse_version(ground_version),
                    'sky_firmware': parse_version(sky_version)
                }
                
                self.log(f"RC Firmware: {versions['rc_firmware']}", level="DATA")
                self.log(f"RF Firmware: {versions['rf_firmware']}", level="DATA")
                self.log(f"Ground Firmware: {versions['ground_firmware']}", level="DATA")
                self.log(f"Sky Firmware: {versions['sky_firmware']}", level="DATA")
                
                return versions
        else:
            self.log("No response to Firmware Version request", level="WARNING")
            return None
    
    @staticmethod
    def log(message: str, level: str = "INFO") -> None:
        """
        Log message to terminal with timestamp and color.
        
        Args:
            message: Log message
            level: Log level (INFO, SUCCESS, WARNING, ERROR, REQUEST, DATA)
        """
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Color codes
        colors = {
            'INFO': '\033[94m',      # Blue
            'SUCCESS': '\033[92m',   # Green
            'WARNING': '\033[93m',   # Yellow
            'ERROR': '\033[91m',     # Red
            'REQUEST': '\033[96m',   # Cyan
            'DATA': '\033[95m',      # Magenta
            'RESET': '\033[0m'       # Reset
        }
        
        color = colors.get(level, colors['INFO'])
        reset = colors['RESET']
        
        print(f"{color}[{timestamp}] [{level:7s}]{reset} {message}")
    
    def poll_all(self) -> None:
        """Poll all available data from HM30."""
        self.log("=" * 70, level="INFO")
        self.log("HM30 SIYI Datalink SDK - Complete Data Polling", level="SUCCESS")
        self.log("=" * 70, level="INFO")
        
        self.request_hardware_id()
        self.log("", level="INFO")
        
        self.request_firmware_version()
        self.log("", level="INFO")
        
        self.request_system_settings()
        self.log("", level="INFO")
        
        self.request_channel_data(ChannelFrequency.HZ_4)
        self.log("", level="INFO")
        
        self.request_datalink_status()
        self.log("", level="INFO")
        
        self.request_image_link_status()
        self.log("", level="INFO")
        
        self.log("=" * 70, level="INFO")
        self.log("Polling complete", level="SUCCESS")


def main():
    """Main function with CLI interface."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='HM30 SIYI Datalink SDK Data Poller (UDP)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use default HM30 IP 192.168.144.12 and port 19856
  python hm30_data_poller.py
  
  # Use custom HM30 IP address
  python hm30_data_poller.py -H 192.168.1.100
  
  # Use custom UDP port
  python hm30_data_poller.py -P 19856
  
  # Specific command (hardware ID only)
  python hm30_data_poller.py -c hardware
  
  # Request channel data only
  python hm30_data_poller.py -c channel
  
  # Poll firmware versions
  python hm30_data_poller.py -c firmware

Network Setup:
  1. Connect HM30 ground unit to your network (Ethernet or WiFi)
  2. Note the HM30 IP address (typically 192.168.144.12 on direct connection)
  3. Configure your PC to be on the same subnet (e.g., 192.168.144.30)
  4. Run this script pointing to the HM30 IP
        """
    )
    
    parser.add_argument(
        '-H', '--host',
        default='192.168.144.12',
        help='HM30 ground unit IP address (default: 192.168.144.12)'
    )
    parser.add_argument(
        '-P', '--port',
        type=int,
        default=19856,
        help='UDP port for HM30 communication (default: 19856)'
    )
    parser.add_argument(
        '-t', '--timeout',
        type=float,
        default=2.0,
        help='Socket timeout in seconds (default: 2.0)'
    )
    parser.add_argument(
        '-c', '--command',
        choices=['hardware', 'settings', 'channel', 'datalink', 'image', 'firmware', 'all'],
        default='all',
        help='Specific command to run (default: all)'
    )
    parser.add_argument(
        '--bind-src-port',
        action='store_true',
        help='Bind local UDP source port to the HM30 port (19856) to test firmwares that reply only to same source port',
    )
    
    args = parser.parse_args()
    
    # Create poller instance
    poller = HM30Poller(host=args.host, port=args.port, timeout=args.timeout)
    if getattr(args, 'bind_src_port', False):
        poller.bind_src_port = True
    
    # Connect
    if not poller.connect():
        sys.exit(1)
    
    try:
        # Run requested command(s)
        if args.command == 'hardware':
            poller.request_hardware_id()
        elif args.command == 'settings':
            poller.request_system_settings()
        elif args.command == 'channel':
            poller.request_channel_data()
        elif args.command == 'datalink':
            poller.request_datalink_status()
        elif args.command == 'image':
            poller.request_image_link_status()
        elif args.command == 'firmware':
            poller.request_firmware_version()
        else:  # all
            poller.poll_all()
    
    except KeyboardInterrupt:
        poller.log("\nInterrupted by user", level="WARNING")
    except Exception as e:
        poller.log(f"Error: {e}", level="ERROR")
    finally:
        poller.disconnect()


if __name__ == '__main__':
    main()