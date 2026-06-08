#!/usr/bin/env python3
"""
HM30 UDP Data Receiver
Reads telemetry data from SIYI HM30 ground unit via UDP at 10Hz
Based on HM30 User Manual v1.7 Protocol Specification
"""

import socket
import struct
import time
from datetime import datetime
import sys

class HM30UDPReader:
    # Protocol Constants
    HM30_GROUND_UNIT_IP = "192.168.144.12"
    HM30_UDP_PORT = 19856
    
    # Packet Structure
    # STX is 0x6655 in the spec (low byte on the wire first -> 0x55 0x66)
    STX = 0x6655  # Start marker (use with '<H' to get bytes 0x55 0x66)
    
    # Command IDs
    CMD_REQUEST_CHANNEL_DATA = 0x42
    CMD_REQUEST_DATALINK_STATUS = 0x43
    CMD_REQUEST_IMAGE_STATUS = 0x44
    
    # Frequency codes
    FREQ_OFF = 0
    FREQ_2HZ = 1
    FREQ_4HZ = 2
    FREQ_5HZ = 3
    FREQ_10HZ = 4  # 10 Hz - what we want
    FREQ_20HZ = 5
    FREQ_50HZ = 6
    FREQ_100HZ = 7
    
    def __init__(self, local_ip="192.168.144.30", frequency=FREQ_10HZ):
        """
        Initialize HM30 UDP Reader
        
        Args:
            local_ip: Your computer's IP address on the same network as HM30
            frequency: Data frequency (default 10Hz)
        """
        self.local_ip = local_ip
        self.frequency = frequency
        self.sequence = 0
        self.socket = None
        self.running = False
        
        # Create UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            # Bind to local interface (optional, helps with multiple interfaces)
            self.socket.bind((self.local_ip, 0))
        except:
            # If binding fails, just connect
            pass
    
    def crc16_ccitt(self, data):
        """
        Calculate CRC16-CCITT checksum
        Poly: 0x1021, Initial: 0x0000
        """
        crc = 0x0000
        poly = 0x1021
        
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                crc = (crc << 1) ^ (poly if crc & 0x8000 else 0)
                crc &= 0xFFFF
        
        return crc
    
    def build_packet(self, cmd_id, data=b'', seq=None):
        """
        Build HM30 protocol packet
        
        Packet format:
        - STX (2 bytes): 0x5566
        - CTRL (1 byte): 0x01 (need_ack)
        - Data_len (2 bytes): length of data field
        - SEQ (2 bytes): sequence number
        - CMD_ID (1 byte): command ID
        - DATA (variable): command data
        - CRC16 (2 bytes): checksum
        """
        # Build packet using provided seq for deterministic tests; otherwise use and increment internal counter
        ctrl = 0x01  # need_ack
        data_len = len(data) + 1  # +1 for CMD_ID

        if seq is None:
            seq = self.sequence
            self.sequence = (self.sequence + 1) & 0xFFFF

        # Build packet body (CTRL, DATA_LEN, SEQ, CMD_ID, DATA)
        packet_body = struct.pack('<BHH', ctrl, data_len, seq & 0xFFFF) + struct.pack('B', cmd_id) + data

        # Prepend STX (little-endian of 0x6655 -> wire bytes 0x55 0x66)
        stx = struct.pack('<H', self.STX)
        packet = stx + packet_body

        # Calculate CRC over CTRL..DATA and append (little-endian)
        crc = self.crc16_ccitt(packet_body)
        crc_bytes = struct.pack('<H', crc)

        complete_packet = packet + crc_bytes
        return complete_packet
    
    def send_request(self, cmd_id, data=b''):
        """Send request packet to HM30"""
        packet = self.build_packet(cmd_id, data)
        try:
            self.socket.sendto(packet, (self.HM30_GROUND_UNIT_IP, self.HM30_UDP_PORT))
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False
    
    def parse_channel_data(self, response_data):
        """
        Parse channel data response
        16 channels × 2 bytes each = 32 bytes
        """
        if len(response_data) < 32:
            return None
        
        channels = []
        for i in range(16):
            value = struct.unpack('<h', response_data[i*2:(i+1)*2])[0]
            channels.append(value)
        
        return channels
    
    def parse_datalink_status(self, response_data):
        """
        Parse datalink status response
        Format: freq(2), loss_rate(1), real_pack(2), real_pack_rate(2), 
                data_up(4), data_down(4)
        """
        if len(response_data) < 15:
            return None
        
        freq, loss_rate, real_pack, real_pack_rate, data_up, data_down = struct.unpack(
            '<HBHHII',
            response_data[:15]
        )
        
        return {
            'frequency': freq,
            'loss_rate': loss_rate,
            'valid_packets': real_pack,
            'valid_rate': real_pack_rate,
            'upload_bps': data_up,
            'download_bps': data_down
        }
    
    def read_response(self, timeout=1.0):
        """
        Read UDP response packet and parse it
        
        Response format similar to request:
        - STX (2 bytes): 0x5566
        - CTRL (1 byte): 0x02 (ack_pack)
        - Data_len (2 bytes)
        - SEQ (2 bytes)
        - CMD_ID (1 byte)
        - DATA (variable)
        - CRC16 (2 bytes)
        """
        self.socket.settimeout(timeout)
        
        try:
            data, addr = self.socket.recvfrom(4096)
        except socket.timeout:
            return None
        except Exception as e:
            print(f"Receive error: {e}")
            return None
        
        # Validate minimum packet size
        if len(data) < 9:
            return None
        
        # Check STX
        stx = struct.unpack('<H', data[0:2])[0]
        if stx != self.STX:
            return None
        
        # Parse header
        ctrl = data[2]
        data_len = struct.unpack('<H', data[3:5])[0]
        seq = struct.unpack('<H', data[5:7])[0]
        cmd_id = data[7]
        
        # Check packet length (total = 9 + data_len)
        if len(data) < (9 + data_len):
            return None

        # Extract payload (DATA_LEN includes CMD_ID as +1) and CRC
        payload = data[8:8 + (data_len - 1)] if data_len > 1 else b""
        crc_received = struct.unpack('<H', data[8 + (data_len - 1): 8 + (data_len - 1) + 2])[0]

        # Verify CRC computed over CTRL..DATA (exclude STX)
        packet_body = data[2: 8 + (data_len - 1)]
        crc_calc = self.crc16_ccitt(packet_body)

        if crc_calc != crc_received:
            print(f"CRC mismatch: calculated {crc_calc:04X}, received {crc_received:04X}")
            return None
        
        return {
            'ctrl': ctrl,
            'seq': seq,
            'cmd_id': cmd_id,
            'data': payload,
            'data_len': data_len
        }
    
    def request_channel_data_10hz(self):
        """Request channel data at 10Hz"""
        # Frequency code: 0x04 = 10Hz
        data = struct.pack('B', self.FREQ_10HZ)
        return self.send_request(self.CMD_REQUEST_CHANNEL_DATA, data)
    
    def request_datalink_status(self):
        """Request datalink status"""
        return self.send_request(self.CMD_REQUEST_DATALINK_STATUS, b'')
    
    def start(self):
        """Start reading data from HM30 at 10Hz"""
        print(f"HM30 UDP Reader - 10Hz Mode")
        print(f"Connecting to HM30 at {self.HM30_GROUND_UNIT_IP}:{self.HM30_UDP_PORT}")
        print(f"Local IP: {self.local_ip}")
        print("-" * 80)
        print(f"{'Time':<12} {'CH1':<7} {'CH2':<7} {'CH3':<7} {'CH4':<7} {'CH5':<7} {'CH6':<7} {'CH7':<7} {'CH8':<7}")
        print("-" * 80)
        
        self.running = True
        request_count = 0
        last_seq = 0
        
        try:
            while self.running:
                # Send request for channel data
                if self.request_channel_data_10hz():
                    request_count += 1
                    
                    # Read response
                    response = self.read_response(timeout=0.5)
                    
                    if response:
                        if response['cmd_id'] == self.CMD_REQUEST_CHANNEL_DATA:
                            channels = self.parse_channel_data(response['data'])
                            
                            if channels:
                                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                                ch_str = " ".join([f"{ch:6d}" for ch in channels[:8]])
                                print(f"{timestamp}  {ch_str}")
                                
                                # Check for sequence gaps
                                if last_seq > 0 and response['seq'] != (last_seq + 1) & 0xFFFF:
                                    print(f"  [!] Sequence gap: {last_seq} -> {response['seq']}")
                                last_seq = response['seq']
                    else:
                        print("No response")
                
                # 10Hz = 100ms per sample
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print("\n\nStopped by user")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.close()
    
    def close(self):
        """Close UDP connection"""
        self.running = False
        if self.socket:
            self.socket.close()
        print("Connection closed")


def main():
    """Main entry point"""
    try:
        # Create reader with default settings
        reader = HM30UDPReader(
            local_ip="192.168.144.30",  # Change this to your PC's IP on the network
            frequency=HM30UDPReader.FREQ_10HZ
        )
        
        # Start reading data
        reader.start()
    
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()