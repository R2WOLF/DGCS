import socket

GIMBAL_IP = "192.168.144.25"
GIMBAL_PORT = 37260

class SiyiGimbal:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, data):
        self.sock.sendto(bytes(data), (GIMBAL_IP, GIMBAL_PORT))

    # --- Commands (prebuilt frames from SDK) ---

    def center(self):
        self.send([0x55,0x66,0x01,0x01,0x00,0x00,0x00,0x08,0x01,0xd1,0x12])

    def rotate(self, pitch):
        # pitch: -100 to 100
        pitch_byte = pitch & 0xFF
        self.send([0x55,0x66,0x01,0x02,0x00,0x00,0x00,0x07,0x00,pitch_byte,0x00,0x00])

    def stop(self):
        self.rotate(0)

    def get_attitude(self):
        self.send([0x55,0x66,0x01,0x00,0x00,0x00,0x00,0x0d,0xe8,0x05])