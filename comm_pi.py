import serial

# ==========================================
# UART CONFIG
# ==========================================

PORT = "/dev/ttyUSB0"
BAUD = 115200

START_BYTE = 0xAA
END_BYTE = 0x55

# ==========================================
# OPEN SERIAL
# ==========================================

ser = serial.Serial(PORT, BAUD, timeout=1)

print("UART Receiver Started...\n")

# ==========================================
# MAIN LOOP
# ==========================================

while True:

    # WAIT FOR START BYTE
    start = ser.read(1)

    if len(start) == 0:
        continue

    if start[0] != START_BYTE:
        continue

    # READ REST OF PACKET
    data = ser.read(6)

    if len(data) != 6:
        continue

    x_high = data[0]
    x_low  = data[1]

    y_high = data[2]
    y_low  = data[3]

    received_checksum = data[4]
    end_byte = data[5]

    # ======================================
    # VERIFY END BYTE
    # ======================================

    if end_byte != END_BYTE:
        print("END BYTE ERROR")
        continue

    # ======================================
    # VERIFY CHECKSUM
    # ======================================

    calculated_checksum = (
        x_high ^
        x_low ^
        y_high ^
        y_low
    )

    if calculated_checksum != received_checksum:
        print("CHECKSUM ERROR")
        continue

    # ======================================
    # REBUILD ADC VALUES
    # ======================================

    xValue = (x_high << 8) | x_low
    yValue = (y_high << 8) | y_low

    # ======================================
    # PRINT VALUES
    # ======================================

    print(f"Joystick X: {xValue:4d} | Y: {yValue:4d}")