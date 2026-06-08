/*
====================================================
 ESP32-C3 UART Binary Joystick Transmitter
 TX -> GPIO21
 RX -> GPIO20
====================================================

PACKET FORMAT

[0] 0xAA   START BYTE
[1] X HIGH
[2] X LOW
[3] Y HIGH
[4] Y LOW
[5] CHECKSUM
[6] 0x55   END BYTE

====================================================
*/

#define JOY_X 0
#define JOY_Y 1

#define START_BYTE 0xAA
#define END_BYTE   0x55

#define DEBUG_SERIAL 0

uint8_t packet[7];
uint32_t lastDebugMs = 0;

void setup()
{
    // UART to Raspberry Pi
    Serial0.begin(115200, SERIAL_8N1, 20, 21);

    // USB Serial Debug
    Serial.begin(115200);

    analogReadResolution(12);

    pinMode(JOY_X, INPUT);
    pinMode(JOY_Y, INPUT);

    Serial.println("ESP32-C3 Joystick UART Started");
}

void loop()
{
    uint16_t xValue = analogRead(JOY_X);
    uint16_t yValue = analogRead(JOY_Y);

    // =========================
    // BUILD PACKET
    // =========================

    packet[0] = START_BYTE;

    // X VALUE
    packet[1] = (xValue >> 8) & 0xFF;
    packet[2] = xValue & 0xFF;

    // Y VALUE
    packet[3] = (yValue >> 8) & 0xFF;
    packet[4] = yValue & 0xFF;

    // XOR CHECKSUM
    packet[5] =
        packet[1] ^
        packet[2] ^
        packet[3] ^
        packet[4];

    packet[6] = END_BYTE;

    // =========================
    // SEND PACKET
    // =========================

    Serial.write(packet, sizeof(packet));

    // =========================
    // DEBUG OUTPUT
    // =========================

    if (DEBUG_SERIAL) {
        uint32_t now = millis();
        if (now - lastDebugMs >= 250) {
            lastDebugMs = now;
            Serial.print("X: ");
            Serial.print(xValue);

            Serial.print(" | Y: ");
            Serial.print(yValue);

            Serial.print(" | CHK: ");
            Serial.println(packet[5], HEX);
        }
    }

    delay(5); // 100Hz
}