/*
====================================================
 STM Nucleo UART Binary Joystick Transmitter
 STM32L476RG (or compatible Nucleo board)
 UART1: TX -> PA9 (USART1_TX), RX -> PA10 (USART1_RX)
 Joystick X -> PA0 (ADC1_IN5)
 Joystick Y -> PA1 (ADC1_IN6)
====================================================

PACKET FORMAT (IDENTICAL TO ESP32 VERSION)

[0] 0xAA   START BYTE
[1] X HIGH
[2] X LOW
[3] Y HIGH
[4] Y LOW
[5] CHECKSUM
[6] 0x55   END BYTE

Baud Rate: 115200
ADC Resolution: 12-bit (0-4095)

====================================================
*/

#include <Arduino.h>

#define JOY_X PA0
#define JOY_Y PA1

#define START_BYTE 0xAA
#define END_BYTE   0x55

#define DEBUG_SERIAL 0

// Packet buffer
uint8_t packet[7];
uint32_t lastDebugMs = 0;

void setup()
{
    // Initialize serial communication at 115200 baud
    // On STM Nucleo, Serial uses USART2 (PA2/PA3) or USART1 (PA9/PA10)
    // We use Serial1 for USART1 to connect to Raspberry Pi
    Serial1.begin(115200);
    
    // Debug serial on USB
    Serial.begin(115200);

    // Configure analog pins for joystick input
    pinMode(JOY_X, INPUT);
    pinMode(JOY_Y, INPUT);

    // STM32 ADC defaults to 12-bit resolution
    analogReadResolution(12);

    Serial.println("STM Nucleo Joystick UART Started");
    delay(100);
}

void loop()
{
    // Read joystick analog values (12-bit: 0-4095)
    uint16_t xValue = analogRead(JOY_X);
    uint16_t yValue = analogRead(JOY_Y);

    // =========================
    // BUILD PACKET
    // =========================

    packet[0] = START_BYTE;

    // X VALUE (16-bit big-endian)
    packet[1] = (xValue >> 8) & 0xFF;
    packet[2] = xValue & 0xFF;

    // Y VALUE (16-bit big-endian)
    packet[3] = (yValue >> 8) & 0xFF;
    packet[4] = yValue & 0xFF;

    // XOR CHECKSUM (bytes 1-4)
    packet[5] =
        packet[1] ^
        packet[2] ^
        packet[3] ^
        packet[4];

    packet[6] = END_BYTE;

    // =========================
    // SEND PACKET
    // =========================

    Serial1.write(packet, sizeof(packet));

    // =========================
    // DEBUG OUTPUT (to USB Serial)
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

    // Send at approximately 100Hz (10ms per packet)
    delay(10);
}

/*
====================================================
SETUP INSTRUCTIONS FOR STM NUCLEO
====================================================

HARDWARE CONNECTIONS:
- Joystick X -> PA0 (ADC1_IN5)
- Joystick Y -> PA1 (ADC1_IN6)
- UART1 TX (PA9) -> Raspberry Pi RX
- UART1 RX (PA10) -> Raspberry Pi TX
- GND -> Raspberry Pi GND

BOARD CONFIGURATION:
1. Open Arduino IDE and select: Tools > Board > STM32 Boards > Nucleo L476RG
2. Select Port (USB connection to Nucleo board)
3. Select Tools > Serial Port: USART1

IMPORTANT NOTES:
- ADC values are 12-bit (0-4095), same as ESP32
- Baud rate is 115200 to match Raspberry Pi UART
- Joystick center should be around 2048 (mid-range)
- Packet transmission rate: ~100Hz

TESTING:
- Enable DEBUG_SERIAL by setting to 1 to see joystick values on USB monitor
- Open Tools > Serial Monitor to view debug output
- Verify X and Y values change when moving joystick
- Checksum (CHK) should change with each packet

====================================================
*/
