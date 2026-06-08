/*
====================================================
STM32 Nucleo Joystick + Telemetry Transmitter (Enhanced)

Hardware:
- Joystick X → PA0 (ADC1_IN5)
- Joystick Y → PA1 (ADC1_IN6)
- Battery Sense → PA3 (ADC1_IN8, with voltage divider)
- Button 1 → PB0 (GPIO)
- Button 2 → PB1 (GPIO)
- Button 3 → PB2 (GPIO)
- Button 4 → PB3 (GPIO)
- USB Connection → Nucleo's built-in ST-LINK USB port
  (/dev/ttyACM0 on Linux, COMx on Windows)

Communication:
- USB Virtual COM Port (Serial) @ 115200 baud
- Data sent to Raspberry Pi via Nucleo's USB port
- Both binary packets AND JSON telemetry transmitted

This firmware sends both binary joystick packets AND JSON telemetry.

Packet Format (Binary):
[0xAA] [X_HIGH] [X_LOW] [Y_HIGH] [Y_LOW] [CHECKSUM] [0x55]

Telemetry Format (JSON):
{
  "type":"telemetry",
  "bat_gcs":3.7,
  "bat_robot":12.5,
  "joy":{"x":2048,"y":2048,"btn":[false,false,false,false]},
  "extra_data":25.5
}

Fields:
- bat_gcs: GCS/controller board battery (V) - read from PA3
- bat_robot: Robot/gimbal system battery (V) - placeholder, update with actual sensor
- joy.x, joy.y: Joystick ADC values (0-4095)
- joy.btn: 4 button states [Emergency Stop, Camera Switch, Control Disable, Reserved]
- extra_data: Custom sensor data (temperature by default)
====================================================
*/

#include <Arduino.h>

// ===== PINOUT =====
#define JOY_X PA0
#define JOY_Y PA1
#define BATT_SENSE PA3

#define BTN1 PB0
#define BTN2 PB1
#define BTN3 PB2
#define BTN4 PB3

// ===== PROTOCOL DEFINITIONS =====
#define START_BYTE 0xAA
#define END_BYTE   0x55

// ===== TIMING =====
#define JOYSTICK_SEND_MS 10    // Binary packets: 100Hz
#define TELEMETRY_SEND_MS 100  // JSON telemetry: 10Hz
#define BATTERY_READ_MS 1000   // Battery reading: 1Hz

// ===== CALIBRATION =====
#define BATTERY_ADC_TO_VOLTAGE 2.0  // Adjust based on voltage divider
#define BATTERY_DIVIDER_RATIO 3.3   // VCC reference
#define BATTERY_FILTER_ALPHA 0.1    // Low-pass filter coefficient

// ===== GLOBALS =====
uint8_t packet[7];
uint32_t lastJoystickMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastBatteryMs = 0;

float filtered_battery = 0.0;
uint16_t last_x = 2048;
uint16_t last_y = 2048;
bool button_state[4] = {false, false, false, false};

// ===== DEBUG =====
#define DEBUG_SERIAL 0

void setup() {
    // USB Serial to Raspberry Pi (via Nucleo's virtual COM port)
    Serial.begin(115200);
    
    // Configure ADC
    analogReadResolution(12);
    
    // Configure GPIO
    pinMode(JOY_X, INPUT);
    pinMode(JOY_Y, INPUT);
    pinMode(BATT_SENSE, INPUT);
    
    pinMode(BTN1, INPUT_PULLUP);
    pinMode(BTN2, INPUT_PULLUP);
    pinMode(BTN3, INPUT_PULLUP);
    pinMode(BTN4, INPUT_PULLUP);
    
    delay(100);
    
    // Initialize battery filter with first reading
    float adc_val = analogRead(BATT_SENSE);
    float raw_voltage = (adc_val / 4095.0) * BATTERY_DIVIDER_RATIO * BATTERY_ADC_TO_VOLTAGE;
    filtered_battery = raw_voltage;
    
    if (DEBUG_SERIAL) {
        Serial.println("STM32 Nucleo Joystick + Telemetry Started");
        Serial.print("Battery: ");
        Serial.print(filtered_battery);
        Serial.println("V");
    }
}

void loop() {
    uint32_t now = millis();
    
    // Read sensors continuously
    uint16_t x_value = analogRead(JOY_X);
    uint16_t y_value = analogRead(JOY_Y);
    
    // Read buttons (inverted logic: LOW = pressed due to pull-ups)
    button_state[0] = !digitalRead(BTN1);
    button_state[1] = !digitalRead(BTN2);
    button_state[2] = !digitalRead(BTN3);
    button_state[3] = !digitalRead(BTN4);
    
    // Update battery reading (low frequency)
    if (now - lastBatteryMs >= BATTERY_READ_MS) {
        float adc_val = analogRead(BATT_SENSE);
        float raw_voltage = (adc_val / 4095.0) * BATTERY_DIVIDER_RATIO * BATTERY_ADC_TO_VOLTAGE;
        
        // Apply low-pass filter for stability
        filtered_battery = (BATTERY_FILTER_ALPHA * raw_voltage) + 
                          ((1.0 - BATTERY_FILTER_ALPHA) * filtered_battery);
        
        lastBatteryMs = now;
        
        if (DEBUG_SERIAL) {
            Serial.print("Battery: ");
            Serial.print(filtered_battery);
            Serial.println("V");
        }
    }
    
    // Send binary joystick packet (high frequency)
    if (now - lastJoystickMs >= JOYSTICK_SEND_MS) {
        send_binary_packet(x_value, y_value);
        last_x = x_value;
        last_y = y_value;
        lastJoystickMs = now;
    }
    
    // Send JSON telemetry (lower frequency)
    if (now - lastTelemetryMs >= TELEMETRY_SEND_MS) {
        send_json_telemetry(x_value, y_value, filtered_battery);
        lastTelemetryMs = now;
    }
}

void send_binary_packet(uint16_t x_value, uint16_t y_value) {
    /*
    Binary packet format (identical to ESP32 version):
    [0] 0xAA       START BYTE
    [1] X HIGH
    [2] X LOW
    [3] Y HIGH
    [4] Y LOW
    [5] CHECKSUM
    [6] 0x55       END BYTE
    */
    
    packet[0] = START_BYTE;
    
    // X VALUE (16-bit big-endian)
    packet[1] = (x_value >> 8) & 0xFF;
    packet[2] = x_value & 0xFF;
    
    // Y VALUE (16-bit big-endian)
    packet[3] = (y_value >> 8) & 0xFF;
    packet[4] = y_value & 0xFF;
    
    // XOR CHECKSUM
    packet[5] = packet[1] ^ packet[2] ^ packet[3] ^ packet[4];
    
    packet[6] = END_BYTE;
    
    // Send binary packet via USB virtual COM port to RPi
    Serial.write(packet, sizeof(packet));
    
    if (DEBUG_SERIAL && (millis() % 500 == 0)) {
        Serial.print("Binary: X=");
        Serial.print(x_value);
        Serial.print(" Y=");
        Serial.print(y_value);
        Serial.print(" CHK=");
        Serial.println(packet[5], HEX);
    }
}

void send_json_telemetry(uint16_t x_value, uint16_t y_value, float battery) {
    /*
    JSON telemetry format expected by Python GCS:
    {
      "type":"telemetry",
      "bat_gcs":3.7,
      "bat_robot":12.5,
      "joy":{
        "x":2048,
        "y":2048,
        "btn":[false,false,false,false]
      },
      "extra_data":25.5
    }
    */
    
    // Build JSON manually (to avoid ArduinoJson library dependency)
    String json = "{";
    json += "\"type\":\"telemetry\",";
    json += "\"bat_gcs\":" + String(battery, 2) + ",";
    json += "\"bat_robot\":12.50,";  // Robot battery - placeholder (set to actual value from sensor)
    json += "\"joy\":{";
    json += "\"x\":" + String(x_value) + ",";
    json += "\"y\":" + String(y_value) + ",";
    json += "\"btn\":[";
    json += button_state[0] ? "true" : "false";
    json += ",";
    json += button_state[1] ? "true" : "false";
    json += ",";
    json += button_state[2] ? "true" : "false";
    json += ",";
    json += button_state[3] ? "true" : "false";
    json += "]},";
    
    // Extra data: temperature from internal sensor (example)
    float temp = get_internal_temperature();
    json += "\"extra_data\":" + String(temp, 1);
    
    json += "}";
    
    // Send JSON via USB virtual COM port to RPi, followed by newline (line-delimited)
    Serial.println(json);
    
    if (DEBUG_SERIAL) {
        Serial.print("JSON: ");
        Serial.println(json);
    }
}

float get_internal_temperature() {
    /*
    Read STM32 internal temperature sensor
    Note: Accuracy is ±5°C. For production, use external sensor.
    */
    #ifdef TSTOP
    // STM32 internal temp sensor on internal ADC channel
    uint16_t adc_val = analogRead(TSTOP);
    // Rough conversion (specific to STM32L476)
    float temp = 25.0 + (float)(adc_val - 600) / 4.3;
    return temp;
    #else
    // Fallback: return constant if sensor not available
    return 25.0;
    #endif
}

/*
====================================================
SETUP INSTRUCTIONS
====================================================

1. HARDWARE WIRING:

   Joystick Module    →    STM32 Nucleo
   ────────────────────────────────────
   VCC              →    +5V
   GND              →    GND
   X axis           →    PA0 (ADC)
   Y axis           →    PA1 (ADC)
   
   Buttons          →    STM32
   ────────────────────
   Button 1         →    PB0
   Button 2         →    PB1
   Button 3         →    PB2
   Button 4         →    PB3
   Button GND       →    GND (via pull-up resistor to +3.3V)
   
   Battery Sense    →    STM32
   ─────────────────────────────
   Battery +        →    PA3 (through 10k + 30k divider)
   Battery GND      →    GND
   
   UART to Pi       →    STM32
   ────────────────────────────
   Pi RX            →    PA9 (USART1_TX)
   Pi TX            →    PA10 (USART1_RX)
   Pi GND           →    GND

2. VOLTAGE DIVIDER FOR BATTERY:
   
   For LiPo 3.7V measurement:
   ─────────────────────────
   Battery+ ─────[10k]─────┬─────┬─── PA3
                           │     │
                          [30k]  [0.1µF]
                           │     │
                          GND    GND
   
   Voltage at PA3 = Battery * 30k / (10k + 30k) = Battery * 0.75
   For 3.7V LiPo → 2.77V at PA3

3. BUTTON PULLUP CONFIGURATION:
   
   Each button:
   ────────────
   Button ─┬─── PB0/PB1/PB2/PB3 (with internal pull-up)
           │
          [Button to GND when pressed]
   
   The STM32 internal pull-up is enabled in firmware.
   When button is pressed: GPIO reads LOW (false)
   When button is released: GPIO reads HIGH (true)
   Inverted in code: button_state = !digitalRead(BTN)

4. CALIBRATION:
   
   a) Battery voltage divider:
      - Measure actual battery voltage
      - Measure voltage at PA3 while measuring battery
      - Calculate: BATTERY_DIVIDER_RATIO = PA3_voltage / Battery_voltage
      - Update BATTERY_DIVIDER_RATIO in code
   
   b) ADC zero offset:
      - Move joystick to center
      - Note X and Y readings
      - Adjust JOYSTICK_CENTER in receiving Python code
   
   c) Button sensitivity:
      - Test each button press/release
      - Verify JSON shows true/false correctly
      - Check for bounce (add debounce if needed)

5. TESTING:
   
   a) USB Debug Output:
      - Open Arduino Serial Monitor (115200 baud)
      - Enable DEBUG_SERIAL = 1 in code
      - Move joystick, press buttons
      - Verify readings on monitor
   
   b) UART to Pi:
      - On Raspberry Pi:
        # Read UART port
        cat /dev/ttyUSB0
      - Move joystick on Nucleo
      - You should see binary packets and JSON lines
   
   c) Full Integration:
      - Run Python application
      - Check logs for telemetry reception
      - Verify overlay displays battery and buttons

6. TROUBLESHOOTING:
   
   Issue: No binary packets received
   ────────────────────────────────
   - Check UART cable connections (TX/RX not swapped)
   - Verify baud rate is 115200
   - Test with `cat /dev/ttyUSB0` on Pi
   
   Issue: Battery reading incorrect
   ────────────────────────────────
   - Verify voltage divider resistor values
   - Measure actual voltage at PA3 with multimeter
   - Recalibrate BATTERY_DIVIDER_RATIO
   
   Issue: Buttons not working
   ──────────────────────────
   - Check GPIO pin connections
   - Test with Serial.println(digitalRead(BTN1))
   - Verify pull-up is working (should read HIGH when not pressed)
   
   Issue: Temperature always 25°C
   ───────────────────────────────
   - Internal STM32 sensor is inaccurate
   - Use external temp sensor (e.g., DS18B20)
   - Replace get_internal_temperature() with external sensor reading

7. PERFORMANCE:
   
   - Binary packet rate: 100Hz (10ms interval)
   - Telemetry rate: 10Hz (100ms interval)
   - Battery filter: 0.1Hz effective (smooth variations)
   - Total latency: <1ms UART, ~10ms UI update on Pi

====================================================
*/
