/*
  esp_telemetry.ino
  ESP (ESP32/ESP8266/Arduino) sketch to read:
    - analog joystick (X/Y and optional buttons)
    - battery voltage via ADC + voltage divider
    - IMU (MPU6050) via I2C
  and send newline-delimited JSON telemetry lines to Serial (115200) for the Raspberry Pi to read.

  Wiring (example):
    - Joystick X -> ADC pin (config JOY_X_PIN)
    - Joystick Y -> ADC pin (config JOY_Y_PIN)
    - Button A -> digital pin (config BUTTON_A_PIN)
    - Button B -> digital pin (config BUTTON_B_PIN)
    - Battery sense -> ADC pin through divider (config BATTERY_PIN)
    - MPU6050 SDA -> SDA pin (Wire), SCL -> SCL pin
    - GND and 3V3 shared

  Notes:
    - Configure voltage divider resistors R1 and R2 to scale your battery into ADC input range.
    - The sketch auto-detects ESP32 vs ESP8266 ADC ranges; on AVR use 10-bit ADC.
    - Output format: JSON object per line, example:
      {"type":"telemetry","bat":11.42,"imu":{"ax":-0.01,"ay":0.05,"az":0.98,"gx":0.35,"gy":-1.2,"gz":0.05},"joy":{"x":512,"y":523,"btn":[0,1]}}
*/

#include <Arduino.h>
#include <Wire.h>

// ----- Config -----
#define SERIAL_BAUD 115200

// Joystick pins - set to analog-capable pins on your board
#define JOY_X_PIN A0
#define JOY_Y_PIN A1
#define BUTTON_A_PIN 2
#define BUTTON_B_PIN 0

// Battery sense pin
#define BATTERY_PIN A2

// Voltage divider: R1 between battery+ and VIN pin, R2 between VIN pin and GND
// Vmeasured = Vbattery * (R2 / (R1 + R2))
// Set these to match your hardware; default assumes equal resistors (factor 2)
const float R1 = 100000.0; // ohms
const float R2 = 100000.0; // ohms

// IMU (MPU6050) I2C address
#define MPU_ADDR 0x68

// Telemetry interval (ms)
const uint32_t TELEMETRY_MS = 100; // 10 Hz by default

// Thresholds for change-based sends (optional)
const int JOY_CHANGE_THRESH = 4; // raw ADC units
const float BAT_VOLT_CHANGE_THRESH = 0.05; // volts

// ----- End config -----

// ADC characteristics - filled in setup based on platform
uint32_t adc_max = 1023; // default 10-bit
float adc_vref = 3.3;

uint32_t last_joy_x = 0;
uint32_t last_joy_y = 0;
float last_bat_v = 0.0;

uint32_t last_sent_ms = 0;

// Helper to initialize ADC calibration per platform
void init_adc_params() {
#if defined(ESP32)
  adc_max = 4095;
  adc_vref = 3.3; // typical; if using ADC calibration, adjust
#elif defined(ESP8266)
  adc_max = 1023;
  adc_vref = 1.0; // on many ESP8266 boards the ADC reads Vbat/Vref through onboard divider; adjust as needed
#else
  // AVR / other
  adc_max = 1023;
  adc_vref = 5.0; // or 3.3 depending on board
#endif
}

// MPU6050 helpers (lightweight, no external library)
void mpu_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpu_read_regs(uint8_t reg, uint8_t count, uint8_t *buf) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, count);
  uint8_t i = 0;
  while (Wire.available() && i < count) buf[i++] = Wire.read();
}

bool mpu_begin() {
  // wake up
  mpu_write(0x6B, 0x00);
  delay(10);
  // set accel range to +/-2g (0x00) and gyro range to +/-250deg/s (0x00) - defaults
  // verify device present by reading WHO_AM_I (0x75)
  uint8_t who = 0;
  mpu_read_regs(0x75, 1, &who);
  return (who == 0x68);
}

void read_mpu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint8_t buf[14];
  mpu_read_regs(0x3B, 14, buf);
  int16_t raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t raw_az = (int16_t)((buf[4] << 8) | buf[5]);
  int16_t raw_temp = (int16_t)((buf[6] << 8) | buf[7]);
  int16_t raw_gx = (int16_t)((buf[8] << 8) | buf[9]);
  int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
  int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

  // conversions
  // MPU6050 default: accel LSB = 16384 per g for +/-2g
  ax = (float)raw_ax / 16384.0;
  ay = (float)raw_ay / 16384.0;
  az = (float)raw_az / 16384.0;
  // gyro default LSB = 131 per deg/s for +/-250
  gx = (float)raw_gx / 131.0;
  gy = (float)raw_gy / 131.0;
  gz = (float)raw_gz / 131.0;
}

float read_battery_voltage() {
  uint32_t raw = analogRead(BATTERY_PIN);
  float v_meas = ((float)raw / (float)adc_max) * adc_vref;
  float divider = R2 / (R1 + R2);
  float v_bat = v_meas / divider;
  return v_bat;
}

uint32_t read_joy_raw(int pin) {
  return analogRead(pin);
}

void send_telemetry(float bat_v, float ax, float ay, float az, float gx, float gy, float gz, uint32_t jx, uint32_t jy, int btnA, int btnB) {
  // Build a compact JSON object as a single line
  // Avoid dynamic allocation; use String for simplicity
  String s = "{";
  s += "\"type\":\"telemetry\",";
  s += "\"bat\":";
  s += String(bat_v, 2);
  s += ",\"imu\":{";
  s += "\"ax\":" + String(ax, 3) + ",\"ay\":" + String(ay, 3) + ",\"az\":" + String(az, 3);
  s += ",\"gx\":" + String(gx, 3) + ",\"gy\":" + String(gy, 3) + ",\"gz\":" + String(gz, 3) + "}";
  s += ",\"joy\":{";
  s += "\"x\":" + String(jx) + ",\"y\":" + String(jy) + ",\"btn\": [" + String(btnA) + "," + String(btnB) + "]}";
  s += "}";

  Serial.println(s);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    ; // wait for Serial on boards that need it
  }
  init_adc_params();

  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_B_PIN, INPUT_PULLUP);

  Wire.begin();
  delay(10);
  bool ok = mpu_begin();
  if (!ok) {
    Serial.println("{\"type\":\"status\",\"mpu\":false}");
  } else {
    Serial.println("{\"type\":\"status\",\"mpu\":true}");
  }

  // warm up reads
  last_joy_x = read_joy_raw(JOY_X_PIN);
  last_joy_y = read_joy_raw(JOY_Y_PIN);
  last_bat_v = read_battery_voltage();
  last_sent_ms = millis();
}

void loop() {
  uint32_t now = millis();
  bool should_send = false;

  uint32_t jx = read_joy_raw(JOY_X_PIN);
  uint32_t jy = read_joy_raw(JOY_Y_PIN);
  int btnA = digitalRead(BUTTON_A_PIN) == LOW ? 1 : 0; // active low
  int btnB = digitalRead(BUTTON_B_PIN) == LOW ? 1 : 0;

  float bat_v = read_battery_voltage();

  // detect changes
  if (abs((int)jx - (int)last_joy_x) > JOY_CHANGE_THRESH || abs((int)jy - (int)last_joy_y) > JOY_CHANGE_THRESH) {
    should_send = true;
  }
  if (fabs(bat_v - last_bat_v) > BAT_VOLT_CHANGE_THRESH) {
    should_send = true;
  }
  if (now - last_sent_ms >= TELEMETRY_MS) {
    should_send = true;
  }

  if (should_send) {
    float ax, ay, az, gx, gy, gz;
    read_mpu(ax, ay, az, gx, gy, gz);
    send_telemetry(bat_v, ax, ay, az, gx, gy, gz, jx, jy, btnA, btnB);

    last_joy_x = jx;
    last_joy_y = jy;
    last_bat_v = bat_v;
    last_sent_ms = now;
  }

  
  delay(5);
}
