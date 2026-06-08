#include <Arduino.h>

/* ---- CRC ---- */
String crc(const String& s) {
  uint8_t sum = 0;
  for (int i = 0; i < s.length(); i++) sum += s[i];

  char buf[3];
  sprintf(buf, "%02X", sum);
  return String(buf);
}

/* ---- helpers ---- */
String hex16(int v) {
  char b[5];
  sprintf(b, "%04X", v & 0xFFFF);
  return String(b);
}

String hex8(int v) {
  char b[3];
  sprintf(b, "%02X", v & 0xFF);
  return String(b);
}

/* ---- GAP command (Pitch Angle Mode) ---- */
String buildGAP(float angle, int speed) {

  int scaled = (int)(angle * 100);   // 50.0 → 5000 → 0x1388

  String data = hex16(scaled) + hex8(speed);   // "138810"

  String body = "wGAP" + data;

  char len[2];
  sprintf(len, "%X", body.length());   // length = 6

  String frame = "#TPUG" + String(len) + body;

  return frame + crc(frame);           // append CRC
}

void sendCmd(const String& s) {
  Serial0.print(s);   // send via UART0 (GPIO21)
  Serial.println("TX: " + s); // debug on USB
}

void setup() {
  Serial.begin(115200);   // USB debug

  // 🔥 Force UART0 to use GPIO20 (RX) and GPIO21 (TX)
  Serial0.begin(115200, SERIAL_8N1, 20, 21);

  delay(3000);  // allow C12 to boot

  String cmd = buildGAP(50.0, 16);

  // send twice (important)
  sendCmd(cmd);
  delay(50);
  sendCmd(cmd);
}

void loop() {
}