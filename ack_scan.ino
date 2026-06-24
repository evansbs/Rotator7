#include <Wire.h>

static void printHex2(uint8_t v) {
  if (v < 16) Serial.print('0');
  Serial.print(v, HEX);
}

void setup() {
  Serial.begin(9600);
  delay(1500);

  Wire.begin();
  Wire.setClock(50000);

  Serial.println("ACK-only I2C scan start");

  int found = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("ACK 0x");
      printHex2(addr);
      Serial.println();
      found++;
    }
    delay(2);
  }

  Serial.print("Scan done, found ");
  Serial.print(found);
  Serial.println(" device(s)");
}

void loop() {
}
