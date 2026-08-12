#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  delay(500); // Allow custom chip boot cycle stability
  
  Wire.beginTransmission(0x42);
  Wire.write(0x00); // Point to WHO_AM_I register
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print("I2C Error Connecting: ");
    Serial.println(error);
    while(1);
  }
  
  Wire.requestFrom(0x42, 1);
  if (Wire.available()) {
    uint8_t who_am_i = Wire.read();
    Serial.print("Read WHO_AM_I: 0x");
    Serial.println(who_am_i, HEX);
    
    if (who_am_i == 0x51) {
      Serial.println("VERIFICATION GATE: M0 PASSED");
    } else {
      Serial.println("VERIFICATION GATE: M0 FAILED - Bad Payload");
    }
  }
}

void loop() {}