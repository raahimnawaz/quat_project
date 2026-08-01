#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C for faster reads
  delay(100); 
  
  Serial.println("time_ms,norm_error,energy,wx,wy,wz");
}

void loop() {
  float q[4];
  float w[3];
  
  // Read q_true (4 floats = 16 bytes) in ONE transaction
  Wire.beginTransmission(0x42);
  Wire.write(0x30); 
  Wire.endTransmission(false);
  Wire.requestFrom(0x42, 16);
  Wire.readBytes((uint8_t*)q, 16);

  // Read w_true (3 floats = 12 bytes) in ONE transaction
  Wire.beginTransmission(0x42);
  Wire.write(0x40);
  Wire.endTransmission(false);
  Wire.requestFrom(0x42, 12);
  Wire.readBytes((uint8_t*)w, 12);

  // 1. Norm Check
  float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  float norm_error = (norm - 1.0f) * 1e6; // Scaled for plotter visibility

  // 2. Energy Conservation (E = 0.5 * sum(J * w^2))
  float energy = 0.5f * (1.0f * w[0]*w[0] + 2.0f * w[1]*w[1] + 3.0f * w[2]*w[2]);

  // Output as CSV
  Serial.print(millis());
  Serial.print(",");
  Serial.print(norm_error, 6);
  Serial.print(",");
  Serial.print(energy, 6);
  Serial.print(",");
  Serial.print(w[0], 4);
  Serial.print(",");
  Serial.print(w[1], 4);
  Serial.print(",");
  Serial.println(w[2], 4);
  
  delay(20); // 50 Hz reporting rate
}
