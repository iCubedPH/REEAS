#include <Wire.h>
const uint8_t MPU6050Address = 0x68;
int16_t accelX, accelY, accelZ;

void setup() {
  Wire.begin(D7, D6);
  Serial.begin(9600);
  MPUSetup();
}

void loop() {
  readAcceleration();
  delay(10);
}

void MPUSetup() {
  Wire.beginTransmission(MPU6050Address);
  Wire.write(0x6B);                                 //Power Management Register
  Wire.write(0x00);                                 //Turn off sleep mode
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050Address);
  Wire.write(0x1C);                                 //Accelerometer Config Register
  Wire.write(0x00);                                 //Setting the accel to +/- 2g
  Wire.endTransmission();
}

void readAcceleration() {
  Wire.beginTransmission(MPU6050Address);
  Wire.write(0x3B);                                 //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(MPU6050Address,6);               //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read();
  accelY = Wire.read()<<8|Wire.read();
  accelZ = Wire.read()<<8|Wire.read(); 
  Serial.print("Accel X: ");
  Serial.print(accelX/ 16384.0);
  Serial.print("  Accel Y: ");
  Serial.print(accelY/ 16384.0);
  Serial.print("  Accel Z: ");
  Serial.println(accelZ/ 16384.0);
}
