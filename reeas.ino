#include <Wire.h>

const uint8_t MPU6050Address = 0x68;
float accelX, accelY, accelZ;
int16_t sampleX, sampleY, sampleZ;
float calibratedX, calibratedY, calibratedZ;
byte counter = 0;
unsigned long currentms, lastms;

void setup() {
  Wire.begin(D7, D6);
  Serial.begin(9600);
  MPUSetup();
  Calibration();                                    //Calibrate sensor during startup
}

void loop() {
  readAcceleration();
  accelX = sampleX - calibratedX;
  accelY = sampleY - calibratedY;
  accelZ = sampleZ - calibratedZ;
  Serial.print("Accel X: ");
  Serial.print(accelX / 16384.0);
  Serial.print("  Accel Y: ");
  Serial.print(accelY / 16384.0);
  Serial.print("  Accel Z: ");
  Serial.println(accelZ / 16384.0);
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
  Wire.requestFrom(MPU6050Address, 6);              //Request Accel Registers (3B - 40)
  while (Wire.available() < 6);
  sampleX = Wire.read() << 8 | Wire.read();
  sampleY = Wire.read() << 8 | Wire.read();
  sampleZ = Wire.read() << 8 | Wire.read();
  /*Serial.print("Accel X: ");
    Serial.print(accelX/ 16384.0);
    //Serial.print("  Accel Y: ");
    Serial.print(accelY/ 16384.0);
    //Serial.print("  Accel Z: ");
    Serial.println(accelZ/ 16384.0);*/
}

void Calibration() {
  counter = 0;
  readAcceleration();
  delay(100);
  Serial.print("Calibrating");
  while (counter <= 100) {
    currentms = millis();
    if (currentms - lastms >= 1) {                  //get sample every 1ms (Accelerometer Sampling rate is 1kz)
      lastms = currentms;
      
      calibratedX = calibratedX + sampleX;
      calibratedY = calibratedY + sampleY;
      calibratedZ = calibratedZ + sampleZ;
      counter++;
      Serial.print(counter);
    }
  }
  Serial.println("Done Sampling");
  counter = 0;
  calibratedX = calibratedX / 100;
  calibratedY = calibratedY / 100;
  calibratedZ = calibratedZ / 100;
  Serial.println("Calibration done.");
  Serial.print("X offset: ");
  Serial.print(calibratedX);
  Serial.print(" Y offset: ");
  Serial.print(calibratedY);
  Serial.print(" Z offset: ");
  Serial.println(calibratedZ);
}
