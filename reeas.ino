#include <Wire.h>

const uint8_t MPU6050Address = 0x68;
float accelX, accelY, accelZ;
int16_t sampleX, sampleY, sampleZ;
float calibratedX, calibratedY, calibratedZ;
float sample2X, sample2Y, sample2Z;
byte counter = 0;
unsigned long currentms, lastms, timer;
const int LPFCounter = 100;
const int CalibrationCounter = 200;

void setup() {
  Wire.begin(D7, D6);
  Serial.begin(9600);
  MPUSetup();
  Calibration();                                    //Calibrate sensor during startup
}

void loop() {
  timer = millis();
  counter = 0;
  while (counter <= LPFCounter) {
    currentms = millis();
    if (currentms - lastms >= 1) {                  //get sample every 1ms (Accelerometer Sampling rate is 1kHz)
      lastms = currentms;
      counter++;
      readAcceleration();
      sample2X += sampleX;
      sample2Y += sampleY;
      sample2Z += sampleZ;
    }
  }
  counter = 0;
  sample2X = sample2X / LPFCounter;
  sample2Y = sample2Y / LPFCounter;
  sample2Z = sample2Z / LPFCounter;
  accelX = sample2X - calibratedX;
  accelY = sample2Y - calibratedY;
  accelZ = sample2Z - calibratedZ;
  sample2X = 0;
  sample2Y = 0;
  sample2Z = 0;
  Serial.print(millis() - timer);
  Serial.print("ms ");
  Serial.print("Accel X: ");
  Serial.print(accelX / 1670.7, 4);
  Serial.print("  Accel Y: ");
  Serial.print(accelY / 1670.7, 4);
  Serial.print("  Accel Z: ");
  Serial.println(accelZ / 1670.7, 4);
  delay(100);
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

  Wire.beginTransmission(MPU6050Address);
  Wire.write(0x1A);                                 //DLPF Config Register
  Wire.write(0x04);                                 //Setting DLPF to 21hz
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
  Serial.print(sampleX / 16384.0, 4);
  Serial.print("  Accel Y: ");
  Serial.print(sampleY / 16384.0, 4);
  Serial.print("  Accel Z: ");
  Serial.println(sampleZ / 16384.0, 4);*/
}

void Calibration() {
  counter = 0;
  readAcceleration();
  delay(100);
  Serial.print("Calibrating");
  while (counter < CalibrationCounter) {
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
  calibratedX = calibratedX / CalibrationCounter;
  calibratedY = calibratedY / CalibrationCounter;
  calibratedZ = calibratedZ / CalibrationCounter;
  Serial.println("Calibration done.");
  Serial.print("X offset: ");
  Serial.print(calibratedX);
  Serial.print(" Y offset: ");
  Serial.print(calibratedY);
  Serial.print(" Z offset: ");
  Serial.println(calibratedZ);
}
