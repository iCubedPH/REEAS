#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <ArduinoJson.h>

const char *ssid     = "AndroidAPEA68";
const char *password = "icubed88";

const int BUFFER_SIZE = 50;
const int BUFFER_SIZE_SAMPLE = 275;
const int STACount = 25, LTACount = 250;
const uint8_t MPU6050Address = 0x68;
const int LPFCounter = 20;
const int CalibrationCounter = 250;
const byte deviceID = 1;

byte counter = 0, waveType = 0, idleCounter = 50, indexOfBuffer, staIndex, ltaIndex;                 //Wave type : 0 for P-Wave, 1 for S-Wave

float accelX, accelY, accelZ;
float xSTA, xLTA, ySTA, yLTA, zSTA, zLTA, xRatio, yRatio, zRatio;
int16_t sampleX, sampleY, sampleZ;
long calibratedX, calibratedY, calibratedZ;
float sample2X, sample2Y, sample2Z;
unsigned long currentms, lastms, timer;

float xAccBuffer[BUFFER_SIZE], yAccBuffer[BUFFER_SIZE], zAccBuffer[BUFFER_SIZE];
float xSampleBuffer[BUFFER_SIZE_SAMPLE], ySampleBuffer[BUFFER_SIZE_SAMPLE], zSampleBuffer[BUFFER_SIZE_SAMPLE];

float temp[3];
int tc;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void setup() {
  Wire.begin(D4, D3);
  Serial.begin(115200);

  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  timeClient.begin();
  timeClient.update();
  timeClient.setTimeOffset(28800);

  MPUSetup();
  Calibration();                                    //Calibrate sensor during startup
}

void loop() {
  readCalibratedAcceleration();
  calculateSTALTARatio();
  
    Serial.print("xRatio: ");
    Serial.print(xRatio, 4);
    Serial.print(" yRatio: ");
    Serial.print(yRatio, 4);
    Serial.print(" zRatio: ");
    Serial.println(zRatio, 4);
  /*
  if (tc < 300) {
    tc++;
    Serial.print(".");
  }
  else {
    Serial.println("");
    if (xRatio > temp[0]) {
      temp[0] = xRatio;
      Serial.print("xRatio: ");
      Serial.print(temp[0], 4);
      Serial.print(" yRatio: ");
      Serial.print(temp[1], 4);
      Serial.print(" zRatio: ");
      Serial.println(temp[2], 4);
    }

    if (yRatio > temp[1]) {
      temp[1] = yRatio;
      Serial.print("xRatio: ");
      Serial.print(temp[0], 4);
      Serial.print(" yRatio: ");
      Serial.print(temp[1], 4);
      Serial.print(" zRatio: ");
      Serial.println(temp[2], 4);
    }

    if (zRatio > temp[2]) {
      temp[2] = zRatio;
      Serial.print("xRatio: ");
      Serial.print(temp[0], 4);
      Serial.print(" yRatio: ");
      Serial.print(temp[1], 4);
      Serial.print(" zRatio: ");
      Serial.println(temp[2], 4);
    }
  }*/
  //-----------------------------------------------------------------------------
  /*
    if ((accelX != 0) || (accelY != 0) || (accelZ != 0)) {
    while (idx < BUFFER_SIZE) {
      readCalibratedAcceleration();
      xAccBuffer[idx] = accelX;
      yAccBuffer[idx] = accelY;
      zAccBuffer[idx] = accelZ;
      idx++;
    }
    idx = 0;
    serializeToJSON();
    }*/
    /*
  Serial.print(millis() - timer);
  Serial.print("ms ");
  Serial.print("Accel X: ");
  Serial.print(accelX, 4);
  Serial.print("  Accel Y: ");
  Serial.print(accelY, 4);
  Serial.print("  Accel Z: ");
  Serial.println(accelZ, 4);
*/
}

void MPUSetup() {
  Serial.println("MPU Setup..");
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
  Wire.write(0x00);                                 //Setting DLPF to 21hz
  Wire.endTransmission();
  Serial.println("Done Setting up..");
}

void readRawAcceleration() {
  Wire.beginTransmission(MPU6050Address);
  Wire.write(0x3B);                                 //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(MPU6050Address, 6);              //Request Accel Registers (3B - 40)
  while (Wire.available() < 6);
  sampleX = Wire.read() << 8 | Wire.read();
  sampleY = Wire.read() << 8 | Wire.read();
  sampleZ = Wire.read() << 8 | Wire.read();
}

void readCalibratedAcceleration() {
  timer = millis();
  counter = 0;
  while (counter <= LPFCounter) {
    currentms = millis();
    if (currentms - lastms >= 1) {                  //get sample every 1ms (Accelerometer Max Sampling rate is 1kHz)
      lastms = currentms;
      counter++;
      readRawAcceleration();
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
  accelX = accelX / 1670.7;
  accelY = accelY / 1670.7;
  accelZ = accelZ / 1670.7;
  /*
    if ((accelX < 0.05) && (accelX > -0.05)) {
    accelX = 0;
    }
    if ((accelY < 0.05) && (accelY > -0.05)) {
    accelY = 0;
    }
    if ((accelZ < 0.5) && (accelZ > -0.5)) {
    accelZ = 0;
    }*/
}

void Calibration() {
  Serial.println("Beginning Calibration..");
  counter = 0;
  readRawAcceleration();
  delay(200);
  Serial.print("Calibrating");
  while (counter < CalibrationCounter) {
    currentms = millis();
    if (currentms - lastms >= 1) {                  //get sample every 1ms (Accelerometer Sampling rate is 1kz)
      lastms = currentms;
      readRawAcceleration();
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
  calibratedZ = calibratedZ + 835;
  Serial.println("Calibration done.");
  Serial.print("X offset: ");
  Serial.print(calibratedX);
  Serial.print(" Y offset: ");
  Serial.print(calibratedY);
  Serial.print(" Z offset: ");
  Serial.println(calibratedZ);
}

void calculateSTALTARatio() {
  xSampleBuffer[indexOfBuffer] = accelX;
  ySampleBuffer[indexOfBuffer] = accelY;
  zSampleBuffer[indexOfBuffer] = accelZ;
  ltaIndex = indexOfBuffer + 1;
  if (ltaIndex >= LTACount + STACount) {
    ltaIndex = 0;
  }
  for (int i = ltaIndex, iteration = 0; iteration < LTACount ; iteration++) {
    if (i >= LTACount + STACount) {
      i = 0;
    }
    xLTA = xLTA + xSampleBuffer[i];
    yLTA = yLTA + ySampleBuffer[i];
    zLTA = zLTA + zSampleBuffer[i];
    i++;
    staIndex = i;
  }
  xLTA = xLTA / LTACount;
  yLTA = yLTA / LTACount;
  zLTA = zLTA / LTACount;

  if (staIndex >= LTACount + STACount) {
    staIndex = 0;
  }

  for (int i = staIndex, iteration = 0; iteration < STACount ; iteration++) {
    if (i >= LTACount + STACount) {
      i = 0;
    }
    xSTA = xSTA + xSampleBuffer[i];
    ySTA = ySTA + ySampleBuffer[i];
    zSTA = zSTA + zSampleBuffer[i];
    i++;
  }
  xSTA = xSTA / STACount;
  ySTA = ySTA / STACount;
  zSTA = zSTA / STACount;
  indexOfBuffer++;
  if (indexOfBuffer >= LTACount + STACount) {
    indexOfBuffer = 0;
  }
  xRatio = xSTA / xLTA;
  yRatio = ySTA / yLTA;
  zRatio = zSTA / zLTA;
}

void serializeToJSON() {
  const size_t capacity = 3 * JSON_ARRAY_SIZE(50) + JSON_OBJECT_SIZE(6);
  DynamicJsonDocument doc(capacity);

  doc["id"] = deviceID;
  doc["wave"] = waveType;
  doc["time"] = timeClient.getEpochTime();
  JsonArray xacc = doc.createNestedArray("xacc");
  for (int i = 0; i < BUFFER_SIZE; i++) {
    xacc.add(xAccBuffer[i]);
  }
  JsonArray yacc = doc.createNestedArray("yacc");
  for (int i = 0; i < BUFFER_SIZE; i++) {
    yacc.add(yAccBuffer[i]);
  }
  JsonArray zacc = doc.createNestedArray("zacc");
  for (int i = 0; i < BUFFER_SIZE; i++) {
    zacc.add(zAccBuffer[i]);
  }
  serializeJson(doc, Serial);
}
