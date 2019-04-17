#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <Ticker.h>

#define STALTATRIGGER 15
#define WIFI_LED D8
#define STATUS_LED D7
#define BUZZER D5
#define CD_TIME 6        //sending cooldown in 10s per count


const float latitude = 14.33333;
const float longitude = 120.96008;
String deviceID = String("CAV02");

const int BUFFER_SIZE = 50;
const int BUFFER_SIZE_SAMPLE = 275;
const int STACount = 25, LTACount = 250;
const uint8_t MPU6050Address = 0x68;
const int LPFCounter = 20;
const int CalibrationCounter = 250;

int counter = 0, indexOfBuffer, staIndex, ltaIndex, start_up = 2;

float accelX, accelY, accelZ;
float xSTA, xLTA, ySTA, yLTA, zSTA, zLTA, xRatio, yRatio, zRatio;
int16_t sampleX, sampleY, sampleZ;
long calibratedX, calibratedY, calibratedZ;
float sample2X, sample2Y, sample2Z;
unsigned long currentms, lastms, timer, beep_ms;

float xAccBuffer[BUFFER_SIZE], yAccBuffer[BUFFER_SIZE], zAccBuffer[BUFFER_SIZE];
float xSampleBuffer[BUFFER_SIZE_SAMPLE], ySampleBuffer[BUFFER_SIZE_SAMPLE], zSampleBuffer[BUFFER_SIZE_SAMPLE];

int tc, idx, beep_counter, beep_counter_2, status_counter = 18, cd_counter_h_axis = 0, cd_counter_v_axis = 0;
bool h_sent = false, v_sent = false, beep = true, send_ready = false, beep_once = false;   //booleans for checking if sensor sent data

Ticker ticker;
Ticker timer1;
Ticker checker;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void setup() {
  Wire.begin(D4, D3);
  Serial.begin(115200);

  pinMode(WIFI_LED, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  digitalWrite(BUZZER, LOW);

  ticker.attach(0.8, tick);

  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.setAPCallback(configModeCallback);
  if (!wifiManager.autoConnect()) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again
    ESP.reset();
    delay(5000);
  }
  Serial.println("Connected to Wifi");
  ticker.detach();
  digitalWrite(WIFI_LED, HIGH);

  /*
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
  */
  timeClient.begin();
  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }
  timeClient.setTimeOffset(28800);
  Serial.println(timeClient.getFormattedTime());
  //-----------------------------
  const size_t capacity = JSON_OBJECT_SIZE(5);
  DynamicJsonDocument doc(capacity);
  char JSONBUFFER[capacity];

  doc["station"] = deviceID;
  doc["latitude"] = latitude;
  doc["longitude"] = longitude;
  doc["enabled"] = 1;
  doc["enabled"] = 1;

  serializeJson(doc, JSONBUFFER);
  //serializeJson(doc, Serial);

  HTTPClient http;    //Declare object of class HTTPClient

  http.begin("http://www.reeas-web.com:3001/stations");      //Specify request destination
  http.addHeader("Content-Type", "application/json");  //Specify content-type header

  int httpCode = http.POST(JSONBUFFER);   //Send the request
  String payload = http.getString();                  //Get the response payload

  Serial.println(httpCode);   //Print HTTP return code
  Serial.println(payload);    //Print request response payload

  http.end();  //Close connection
  //--------------------------
  MPUSetup();
  Calibration();                                    //Calibrate sensor during startup
  Serial.print("Filling Sample Buffer...");
  /*
    for (int i = 0; i < 50; i++) {      //flush initial values to eliminate noise
    readCalibratedAcceleration();
    calculateSTALTARatio();
    }

    //for (int x = 0; x <= 100; x++) {
    while ((xRatio > 2) || (yRatio > 2) || (zRatio > 2) || (xRatio < -2) || (yRatio < -2) || (zRatio < -2)) {
    readCalibratedAcceleration();
    calculateSTALTARatio();
    Serial.print(".");
    }
    //}
  */
  /*
    currentms = millis();
    while (millis() - currentms <= 30000) {
      readCalibratedAcceleration();
      calculateSTALTARatio();
      Serial.print(".");
    }
  */
  Serial.println("");
  timer1.attach(10, cooldown);
  digitalWrite(STATUS_LED, HIGH);
  checker.attach(30, check_connection);
}

void loop() {

  readCalibratedAcceleration();
  //calculateSTALTARatio();
  /*
    Serial.print(millis() - timer);
    Serial.print("ms ");
    Serial.print("Accel X: ");
    Serial.print(accelX, 4);
    Serial.print("  Accel Y: ");
    Serial.print(accelY, 4);
    Serial.print("  Accel Z: ");
    Serial.println(accelZ, 4);
    Serial.print("xRatio: ");
    Serial.print(xRatio, 4);
    Serial.print(" yRatio: ");
    Serial.print(yRatio, 4);
    Serial.print(" zRatio: ");
    Serial.println(zRatio, 4);


  */

  if (send_ready == true) {
    if (v_sent == false) {
      /*
        if (zRatio > STALTATRIGGER) {
        idx = 0;
        while (idx < BUFFER_SIZE) {
          readCalibratedAcceleration();
          calculateSTALTARatio();
          xAccBuffer[idx] = accelX;
          yAccBuffer[idx] = accelY;
          zAccBuffer[idx] = accelZ;
          idx++;
        }
        idx = 0;
        serializeToJSON(0);
      */
      if ((accelX > 167) || (accelX < -167)) {
        while (idx < BUFFER_SIZE) {
          readCalibratedAcceleration();
          xAccBuffer[idx] = accelX;
          yAccBuffer[idx] = accelY;
          zAccBuffer[idx] = accelZ;
          idx++;
        }
        idx = 0;
        serializeToJSON(0);
        v_sent = true;
        beep = true;
      }
    }

    if (h_sent == false) {
      /*
        if ((xRatio > STALTATRIGGER) || (yRatio > STALTATRIGGER)) {
        idx = 0;
        while (idx < BUFFER_SIZE) {
          readCalibratedAcceleration();
          calculateSTALTARatio();
          xAccBuffer[idx] = accelX;
          yAccBuffer[idx] = accelY;
          zAccBuffer[idx] = accelZ;
          idx++;
        }
        idx = 0;
      */
      if ((accelY > 100) || (accelY < -100) || (accelZ > 100) || (accelZ < -100)) {
        idx = 0;
        while (idx < BUFFER_SIZE) {
          readCalibratedAcceleration();
          xAccBuffer[idx] = accelX;
          yAccBuffer[idx] = accelY;
          zAccBuffer[idx] = accelZ;
          idx++;
        }
        idx = 0;
        serializeToJSON(1);
        v_sent = true;
        beep = true;
      }

    }
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

    //beeper();
  }
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
  sampleZ = Wire.read() << 8 | Wire.read();
  sampleY = Wire.read() << 8 | Wire.read();
  sampleX = Wire.read() << 8 | Wire.read();
}

void readCalibratedAcceleration() {
  timer = millis();
  counter = 0;
  while (counter < LPFCounter) {
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
  //accelX = accelX / 1670.13;
  //accelY = accelY / 1670.13;
  //accelZ = accelZ / 1670.13;
  /*
    if ((accelX < 84) && (accelX > -84)) {
      accelX = 0;
    }
    if ((accelY < 84) && (accelY > -84)) {
      accelY = 0;
    }
    if ((accelZ < 84) && (accelZ > -84)) {
      accelZ = 0;
    }
  */
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
  //calibratedZ = calibratedZ + 835;
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

void serializeToJSON(int waveType) {    //Wave type : 0 for P-Wave, 1 for S-Wave
  const size_t capacity = 3 * JSON_ARRAY_SIZE(BUFFER_SIZE) + JSON_OBJECT_SIZE(7);
  DynamicJsonDocument doc(capacity);
  char JSONBUFFER[capacity];

  doc["station"] = deviceID;
  doc["timestamp"] = timeClient.getEpochTime();
  doc["wave"] = waveType;


  JsonArray va = doc.createNestedArray("va");
  for (int i = 0; i < BUFFER_SIZE; i++) {
    va.add(zAccBuffer[i]);
  }

  JsonArray ewa = doc.createNestedArray("ewa");
  for (int i = 0; i < BUFFER_SIZE; i++) {
    ewa.add(yAccBuffer[i]);
  }

  JsonArray nsa = doc.createNestedArray("nsa");
  for (int i = 0; i < BUFFER_SIZE; i++) {
    nsa.add(xAccBuffer[i]);
  }
  doc["wave"] = waveType;
  serializeJson(doc, JSONBUFFER);
  serializeJson(doc, Serial);

  HTTPClient http;    //Declare object of class HTTPClient

  http.begin("http://www.reeas-web.com:3001/detections");      //Specify request destination
  http.addHeader("Content-Type", "application/json");  //Specify content-type header

  int httpCode = http.POST(JSONBUFFER);   //Send the request
  String payload = http.getString();                  //Get the response payload

  Serial.println(httpCode);   //Print HTTP return code
  Serial.println(payload);    //Print request response payload

  http.end();  //Close connection
}

void tick()
{
  //toggle state
  int state = digitalRead(WIFI_LED);  // get the current state of GPIO1 pin
  digitalWrite(WIFI_LED, !state);     // set pin to the opposite state
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

void cooldown() {
  if (start_up > 0) {
    start_up--;
    if (start_up == 0) {
      send_ready = true;
    }
  }
  if (h_sent == true) {
    cd_counter_h_axis++;
    if (cd_counter_h_axis > CD_TIME) {
      cd_counter_h_axis = 0;
      h_sent = false;
    }
  }
  if (v_sent == true) {
    cd_counter_v_axis++;
    if (cd_counter_v_axis > CD_TIME) {
      cd_counter_v_axis = 0;
      v_sent = false;
    }
  }
}

void beeper() {
  if (beep == true) {
    currentms = millis();
    if (currentms - beep_ms >= 500) {
      beep_ms = currentms;
      int state = digitalRead(BUZZER);
      digitalWrite(BUZZER, !state);
      beep_counter++;
      if (beep_counter > 6) {
        beep_counter = 0;
        beep = false;
        digitalWrite(BUZZER, LOW);
      }
    }
  }
  if (beep_once == true) {
    currentms = millis();
    if (currentms - beep_ms >= 500) {
      beep_ms = currentms;
      int state = digitalRead(BUZZER);
      digitalWrite(BUZZER, !state);
      beep_counter_2++;
      if (beep_counter_2 > 2) {
        beep_counter_2 = 0;
        beep_once = false;
        digitalWrite(BUZZER, LOW);
      }
    }
  }
}

void check_connection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wifi Disconnected");
    //reset and try again
    ESP.reset();
    delay(5000);
  }

  status_counter++;
  if (status_counter > 10) {
    const size_t capacity = JSON_OBJECT_SIZE(5);
    DynamicJsonDocument doc(capacity);
    char JSONBUFFER[capacity];

    doc["station"] = deviceID;
    doc["latitude"] = latitude;
    doc["longitude"] = longitude;
    doc["enabled"] = 1;
    doc["enabled"] = 1;

    serializeJson(doc, JSONBUFFER);
    //serializeJson(doc, Serial);

    HTTPClient http;    //Declare object of class HTTPClient

    http.begin("http://www.reeas-web.com:3001/stations");      //Specify request destination
    http.addHeader("Content-Type", "application/json");  //Specify content-type header

    int httpCode = http.POST(JSONBUFFER);   //Send the request
    String payload = http.getString();                  //Get the response payload

    Serial.println(httpCode);   //Print HTTP return code
    Serial.println(payload);    //Print request response payload

    http.end();  //Close connection
    status_counter = 0;
    beep_once = false;
  }
}
