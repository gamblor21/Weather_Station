/******
The MIT License (MIT)

Copyright (c) 2020 Mark Komus

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
******/

#include <Adafruit_SleepyDog.h>
#include "SparkFunBME280.h"
#include "Wire.h"
#include "config.h"
#include "RTCZero.h"
#include "ArduinoLowPower.h"

#define VBATPIN A7
#define LED_PIN 5
#define WIND_PIN 6
#define RAIN_PIN 11
#define WIND_DIR_PIN A2

BME280 bmeSensor;

AdafruitIO_Feed *batVoltage = io.feed("Battery Voltage");
AdafruitIO_Feed *temperature = io.feed("Temperature");
AdafruitIO_Feed *humidity = io.feed("Humidity");
AdafruitIO_Feed *pressure = io.feed("Pressure");
AdafruitIO_Feed *rain = io.feed("Rain");
AdafruitIO_Feed *windDir = io.feed("Wind Direction");
AdafruitIO_Feed *windSpeed = io.feed("Wind Speed");
AdafruitIO_Feed *start = io.feed("Start");

volatile unsigned int rainTicks = 0;
volatile unsigned int windTicks = 0;
volatile bool alarmWent = false;

RTCZero rtc;

void setup() {
  Serial.begin(115200);
  //while (!Serial) {;}
  
  Serial.println("Starting up...");
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Show we're awake
  
  Serial.print("Previous reset cause: ");
  Serial.println(PM->RCAUSE.reg);

  pinMode(WIND_PIN, INPUT_PULLUP);
  pinMode(RAIN_PIN, INPUT_PULLUP);
  pinMode(WIND_DIR_PIN, INPUT);

  LowPower.attachInterruptWakeup(WIND_PIN, windIRQ, FALLING);
  LowPower.attachInterruptWakeup(RAIN_PIN, rainIRQ, FALLING);

  rtc.begin();
  rtc.setTime(0,0,0);
  rtc.attachInterrupt(alarmIRQ);
  
  Serial.print("Connecting to Adafruit IO");
  io.connect();

  // wait for a connection
  while (io.status() < AIO_CONNECTED) {
    Serial.print(".");
    Serial.println(io.statusText());
    delay(500);
  }

  // we are connected
  Serial.println(io.statusText());

  WiFi.lowPowerMode();

  // Setup BME280 sensor
  bmeSensor.settings.commInterface = I2C_MODE;
  bmeSensor.settings.I2CAddress = 0x77;
  bmeSensor.settings.runMode = 2; // Forced mode
  bmeSensor.settings.tStandby = 0;  //  4, 500ms
  bmeSensor.settings.filter = 0; // off
  bmeSensor.settings.tempOverSample = 1;
  bmeSensor.settings.pressOverSample = 1;
  bmeSensor.settings.humidOverSample = 1;

  delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  Serial.println(bmeSensor.begin(), HEX);
  Serial.print("ctrl_meas(0xF4): 0x");
  Serial.println(bmeSensor.readRegister(BME280_CTRL_MEAS_REG), HEX);

  start->save(PM->RCAUSE.reg);

  rtc.setAlarmEpoch(rtc.getEpoch() + 9);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);

  int countdownMS = Watchdog.enable(16000);
  Serial.println(countdownMS);
}

unsigned long prevMillis = 0;
unsigned long prevThirtyMillis = 0;
volatile unsigned long alarmCount = 0;

void loop() {
  Serial.end();

  digitalWrite(LED_BUILTIN, LOW); 

  LowPower.sleep();

  Serial.begin(115200);
  //while (!Serial) {;}
  Serial.println("Awakened");

  if (alarmWent) {
    Watchdog.reset();
    if (alarmCount <= 5) {
      rtc.setAlarmEpoch(rtc.getEpoch() + 9);
      rtc.enableAlarm(rtc.MATCH_HHMMSS);
    }
    alarmWent = false;
  }
  
  if (alarmCount > 5) {
    digitalWrite(LED_BUILTIN, HIGH);

    rtc.setAlarmEpoch(rtc.getEpoch() + 8);
    rtc.enableAlarm(rtc.MATCH_HHMMSS);
    alarmCount = 0;
    
    io.run();

    measure();
  }
}

void measure() {
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  batVoltage->save(measuredvbat);
  Serial.print("VBat: " ); Serial.println(measuredvbat);

  bmeSensor.writeRegister(BME280_CTRL_MEAS_REG, 0x26); 
  delay(10);

  //Serial.print("Temperature: ");
  //Serial.print(bmeSensor.readTempC(), 2);
  //Serial.println(" degrees C");
  temperature->save(bmeSensor.readTempC());
  
  //Serial.print("Temperature: ");
  //Serial.print(bmeSensor.readTempF(), 2);
  //Serial.println(" degrees F");

  //Serial.print("Pressure: ");
  //Serial.print(bmeSensor.readFloatPressure(), 2);
  //Serial.println(" Pa");
  
  pressure->save(bmeSensor.readFloatPressure());

  io.run();

  //Serial.print("Altitude: ");
  //Serial.print(bmeSensor.readFloatAltitudeMeters(), 2);
  //Serial.println("m");

  //Serial.print("Altitude: ");
  //Serial.print(bmeSensor.readFloatAltitudeFeet(), 2);
  //Serial.println("ft");

  //Serial.print("%RH: ");
  //Serial.print(bmeSensor.readFloatHumidity(), 2);
  //Serial.println(" %");

  humidity->save(bmeSensor.readFloatHumidity());

  //Serial.print("ctrl_meas(0xF4): 0x");
  //Serial.println(bmeSensor.readRegister(BME280_CTRL_MEAS_REG), HEX);

  //Serial.println(io.statusText());

  float rainAmount = rainTicks * 0.011;
  rainTicks = 0;
  //Serial.print("Rain: ");
  //Serial.println(rainAmount);
  rain->save(rainAmount);

  int windDirDegrees = getWindDirection();
  //Serial.print("Wind Dir: ");
  //Serial.println(windDirDegrees);
  //int windDirDegrees = 0;
  windDir->save(windDirDegrees);

  io.run();

  float wind = (float)windTicks / (float)60; //60 seconds)
  windTicks = 0;
  wind *= 2.4; // 2.4 km/h
  //Serial.print("Wind Speed: ");
  //Serial.println(wind);
  windSpeed->save(wind);

  //Serial.println();
  
}

int getWindDirection() {
  unsigned int windDir = analogRead(WIND_DIR_PIN);
  if (windDir < 74) return 113;
  if (windDir < 88) return 67;
  if (windDir < 110) return 90;
  if (windDir < 150) return 158;
  if (windDir < 210) return 135;
  if (windDir < 260) return 203;
  if (windDir < 340) return 180;
  if (windDir < 430) return 23;
  if (windDir < 530) return 45;
  if (windDir < 615) return 248;
  if (windDir < 660) return 225;
  if (windDir < 740) return 338;
  if (windDir < 800) return 0;
  if (windDir < 860) return 293;
  if (windDir < 960) return 270;
  if (windDir < 1010) return 315;
  return -1;
    /*if (windDir < 74) Serial.println("ESE");
  else if (windDir < 88) Serial.println("ENE");
  else if (windDir < 110) Serial.println("E");
  else if (windDir < 150) Serial.println("SSE");
  else if (windDir < 210) Serial.println("SE");
  else if (windDir < 260) Serial.println("SSW");
  else if (windDir < 340) Serial.println("S");
  else if (windDir < 430) Serial.println("NNE");
  else if (windDir < 530) Serial.println("NE");
  else if (windDir < 615) Serial.println("WSW");
  else if (windDir < 660) Serial.println("SW");
  else if (windDir < 740) Serial.println("NNW");
  else if (windDir < 800) Serial.println("N");
  else if (windDir < 860) Serial.println("WNW");
  else if (windDir < 960) Serial.println("W");
  else if (windDir < 1010) Serial.println("NW");*/
}

void windIRQ() {
  windTicks++;
}

void rainIRQ() {
  rainTicks++;
}

void alarmIRQ() {
  alarmWent = true;
  alarmCount++;
}
