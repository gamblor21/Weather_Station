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

// Pins for the weather gauages. Wind/Rain are digital, Wind direction must be analog
#define VBAT_PIN A7
#define LED_PIN 5
#define WIND_PIN 6
#define RAIN_PIN 11
#define WIND_DIR_PIN A2

// Set this to your location's altitude above sea level in meters
#define ALTITUDE 235  

// Our temperature/pressure/humidity sensor
BME280 bmeSensor;

// All the Adafruit IO Feeds we measure
AdafruitIO_Feed *batVoltageFeed = io.feed("Battery Voltage");
AdafruitIO_Feed *temperatureFeed = io.feed("Temperature");
AdafruitIO_Feed *humidityFeed = io.feed("Humidity");
AdafruitIO_Feed *pressureFeed = io.feed("Pressure");
AdafruitIO_Feed *rainFeed = io.feed("Rain");
AdafruitIO_Feed *windDirFeed = io.feed("Wind Direction");
AdafruitIO_Feed *windSpeedFeed = io.feed("Wind Speed");
AdafruitIO_Feed *startFeed = io.feed("Start");

// Interrupts will increase any measurements of rain and wind speed
volatile unsigned int rainTicks = 0;
volatile unsigned int windTicks = 0;

// Set if our RTC alarm goes
volatile bool alarmWent = false;
volatile unsigned long alarmCount = 0;

// Keeps track of when our next alarm will be (every 10 seconds)
byte nextAlarmSecond = 0;

RTCZero rtc;

void setup() {
  Serial.begin(115200);
  //while (!Serial) {;}  // uncomment to wait for the code to run until you connect serial. Don't do this if its installed!
  
  Serial.println("Starting up...");
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Show we're awake
  
  Serial.print("Previous reset cause: "); // Used for debugging if the watchdog triggered there is a record of it
  Serial.println(PM->RCAUSE.reg); // This is SAMD21 specific (may work on SAMD51)

  // Set input pins up for wind and rain
  pinMode(WIND_PIN, INPUT_PULLUP);
  pinMode(RAIN_PIN, INPUT_PULLUP);
  pinMode(WIND_DIR_PIN, INPUT);

  // Ensure we wake up from sleep to register the interrupts
  LowPower.attachInterruptWakeup(WIND_PIN, windIRQ, FALLING);
  LowPower.attachInterruptWakeup(RAIN_PIN, rainIRQ, FALLING);

  // Setup connection to Adafruit IO
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

  // Ensure WiFi adapter is working on low power mode. Had issues with maxLowPowerMode()
  WiFi.lowPowerMode();

  // Setup BME280 sensor, settings based on recommendation for weather station in BME280 specification sheet
  bmeSensor.settings.commInterface = I2C_MODE;
  bmeSensor.settings.I2CAddress = 0x77;
  bmeSensor.settings.runMode = 2; // Forced mode
  bmeSensor.settings.tStandby = 0;  //  4, 500ms
  bmeSensor.settings.filter = 0; // off
  bmeSensor.settings.tempOverSample = 1;
  bmeSensor.settings.pressOverSample = 1;
  bmeSensor.settings.humidOverSample = 1;

  delay(20);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.

  bmeSensor.reset(); // reset the sensor like after a power on (in case the reset button was pressed)
  delay(100); // not sure how long this takes just to make sure
  
  uint8_t bmeStart = bmeSensor.begin();
  Serial.println(bmeStart, HEX);
  Serial.print("ctrl_meas(0xF4): 0x");
  Serial.println(bmeSensor.readRegister(BME280_CTRL_MEAS_REG), HEX);
  bmeSensor.setMode(MODE_SLEEP);

  // Save our reason for startup to Adafruit IO
  startFeed->save(PM->RCAUSE.reg);

  // Turn on the watchdog timer for 16 seconds
  // The code wakes up every 10 seconds so this should never be reached
  int countdownMS = Watchdog.enable(16000);

  // Start the RTC to wake us up. Only clock that is running while sleeping
  rtc.begin();
  rtc.attachInterrupt(alarmIRQ);
  rtc.setTime(0,0,0);

  // Set our first alarm
  nextAlarmSecond = 10;  // Our first alarm will be 10 seconds in
  rtc.setAlarmTime(0, 0, nextAlarmSecond);
  nextAlarmSecond = (nextAlarmSecond + 10) % 60;
  rtc.enableAlarm(rtc.MATCH_SS);
}

void loop() {
  // At the start of the loop go to lower power sleep
  Serial.end();
  digitalWrite(LED_BUILTIN, LOW); 

  LowPower.sleep(); // put the controller to sleep until an interrupt (including alarm) wakes us

  // Anything after this is when we have been woken up due to the timer or sensor interupt
  Serial.begin(115200);
  Serial.println("Awakened");

  // Every 10 seconds we get woke up to pet the watchdog
  // Need this as an interrupt from wind/rain will also re-awaken us
  if (alarmWent) {
    Watchdog.reset(); // pet the watchdog so it does not trigger
    // measureWindGusts();
    // measureWindDirection(); // to get average over 2 min

    // 60 seconds have passed
    if (alarmCount > 5) {
      digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED on so we know we ran the main loop, can be disabled
    
      alarmCount = 0;
    
      io.run(); // needed to keep Adafruit IO connected

      // take our 1 minute weather measurements
      measure();
    }

    // set the alarm for another 10 seconds
    rtc.setAlarmTime(0, 0, nextAlarmSecond);
    nextAlarmSecond = (nextAlarmSecond + 10) % 60;
    rtc.enableAlarm(rtc.MATCH_SS);

    // Reset our flag that the alarm went
    alarmWent = false;
  }
}

void measure() {
  // Measure the battery voltage
  float measuredvbat = analogRead(VBAT_PIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  batVoltageFeed->save(measuredvbat);

  // Wake up the bme280
  bmeSensor.writeRegister(BME280_CTRL_MEAS_REG, 0x26); 
  delay(20); // give the sensor time to read

  float temperature = bmeSensor.readTempC();
  temperatureFeed->save(temperature);

  float pressure = bmeSensor.readFloatPressure();
  float seaPressure = ((pressure/100) * pow(1 - (0.0065 * ALTITUDE / (temperature + 0.0065 * ALTITUDE + 273.15)), -5.257));
  pressureFeed->save(seaPressure);
  
  io.run();

  humidityFeed->save(bmeSensor.readFloatHumidity());

  float rainAmount = rainTicks * 0.011;
  rainTicks = 0;
  rainFeed->save(rainAmount);

  int windDirDegrees = getWindDirection();
  windDirFeed->save(windDirDegrees);

  io.run();

  float wind = (float)windTicks / (float)60; //60 seconds)
  windTicks = 0;
  wind *= 2.4; // 2.4 km/h
  windSpeedFeed->save(wind);

  //Serial.println(bmeSensor.readRegister(BME280_CTRL_MEAS_REG), HEX);
  //Serial.println(io.statusText());
}

// Get and measure the wind direction
int getWindDirection() {
  unsigned int windDir = analogRead(WIND_DIR_PIN);
  if (windDir < 74) return 113; // ESE
  if (windDir < 88) return 67;  // ENE
  if (windDir < 110) return 90; // E
  if (windDir < 150) return 158;// SSE
  if (windDir < 210) return 135;// SS
  if (windDir < 260) return 203;// SSW
  if (windDir < 340) return 180;// S
  if (windDir < 430) return 23; // NNE
  if (windDir < 530) return 45; // NE
  if (windDir < 615) return 248;// WSW
  if (windDir < 660) return 225;// SW
  if (windDir < 740) return 338;// NNW
  if (windDir < 800) return 0;  // N
  if (windDir < 860) return 293;// WNW
  if (windDir < 960) return 270;// W
  if (windDir < 1010) return 315;// NW
  return -1;
}

// Called whenever we receive one tick from the anemometer
void windIRQ() {
  windTicks++;
}

// Called whenever we receive on tick from the rain gauge
void rainIRQ() {
  rainTicks++;
}

// Called whenever our RTC alarm triggers (should be every 10 seconds)
void alarmIRQ() {
  alarmWent = true;
  alarmCount++;
}
