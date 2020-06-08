/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME  "your_username"
#define IO_KEY       "your_key"

#define WIFI_SSID   "your_ssuid"
#define WIFI_PASS   "your_pass"

// uncomment the following line if you are using winc1500
#define USE_WINC1500

// comment out the following lines if you are using fona or ethernet
#include "AdafruitIO_WiFi.h"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
