#include "funcionesSetup.h"

extern Adafruit_SSD1306 display;
extern TofSensors MySensors;

#if defined(ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

void configDisplay()
{
    // ------------------------- Movidas del display --------------------
if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); 
  }

  delay(1000); //antes 2000
display.display();

  // Clear the buffer
  display.clearDisplay();
  display.display();
  delay(1000); //antes 2000
  display.clearDisplay();


}

