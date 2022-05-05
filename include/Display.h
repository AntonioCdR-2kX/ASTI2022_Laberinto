#ifndef DISPLAY_H
#define DISPLAY_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <SPI.h>
#include "Logo.h"
#include "Config.h"

#ifndef SCREEN_WIDTH
#define SCREEN_WIDTH 128 
#endif

#ifndef SCREEN_HEIGHT
#define SCREEN_HEIGHT 64 
#endif

#ifndef OLED_RESET
#define OLED_RESET     4 
#endif

#ifndef SCREEN_ADDRESS
#define SCREEN_ADDRESS 0x3
#endif

extern Adafruit_SSD1306 display;

void display_config();

void display_printLogo();

void display_printLine(String);



#endif