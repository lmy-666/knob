#ifndef RGB_H
#define RGB_H

#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;

void RGB_init();
void RGB_start(unsigned long currentMillis);
void colorWipe(uint32_t color, int wait);
void theaterChase(uint32_t color, int wait) ;
void myrainbow(uint8_t wait);
void theaterChaseRainbow(uint8_t wait);
uint32_t Wheel(byte WheelPos) ;

#endif 