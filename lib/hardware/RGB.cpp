#include "RGB.h"

#define PIXEL_PIN    38  // Digital IO pin connected to the NeoPixels.

#define PIXEL_COUNT 8  // Number of NeoPixels

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

unsigned long pixelPrevious = 0;        // Previous Pixel Millis
unsigned long patternPrevious = 0;      // Previous Pattern Millis
int           patternCurrent = 0;       // Current Pattern Number
int           patternInterval = 5000;   // Pattern Interval (ms)
bool          patternComplete = false;

int           pixelInterval = 50;       // Pixel Interval (ms)
int           pixelQueue = 0;           // Pattern Pixel Queue
int           pixelCycle = 0;           // Pattern Pixel Cycle
uint16_t      pixelNumber = PIXEL_COUNT;  // Total Number of Pixels
void RGB_init() {
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'
  strip.setBrightness(125); // Set BRIGHTNESS to about 1/5 (max = 255)
}

void RGB_start(unsigned long currentMillis)
{
    if (currentMillis - pixelPrevious >= pixelInterval) {  //  Check for expired time
      pixelPrevious = currentMillis; }                      //  Run current frame
}
// Some functions of our own for creating animated effects -----------------

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  static uint16_t current_pixel = 0;
  pixelInterval = wait;                        //  Update delay time
  strip.setPixelColor(current_pixel++, color); //  Set pixel's color (in RAM)
  strip.show();                                //  Update strip to match
  if(current_pixel >= pixelNumber) {           //  Loop the pattern from the first LED
    current_pixel = 0;
    patternComplete = true;
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  static uint32_t loop_count = 0;
  static uint16_t current_pixel = 0;

  pixelInterval = wait;                   //  Update delay time

  strip.clear();

  for(int c=current_pixel; c < pixelNumber; c += 3) {
    strip.setPixelColor(c, color);
  }
  strip.show();

  current_pixel++;
  if (current_pixel >= 3) {
    current_pixel = 0;
    loop_count++;
  }

  if (loop_count >= 10) {
    current_pixel = 0;
    loop_count = 0;
    patternComplete = true;
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void myrainbow(uint8_t wait) {
  if(pixelInterval != wait)
    pixelInterval = wait;                   
  for(uint16_t i=0; i < pixelNumber; i++) {
    strip.setPixelColor(i, Wheel((i + pixelCycle) & 255)); //  Update delay time  
  }
  strip.show();                             //  Update strip to match
  pixelCycle++;                             //  Advance current cycle
  if(pixelCycle >= 256)
    pixelCycle = 0;                         //  Loop the cycle back to the begining
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  if(pixelInterval != wait)
    pixelInterval = wait;                   //  Update delay time  
  for(int i=0; i < pixelNumber; i+=3) {
    strip.setPixelColor(i + pixelQueue, Wheel((i + pixelCycle) % 255)); //  Update delay time  
  }
  strip.show();
  for(int i=0; i < pixelNumber; i+=3) {
    strip.setPixelColor(i + pixelQueue, strip.Color(0, 0, 0)); //  Update delay time  
  }      
  pixelQueue++;                           //  Advance current queue  
  pixelCycle++;                           //  Advance current cycle
  if(pixelQueue >= 3)
    pixelQueue = 0;                       //  Loop
  if(pixelCycle >= 256)
    pixelCycle = 0;                       //  Loop
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
