#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Uncomment the line that matches your board version
// #define BOARD_V1  // If your LED is on pin 48
#define BOARD_LATEST // If your LED is on pin 38

#ifdef BOARD_V1
const int RGB_PIN = 48;
#else
const int RGB_PIN = 38;
#endif

Adafruit_NeoPixel pixel(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pixel.begin();
  pixel.setBrightness(30); // Start at 30% brightness
}

void loop() {
  // Cycle through colors
  pixel.setPixelColor(0, 255, 0, 0); // Red
  pixel.show();
  delay(1000);

  pixel.setPixelColor(0, 0, 255, 0); // Green
  pixel.show();
  delay(1000);

  pixel.setPixelColor(0, 0, 0, 255); // Blue
  pixel.show();
  delay(1000);

  // Custom color (white)
  pixel.setPixelColor(0, 255, 255, 255);
  pixel.show();
  delay(1000);
}