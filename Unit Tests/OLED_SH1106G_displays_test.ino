/**************************************************************************
 This is an example for the Adafruit SH1106G displays

 This example is for a 128x64 display using I2C

 Written by Adafruit Industries
**************************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// I2C address for SH1106 is usually 0x3C
#define I2C_ADDRESS 0x3C

// Create display object
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println("SH1106 OLED test");

  // Initialize I2C
  Wire.begin();

  // Initialize display
  if (!display.begin(I2C_ADDRESS, true)) {
    Serial.println("SH1106 allocation failed");
    while (1);
  }

  Serial.println("SH1106 OK");

  display.clearDisplay();

  // Draw text
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("Hello!");

  display.setTextSize(1);
  display.println("SH1106 128x64");
  display.println("I2C test");

  display.display();
}

void loop() {
  // nothing here
}
