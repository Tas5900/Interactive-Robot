#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <AudioFileSourceSD.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutputI2S.h>

// SD card CS pin
#define SD_CS 5

AudioGeneratorMP3 *mp3;
AudioFileSourceSD *file;
AudioOutputI2S *out;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== SD + MAX98357A Test ===");

  // ---- SD CARD INIT ----
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("SD init FAILED!");
    while (1);
  }
  Serial.println("SD OK!");

  // ---- AUDIO OUTPUT INIT ----
  out = new AudioOutputI2S();
  out->SetPinout(27, 26, 25);  // BCLK, LRC, DIN
  out->SetGain(0.6);           // Adjust volume 0.0–1.0

  // ---- OPEN FILE ----
  Serial.println("Opening /test.mp3");
  file = new AudioFileSourceSD("/test.mp3");
  if (!file) {
    Serial.println("Could not open test.mp3");
    while (1);
  }

  // ---- START MP3 DECODER ----
  mp3 = new AudioGeneratorMP3();
  if (!mp3->begin(file, out)) {
    Serial.println("MP3 begin FAILED!");
    while (1);
  }

  Serial.println("Playing test.mp3...");
}

void loop() {
  if (mp3 && mp3->isRunning()) {
    if (!mp3->loop()) {
      Serial.println("Playback finished.");
      mp3->stop();
    }
  }
}
