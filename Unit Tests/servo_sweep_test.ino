/*
  StreamMP3fromHTTP.ino

  Streams an MP3 file from an HTTP server
  using the ESP8266Audio library.

  Works on ESP32.
*/

#include <WiFi.h>
#include <AudioFileSourceICYStream.h>
#include <AudioFileSourceBuffer.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutputI2S.h>

// =========================
// WiFi credentials
// =========================
const char* ssid     = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";

// =========================
// MP3 stream URL (HTTP only)
// =========================
const char* streamURL = "http://icecast.somafm.com/groovesalad";

// =========================
// Audio objects
// =========================
AudioGeneratorMP3 *mp3;
AudioFileSourceICYStream *file;
AudioFileSourceBuffer *buff;
AudioOutputI2S *out;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // =========================
  // Setup audio output (I2S)
  // =========================
  out = new AudioOutputI2S();
  out->SetPinout(27, 26, 25);   // BCLK, LRC, DIN
  out->SetGain(0.5);            // volume 0.0–1.0

  // =========================
  // Setup MP3 stream
  // =========================
  file = new AudioFileSourceICYStream(streamURL);
  buff = new AudioFileSourceBuffer(file, 2048);
  buff->begin();

  mp3 = new AudioGeneratorMP3();
  mp3->begin(buff, out);

  Serial.println("Streaming started");
}

void loop() {
  if (mp3->isRunning()) {
    if (!mp3->loop()) {
      mp3->stop();
      Serial.println("MP3 stream stopped");
    }
  } else {
    delay(1000);
  }
}
