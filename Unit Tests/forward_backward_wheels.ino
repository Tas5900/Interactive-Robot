#include <L298N.h>

// =========================
// Pin definition (ESP32)
// =========================

// Motor A
const int ENA = 4;
const int IN1 = 13;
const int IN2 = 14;

// Motor B (NEW pins)
const int ENB = 4;
const int IN3 = 16;
const int IN4 = 17;

// =========================
// Create motor objects
// =========================
L298N motor1(ENA, IN1, IN2);
L298N motor2(ENB, IN3, IN4);

int count = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== L298N Speed Ramp Test (NEW PINS) ===");

  // Initial speed
  motor1.setSpeed(40);
  motor2.setSpeed(40);
}

void loop() {

  // =========================
  // FORWARD ramp (40 → 255)
  // =========================
  Serial.println("FORWARD ramp");
  for (count = 40; count < 255; count++) {
    motor1.setSpeed(count);
    motor1.forward();

    motor2.setSpeed(count);
    motor2.forward();

    delay(8);
  }

  delay(4000);

  motor1.stop();
  motor2.stop();
  delay(1000);

  // =========================
  // BACKWARD ramp (40 → 255)
  // =========================
  Serial.println("BACKWARD ramp");
  for (count = 40; count < 255; count++) {
    motor1.setSpeed(count);
    motor1.backward();

    motor2.setSpeed(count);
    motor2.backward();

    delay(8);
  }

  delay(4000);

  motor1.stop();
  motor2.stop();
  delay(2000);
}
