// ===== BASIC ESP32 TOUCH SENSOR TEST =====
// Tests GPIO32 and GPIO33
// Prints raw values and touch duration

const int HEAD_PIN = 32;   // Touch pin T9
const int TAIL_PIN = 33;   // Touch pin T8

unsigned long headStart = 0;
unsigned long tailStart = 0;

bool headWasTouched = false;
bool tailWasTouched = false;

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("=== ESP32 TOUCH SENSOR TEST ===");
  Serial.println("Reading raw values from GPIO32 (HEAD) & GPIO33 (TAIL)");
  Serial.println("Touch to see duration.\n");
}

void loop() {
  unsigned long now = millis();

  // Read raw values
  int headVal = touchRead(HEAD_PIN);
  int tailVal = touchRead(TAIL_PIN);

  // Print raw values continuously
  Serial.print("[RAW] Head=");
  Serial.print(headVal);
  Serial.print("   Tail=");
  Serial.println(tailVal);

  bool headTouched = (headVal < 1000); // Temporary threshold to detect touch
  bool tailTouched = (tailVal < 1000);

  // -------------------------
  // HEAD touch logic
  // -------------------------
  if (headTouched && !headWasTouched) {
    headWasTouched = true;
    headStart = now;
    Serial.println(">>> HEAD TOUCH START");
  }

  if (!headTouched && headWasTouched) {
    headWasTouched = false;
    unsigned long duration = now - headStart;

    Serial.print("<<< HEAD TOUCH END — Duration: ");
    Serial.print(duration / 1000.0);
    Serial.println(" seconds");
  }

  // -------------------------
  // TAIL touch logic
  // -------------------------
  if (tailTouched && !tailWasTouched) {
    tailWasTouched = true;
    tailStart = now;
    Serial.println(">>> TAIL TOUCH START");
  }

  if (!tailTouched && tailWasTouched) {
    tailWasTouched = false;
    unsigned long duration = now - tailStart;

    Serial.print("<<< TAIL TOUCH END — Duration: ");
    Serial.print(duration / 1000.0);
    Serial.println(" seconds");
  }

  delay(200); // Slow down printing
}
