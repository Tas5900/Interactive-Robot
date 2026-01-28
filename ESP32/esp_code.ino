/*******************************************************
  ESPuppy: Touch + Audio + 2xOLED + Motors + Tail(LEDC) + Ear(LEDC)
  + Adafruit IO (event-based publish, debounce, queue, rate-limit, non-blocking reconnect)

  SESSION LOGGING:
    - session=1 -> publish "state" and "event" (your experiment logging)
    - session=0 -> STOP logging state/event

  DASHBOARD:
    - current_state: always updated on state entry
    - head_count / tail_count: GAUGES updated live when counts change
    - stats: published on STOP

  ✅ Movement changes requested:
    - HAPPY_SHORT: unchanged (forward for HAPPY_SHORT_MOVE_MS)
    - HAPPY_LONG : unchanged (spin right for HAPPY_LONG_TURN_MS)
    - HAPPY_MEDIUM: forward a bit more than HAPPY_SHORT
    - ANNOYED: backward same duration as HAPPY_SHORT
    - ANGRY: backward a lot

  ✅ EAR ANGLE behavior (NEW):
    - NEUTRAL, CONFUSED, ANNOYED, HAPPY_SHORT, HAPPY_MEDIUM, HAPPY_LONG -> 90
    - SLEEPY -> 30
    - ANGRY -> 150

  ✅ TAIL SERVOS (NEW mapping + behavior):
    - GPIO12 = sideways tail
    - GPIO15 = up/down tail

    - NEUTRAL / CONFUSED / ANNOYED: up/down=90, sideways=90
    - HAPPY_SHORT / HAPPY_MEDIUM / HAPPY_LONG: up/down=90, sideways sweeps smoothly 30<->150 loop
    - ANGRY: sideways=90, up/down=30
    - SLEEPY: sideways=90, up/down=150
*******************************************************/

#include <SD.h>
#include <SPI.h>
#include <AudioFileSourceSD.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutputI2S.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <math.h>

#include <L298N.h>
#include <esp32-hal-ledc.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "AdafruitIO_WiFi.h"

// --- Fill these ---
#define WIFI_SSID    "ICST"
#define WIFI_PASS    "arduino123"

#define AIO_USERNAME "tas59"
#define AIO_KEY      "aio_WKLX78IGRyMCOUyyPFWPOTKXgZtm"

// =========================
// Feed routing
// =========================
enum FeedId : uint8_t {
  FEED_EVENT = 0,
  FEED_STATE_LOG = 1,     // session-gated
  FEED_CURRENT_STATE = 2, // always
  FEED_HEAD_COUNT = 3,    // numeric gauge
  FEED_TAIL_COUNT = 4,    // numeric gauge
  FEED_STATS_TEXT = 5     // text
};

struct NetMsg {
  FeedId feed;
  bool sessionGated;
  char text[220];
};

// Forward declarations
void enqueueFeed(FeedId feed, bool gated, const char* txt);
void networkTask(void *param);
void handleSession(AdafruitIO_Data *data);

QueueHandle_t netQueue = nullptr;

// =========================
// Adafruit IO instance + feeds
// =========================
AdafruitIO_WiFi io(AIO_USERNAME, AIO_KEY, WIFI_SSID, WIFI_PASS);

AdafruitIO_Feed *stateFeed        = io.feed("state");
AdafruitIO_Feed *eventFeed        = io.feed("event");
AdafruitIO_Feed *sessionFeed      = io.feed("session");
AdafruitIO_Feed *currentStateFeed = io.feed("current_state");

AdafruitIO_Feed *headCountFeed    = io.feed("head_count");
AdafruitIO_Feed *tailCountFeed    = io.feed("tail_count");
AdafruitIO_Feed *statsFeed        = io.feed("stats");

volatile bool sessionActive = false;
volatile bool sessionKnown  = false;

// =========================
// SD Card
// =========================
#define SD_CS 5

// =========================
// OLED (SH1106)
// =========================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_ADDR_LEFT  0x3C
#define OLED_ADDR_RIGHT 0x3D

Adafruit_SH1106G displayLeft(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_SH1106G displayRight(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// =========================
// ESP32 touch pins
// =========================
const int HEAD_TOUCH_PIN = 32;
const int TAIL_TOUCH_PIN = 33;

int HEAD_THRESHOLD = 1000;
int TAIL_THRESHOLD = 1000;

const uint32_t DEBOUNCE_MS = 30;

// =========================
// Audio
// =========================
AudioGeneratorMP3 *mp3  = nullptr;
AudioFileSourceSD *file = nullptr;
AudioOutputI2S    *out  = nullptr;

const char* FILE_HAPPY_SHORT  = "/happy_short.mp3";
const char* FILE_HAPPY_MEDIUM = "/happy_medium.mp3";
const char* FILE_HAPPY_LONG   = "/happy_long.mp3";
const char* FILE_ANNOYED      = "/annoyed.mp3";
const char* FILE_ANGRY        = "/angry.mp3";
const char* FILE_SLEEPY       = "/sleepy.mp3";
const char* FILE_CONFUSED     = "/confused.mp3";

// =========================
// Emotion states
// =========================
enum EmotionState {
  NEUTRAL,
  HAPPY_SHORT,
  HAPPY_MEDIUM,
  HAPPY_LONG,
  ANNOYED,
  ANGRY,
  SLEEPY,
  CONFUSED,
  EMO_COUNT
};

EmotionState currentState = NEUTRAL;

// =========================
// Session counters
// =========================
volatile uint32_t sessionHeadCount = 0;
volatile uint32_t sessionTailCount = 0;
volatile uint32_t stateEnterCount[EMO_COUNT] = {0};

// publish only when changed
uint32_t lastPublishedHead = 0xFFFFFFFF;
uint32_t lastPublishedTail = 0xFFFFFFFF;

void publishCountsIfChanged() {
  if (!sessionActive) return;

  char num[24];

  if (sessionHeadCount != lastPublishedHead) {
    lastPublishedHead = sessionHeadCount;
    snprintf(num, sizeof(num), "%lu", (unsigned long)sessionHeadCount);
    enqueueFeed(FEED_HEAD_COUNT, false, num);
  }

  if (sessionTailCount != lastPublishedTail) {
    lastPublishedTail = sessionTailCount;
    snprintf(num, sizeof(num), "%lu", (unsigned long)sessionTailCount);
    enqueueFeed(FEED_TAIL_COUNT, false, num);
  }
}

void resetSessionCounters() {
  sessionHeadCount = 0;
  sessionTailCount = 0;
  for (int i = 0; i < EMO_COUNT; i++) stateEnterCount[i] = 0;

  lastPublishedHead = 0xFFFFFFFF;
  lastPublishedTail = 0xFFFFFFFF;
}

void publishSessionSummary() {
  char buf[220];

  snprintf(
    buf, sizeof(buf),
    "Session summary:\n"
    "NEUTRAL: %lu\n"
    "HAPPY_SHORT: %lu\n"
    "HAPPY_MEDIUM: %lu\n"
    "HAPPY_LONG: %lu\n"
    "ANNOYED: %lu\n"
    "ANGRY: %lu\n"
    "SLEEPY: %lu\n"
    "CONFUSED: %lu\n",
    (unsigned long)stateEnterCount[NEUTRAL],
    (unsigned long)stateEnterCount[HAPPY_SHORT],
    (unsigned long)stateEnterCount[HAPPY_MEDIUM],
    (unsigned long)stateEnterCount[HAPPY_LONG],
    (unsigned long)stateEnterCount[ANNOYED],
    (unsigned long)stateEnterCount[ANGRY],
    (unsigned long)stateEnterCount[SLEEPY],
    (unsigned long)stateEnterCount[CONFUSED]
  );

  char num[24];
  snprintf(num, sizeof(num), "%lu", (unsigned long)sessionHeadCount);
  enqueueFeed(FEED_HEAD_COUNT, false, num);

  snprintf(num, sizeof(num), "%lu", (unsigned long)sessionTailCount);
  enqueueFeed(FEED_TAIL_COUNT, false, num);

  enqueueFeed(FEED_STATS_TEXT, false, buf);
}

// =========================
// Timing
// =========================
unsigned long neutralStartTime   = 0;
unsigned long emotionalStartTime = 0;
unsigned long stateStartTime     = 0;

unsigned long headTouchStartTime = 0;
bool headWasTouched = false;

bool tailWasTouched              = false;
unsigned long tailTouchStartTime = 0;
unsigned long lastTailTouchTime  = 0;

const unsigned long ANNOYED_CONFUSED_WINDOW = 20000;

const unsigned long SLEEPY_TIMEOUT  = 30000;
const unsigned long RESET_TIMEOUT   = 20000;

const unsigned long PET_IGNORE_MAX  = 3000;
const unsigned long PET_SHORT_MAX   = 5000;
const unsigned long PET_MEDIUM_MAX  = 12000;

int lastEmoSecPrinted = 0;
bool emotionalTimeoutPending = false;

// =========================
// MOTOR (L298N)
// =========================
const int ENA = 4;
const int IN1 = 13;
const int IN2 = 14;

const int ENB = 4;     // shared enable pin
const int IN3 = 16;
const int IN4 = 17;

L298N motor1(ENA, IN1, IN2);
L298N motor2(ENB, IN3, IN4);

const int HAPPY_SHORT_SPEED     = 200;
const int HAPPY_LONG_TURN_SPEED = 200;

const unsigned long HAPPY_SHORT_MOVE_MS = 2000;
const unsigned long HAPPY_LONG_TURN_MS  = 6500;

unsigned long happyShortMoveStart = 0;
bool happyShortMoveActive = false;

unsigned long happyLongTurnStart = 0;
bool happyLongTurnActive = false;

// ✅ NEW movement behaviors
const unsigned long HAPPY_MEDIUM_MOVE_MS = 3000;                 // forward a bit more than happy short
const unsigned long ANNOYED_MOVE_MS      = HAPPY_SHORT_MOVE_MS;  // same amount, but backwards
const unsigned long ANGRY_MOVE_MS        = 6500;                 // backwards a lot

unsigned long happyMediumMoveStart = 0;
bool happyMediumMoveActive = false;

unsigned long annoyedMoveStart = 0;
bool annoyedMoveActive = false;

unsigned long angryMoveStart = 0;
bool angryMoveActive = false;

const int HAPPY_MEDIUM_SPEED = HAPPY_SHORT_SPEED;
const int ANNOYED_BACK_SPEED = 180;
const int ANGRY_BACK_SPEED   = 220;

void motorsStop() { motor1.stop(); motor2.stop(); }

void motorsForward(int spd) {
  motor1.setSpeed(spd); motor2.setSpeed(spd);
  motor1.forward(); motor2.forward();
}

void motorsBackward(int spd) {
  motor1.setSpeed(spd); motor2.setSpeed(spd);
  motor1.backward(); motor2.backward();
}

void motorsSpinRight(int spd) {
  motor1.setSpeed(spd); motor2.setSpeed(spd);
  motor1.forward(); motor2.backward();
}

void resetMovementOneShots() {
  happyShortMoveActive  = false;
  happyLongTurnActive   = false;

  happyMediumMoveActive = false;
  annoyedMoveActive     = false;
  angryMoveActive       = false;
}

// =========================
// SERVO PWM shared config (LEDC)
// =========================
const int SERVO_FREQ_HZ = 50;
const int SERVO_RES_BITS = 16;
const uint32_t SERVO_PERIOD_US = 20000;
const uint32_t SERVO_MIN_US = 1000;
const uint32_t SERVO_MAX_US = 2000;

uint32_t servoUsToDuty(uint32_t pulse_us) {
  if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
  if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;
  const uint32_t maxDuty = (1UL << SERVO_RES_BITS) - 1UL;
  return (uint32_t)((pulse_us * maxDuty) / SERVO_PERIOD_US);
}

uint32_t angleToUs(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  return SERVO_MIN_US + (uint32_t)((angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180);
}

void servoWriteAngleByChannel(int channel, int angle) {
  uint32_t us = angleToUs(angle);
  uint32_t duty = servoUsToDuty(us);
  ledcWriteChannel(channel, duty);
}

// =========================
// TAIL SERVOS using LEDC (GPIO12=sideways, GPIO15=up/down)
// =========================
const int TAIL_SIDE_PIN = 12;   // sideways
const int TAIL_UP_PIN   = 15;   // up/down

const int TAIL_SIDE_CH  = 6;
const int TAIL_UP_CH    = 7;

bool tailAttached = false;

// angles
const int TAIL_CENTER = 90;
const int TAIL_MIN    = 30;
const int TAIL_MAX    = 150;

// smooth sweep settings
bool tailSweepActive = false;
int  tailSideAngle   = TAIL_MIN;
int  tailSideDir      = +1;                 // +1 up, -1 down
uint32_t tailLastTick  = 0;

const uint32_t TAIL_SWEEP_TICK_MS = 18;     // smooth (smaller = faster)
const int      TAIL_SWEEP_STEP_DEG = 2;     // smaller step = smoother

void tailAttachIfNeeded() {
  if (tailAttached) return;

  bool okSide = ledcAttachChannel(TAIL_SIDE_PIN, SERVO_FREQ_HZ, SERVO_RES_BITS, TAIL_SIDE_CH);
  bool okUp   = ledcAttachChannel(TAIL_UP_PIN,   SERVO_FREQ_HZ, SERVO_RES_BITS, TAIL_UP_CH);

  if (!okSide || !okUp) {
    Serial.printf("[TAIL] ledcAttachChannel FAILED! okSide=%d okUp=%d\n", okSide, okUp);
    tailAttached = false;
    tailSweepActive = false;
    return;
  }

  // default center
  servoWriteAngleByChannel(TAIL_SIDE_CH, TAIL_CENTER);
  servoWriteAngleByChannel(TAIL_UP_CH,   TAIL_CENTER);

  tailAttached = true;
  Serial.printf("[TAIL] LEDC attached. side pin=%d ch=%d | up pin=%d ch=%d\n",
                TAIL_SIDE_PIN, TAIL_SIDE_CH, TAIL_UP_PIN, TAIL_UP_CH);
}

void tailSetFixed(int sideAngle, int upAngle) {
  tailAttachIfNeeded();
  if (!tailAttached) return;

  tailSweepActive = false;

  servoWriteAngleByChannel(TAIL_SIDE_CH, sideAngle);
  servoWriteAngleByChannel(TAIL_UP_CH,   upAngle);
}

void tailStartSmoothSweep(uint32_t now) {
  tailAttachIfNeeded();
  if (!tailAttached) return;

  // up/down fixed at 90 in happy states
  servoWriteAngleByChannel(TAIL_UP_CH, TAIL_CENTER);

  tailSweepActive = true;
  tailSideAngle   = TAIL_MIN;
  tailSideDir     = +1;
  tailLastTick    = now;

  servoWriteAngleByChannel(TAIL_SIDE_CH, tailSideAngle);
}

void tailUpdate(uint32_t now) {
  if (!tailSweepActive) return;
  if (now - tailLastTick < TAIL_SWEEP_TICK_MS) return;
  tailLastTick = now;

  tailSideAngle += tailSideDir * TAIL_SWEEP_STEP_DEG;

  if (tailSideAngle >= TAIL_MAX) {
    tailSideAngle = TAIL_MAX;
    tailSideDir = -1;
  } else if (tailSideAngle <= TAIL_MIN) {
    tailSideAngle = TAIL_MIN;
    tailSideDir = +1;
  }

  servoWriteAngleByChannel(TAIL_SIDE_CH, tailSideAngle);
}

// Apply tail behavior on state enter
void setTailForState(EmotionState s, uint32_t now) {
  switch (s) {
    // center both at 90
    case NEUTRAL:
    case CONFUSED:
    case ANNOYED:
      tailSetFixed(TAIL_CENTER, TAIL_CENTER);
      break;

    // happy: up/down 90, sideways smooth sweep 30<->150
    case HAPPY_SHORT:
    case HAPPY_MEDIUM:
    case HAPPY_LONG:
      tailStartSmoothSweep(now);
      break;

    // angry: sideways 90, up/down 30
    case ANGRY:
      tailSetFixed(TAIL_CENTER, TAIL_MIN);
      break;

    // sleepy: sideways 90, up/down 150
    case SLEEPY:
      tailSetFixed(TAIL_CENTER, TAIL_MAX);
      break;

    default:
      tailSetFixed(TAIL_CENTER, TAIL_CENTER);
      break;
  }
}

// =========================
// EAR SERVO on GPIO2 using LEDC (FIXED ANGLE PER STATE)
// =========================
const int EAR_PIN = 2;
const int EAR_CH  = 5;

bool earAttached = false;

void earAttachIfNeeded() {
  if (earAttached) return;

  bool ok = ledcAttachChannel(EAR_PIN, SERVO_FREQ_HZ, SERVO_RES_BITS, EAR_CH);
  if (!ok) {
    Serial.printf("[EAR] ledcAttachChannel FAILED! pin=%d ch=%d\n", EAR_PIN, EAR_CH);
    earAttached = false;
    return;
  }

  // default at attach
  servoWriteAngleByChannel(EAR_CH, 90);
  earAttached = true;
  Serial.printf("[EAR] LEDC attached. pin=%d ch=%d\n", EAR_PIN, EAR_CH);
}

void setEarAngleForState(EmotionState s) {
  earAttachIfNeeded();
  if (!earAttached) return;

  int angle = 90; // default

  switch (s) {
    case NEUTRAL:
    case CONFUSED:
    case ANNOYED:
    case HAPPY_SHORT:
    case HAPPY_MEDIUM:
    case HAPPY_LONG:
      angle = 90;
      break;

    case SLEEPY:
      angle = 30;
      break;

    case ANGRY:
      angle = 150;
      break;

    default:
      angle = 90;
      break;
  }

  servoWriteAngleByChannel(EAR_CH, angle);
}

// =========================
// Helpers
// =========================
const char* getStateName(EmotionState s) {
  switch (s) {
    case NEUTRAL:      return "NEUTRAL";
    case HAPPY_SHORT:  return "HAPPY_SHORT";
    case HAPPY_MEDIUM: return "HAPPY_MEDIUM";
    case HAPPY_LONG:   return "HAPPY_LONG";
    case ANNOYED:      return "ANNOYED";
    case ANGRY:        return "ANGRY";
    case SLEEPY:       return "SLEEPY";
    case CONFUSED:     return "CONFUSED";
    default:           return "UNKNOWN";
  }
}

void debugPrint(const String &msg) {
  Serial.print("[STATE: ");
  Serial.print(getStateName(currentState));
  Serial.print("] ");
  Serial.println(msg);
}

void stopSound() {
  if (mp3) { mp3->stop(); delete mp3; mp3 = nullptr; }
  if (file) { delete file; file = nullptr; }
}

// =========================
// Network task config
// =========================
const uint32_t NET_MIN_INTERVAL_MS = 333;
const uint32_t RECONNECT_EVERY_MS  = 5000;

void handleSession(AdafruitIO_Data *data) {
  int v = data->toInt();
  sessionActive = (v == 1);
  sessionKnown = true;

  if (sessionActive) {
    Serial.println("[SESSION] START -> reset counters");
    resetSessionCounters();

    publishCountsIfChanged();
    enqueueFeed(FEED_CURRENT_STATE, false, getStateName(currentState));

    stateEnterCount[currentState]++;

    enqueueFeed(FEED_STATS_TEXT, false, "Session running...");
  } else {
    Serial.println("[SESSION] STOP -> publish summary");
    publishSessionSummary();
  }
}

void enqueueFeed(FeedId feed, bool gated, const char* txt) {
  if (!netQueue) return;
  if (gated && !sessionActive) return;

  NetMsg m;
  m.feed = feed;
  m.sessionGated = gated;
  strncpy(m.text, txt, sizeof(m.text) - 1);
  m.text[sizeof(m.text) - 1] = '\0';

  xQueueSend(netQueue, &m, 0);
}

void networkTask(void *param) {
  uint32_t lastSendMs = 0;
  uint32_t lastReconnectAttempt = 0;
  bool askedSessionOnce = false;

  io.connect();

  for (;;) {
    uint32_t now = millis();
    io.run();

    bool aioConnected = (io.status() == AIO_CONNECTED);

    if (!aioConnected) {
      askedSessionOnce = false;
      if (now - lastReconnectAttempt >= RECONNECT_EVERY_MS) {
        lastReconnectAttempt = now;
        io.connect();
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (!askedSessionOnce) {
      askedSessionOnce = true;
      sessionFeed->get();
    }

    if (now - lastSendMs >= NET_MIN_INTERVAL_MS) {
      NetMsg msg;
      if (xQueueReceive(netQueue, &msg, 0) == pdTRUE) {
        lastSendMs = now;

        switch (msg.feed) {
          case FEED_EVENT:         eventFeed->save(msg.text); break;
          case FEED_STATE_LOG:     stateFeed->save(msg.text); break;
          case FEED_CURRENT_STATE: currentStateFeed->save(msg.text); break;
          case FEED_HEAD_COUNT:    headCountFeed->save(msg.text); break;
          case FEED_TAIL_COUNT:    tailCountFeed->save(msg.text); break;
          case FEED_STATS_TEXT:    statsFeed->save(msg.text); break;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// =========================
// Touch debounce helper
// =========================
struct DebouncedTouch {
  bool stable = false;
  bool lastRaw = false;
  uint32_t lastChangeMs = 0;

  bool update(bool raw, uint32_t nowMs) {
    if (raw != lastRaw) {
      lastRaw = raw;
      lastChangeMs = nowMs;
    }
    if ((nowMs - lastChangeMs) >= DEBOUNCE_MS && stable != raw) {
      stable = raw;
      return true;
    }
    return false;
  }
};

DebouncedTouch headDeb;
DebouncedTouch tailDeb;

// =====================================================
// EYE ANIMATION (animated style on SH1106)
// =====================================================
static const int E_W = 128;
static const int E_H = 64;
static const int CX = 64;
static const int CY = 32;
static const int R  = 30;

int pupilX = 0;
int pupilY = 0;
int pupilR = 8;

unsigned long lastMove = 0;
unsigned long lastBlink = 0;
unsigned long lastEyeRender = 0;

unsigned long moveInterval = 900;
unsigned long blinkInterval = 3500;

bool blinking = false;
int blinkStep = 0;

int zzzFrame = 0;
int confusedQFrame = 0;

unsigned long lastZzzTick = 0;
unsigned long lastConfusedTick = 0;

const unsigned long EYE_RENDER_INTERVAL_MS = 30;
const unsigned long ZZZ_TICK_MS = 90;
const unsigned long CONFUSED_TICK_MS = 90;

void drawEyeBase(Adafruit_SH1106G &d) {
  d.fillCircle(CX, CY, R, SH110X_WHITE);
  d.fillCircle(CX + pupilX, CY + pupilY, pupilR, SH110X_BLACK);
}

void drawBlinkMask(Adafruit_SH1106G &d) {
  d.fillRect(0, 0, E_W, blinkStep, SH110X_BLACK);
  d.fillRect(0, E_H - blinkStep, E_W, blinkStep, SH110X_BLACK);
}

void drawClosedEye(Adafruit_SH1106G &d) {
  d.drawLine(CX - 20, CY, CX + 20, CY, SH110X_WHITE);
}

void drawPuppyLids(Adafruit_SH1106G &d) {
  d.fillRect(0, E_H - 8, E_W, 8, SH110X_BLACK);
}

void drawAngryBrowLeft(Adafruit_SH1106G &d) {
  d.fillTriangle(0, 0, E_W, 0, E_W, 18, SH110X_BLACK);
}
void drawAngryBrowRight(Adafruit_SH1106G &d) {
  d.fillTriangle(0, 0, E_W, 0, 0, 18, SH110X_BLACK);
}

void drawConfusedQuestionMark(Adafruit_SH1106G &d) {
  int startY = E_H - 20;
  int qY = startY - confusedQFrame * 3;
  if (qY < E_H / 2) return;

  d.setTextSize(3);
  d.setTextColor(SH110X_WHITE);

  d.setCursor(10, qY);
  d.print("?");

  d.setCursor(11, qY);
  d.print("?");
}

void drawZzz(Adafruit_SH1106G &d) {
  int startY = E_H - 8;
  int zY = startY - zzzFrame * 4;
  if (zY < E_H / 2) return;

  d.setTextSize(1);
  d.setTextColor(SH110X_WHITE);

  d.setCursor(92, zY);
  d.print("Z");
  d.setCursor(102, zY - 6);
  d.print("z");
  d.setCursor(112, zY - 12);
  d.print("z");
}

void updateEyeConfigOnStateEnter(EmotionState s) {
  switch (s) {
    case HAPPY_SHORT:  pupilR = 10; break;
    case HAPPY_MEDIUM: pupilR = 14; break;
    case HAPPY_LONG:   pupilR = 18; break;
    case ANNOYED:      pupilR = 8;  break;
    case ANGRY:        pupilR = 6;  break;
    case CONFUSED:     pupilR = 8;  confusedQFrame = 0; break;
    case SLEEPY:       pupilR = 0;  zzzFrame = 0; break;
    default:           pupilR = 8;  break;
  }
}

void updateMovement(unsigned long now) {
  if (now - lastMove < moveInterval) return;
  lastMove = now;

  switch (currentState) {
    case ANNOYED:
      pupilX = random(-8, 8);
      pupilY = random(-6, 6);
      moveInterval = 300;
      break;

    case CONFUSED:
      pupilX = random(-4, 4);
      pupilY = random(-3, 3);
      moveInterval = 800;
      break;

    case SLEEPY:
      pupilX = 0;
      pupilY = 0;
      break;

    default:
      pupilX = random(-3, 3);
      pupilY = random(-2, 2);
      moveInterval = 900;
      break;
  }
}

void updateBlink(unsigned long now) {
  if (currentState == SLEEPY) return;

  if (!blinking && (now - lastBlink > blinkInterval)) {
    blinking = true;
    blinkStep = 0;
  }

  if (blinking) {
    blinkStep += 4;
    if (blinkStep >= R) {
      blinking = false;
      blinkStep = 0;
      lastBlink = now;
      blinkInterval = random(2500, 5000);
    }
  }
}

void updateSpecialFrames(unsigned long now) {
  if (currentState == SLEEPY) {
    if (now - lastZzzTick >= ZZZ_TICK_MS) {
      lastZzzTick = now;
      zzzFrame++;
      if (zzzFrame > 12) zzzFrame = 0;
    }
  }

  if (currentState == CONFUSED) {
    if (now - lastConfusedTick >= CONFUSED_TICK_MS) {
      lastConfusedTick = now;
      confusedQFrame++;
      if (confusedQFrame > 10) confusedQFrame = 0;
    }
  }
}

void renderEyes(unsigned long now) {
  if (now - lastEyeRender < EYE_RENDER_INTERVAL_MS) return;
  lastEyeRender = now;

  displayLeft.clearDisplay();
  displayRight.clearDisplay();

  if (currentState == SLEEPY) {
    drawClosedEye(displayLeft);
    drawClosedEye(displayRight);
    drawZzz(displayRight);
  } else {
    drawEyeBase(displayLeft);
    drawEyeBase(displayRight);

    if (currentState == HAPPY_SHORT || currentState == HAPPY_MEDIUM || currentState == HAPPY_LONG) {
      drawPuppyLids(displayLeft);
      drawPuppyLids(displayRight);
    }

    if (currentState == ANGRY) {
      drawAngryBrowLeft(displayLeft);
      drawAngryBrowRight(displayRight);
    }

    if (currentState == CONFUSED) {
      drawConfusedQuestionMark(displayLeft);
    }

    if (blinking) {
      drawBlinkMask(displayLeft);
      drawBlinkMask(displayRight);
    }
  }

  displayLeft.display();
  displayRight.display();
}

// =========================
// playSound
// =========================
void playSound(const char* filename) {
  stopSound();
  debugPrint(String("[AUDIO] Starting sound: ") + filename);

  if (!SD.exists(filename)) {
    debugPrint(String("[AUDIO] File NOT FOUND on SD: ") + filename);
    return;
  }

  file = new AudioFileSourceSD(filename);
  mp3  = new AudioGeneratorMP3();

  if (!mp3->begin(file, out)) {
    debugPrint("[AUDIO] mp3->begin FAILED");
    stopSound();
  } else {
    debugPrint("[AUDIO] Sound started");
  }
}

// =========================
// State change (time-safe: pass 'now' in)
// =========================
bool setEmotionState(EmotionState newState, unsigned long now) {
  emotionalTimeoutPending = false;

  if (newState == currentState) return false;

  debugPrint(String("[STATE] Changing from ") +
             getStateName(currentState) + " -> " +
             getStateName(newState));

  currentState   = newState;
  stateStartTime = now;

  enqueueFeed(FEED_STATE_LOG, true, getStateName(currentState));
  enqueueFeed(FEED_CURRENT_STATE, false, getStateName(currentState));

  if (sessionActive) stateEnterCount[currentState]++;

  updateEyeConfigOnStateEnter(currentState);

  resetMovementOneShots();

  // ✅ EAR: set fixed angle on EVERY state change
  setEarAngleForState(currentState);

  // ✅ TAIL: apply requested behavior on EVERY state change
  setTailForState(currentState, (uint32_t)now);

  // ✅ movement triggers
  if (currentState == HAPPY_SHORT) {
    happyShortMoveActive = true;
    happyShortMoveStart  = now;
  }
  if (currentState == HAPPY_MEDIUM) {
    happyMediumMoveActive = true;
    happyMediumMoveStart  = now;
  }
  if (currentState == HAPPY_LONG) {
    happyLongTurnActive = true;
    happyLongTurnStart  = now;
  }
  if (currentState == ANNOYED) {
    annoyedMoveActive = true;
    annoyedMoveStart  = now;
  }
  if (currentState == ANGRY) {
    angryMoveActive = true;
    angryMoveStart  = now;
  }

  if (currentState == NEUTRAL) neutralStartTime = now;
  else if (currentState != SLEEPY) {
    emotionalStartTime = now;
    lastEmoSecPrinted  = 0;
  }

  switch (currentState) {
    case NEUTRAL:      stopSound();                  break;
    case HAPPY_SHORT:  playSound(FILE_HAPPY_SHORT);  break;
    case HAPPY_MEDIUM: playSound(FILE_HAPPY_MEDIUM); break;
    case HAPPY_LONG:   playSound(FILE_HAPPY_LONG);   break;
    case ANNOYED:      playSound(FILE_ANNOYED);      break;
    case ANGRY:        playSound(FILE_ANGRY);        break;
    case SLEEPY:       playSound(FILE_SLEEPY);       break;
    case CONFUSED:     playSound(FILE_CONFUSED);     break;
    default: break;
  }

  return true;
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  delay(500);

  debugPrint("=== ESPuppy + Adafruit IO (queue + non-block reconnect) ===");

  pinMode(HEAD_TOUCH_PIN, INPUT);
  pinMode(TAIL_TOUCH_PIN, INPUT);

  Wire.begin(); // SDA=21, SCL=22

  if (!displayLeft.begin(OLED_ADDR_LEFT, true)) debugPrint("[OLED] Left display init FAILED!");
  else { displayLeft.setRotation(0); displayLeft.clearDisplay(); displayLeft.display(); debugPrint("[OLED] Left display OK"); }

  if (!displayRight.begin(OLED_ADDR_RIGHT, true)) debugPrint("[OLED] Right display init FAILED!");
  else { displayRight.setRotation(0); displayRight.clearDisplay(); displayRight.display(); debugPrint("[OLED] Right display OK"); }

  randomSeed((uint32_t)esp_random());

  debugPrint("[SD] Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    debugPrint("[SD] SD init FAILED! Check wiring and card.");
    while (1) delay(1000);
  }
  debugPrint("[SD] SD OK!");

  out = new AudioOutputI2S();
  out->SetPinout(27, 26, 25);
  out->SetGain(0.3);

  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motorsStop();

  netQueue = xQueueCreate(40, sizeof(NetMsg));
  sessionFeed->onMessage(handleSession);

  xTaskCreatePinnedToCore(networkTask, "networkTask", 4096, NULL, 1, NULL, 0);

  Serial.println(io.statusText());

  unsigned long now = millis();
  updateEyeConfigOnStateEnter(NEUTRAL);

  // set initial state + ear + tail
  setEmotionState(NEUTRAL, now);

  lastEyeRender = 0;
  renderEyes(now);
}

// =========================
// Loop
// =========================
void loop() {
  unsigned long now = millis();

  int headValueRaw = touchRead(HEAD_TOUCH_PIN);
  int tailValueRaw = touchRead(TAIL_TOUCH_PIN);

  bool headTouchedRaw = (headValueRaw < HEAD_THRESHOLD);
  bool tailTouchedRaw = (tailValueRaw < TAIL_THRESHOLD);

  bool headChanged = headDeb.update(headTouchedRaw, now);
  bool tailChanged = tailDeb.update(tailTouchedRaw, now);

  bool headTouched = headDeb.stable;
  bool tailTouched = tailDeb.stable;

  // =======================
  // HEAD PET
  // =======================
  if (headChanged && headTouched) {
    headWasTouched = true;
    headTouchStartTime = now;
    debugPrint(String("[TOUCH] Head START ms=") + String(now) + " value=" + String(headValueRaw));
    enqueueFeed(FEED_EVENT, true, "head_pet");
  }

  if (headChanged && !headTouched && headWasTouched) {
    headWasTouched = false;
    unsigned long duration = now - headTouchStartTime;

    debugPrint(String("[TOUCH] Head END ms=") + String(now) +
               " duration=" + String(duration) + " ms");

    if (currentState == ANNOYED) {
      unsigned long timeSinceAnnoyedStart = headTouchStartTime - stateStartTime;
      if (timeSinceAnnoyedStart <= ANNOYED_CONFUSED_WINDOW && duration >= PET_IGNORE_MAX) {
        setEmotionState(CONFUSED, now);
      }
    }
    else if (currentState == SLEEPY) {
      if (duration >= PET_IGNORE_MAX) setEmotionState(HAPPY_SHORT, now);
    }
    else {
      if (duration >= PET_IGNORE_MAX) {
        switch (currentState) {
          case NEUTRAL:
            if (duration < PET_SHORT_MAX) setEmotionState(HAPPY_SHORT, now);
            else if (duration < PET_MEDIUM_MAX) setEmotionState(HAPPY_MEDIUM, now);
            else setEmotionState(HAPPY_LONG, now);
            break;

          case HAPPY_SHORT:  setEmotionState(HAPPY_MEDIUM, now); break;
          case HAPPY_MEDIUM: setEmotionState(HAPPY_LONG, now);   break;
          case CONFUSED:     setEmotionState(HAPPY_SHORT, now);  break;
          default: break;
        }
      }
    }

    if (sessionActive && duration >= PET_IGNORE_MAX) {
      sessionHeadCount++;
      publishCountsIfChanged();
    }
  }

  // =======================
  // TAIL TOUCH
  // =======================
  if (tailChanged && tailTouched) {
    tailWasTouched = true;
    tailTouchStartTime = now;
    lastTailTouchTime  = now;
    debugPrint(String("[TOUCH] Tail START ms=") + String(now) + " value=" + String(tailValueRaw));
    enqueueFeed(FEED_EVENT, true, "tail_touch");
  }

  if (tailChanged && !tailTouched && tailWasTouched) {
    tailWasTouched = false;
    unsigned long duration = now - tailTouchStartTime;

    debugPrint(String("[TOUCH] Tail END ms=") + String(now) + " duration=" + String(duration));

    if (currentState == SLEEPY) {
      if (duration >= PET_IGNORE_MAX) setEmotionState(ANNOYED, now);
    }
    else if (currentState == ANNOYED) {
      if (duration >= PET_IGNORE_MAX) setEmotionState(ANGRY, now);
    }
    else if (currentState == CONFUSED) {
      if (duration >= PET_IGNORE_MAX) setEmotionState(ANNOYED, now);
    }
    else if (currentState == ANGRY) {
      // stay angry
    }
    else {
      if (duration >= PET_IGNORE_MAX) setEmotionState(ANNOYED, now);
    }

    if (sessionActive && duration >= PET_IGNORE_MAX) {
      sessionTailCount++;
      publishCountsIfChanged();
    }
  }

  // =======================
  // EMO TIMER -> NEUTRAL
  // =======================
  if (currentState != NEUTRAL && currentState != SLEEPY) {
    unsigned long elapsed = now - emotionalStartTime;
    unsigned long sec     = elapsed / 1000;

    if (sec != (unsigned long)lastEmoSecPrinted && elapsed < RESET_TIMEOUT) {
      lastEmoSecPrinted = (int)sec;
      debugPrint("[EMO TIMER] " + String(sec) + "s / " + String(RESET_TIMEOUT / 1000) + "s");
    }

    bool interactingNow = headWasTouched || tailWasTouched;

    if (elapsed >= RESET_TIMEOUT) {
      if (interactingNow) {
        if (!emotionalTimeoutPending) {
          emotionalTimeoutPending = true;
          debugPrint("[TIMER] Emotional timeout reached BUT interaction active -> pending");
        }
      } else {
        debugPrint("[TIMER] Emotional timeout -> NEUTRAL");
        setEmotionState(NEUTRAL, now);
      }
    }
  }

  // =======================
  // NEUTRAL -> SLEEPY
  // =======================
  if (currentState == NEUTRAL && (now - neutralStartTime) >= SLEEPY_TIMEOUT) {
    debugPrint("[TIMER] NEUTRAL 30s -> SLEEPY");
    setEmotionState(SLEEPY, now);
  }

  // =======================
  // MOTORS (timed)
  // =======================
  if (currentState == HAPPY_SHORT && happyShortMoveActive) {
    if (now - happyShortMoveStart <= HAPPY_SHORT_MOVE_MS) motorsForward(HAPPY_SHORT_SPEED);
    else { motorsStop(); happyShortMoveActive = false; }
  }
  else if (currentState == HAPPY_MEDIUM && happyMediumMoveActive) {
    if (now - happyMediumMoveStart <= HAPPY_MEDIUM_MOVE_MS) motorsForward(HAPPY_MEDIUM_SPEED);
    else { motorsStop(); happyMediumMoveActive = false; }
  }
  else if (currentState == HAPPY_LONG && happyLongTurnActive) {
    if (now - happyLongTurnStart <= HAPPY_LONG_TURN_MS) motorsSpinRight(HAPPY_LONG_TURN_SPEED);
    else { motorsStop(); happyLongTurnActive = false; }
  }
  else if (currentState == ANNOYED && annoyedMoveActive) {
    if (now - annoyedMoveStart <= ANNOYED_MOVE_MS) motorsBackward(ANNOYED_BACK_SPEED);
    else { motorsStop(); annoyedMoveActive = false; }
  }
  else if (currentState == ANGRY && angryMoveActive) {
    if (now - angryMoveStart <= ANGRY_MOVE_MS) motorsBackward(ANGRY_BACK_SPEED);
    else { motorsStop(); angryMoveActive = false; }
  }
  else {
    motorsStop();
  }

  // ✅ Tail update (runs only when sweep active)
  tailUpdate((uint32_t)now);

  // AUDIO
  if (mp3 && mp3->isRunning()) {
    if (!mp3->loop()) {
      stopSound();
      debugPrint("[AUDIO] Playback finished");
    }
  }

  // EYES
  updateMovement(now);
  updateBlink(now);
  updateSpecialFrames(now);
  renderEyes(now);

  delay(10);
} 
