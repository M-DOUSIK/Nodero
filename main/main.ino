/*
 * ═══════════════════════════════════════════════════════════════════
 * NODERO — Unified Firmware v2.0
 *
 * FLOW:
 * IDLE → TIMER → ALARM → WAKE_WINDOW → TEST → AWAIT_EXIT
 * → ROAMING → RECHECK → COMPLETE
 *
 * TOKEN DEDUCTIONS:
 * • SleepWatch score < 50          → deductOnFail tokens
 * • Wake window expires            → deductOnFail × 2 tokens
 * • Did not exit room in 60 s      → deductOnFail tokens
 * • GY-87 tamper not fixed in 30 s → deductOnFail tokens
 * • Missed random recheck          → deductOnFail tokens
 *
 * OLED (I2C 0x3C, 128×64):
 * Shows state, countdowns, tokens, instructions
 * Questions are NOT shown on OLED — only on phone
 *
 * WIRING:
 * MPU6050 / QMC5883P / C4001 / SSD1306   SDA→GPIO8   SCL→GPIO9
 * NeoPixel RGB                           GPIO 38
 * Buzzer (passive)                       GPIO 5
 *
 * BOARD CONFIG (Arduino IDE):
 * Board     → ESP32S3 Dev Module
 * PSRAM     → OPI PSRAM
 * CPU       → 240 MHz
 * Partition → Huge APP (3MB No OTA)
 *
 * LIBRARIES:
 * ESPAsyncWebServer  AsyncTCP  Adafruit_NeoPixel
 * DFRobot_C4001  Adafruit_GFX  Adafruit_SSD1306
 *
 * CONNECT: WiFi=nodero  Pass=nodero123  → http://192.168.4.1
 * ═══════════════════════════════════════════════════════════════════
 */

// ── Enums FIRST — Arduino IDE preprocessor requirement ───────────
enum NodState {
  NS_IDLE,          // 0  waiting for alarm to be set
  NS_TIMER,         // 1  countdown to alarm
  NS_ALARM,         // 2  buzzer ringing
  NS_WAKE_WINDOW,   // 3  user has N minutes to complete test
  NS_TEST,          // 4  SleepWatch in progress
  NS_AWAIT_EXIT,    // 5  passed test, must leave room in 60s
  NS_ROAMING,       // 6  left room, random recheck pending
  NS_RECHECK,       // 7  must return to radar station
  NS_COMPLETE       // 8  session done
};

// ── Includes ──────────────────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DFRobot_C4001.h"
#include "sleep_scorer.h"

// ── Pins & I2C addresses ──────────────────────────────────────────
#define SDA_PIN       8
#define SCL_PIN       9
#define RGB_PIN       38
#define BUZZER_PIN    19
#define MPU_ADDR      0x68
#define COMPASS_ADDR  0x2C
#define OLED_ADDR     0x3C   // change to 0x3D if your OLED uses that address
#define OLED_W        128
#define OLED_H        64

// ── Tuning ────────────────────────────────────────────────────────
#define TILT_THRESHOLD       4000
#define YAW_THRESHOLD         500
#define EXIT_POLLS             15    // consecutive empty polls to confirm room exit
#define TAMPER_GRACE_MS     30000UL  // 30 s to restore device after tamper
#define EXIT_TIMEOUT_MS     60000UL  // 60 s to leave room after completing test
#define RECHECK_TIMEOUT_MS  90000UL  // 90 s to return to station for recheck
#define WAKE_WINDOW_DEF_S     420    // 7 min default wake window
#define SCORE_PASS            50     // minimum SleepWatch score
#define SW_POOL               10
#define SW_RDS                 3
#define SW_QPR                 3

// ── PREPROCESSOR FIX: Structs moved up here ───────────────────────
struct SWQ { const char* text; const char* answer; uint16_t budget; };
struct SwAns { bool ok; uint32_t ms; };
struct SwSession {
  bool     active, done;
  uint8_t  round, qi;
  uint8_t  idx[SW_RDS][SW_QPR];
  SwAns    ans[SW_RDS][SW_QPR];
  uint32_t qStart;
  float    score;
};

// ── WiFi ──────────────────────────────────────────────────────────
const char* AP_SSID = "nodero";
const char* AP_PASS = "nodero123";

// ── Hardware objects ──────────────────────────────────────────────
AsyncWebServer    server(80);
Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);
DFRobot_C4001_I2C radar(&Wire, DEVICE_ADDR_0);
Adafruit_SSD1306  oled(OLED_W, OLED_H, &Wire, -1);

// ════════════════════════════════════════════════════════════════════
//  TOKEN SYSTEM
// ════════════════════════════════════════════════════════════════════
struct Tokens {
  int32_t balance      = 100;
  int32_t staked       = 0;
  int32_t deductOnFail = 10;
  int32_t totalLost    = 0;
  String  lastReason   = "";
} tok;

void deductTokens(int32_t n, const char* reason) {
  tok.balance   -= n;
  tok.totalLost += n;
  tok.lastReason = String(reason);
  if (tok.balance < 0) tok.balance = 0;
  Serial.printf("[TOKEN] -%d (%s) balance=%d\n", n, reason, tok.balance);
}

// ════════════════════════════════════════════════════════════════════
//  STATE MACHINE GLOBALS
// ════════════════════════════════════════════════════════════════════
NodState nodState       = NS_IDLE;
uint32_t wakeWindowEndMs = 0;          // absolute deadline for test + exit
uint32_t exitDeadlineMs  = 0;          // absolute deadline to leave room
uint32_t wakeWindowS     = WAKE_WINDOW_DEF_S;
bool     testDone        = false;
bool     testPassed      = false;
float    sessionScore    = 0;
bool     roomExited      = false;

// Recheck
uint32_t recheckMaxS      = 60;
uint32_t recheckFireMs    = 0;
bool     recheckPending   = false;
bool     recheckActive    = false;
uint32_t recheckStartMs   = 0;

// Forward declarations
void setNodState(NodState s);
void triggerAlarm(bool fromTamper);
void silenceAlarm();
void armWatchdog();

void setNodState(NodState s) {
  nodState = s;
  Serial.printf("[STATE] → %d\n", (int)s);
}

// ════════════════════════════════════════════════════════════════════
//  ALARM
// ════════════════════════════════════════════════════════════════════
bool     alarmActive   = false;
bool     alarmTamper   = false;
uint32_t alarmUntilMs  = 0;
bool     timerRunning  = false;
uint32_t timerEndMs    = 0;
uint32_t buzzerNextMs  = 0;
bool     buzzerState   = false;

void triggerAlarm(bool fromTamper) {
  alarmActive  = true;
  alarmTamper  = fromTamper;
  alarmUntilMs = fromTamper ? millis() + 30000UL : 0;
  Serial.println("[ALARM] " + String(fromTamper ? "TAMPER" : "WAKE"));
}
void silenceAlarm() {
  alarmActive = alarmTamper = false;
  noTone(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
}
void pollAlarm() {
  uint32_t now = millis();
  // Tamper alarm auto-silences after 30 s
  if (alarmActive && alarmTamper && now > alarmUntilMs) silenceAlarm();
  if (alarmActive && now >= buzzerNextMs) {
    buzzerState = !buzzerState;
    buzzerState ? tone(BUZZER_PIN, alarmTamper ? 2400 : 1800)
                : noTone(BUZZER_PIN);
    buzzerNextMs = now + (alarmTamper ? 100 : 250);
  }
  if (!alarmActive) digitalWrite(BUZZER_PIN, LOW);
}

// ════════════════════════════════════════════════════════════════════
//  WATCHDOG — GY-87 (MPU6050 + QMC5883P)
// ════════════════════════════════════════════════════════════════════
int16_t  baseTiltX, baseTiltY, baseYawX, baseYawY;
bool     watchdogArmed  = false;
bool     tamperDetected = false;
bool     tamperInGrace  = false;
uint32_t tamperGraceMs  = 0;
String   tamperReason   = "";
uint32_t lastWdMs       = 0;
bool     wdReady        = false;

void readMPU(int16_t& tx, int16_t& ty) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)4, true);
  tx = Wire.read()<<8 | Wire.read();
  ty = Wire.read()<<8 | Wire.read();
}
void readCompass(int16_t& cx, int16_t& cy) {
  Wire.beginTransmission(COMPASS_ADDR); Wire.write(0x01); Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)COMPASS_ADDR, (uint8_t)4, true);
  uint8_t xL=Wire.read(), xH=Wire.read(), yL=Wire.read(), yH=Wire.read();
  cx = xL|(xH<<8); cy = yL|(yH<<8);
}
void initWatchdog() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x37); Wire.write(0x02); Wire.endTransmission();
  delay(100);
  Wire.beginTransmission(COMPASS_ADDR); Wire.write(0x29); Wire.write(0x06); Wire.endTransmission();
  Wire.beginTransmission(COMPASS_ADDR); Wire.write(0x0B); Wire.write(0x08); Wire.endTransmission();
  Wire.beginTransmission(COMPASS_ADDR); Wire.write(0x0A); Wire.write(0xCD); Wire.endTransmission();
  delay(500);
  wdReady = true;
  Serial.println("[WD] Sensors OK");
}
void armWatchdog() {
  if (!wdReady) return;
  readMPU(baseTiltX, baseTiltY);
  readCompass(baseYawX, baseYawY);
  tamperDetected = tamperInGrace = false;
  tamperReason = "";
  watchdogArmed = true;
  Serial.println("[WD] ARMED");
}
void pollWatchdog() {
  if (!watchdogArmed || !wdReady) return;
  if (millis()-lastWdMs < 500) return;
  lastWdMs = millis();

  int16_t tx, ty, cx, cy;
  readMPU(tx, ty);
  readCompass(cx, cy);

  bool tiltOk = abs(tx-baseTiltX)<=TILT_THRESHOLD && abs(ty-baseTiltY)<=TILT_THRESHOLD;
  bool yawOk  = abs(cx-baseYawX)<=YAW_THRESHOLD   && abs(cy-baseYawY)<=YAW_THRESHOLD;
  bool nowOk  = tiltOk && yawOk;

  // New tamper detected → start grace period
  if (!nowOk && !tamperDetected && !tamperInGrace) {
    tamperInGrace = true;
    tamperGraceMs = millis();
    tamperReason  = tiltOk ? "ROTATION" : "TILT";
    triggerAlarm(true);
    Serial.println("[WD] TAMPER — 30s grace started");
  }

  if (tamperInGrace) {
    if (nowOk) {
      // Restored within grace — no penalty
      tamperInGrace = false;
      tamperReason  = "";
      silenceAlarm();
      Serial.println("[WD] Restored in time — no deduction");
    } else if (millis()-tamperGraceMs > TAMPER_GRACE_MS) {
      // Grace expired → penalise
      tamperDetected = true;
      tamperInGrace  = false;
      deductTokens(tok.deductOnFail, "GY-87 tamper not restored");
    }
  }
}

// ════════════════════════════════════════════════════════════════════
//  RADAR — DFRobot C4001
// ════════════════════════════════════════════════════════════════════
int      radarTargets = 0;
float    radarDist    = 0, radarSpeed = 0, radarEnergy = 0;
bool     roomOccupied = false;
bool     radarReady   = false;
int      emptyStreak  = 0;
uint32_t lastRadarMs  = 0;

void pollRadar() {
  if (!radarReady) return;
  if (millis()-lastRadarMs < 200) return;
  lastRadarMs = millis();

  int   n = radar.getTargetNumber(); delay(5);
  float d = radar.getTargetRange();  delay(5);
  float s = radar.getTargetSpeed();  delay(5);
  float e = radar.getTargetEnergy();

  if (n > 0) {
    emptyStreak  = 0;
    roomOccupied = true;
    radarTargets = n; radarDist = d; radarSpeed = s; radarEnergy = e;
  } else {
    emptyStreak++;
    radarTargets = 0; radarDist = 0; radarSpeed = 0; radarEnergy = 0;
    if (emptyStreak >= EXIT_POLLS) roomOccupied = false;
  }
}

bool isRoomEmpty() { return !roomOccupied && emptyStreak >= EXIT_POLLS; }

// ════════════════════════════════════════════════════════════════════
//  SLEEPWATCH QUESTION BANK
// ════════════════════════════════════════════════════════════════════
const SWQ EASY[] = {
  {"What is 4 + 7?",                        "11",      7000},
  {"What is 9 - 3?",                        "6",       7000},
  {"Type the word: CLOUD",                  "cloud",   6000},
  {"Odd one: Cat, Dog, Table, Bird?",       "table",   9000},
  {"Complete: 2, 4, 6, ___",               "8",       7000},
  {"Sun rises in EAST or WEST?",           "east",    7000},
  {"What is 5 x 3?",                       "15",      8000},
  {"Type the word: RIVER",                 "river",   6000},
  {"What comes after Sunday?",             "monday",  7000},
  {"What is 20 / 4?",                      "5",       8000},
};
const SWQ MEDIUM[] = {
  {"What is 13 x 7?",                      "91",      11000},
  {"Spell backwards: CAT",                 "tac",     9000},
  {"Next: 2, 4, 8, 16, ___",              "32",      11000},
  {"Opposite of ANCIENT?",                "modern",  9000},
  {"Repeat this: 7 3 9",                  "7 3 9",   10000},
  {"Odd one: Tennis, Football, Bat?",      "bat",     11000},
  {"What is 8 squared?",                   "64",      11000},
  {"Spell backwards: DOG",                "god",     9000},
  {"What is 144 / 12?",                   "12",      11000},
  {"Opposite of SHALLOW?",                "deep",    9000},
};
const SWQ HARD[] = {
  {"What is 17 x 13?",                     "221",       14000},
  {"Spell backwards: SLEEP",              "peels",     12000},
  {"Next: 1, 1, 2, 3, 5, 8, ___",        "13",        12000},
  {"Odd: Oxygen, Nitrogen, Iron, Helium?","iron",      13000},
  {"Repeat: 4 7 2 9 1",                   "4 7 2 9 1", 13000},
  {"Next: 3, 9, 27, 81, ___",            "243",       13000},
  {"What is 256 / 16?",                   "16",        14000},
  {"Odd: Jupiter, Mars, Moon, Saturn?",   "moon",      13000},
  {"Synonym of FATIGUED?",               "tired",     12000},
  {"12 squared minus 44?",               "100",       14000},
};

SwSession sw;

void swShuffle(uint8_t* a, uint8_t n) {
  for (int i=n-1;i>0;i--){uint8_t j=random(i+1),t=a[i];a[i]=a[j];a[j]=t;}
}
void swPickRound(uint8_t r) {
  uint8_t p[SW_POOL];
  for (uint8_t i=0;i<SW_POOL;i++) p[i]=i;
  swShuffle(p, SW_POOL);
  for (uint8_t i=0;i<SW_QPR;i++) sw.idx[r][i]=p[i];
}
const SWQ& swCurQ() {
  uint8_t i = sw.idx[sw.round][sw.qi];
  return sw.round==0 ? EASY[i] : sw.round==1 ? MEDIUM[i] : HARD[i];
}
bool swCheckAns(const String& raw) {
  String u=raw; u.trim(); u.toLowerCase();
  String e=String(swCurQ().answer); e.toLowerCase();
  return u==e;
}
const char* swVerdict(float s) {
  if (s>=85) return "FULLY ALERT";
  if (s>=68) return "MILDLY DROWSY";
  if (s>=50) return "MODERATELY SLEEPY";
  if (s>=30) return "SEVERELY IMPAIRED";
  return "CRITICALLY SLEEP-DEPRIVED";
}
float swRunScorer() {
  double ra[3]={}, rs[3]={}, all[9]; uint8_t ti=0;
  for (uint8_t r=0;r<SW_RDS;r++){
    uint8_t ok=0; double ss=0;
    for (uint8_t q=0;q<SW_QPR;q++){
      if (sw.ans[r][q].ok) ok++;
      ss+=sw.ans[r][q].ms; all[ti++]=sw.ans[r][q].ms;
    }
    ra[r]=ok/(double)SW_QPR; rs[r]=ss/SW_QPR;
  }
  double mn=0; for(uint8_t i=0;i<9;i++) mn+=all[i]; mn/=9;
  double v=0;  for(uint8_t i=0;i<9;i++) v+=(all[i]-mn)*(all[i]-mn);
  double feats[9]={ra[0],ra[1],ra[2],rs[0],rs[1],rs[2],
                   rs[2]-rs[0], ra[2]-ra[0], sqrt(v/9.0)};
  return score_session(feats);
}
void swStart() {
  memset(&sw, 0, sizeof(sw));
  sw.active = true;
  for (uint8_t r=0;r<SW_RDS;r++) swPickRound(r);
  sw.qStart = millis();
  setNodState(NS_TEST);
}
void swAnswer(const String& input) {
  uint32_t ms = millis()-sw.qStart;
  sw.ans[sw.round][sw.qi] = { swCheckAns(input), ms };
  sw.qi++;
  if (sw.qi>=SW_QPR) { sw.round++; sw.qi=0; }
  if (sw.round>=SW_RDS) {
    sw.score     = swRunScorer();
    sw.done      = true;
    sw.active    = false;
    testDone     = true;
    testPassed   = (sw.score >= SCORE_PASS);
    sessionScore = sw.score;
    if (!testPassed)
      deductTokens(tok.deductOnFail, "SleepWatch score too low");
    // Move to exit phase regardless of score
    exitDeadlineMs = millis() + EXIT_TIMEOUT_MS;
    setNodState(NS_AWAIT_EXIT);
  }
  sw.qStart = millis();
}

// ════════════════════════════════════════════════════════════════════
//  LED
// ════════════════════════════════════════════════════════════════════
uint8_t  ledPulse = 0;
bool     ledDir   = true;
uint32_t lastLedMs = 0;

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r,g,b)); pixels.show();
}
void updateLed() {
  if (millis()-lastLedMs < 40) return;
  lastLedMs = millis();
  switch (nodState) {
    case NS_IDLE:        setLed(0,5,0);   break;
    case NS_TIMER:       setLed(0,0,30);  break;
    case NS_ALARM:
      if(ledDir){ledPulse+=12;if(ledPulse>=200)ledDir=false;}
      else{ledPulse-=12;if(ledPulse<12)ledDir=true;}
      setLed(ledPulse,0,0); break;
    case NS_WAKE_WINDOW: setLed(40,20,0); break;
    case NS_TEST:        setLed(30,30,0); break;
    case NS_AWAIT_EXIT:  setLed(60,30,0); break;
    case NS_ROAMING:     setLed(0,20,0);  break;
    case NS_RECHECK:
      if(ledDir){ledPulse+=10;if(ledPulse>=180)ledDir=false;}
      else{ledPulse-=10;if(ledPulse<10)ledDir=true;}
      setLed(ledPulse,ledPulse,0); break;
    case NS_COMPLETE:    setLed(0,60,0);  break;
  }
}

// ════════════════════════════════════════════════════════════════════
//  OLED
// ════════════════════════════════════════════════════════════════════
bool     oledReady  = false;
uint32_t lastOledMs = 0;

String fmtCountdown(uint32_t remMs) {
  if (remMs == 0) return "00:00";
  uint32_t s = remMs/1000;
  char b[8];
  snprintf(b,sizeof(b),"%02lu:%02lu",(unsigned long)(s/60),(unsigned long)(s%60));
  return String(b);
}
// Draw a simple centered big number (2× text size)
void oledBig(const String& s, uint8_t y) {
  oled.setTextSize(2);
  int16_t x = (OLED_W - s.length()*12) / 2;
  if (x<0) x=0;
  oled.setCursor(x, y);
  oled.print(s);
}
void oledSmall(const String& s, uint8_t x, uint8_t y) {
  oled.setTextSize(1); oled.setCursor(x,y); oled.print(s);
}

void oledUpdate() {
  if (!oledReady) return;
  if (millis()-lastOledMs < 150) return;
  lastOledMs = millis();

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  uint32_t now = millis();

  // ── Top bar ───────────────────────────────────────────────
  oled.setTextSize(1);
  oled.setCursor(0,0);   oled.print("NODERO");
  oled.setCursor(74,0);  oled.print("TOK:");
  oled.print(tok.balance);
  oled.drawLine(0,9,127,9,SSD1306_WHITE);

  // ── State content ─────────────────────────────────────────
  switch (nodState) {

    case NS_IDLE:
      oledSmall("Set alarm in app", 0, 14);
      oledSmall("192.168.4.1", 0, 26);
      oledSmall("Balance: " + String(tok.balance), 0, 42);
      break;

    case NS_TIMER: {
      uint32_t rem = timerEndMs > now ? timerEndMs-now : 0;
      oledSmall("ALARM IN", 38, 14);
      oledBig(fmtCountdown(rem), 26);
      oledSmall("Stake:" + String(tok.staked) + " tok", 0, 54);
      break;
    }

    case NS_ALARM:
      oledBig("WAKE UP!", 16);
      oledSmall("Open app > ALARM tab", 0, 40);
      oledSmall("Tap I'm Awake", 12, 52);
      break;

    case NS_WAKE_WINDOW: {
      uint32_t rem = wakeWindowEndMs > now ? wakeWindowEndMs-now : 0;
      oledSmall("TAKE TEST NOW", 14, 14);
      oledBig(fmtCountdown(rem), 26);
      oledSmall("Open app > SLEEP tab", 0, 54);
      break;
    }

    case NS_TEST: {
      uint32_t rem = wakeWindowEndMs > now ? wakeWindowEndMs-now : 0;
      oledSmall("TEST IN PROGRESS", 4, 14);
      oledSmall("Q" + String(sw.round*SW_QPR+sw.qi+1) + "/9  Round " +
                String(sw.round+1), 0, 26);
      oledSmall("Answer on phone", 8, 38);
      oledSmall("Window: " + fmtCountdown(rem), 0, 52);
      break;
    }

    case NS_AWAIT_EXIT: {
      uint32_t rem = exitDeadlineMs > now ? exitDeadlineMs-now : 0;
      oledSmall(testPassed ? "TEST PASSED!" : "TEST DONE", 20, 14);
      oledSmall("Score: " + String((int)sessionScore), 36, 26);
      oledBig("LEAVE!", 36);
      oledSmall(fmtCountdown(rem), 48, 54);
      break;
    }

    case NS_ROAMING:
      oledSmall("ROOM EXITED", 20, 14);
      oledSmall("Tokens: " + String(tok.balance), 20, 28);
      if (recheckPending)
        oledSmall("Recheck coming...", 4, 44);
      else
        oledSmall("Stay awake!", 24, 44);
      break;

    case NS_RECHECK: {
      uint32_t rem = (now-recheckStartMs < RECHECK_TIMEOUT_MS)
                     ? RECHECK_TIMEOUT_MS-(now-recheckStartMs) : 0;
      oledSmall("RETURN TO STATION", 0, 14);
      oledBig(fmtCountdown(rem), 26);
      oledSmall("Stand in front NOW", 2, 54);
      break;
    }

    case NS_COMPLETE:
      oledSmall("COMPLETE!", 28, 14);
      oledSmall("Score: " + String((int)sessionScore), 20, 26);
      oledSmall("Tokens: " + String(tok.balance) +
                "/" + String(tok.balance+tok.totalLost), 0, 40);
      oledSmall("Lost: " + String(tok.totalLost), 0, 52);
      break;
  }

  oled.display();
}

// ════════════════════════════════════════════════════════════════════
//  NODERO STATE MACHINE
// ════════════════════════════════════════════════════════════════════
void pollNodero() {
  uint32_t now = millis();

  switch (nodState) {

    case NS_IDLE:
      break;

    case NS_TIMER:
      if (now >= timerEndMs) {
        timerRunning = false;
        triggerAlarm(false);
        setNodState(NS_ALARM);
      }
      break;

    case NS_ALARM:
      // Waits for /api/awake POST from user
      break;

    case NS_WAKE_WINDOW:
      if (now > wakeWindowEndMs) {
        deductTokens(tok.deductOnFail*2, "Wake window expired without completing test");
        exitDeadlineMs = now + EXIT_TIMEOUT_MS;
        setNodState(NS_AWAIT_EXIT);
      }
      break;

    case NS_TEST:
      if (now > wakeWindowEndMs && !sw.done) {
        sw.done = sw.active = false;
        testDone = true; testPassed = false; sessionScore = 0;
        deductTokens(tok.deductOnFail*2, "Wake window expired during test");
        exitDeadlineMs = now + EXIT_TIMEOUT_MS;
        setNodState(NS_AWAIT_EXIT);
      }
      break;

    case NS_AWAIT_EXIT:
      if (isRoomEmpty()) {
        roomExited = true;
        uint32_t delay_ms = (uint32_t)(random(10, (long)recheckMaxS+1)) * 1000UL;
        recheckFireMs  = now + delay_ms;
        recheckPending = true;
        recheckActive  = false;
        Serial.printf("[RECHECK] fires in %lu s\n", delay_ms/1000);
        setNodState(NS_ROAMING);
      } else if (now > exitDeadlineMs) {
        deductTokens(tok.deductOnFail, "Did not exit room in time");
        uint32_t delay_ms = (uint32_t)(random(10, (long)recheckMaxS+1)) * 1000UL;
        recheckFireMs  = now + delay_ms;
        recheckPending = true;
        setNodState(NS_ROAMING);
      }
      break;

    case NS_ROAMING:
      if (recheckPending && now >= recheckFireMs) {
        recheckPending  = false;
        recheckActive   = true;
        recheckStartMs  = now;
        triggerAlarm(false);  // buzz to signal recheck
        setNodState(NS_RECHECK);
      }
      // No recheck configured → session complete
      if (!recheckPending && !recheckActive) {
        setNodState(NS_COMPLETE);
      }
      break;

    case NS_RECHECK:
      if (radarTargets > 0) {
        // User returned
        silenceAlarm();
        Serial.println("[RECHECK] PASS — user returned");
        setNodState(NS_COMPLETE);
      } else if (now-recheckStartMs > RECHECK_TIMEOUT_MS) {
        silenceAlarm();
        deductTokens(tok.deductOnFail, "Missed random recheck");
        setNodState(NS_COMPLETE);
      }
      break;

    case NS_COMPLETE:
      break;
  }
}

// ════════════════════════════════════════════════════════════════════
//  WEB PAGE (PROGMEM)
// ════════════════════════════════════════════════════════════════════
const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Nodero</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Orbitron:wght@700;900&display=swap');
:root{--bg:#07090a;--surf:#0d1214;--bord:#1a2830;--grn:#3ddc84;--amb:#f5a623;--red:#ff453a;--blu:#38bdf8;--muted:#3d5a4a;--txt:#b8d4c0;--sub:#476052;}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font-family:'Share Tech Mono',monospace;min-height:100vh;display:flex;flex-direction:column;align-items:center;padding:14px 12px 60px}
header{width:100%;max-width:600px;text-align:center;padding-bottom:12px;border-bottom:1px solid var(--bord);margin-bottom:14px}
.logo{font-family:'Orbitron',sans-serif;font-weight:900;font-size:clamp(20px,5vw,28px);letter-spacing:.35em;color:var(--grn)}.logo b{color:var(--amb)}
.tok-row{display:flex;justify-content:center;gap:16px;margin-top:8px;font-size:11px;color:var(--sub)}.tok-row span{color:var(--amb)}
.tok-row .lost{color:var(--red)}
.state-pill{display:inline-block;padding:3px 14px;border-radius:2px;font-family:'Orbitron',sans-serif;font-size:9px;letter-spacing:.15em;margin-top:7px;border:1px solid var(--grn);color:var(--grn)}
.alert{width:100%;max-width:600px;background:rgba(255,69,58,.1);border:1px solid var(--red);border-radius:3px;padding:12px;text-align:center;margin-bottom:12px;display:none;font-family:'Orbitron',sans-serif;font-size:12px;letter-spacing:.1em;color:var(--red)}
.alert.on{display:block}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.2}}
.dot{display:inline-block;width:6px;height:6px;border-radius:50%;background:var(--grn);margin-right:5px;vertical-align:middle;animation:blink 1.4s infinite}
.tabs{display:flex;width:100%;max-width:600px;gap:6px;margin-bottom:12px}
.tab{flex:1;padding:9px 4px;font-family:'Orbitron',sans-serif;font-size:9px;font-weight:700;letter-spacing:.1em;text-align:center;cursor:pointer;border-radius:3px;background:var(--surf);border:1px solid var(--bord);color:var(--sub);transition:all .2s}
.tab.on{color:var(--bg);border-color:transparent}
.tab.t0.on{background:var(--grn)}.tab.t1.on{background:var(--amb)}.tab.t2.on{background:var(--blu)}.tab.t3.on{background:var(--red)}
.panel{display:none;width:100%;max-width:600px}.panel.on{display:block}
.card{background:var(--surf);border:1px solid var(--bord);border-radius:3px;padding:18px 16px;position:relative;overflow:hidden;margin-bottom:12px}
.card::after{content:'';position:absolute;top:0;left:10%;right:10%;height:1px;background:linear-gradient(90deg,transparent,rgba(61,220,132,.3),transparent)}
@keyframes scan{from{transform:translateY(-100%)}to{transform:translateY(700%)}}
.scan{position:absolute;inset:0 0 auto;height:1px;background:rgba(61,220,132,.05);animation:scan 7s linear infinite;pointer-events:none}
.sect-title{font-family:'Orbitron',sans-serif;font-size:11px;font-weight:700;letter-spacing:.2em;margin-bottom:14px}
.row{display:flex;justify-content:space-between;align-items:center;padding:7px 0;border-bottom:1px solid var(--bord);font-size:12px}
.row:last-child{border-bottom:none}.rl{color:var(--sub)}.rv{color:var(--amb)}
.badge{font-size:9px;padding:2px 8px;border-radius:2px;font-family:'Orbitron',sans-serif}
.b-grn{background:rgba(61,220,132,.13);color:var(--grn)}.b-amb{background:rgba(245,166,35,.13);color:var(--amb)}.b-red{background:rgba(255,69,58,.13);color:var(--red)}.b-blu{background:rgba(56,189,248,.13);color:var(--blu)}
.stat-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:14px}
.stat{background:var(--bg);border:1px solid var(--bord);border-radius:3px;padding:12px 8px;text-align:center}
.stat-val{font-family:'Orbitron',sans-serif;font-size:20px;font-weight:700;color:var(--grn)}.stat-lbl{font-size:9px;color:var(--sub);letter-spacing:.1em;margin-top:3px}
.token-big{font-family:'Orbitron',sans-serif;font-size:42px;font-weight:900;text-align:center;color:var(--grn);margin:6px 0}
button{display:block;width:100%;padding:11px;font-family:'Orbitron',sans-serif;font-size:10px;font-weight:700;letter-spacing:.18em;border:none;border-radius:3px;cursor:pointer;transition:all .18s;margin-top:8px}
.btn-grn{background:var(--grn);color:var(--bg)}.btn-grn:hover{background:#5feba0}
.btn-amb{background:var(--amb);color:var(--bg)}.btn-amb:hover{background:#ffc040}
.btn-red{background:var(--red);color:#fff}.btn-red:hover{background:#ff6b5b}
.btn-ghost{background:transparent;border:1px solid var(--bord);color:var(--sub)}.btn-ghost:hover{border-color:var(--muted);color:var(--txt)}
input[type=text],input[type=number]{width:100%;background:var(--bg);border:1px solid var(--muted);border-radius:3px;color:var(--grn);font-family:'Share Tech Mono',monospace;font-size:17px;padding:11px 13px;outline:none;transition:border-color .2s;margin-bottom:10px}
input[type=text]:focus,input[type=number]:focus{border-color:var(--amb)}
input.ok{border-color:var(--grn)!important}input.bad{border-color:var(--red)!important}
.cfg-label{font-size:10px;color:var(--sub);letter-spacing:.12em;margin-bottom:6px;margin-top:10px}
.row2{display:flex;gap:8px;align-items:center;margin-bottom:8px}.row2 input{margin-bottom:0;flex:1}.row2 span{color:var(--sub);font-size:12px;white-space:nowrap}
.cd-big{font-family:'Orbitron',sans-serif;font-size:42px;font-weight:900;text-align:center;color:var(--amb);margin:8px 0}
.sw-screen{display:none}.sw-screen.on{display:block}
.prog-wrap{height:2px;background:var(--bord);border-radius:1px;margin-bottom:16px}
.prog{height:100%;border-radius:1px;background:var(--amb);box-shadow:0 0 5px var(--amb);transition:width .4s ease;width:0%}
.round-tag{font-size:10px;letter-spacing:.2em;color:var(--sub);margin-bottom:4px}
.diff{display:inline-block;font-size:9px;padding:2px 6px;border-radius:2px;margin-left:5px;vertical-align:middle}
.d0{background:rgba(61,220,132,.13);color:var(--grn)}.d1{background:rgba(245,166,35,.13);color:var(--amb)}.d2{background:rgba(255,69,58,.13);color:var(--red)}
.q-text{font-family:'Orbitron',sans-serif;font-size:clamp(14px,3.5vw,17px);font-weight:700;color:#e8f0ec;line-height:1.45;margin-bottom:14px}
.t-row{display:flex;align-items:center;gap:8px;margin-bottom:10px}
.t-lbl{font-size:9px;letter-spacing:.1em;color:var(--sub)}
.t-bar{flex:1;height:3px;background:var(--bord);border-radius:2px;overflow:hidden}
.t-fill{height:100%;border-radius:2px;transition:width .08s linear,background .25s}
.t-val{font-size:11px;min-width:34px;text-align:right}
.fb{padding:8px 11px;border-radius:3px;font-size:11px;margin-bottom:10px;display:none}
.f-ok{background:rgba(61,220,132,.09);color:var(--grn);border:1px solid rgba(61,220,132,.25)}
.f-bad{background:rgba(255,69,58,.09);color:var(--red);border:1px solid rgba(255,69,58,.25)}
.rs-num{font-family:'Orbitron',sans-serif;font-size:48px;font-weight:900;text-align:center;line-height:1;margin:8px 0 2px}
.rs-lbl{font-size:10px;letter-spacing:.15em;color:var(--sub);text-align:center;margin-bottom:14px}
.q-list{list-style:none;margin-bottom:14px}
.q-list li{display:flex;align-items:center;justify-content:space-between;padding:6px 0;border-bottom:1px solid var(--bord);font-size:11px}
.q-list li:last-child{border-bottom:none}
.q-txt{flex:1;padding-right:8px;color:var(--txt)}.q-ms{color:var(--sub);min-width:36px;text-align:right;margin-left:5px}
.final-num{font-family:'Orbitron',sans-serif;font-size:clamp(64px,16vw,88px);font-weight:900;line-height:1;text-align:center}
.final-verdict{font-family:'Orbitron',sans-serif;font-size:clamp(11px,2.5vw,14px);font-weight:700;letter-spacing:.1em;text-align:center;margin:6px 0 18px}
.ml-tag{font-size:9px;padding:2px 9px;border-radius:2px;display:block;width:fit-content;margin:0 auto 14px;border:1px solid rgba(61,220,132,.25);color:var(--grn);background:rgba(61,220,132,.06);letter-spacing:.1em}
.next-action{background:var(--bg);border:1px solid var(--amb);border-radius:3px;padding:12px;text-align:center;margin-top:14px;font-family:'Orbitron',sans-serif;font-size:11px;color:var(--amb);letter-spacing:.08em;display:none}
.total-row{border-top:1px solid var(--amb)!important;margin-top:3px;padding-top:10px!important}
</style>
</head>
<body>
<header>
  <div class="logo">NODE<b>RO</b></div>
  <div class="tok-row">
    <div>BAL: <span id="hBal">—</span></div>
    <div>STAKED: <span id="hStake">—</span></div>
    <div>LOST: <span class="lost" id="hLost">—</span></div>
  </div>
  <div class="state-pill" id="hState"><span class="dot"></span>IDLE</div>
</header>

<div class="alert" id="recheckAlert">⚠ RECHECK — RETURN TO RADAR STATION NOW!</div>

<div class="tabs">
  <div class="tab t0 on" onclick="go(0)">SLEEP</div>
  <div class="tab t1" onclick="go(1)">ALARM</div>
  <div class="tab t2" onclick="go(2)">RADAR</div>
  <div class="tab t3" onclick="go(3)">WATCHDOG</div>
</div>

<div class="panel on" id="p0">
  <div class="card">
    <div class="scan"></div>
    <div class="prog-wrap"><div class="prog" id="swProg"></div></div>
    <div class="sw-screen on" id="sw0">
      <div class="sect-title" style="color:var(--grn)">ALERTNESS TEST</div>
      <div style="font-size:12px;line-height:1.8;margin-bottom:16px">3 rounds · 9 questions · Adaptive difficulty · ML scoring</div>
      <div class="stat-grid">
        <div class="stat"><div class="stat-val">3</div><div class="stat-lbl">ROUNDS</div></div>
        <div class="stat"><div class="stat-val">3</div><div class="stat-lbl">Q/ROUND</div></div>
        <div class="stat"><div class="stat-val" style="font-size:13px;color:var(--amb)">ML</div><div class="stat-lbl">SCORING</div></div>
      </div>
      <button class="btn-grn" onclick="swStart()">BEGIN TEST</button>
    </div>
    <div class="sw-screen" id="sw1">
      <div class="round-tag" id="swRtag">ROUND 1 OF 3</div>
      <div class="q-text" id="swQtxt">—</div>
      <div class="t-row">
        <span class="t-lbl">TIME</span>
        <div class="t-bar"><div class="t-fill" id="swTfill" style="width:100%;background:var(--grn)"></div></div>
        <span class="t-val" id="swTval" style="color:var(--grn)">—</span>
      </div>
      <div class="fb" id="swFb"></div>
      <input type="text" id="swAns" placeholder="TYPE YOUR ANSWER…" autocomplete="off" autocorrect="off" spellcheck="false" onkeydown="if(event.key==='Enter')swSub()">
      <button class="btn-amb" id="swSbtn" onclick="swSub()">SUBMIT</button>
    </div>
    <div class="sw-screen" id="sw2">
      <div class="rs-lbl" id="swRsLbl">ROUND 1 COMPLETE</div>
      <div class="rs-num" id="swRsNum">0/3</div>
      <div class="rs-lbl">CORRECT</div>
      <ul class="q-list" id="swRsList"></ul>
      <button class="btn-amb" id="swRsNext">NEXT ROUND →</button>
    </div>
    <div class="sw-screen" id="sw3">
      <div class="ml-tag">▸ SCORED BY ON-DEVICE ML MODEL</div>
      <div style="font-size:10px;letter-spacing:.2em;color:var(--sub);text-align:center;margin-bottom:4px">SLEEP SCORE</div>
      <div class="final-num" id="swFnum">—</div>
      <div class="final-verdict" id="swFv">—</div>
      <div id="swFbd"></div>
      <div class="next-action" id="swNext">LEAVE THE ROOM WITHIN 60 SECONDS</div>
      <button class="btn-ghost" style="margin-top:14px" onclick="swRst()">TEST AGAIN</button>
    </div>
  </div>
</div>

<div class="panel" id="p1">
  <div class="card">
    <div class="scan"></div>
    <div class="sect-title" style="color:var(--amb)">ALARM SETUP</div>
    <div style="text-align:center;padding:14px 0 8px">
      <div style="font-size:42px" id="alIcon">🔕</div>
      <div style="font-family:'Orbitron',sans-serif;font-size:16px;font-weight:700;letter-spacing:.15em" id="alTxt">SILENT</div>
    </div>
    <div id="cdWrap" style="display:none"><div class="cd-big" id="cd">00:00</div></div>

    <div class="cfg-label">TOKENS PER FAILURE</div>
    <div class="row2"><input type="number" id="cfgStake" placeholder="10" min="1" max="999" style="text-align:center;font-size:22px"><span>tokens / fail</span></div>

    <div class="cfg-label">ALARM TIMER</div>
    <div class="row2">
      <input type="number" id="cfgMins" placeholder="MM" min="0" max="99" style="text-align:center;font-size:22px">
      <span>min</span>
      <input type="number" id="cfgSecs" placeholder="SS" min="0" max="59" style="text-align:center;font-size:22px">
      <span>sec</span>
    </div>

    <div class="cfg-label">WAKE WINDOW (minutes to complete test + exit)</div>
    <div class="row2"><input type="number" id="cfgWW" placeholder="7" min="2" max="30" style="text-align:center;font-size:22px"><span>minutes</span></div>

    <div class="cfg-label">MAX RECHECK DELAY (seconds, random 10s–this value)</div>
    <div class="row2"><input type="number" id="cfgRC" placeholder="60" min="10" max="300" style="text-align:center;font-size:22px"><span>seconds</span></div>

    <button class="btn-amb" onclick="startTimer()">START ALARM TIMER</button>
    <button class="btn-red" onclick="ringNow()">▶ RING NOW (TEST)</button>
    <button class="btn-grn" onclick="iAmAwake()">✓ I'M AWAKE — START TEST WINDOW</button>
    <button class="btn-ghost" onclick="resetSession()">RESET SESSION</button>
  </div>

  <div class="card">
    <div class="scan"></div>
    <div class="sect-title" style="color:var(--grn)">TOKEN LEDGER</div>
    <div class="token-big" id="tokBig">—</div>
    <div class="row"><span class="rl">BALANCE</span><span class="rv" id="tBal">—</span></div>
    <div class="row"><span class="rl">STAKED THIS SESSION</span><span class="rv" id="tStake">—</span></div>
    <div class="row"><span class="rl">LOST THIS SESSION</span><span style="color:var(--red)" id="tLost">—</span></div>
    <div class="row"><span class="rl">LAST DEDUCTION REASON</span><span class="rv" style="font-size:10px;max-width:60%;text-align:right" id="tReason">—</span></div>
    <button class="btn-ghost" onclick="resetTok()">RESET TOKENS TO 100</button>
  </div>
</div>

<div class="panel" id="p2">
  <div class="card">
    <div class="scan"></div>
    <div class="sect-title" style="color:var(--blu)">C4001 RADAR</div>
    <div class="stat-grid">
      <div class="stat"><div class="stat-val" id="rD" style="color:var(--blu)">—</div><div class="stat-lbl">DIST (m)</div></div>
      <div class="stat"><div class="stat-val" id="rS" style="color:var(--blu)">—</div><div class="stat-lbl">SPEED m/s</div></div>
      <div class="stat"><div class="stat-val" id="rE" style="color:var(--blu)">—</div><div class="stat-lbl">ENERGY</div></div>
    </div>
    <div class="row"><span class="rl">PRESENCE</span><span id="rOcc" class="badge b-grn">UNKNOWN</span></div>
    <div class="row"><span class="rl">TARGETS</span><span class="rv" id="rT">0</span></div>
    <div class="row"><span class="rl">EMPTY STREAK</span><span class="rv" id="rStr">0</span></div>
  </div>
</div>

<div class="panel" id="p3">
  <div class="card">
    <div class="scan"></div>
    <div class="sect-title" style="color:var(--red)">TAMPER WATCHDOG</div>
    <div style="text-align:center;padding:14px 0 10px">
      <div style="font-size:40px;margin-bottom:8px" id="wdIcon">🔓</div>
      <div style="font-family:'Orbitron',sans-serif;font-size:15px;font-weight:700;letter-spacing:.15em" id="wdSt">DISARMED</div>
    </div>
    <div class="row"><span class="rl">STATUS</span><span class="badge b-grn" id="wdBadge">OK</span></div>
    <div class="row"><span class="rl">GRACE REMAINING</span><span class="rv" id="wdGrace">—</span></div>
    <div class="row"><span class="rl">REASON</span><span class="rv" id="wdR">—</span></div>
    <button class="btn-red" onclick="armWD()">ARM WATCHDOG</button>
    <button class="btn-ghost" onclick="disarmWD()">DISARM</button>
  </div>
</div>

<script>
const $=id=>document.getElementById(id);
const wait=ms=>new Promise(r=>setTimeout(r,ms));
const SN=['IDLE','TIMER','ALARM','WAKE WINDOW','TEST','AWAIT EXIT','ROAMING','RECHECK','COMPLETE'];
const swC=s=>s>=68?'var(--grn)':s>=40?'var(--amb)':'var(--red)';
const tr=(s,n)=>s.length>n?s.slice(0,n)+'…':s;

function go(n){
  document.querySelectorAll('.tab').forEach((t,i)=>t.classList.toggle('on',i===n));
  document.querySelectorAll('.panel').forEach((p,i)=>p.classList.toggle('on',i===n));
  if(n===2)pollRadar();if(n===3)pollWD();
}

// ── Global status ─────────────────────────────────────────────────
async function pollStatus(){
  const d=await fetch('/api/status').then(r=>r.json()).catch(()=>null);
  if(!d)return;
  $('hBal').textContent=d.tokBal;
  $('hStake').textContent=d.tokStake;
  $('hLost').textContent=d.tokLost;
  $('hState').innerHTML='<span class="dot"></span>'+SN[d.state];
  $('tokBig').textContent=d.tokBal;
  $('tBal').textContent=d.tokBal;
  $('tStake').textContent=d.tokStake;
  $('tLost').textContent=d.tokLost;
  $('tReason').textContent=d.tokReason||'—';
  $('recheckAlert').classList.toggle('on',d.state===7);
  // Alarm UI
  const ia=$('alIcon'),at=$('alTxt');
  if(d.alarmActive){ia.textContent='🚨';at.textContent=d.alarmTamper?'TAMPER!':'WAKE UP!';at.style.color=d.alarmTamper?'var(--red)':'var(--amb)';}
  else if(d.timerRunning){ia.textContent='⏱';at.textContent='TIMER RUNNING';at.style.color='var(--amb)';}
  else{ia.textContent='🔕';at.textContent='SILENT';at.style.color='var(--sub)';}
  // Countdown
  if(d.timerRunning&&d.timerRemS>0){
    $('cdWrap').style.display='block';
    const m=Math.floor(d.timerRemS/60),s=d.timerRemS%60;
    $('cd').textContent=String(m).padStart(2,'0')+':'+String(s).padStart(2,'0');
  }else $('cdWrap').style.display='none';
}
setInterval(pollStatus,1500);

// ── SLEEPWATCH ────────────────────────────────────────────────────
const DC=['d0','d1','d2'],DL=['EASY','MEDIUM','HARD'];
let swR=0,swQ=0,swLim=0,swT0=0,swTmr=null,swRes=[[],[],[]],swCorr=[];
const swShow=id=>{document.querySelectorAll('.sw-screen').forEach(s=>s.classList.remove('on'));$(id).classList.add('on');}
const swProg=p=>$('swProg').style.width=p+'%';
const swClr=()=>{clearInterval(swTmr);swTmr=null;}

async function swStart(){
  swR=0;swQ=0;swRes=[[],[],[]];swCorr=[];swProg(0);
  await fetch('/api/sw/start',{method:'POST'});
  await swLoad();swShow('sw1');
}
function swRst(){swShow('sw0');swProg(0);}

async function swLoad(){
  swClr();$('swAns').value='';$('swAns').className='';
  $('swFb').style.display='none';$('swSbtn').disabled=false;
  swProg(((swR*3+swQ)/9)*100);
  const d=await fetch('/api/sw/q').then(r=>r.json());
  $('swRtag').innerHTML=`ROUND ${swR+1} OF 3 <span class="diff ${DC[swR]}">${DL[swR]}</span> &nbsp;Q${swQ+1}/3`;
  $('swQtxt').textContent=d.q;swLim=d.ms;swTick();
  setTimeout(()=>$('swAns').focus(),60);
}
function swTick(){
  swClr();swT0=Date.now();
  const fill=$('swTfill'),val=$('swTval');
  swTmr=setInterval(()=>{
    const rem=Math.max(0,swLim-(Date.now()-swT0)),p=rem/swLim*100;
    fill.style.width=p+'%';
    const c=p>50?'var(--grn)':p>22?'var(--amb)':'var(--red)';
    fill.style.background=c;val.style.color=c;val.textContent=(rem/1000).toFixed(1)+'s';
    if(!rem){swClr();swSub(true);}
  },80);
}
async function swSub(tout=false){
  swClr();const el=Date.now()-swT0;
  const a=tout?'':$('swAns').value.trim();$('swSbtn').disabled=true;
  const d=await fetch('/api/sw/ans',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({a,ms:el})}).then(r=>r.json());
  const fb=$('swFb');fb.style.display='block';
  if(d.ok){fb.className='fb f-ok';fb.textContent='✓ CORRECT — '+d.ms+'ms';$('swAns').classList.add('ok');}
  else{fb.className='fb f-bad';fb.textContent=(tout?'✗ TIMED OUT':'✗ WRONG')+' — ans: '+d.ans;$('swAns').classList.add('bad');}
  swRes[swR].push({ok:d.ok,ms:d.ms,q:d.q});
  await wait(1350);swQ++;
  if(swQ>=3){swCorr.push(swRes[swR].filter(x=>x.ok).length);swShowRS();}
  else await swLoad();
}
function swShowRS(){
  swShow('sw2');const corr=swCorr[swR];
  $('swRsLbl').textContent=`ROUND ${swR+1} COMPLETE`;
  const el=$('swRsNum');el.textContent=corr+'/3';
  el.style.color=corr>=2?'var(--grn)':corr>=1?'var(--amb)':'var(--red)';
  const ul=$('swRsList');ul.innerHTML='';
  swRes[swR].forEach(x=>{
    const li=document.createElement('li');
    li.innerHTML=`<span class="q-txt">${tr(x.q,34)}</span><span class="badge ${x.ok?'b-grn':'b-red'}">${x.ok?'OK':'FAIL'}</span><span class="q-ms">${(x.ms/1000).toFixed(1)}s</span>`;
    ul.appendChild(li);
  });
  swProg((swR+1)/3*100);
  const nb=$('swRsNext');
  if(swR+1>=3){nb.textContent='VIEW RESULTS →';nb.onclick=swFinal;}
  else{nb.textContent=`START ROUND ${swR+2} →`;nb.onclick=()=>{swR++;swQ=0;swShow('sw1');swLoad();};}
}
async function swFinal(){
  swShow('sw3');swProg(100);
  const d=await fetch('/api/sw/result').then(r=>r.json());
  const s=Math.round(d.score);
  $('swFnum').textContent=s;$('swFnum').style.color=swC(d.score);
  $('swFv').textContent=d.verdict;$('swFv').style.color=swC(d.score);
  $('swFbd').innerHTML=`
    <div class="row"><span class="rl">ROUND 1 (EASY)</span><span class="rv">${d.acc[0]}/3 correct</span></div>
    <div class="row"><span class="rl">ROUND 2 (MEDIUM)</span><span class="rv">${d.acc[1]}/3 correct</span></div>
    <div class="row"><span class="rl">ROUND 3 (HARD)</span><span class="rv">${d.acc[2]}/3 correct</span></div>
    <div class="row"><span class="rl">AVG RESPONSE</span><span class="rv">${d.avgMs}ms</span></div>
    <div class="row total-row"><span class="rl" style="color:var(--txt)">ML SLEEP SCORE</span>
      <span style="font-size:14px;color:${swC(d.score)}">${d.score.toFixed(1)}</span></div>`;
  const nxt=$('swNext');nxt.style.display='block';
  nxt.textContent=d.score>=50
    ?'TEST PASSED — LEAVE THE ROOM WITHIN 60 SECONDS'
    :'SCORE TOO LOW (tokens deducted) — STILL LEAVE ROOM WITHIN 60s';
  nxt.style.borderColor=d.score>=50?'var(--grn)':'var(--red)';
  nxt.style.color=d.score>=50?'var(--grn)':'var(--red)';
}

// ── ALARM TAB ─────────────────────────────────────────────────────
async function startTimer(){
  const m=parseInt($('cfgMins').value)||0,s=parseInt($('cfgSecs').value)||0;
  if(m+s===0){alert('Set a time first.');return;}
  await fetch('/api/alarm/timer',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({
      seconds:m*60+s,
      wakeWindow:(parseInt($('cfgWW').value)||7)*60,
      recheckMax:parseInt($('cfgRC').value)||60,
      stake:parseInt($('cfgStake').value)||10
    })});
  pollStatus();
}
async function ringNow(){await fetch('/api/alarm/trigger',{method:'POST'});pollStatus();}
async function iAmAwake(){await fetch('/api/awake',{method:'POST'});go(0);pollStatus();}
async function resetSession(){await fetch('/api/reset',{method:'POST'});pollStatus();}
async function resetTok(){await fetch('/api/tokens/reset',{method:'POST'});pollStatus();}

// ── RADAR ─────────────────────────────────────────────────────────
async function pollRadar(){
  const d=await fetch('/api/radar').then(r=>r.json()).catch(()=>null);if(!d)return;
  $('rD').textContent=d.dist.toFixed(2);$('rS').textContent=d.speed.toFixed(3);
  $('rE').textContent=Math.round(d.energy);$('rT').textContent=d.targets;
  $('rStr').textContent=d.emptyStreak;
  $('rOcc').textContent=d.occupied?'OCCUPIED':'EMPTY';
  $('rOcc').className='badge '+(d.occupied?'b-grn':'b-red');
}
setInterval(()=>{if($('p2').classList.contains('on'))pollRadar();},500);

// ── WATCHDOG ──────────────────────────────────────────────────────
async function armWD(){await fetch('/api/wd/arm',{method:'POST'});pollWD();}
async function disarmWD(){await fetch('/api/wd/disarm',{method:'POST'});pollWD();}
async function pollWD(){
  const d=await fetch('/api/wd/status').then(r=>r.json()).catch(()=>null);if(!d)return;
  $('wdIcon').textContent=d.tamper?'🚨':d.inGrace?'⚠️':d.armed?'🛡':'🔓';
  const st=$('wdSt');
  if(d.tamper){st.textContent='TAMPER — TOKENS DEDUCTED';st.style.color='var(--red)';}
  else if(d.inGrace){st.textContent='TAMPER! RESTORE IN '+d.graceRemS+'s';st.style.color='var(--amb)';}
  else if(d.armed){st.textContent='ARMED — SECURE';st.style.color='var(--grn)';}
  else{st.textContent='DISARMED';st.style.color='var(--sub)';}
  $('wdGrace').textContent=d.inGrace?d.graceRemS+'s':'—';
  $('wdR').textContent=d.reason||'—';
  const b=$('wdBadge');
  if(d.tamper||d.inGrace){b.className='badge b-red';b.textContent='TRIGGERED';}
  else{b.className='badge b-grn';b.textContent='OK';}
}
setInterval(()=>{if($('p3').classList.contains('on'))pollWD();},1000);
</script>
</body></html>
)HTML";

// ════════════════════════════════════════════════════════════════════
//  API HANDLERS
// ════════════════════════════════════════════════════════════════════

// Helper: extract int from JSON body
int jsonInt(const String& body, const char* key, int def=0) {
  String k = "\"" + String(key) + "\":";
  int i = body.indexOf(k);
  return i<0 ? def : body.substring(i+k.length()).toInt();
}

// /api/status
void apiStatus(AsyncWebServerRequest* req) {
  uint32_t remS = (timerRunning && timerEndMs>millis()) ? (timerEndMs-millis())/1000 : 0;
  req->send(200,"application/json",
    "{\"state\":" + String((int)nodState) +
    ",\"tokBal\":"   + String(tok.balance) +
    ",\"tokStake\":"  + String(tok.staked) +
    ",\"tokLost\":"   + String(tok.totalLost) +
    ",\"tokReason\":\"" + tok.lastReason + "\"" +
    ",\"alarmActive\":" + String(alarmActive?"true":"false") +
    ",\"alarmTamper\":" + String(alarmTamper?"true":"false") +
    ",\"timerRunning\":" + String(timerRunning?"true":"false") +
    ",\"timerRemS\":" + String(remS) + "}");
}

// /api/awake — user confirmed awake
void apiAwake(AsyncWebServerRequest* req) {
  silenceAlarm();
  if (nodState==NS_ALARM || nodState==NS_TIMER) {
    wakeWindowEndMs = millis() + wakeWindowS*1000UL;
    armWatchdog();
    setNodState(NS_WAKE_WINDOW);
  }
  req->send(200,"application/json","{\"ok\":true}");
}

// /api/alarm/timer — configure & start alarm
void apiAlarmTimer(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
  String body((char*)data, len);
  uint32_t secs        = jsonInt(body,"seconds",0);
  wakeWindowS          = jsonInt(body,"wakeWindow",WAKE_WINDOW_DEF_S);
  recheckMaxS          = jsonInt(body,"recheckMax",60);
  tok.deductOnFail     = jsonInt(body,"stake",10);
  tok.staked           = tok.deductOnFail * 5;
  timerEndMs           = millis() + secs*1000UL;
  timerRunning         = true;
  setNodState(NS_TIMER);
  req->send(200,"application/json","{\"ok\":true}");
}

// /api/alarm/trigger — ring immediately
void apiAlarmTrigger(AsyncWebServerRequest* req) {
  triggerAlarm(false);
  setNodState(NS_ALARM);
  req->send(200,"application/json","{\"ok\":true}");
}

// /api/reset — full session reset
void apiReset(AsyncWebServerRequest* req) {
  silenceAlarm();
  timerRunning = false;
  testDone = testPassed = false;
  sessionScore = 0;
  roomExited = false;
  emptyStreak = 0;
  roomOccupied = false;
  recheckPending = recheckActive = false;
  watchdogArmed = false;
  tamperDetected = tamperInGrace = false;
  tok.staked = 0; tok.totalLost = 0; tok.lastReason = "";
  memset(&sw,0,sizeof(sw));
  setNodState(NS_IDLE);
  req->send(200,"application/json","{\"ok\":true}");
}

// /api/tokens/reset
void apiTokReset(AsyncWebServerRequest* req) {
  tok.balance=100; tok.totalLost=0; tok.lastReason="";
  req->send(200,"application/json","{\"ok\":true}");
}

// SleepWatch
void apiSwStart(AsyncWebServerRequest* req) { swStart(); req->send(200,"application/json","{\"ok\":true}"); }
void apiSwQ    (AsyncWebServerRequest* req) {
  const SWQ& q=swCurQ();
  req->send(200,"application/json","{\"q\":\""+String(q.text)+"\",\"ms\":"+String(q.budget)+"}");
}
void apiSwAns(AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
  String body((char*)data,len);
  String input="";
  int ai=body.indexOf("\"a\":\"");
  if(ai>=0){int s=ai+5,e=body.indexOf("\"",s);if(e>s)input=body.substring(s,e);}
  bool ok=swCheckAns(input);
  String qt=String(swCurQ().text), qa=String(swCurQ().answer);
  uint32_t ms=millis()-sw.qStart;
  swAnswer(input);
  req->send(200,"application/json",
    "{\"ok\":"+String(ok?"true":"false")+",\"ms\":"+String(ms)+",\"ans\":\""+qa+"\",\"q\":\""+qt+"\"}");
}
void apiSwResult(AsyncWebServerRequest* req) {
  uint8_t acc[3]={}; double allMs[9]; uint8_t ti=0; double sumMs=0;
  for(uint8_t r=0;r<SW_RDS;r++)
    for(uint8_t q=0;q<SW_QPR;q++){
      if(sw.ans[r][q].ok) acc[r]++;
      allMs[ti]=sw.ans[r][q].ms; sumMs+=allMs[ti++];
    }
  double mn=sumMs/9,v=0;
  for(uint8_t i=0;i<9;i++) v+=(allMs[i]-mn)*(allMs[i]-mn);
  req->send(200,"application/json",
    "{\"score\":"+String(sw.score,1)+
    ",\"verdict\":\""+String(swVerdict(sw.score))+"\""+
    ",\"acc\":["+String(acc[0])+","+String(acc[1])+","+String(acc[2])+"]"+
    ",\"avgMs\":"+String((uint32_t)(sumMs/9))+
    ",\"variance\":"+String((uint32_t)sqrt(v/9.0))+"}");
}

// Radar
void apiRadar(AsyncWebServerRequest* req) {
  req->send(200,"application/json",
    "{\"targets\":" + String(radarTargets) +
    ",\"dist\":" + String(radarDist,2) +
    ",\"speed\":" + String(radarSpeed,3) +
    ",\"energy\":" + String(radarEnergy,0) +
    ",\"occupied\":" + String(roomOccupied?"true":"false") +
    ",\"emptyStreak\":" + String(emptyStreak) + "}");
}

// Watchdog
void apiWdArm(AsyncWebServerRequest* req)    { armWatchdog(); req->send(200,"application/json","{\"ok\":true}"); }
void apiWdDisarm(AsyncWebServerRequest* req) {
  watchdogArmed=false; tamperDetected=tamperInGrace=false; tamperReason="";
  req->send(200,"application/json","{\"ok\":true}");
}
void apiWdStatus(AsyncWebServerRequest* req) {
  uint32_t graceRem=0;
  if(tamperInGrace && tamperGraceMs>0){
    uint32_t el=millis()-tamperGraceMs;
    graceRem=(el<TAMPER_GRACE_MS)?(TAMPER_GRACE_MS-el)/1000:0;
  }
  req->send(200,"application/json",
    "{\"armed\":"+String(watchdogArmed?"true":"false")+
    ",\"tamper\":"+String(tamperDetected?"true":"false")+
    ",\"inGrace\":"+String(tamperInGrace?"true":"false")+
    ",\"graceRemS\":"+String(graceRem)+
    ",\"reason\":\""+tamperReason+"\"}");
}

// ════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  randomSeed(esp_random());

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pixels.begin();
  pixels.setBrightness(80);
  setLed(0,0,255); // boot = blue

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100kHz — reliable on breadboard

  // OLED
  if (oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    oledReady = true;
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setCursor(10,20); oled.print("NODERO");
    oled.setTextSize(1);
    oled.setCursor(10,44); oled.print("initialising...");
    oled.display();
    Serial.println("[OLED] OK at 0x" + String(OLED_ADDR,HEX));
  } else {
    Serial.println("[OLED] Not found at 0x" + String(OLED_ADDR,HEX));
  }

  // GY-87
  initWatchdog();

  // Radar
  Serial.print("[Radar] Init ");
  bool radarOk = false;
  for (int i=1; i<=5; i++) {
    if (radar.begin()) {
      radar.setSensorMode(eSpeedMode);
      radar.setDetectionRange(0, 300, 30);
      radar.setFrettingDetection(eON);
      radar.setTrigSensitivity(8);
      radar.setKeepSensitivity(7);
      radarReady = true;
      radarOk    = true;
      Serial.printf("OK (attempt %d)\n", i);
      break;
    }
    Serial.print(".");
    delay(300);
  }
  if (!radarOk) Serial.println("\n[Radar] NOT FOUND — radar features disabled");

  // WiFi AP
  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(200);
  Serial.println("[WiFi] AP: nodero → http://" + WiFi.softAPIP().toString());

  // Routes
  server.on("/",                HTTP_GET,  [](AsyncWebServerRequest* r){ r->send_P(200,"text/html",PAGE); });
  server.on("/api/status",      HTTP_GET,  apiStatus);
  server.on("/api/awake",       HTTP_POST, apiAwake);
  server.on("/api/reset",       HTTP_POST, apiReset);
  server.on("/api/tokens/reset",HTTP_POST, apiTokReset);
  server.on("/api/alarm/trigger",HTTP_POST,apiAlarmTrigger);
  server.on("/api/alarm/timer", HTTP_POST, [](AsyncWebServerRequest*){}, NULL, apiAlarmTimer);
  server.on("/api/sw/start",    HTTP_POST, apiSwStart);
  server.on("/api/sw/q",        HTTP_GET,  apiSwQ);
  server.on("/api/sw/ans",      HTTP_POST, [](AsyncWebServerRequest*){}, NULL, apiSwAns);
  server.on("/api/sw/result",   HTTP_GET,  apiSwResult);
  server.on("/api/radar",       HTTP_GET,  apiRadar);
  server.on("/api/wd/arm",      HTTP_POST, apiWdArm);
  server.on("/api/wd/disarm",   HTTP_POST, apiWdDisarm);
  server.on("/api/wd/status",   HTTP_GET,  apiWdStatus);

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin","*");
  server.begin();

  setLed(0,30,0);
  setNodState(NS_IDLE);
  Serial.println("[Nodero] Ready.");
}

// ════════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════════
void loop() {
  pollWatchdog();
  pollRadar();
  pollAlarm();
  pollNodero();
  updateLed();
  oledUpdate();
}