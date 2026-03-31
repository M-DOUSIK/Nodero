# NODERO — Zero-Trust Sleep Alarm System

Nodero is a hardware-software system that forces you to prove you are cognitively awake before your morning is allowed to continue. It combines a 60 GHz mmWave radar, a 6-axis IMU with compass, and an on-device ML model to detect presence, measure alertness, and penalise you in tokens if you try to cheat.

---

## Table of Contents

- [How It Works](#how-it-works)
- [Hardware](#hardware)
- [Repository Structure](#repository-structure)
- [Wiring](#wiring)
- [Software Architecture](#software-architecture)
- [State Machine](#state-machine)
- [Token System](#token-system)
- [ML Model — SleepScorer](#ml-model--sleepscorer)
- [Web Interface](#web-interface)
- [OLED Display](#oled-display)
- [Setup & Flashing](#setup--flashing)
- [Library Dependencies](#library-dependencies)
- [Configuration Reference](#configuration-reference)
- [Component Tests](#component-tests)
- [Roadmap](#roadmap)

---

## How It Works

1. **Set your alarm** in the web UI — configure timer duration, wake window, token stake, and recheck delay.
2. **Alarm fires** — buzzer rings, OLED displays "WAKE UP", NeoPixel goes red.
3. **Tap "I'm Awake"** on the web app within your wake window. This starts the SleepWatch alertness test and arms the GY-87 tamper watchdog.
4. **Complete the test** — 3 rounds of 9 adaptive questions (Easy → Medium → Hard) on your phone. An on-device ML model scores your cognitive alertness from your accuracy and response time patterns.
5. **Leave the room** — the mmWave radar must detect an empty room within 60 seconds of completing the test. Staying in bed costs tokens.
6. **Random recheck** — at a random time after you leave, the buzzer fires again. Return to the radar station within 90 seconds or lose more tokens.
7. **GY-87 tamper protection** — if the device is moved or rotated at any point, you have 30 seconds to restore it to its original position before tokens are deducted.

---

## Hardware

| Component | Role | Interface |
|---|---|---|
| ESP32-S3 Dev Module | Main MCU | — |
| DFRobot C4001 | 60 GHz mmWave radar — presence & speed | I2C `0x2A` |
| GY-87 (MPU6050) | Tilt / accelerometer — tamper detection | I2C `0x68` |
| GY-87 (QMC5883P) | Compass — rotation tamper detection | I2C `0x2C` (via MPU bypass) |
| SSD1306 OLED 128×64 | Secondary display — status & countdowns | I2C `0x3C` |
| WS2812B NeoPixel | RGB status LED | GPIO 38 |
| Passive Buzzer | Alarm tones | GPIO 5 |

All I2C devices share a single bus on **GPIO 8 (SDA)** and **GPIO 9 (SCL)**.

---

## Repository Structure

```
nodero/
│
├── main/
│   ├── main.ino                   # Unified firmware — all modules integrated
│   ├── sleep_scorer.h             # ML model inference (generated from Colab)
│   └── SleepScorer_Train.ipynb    # Colab notebook — train & export the model
│
├── components_tests/              # Individual hardware validation sketches
│   ├── buzzer/
│   │   └── buzzer.ino             # Tone patterns, frequency sweep
│   ├── c4001/
│   │   └── c4001.ino              # Raw radar distance meter + WiFi web monitor
│   ├── gy/
│   │   └── gy.ino                 # MPU6050 tilt + QMC5883P compass baseline test
│   ├── oled/
│   │   └── oled.ino               # SSD1306 text & graphics test
│   └── sat/
│       ├── SleepAlertnessTester.ino   # Standalone SleepWatch test harness
│       ├── SleepScorer_Train.ipynb    # Model training notebook
│       └── sleep_scorer.h             # Exported model header
│
├── LICENSE
└── README.md
```

---

## Wiring

```
ESP32-S3
│
├── GPIO 8  (SDA) ──┬── MPU6050 SDA
│                   ├── QMC5883P SDA  (via MPU bypass gate)
│                   ├── C4001 SDA
│                   └── SSD1306 SDA
│
├── GPIO 9  (SCL) ──┬── MPU6050 SCL
│                   ├── QMC5883P SCL
│                   ├── C4001 SCL
│                   └── SSD1306 SCL
│
├── GPIO 38 ──────── WS2812B NeoPixel DIN
├── GPIO 5  ──────── Buzzer (+)
│
├── 3.3V ──┬── 4.7kΩ ──► SDA
│          └── 4.7kΩ ──► SCL
│
└── GND ─────────── All component GNDs
```

> Set I2C clock to **100 kHz** (`Wire.setClock(100000)`) on breadboard builds. 400 kHz is only reliable with short, direct PCB traces.

---

## Software Architecture

The firmware is structured as a **cooperative task loop** — no RTOS, no blocking delays in the main loop. Each subsystem has a `poll` function that checks `millis()` internally and returns immediately if it is not yet time to act.

```
loop()
  ├── pollWatchdog()    every 500 ms
  ├── pollRadar()       every 200 ms
  ├── pollAlarm()       every ~100–300 ms  (buzzer tone control)
  ├── pollNodero()      every iteration    (state machine transitions)
  ├── updateLed()       every 40 ms
  └── oledUpdate()      every 150 ms
```

The web server runs on **ESPAsyncWebServer** which handles HTTP requests asynchronously on a background task, so it never blocks the sensor loop.

---

## State Machine

```
  power on
     │
     ▼
  NS_IDLE ──► (set timer in app) ──► NS_TIMER ──► timer expires ──► NS_ALARM
                                                                         │
                                                             tap "I'm Awake"
                                                                         │
                                                                         ▼
                                                               NS_WAKE_WINDOW
                                                                         │
                                                            start test on app
                                                                         │
                                                                         ▼
                                                                     NS_TEST
                                                                         │
                                                             test submitted
                                                                         │
                                                                         ▼
                                                               NS_AWAIT_EXIT
                                                                         │
                                                    radar detects empty room
                                                                         │
                                                                         ▼
                                                                  NS_ROAMING
                                                                         │
                                                        random recheck fires
                                                                         │
                                                                         ▼
                                                                  NS_RECHECK
                                                                         │
                                                       user detected by radar
                                                                         │
                                                                         ▼
                                                                 NS_COMPLETE
                                                                         │
                                                               reset session
                                                                         │
                                                                         ▼
                                                                    NS_IDLE
```

---

## Token System

Tokens represent a real or symbolic stake. You configure `tokens per failure` in the alarm setup screen before each session.

| Event | Deduction |
|---|---|
| SleepWatch score below 50 | `deductOnFail` |
| Wake window expires without completing test | `deductOnFail × 2` |
| Wake window expires mid-test | `deductOnFail × 2` |
| Did not leave room within 60 seconds | `deductOnFail` |
| GY-87 moved and not restored within 30 seconds | `deductOnFail` |
| Missed random recheck (90 s timeout) | `deductOnFail` |

Token balance, stake, and loss history are displayed live in the header bar of the web UI and on the OLED at all times.

---

## ML Model — SleepScorer

The alertness scoring model lives in `sleep_scorer.h` and is called once at the end of each SleepWatch session.

### Input features (9-element double array)

| Index | Feature | Description |
|---|---|---|
| 0 | `acc_r1` | Accuracy in round 1 (0.0 – 1.0) |
| 1 | `acc_r2` | Accuracy in round 2 |
| 2 | `acc_r3` | Accuracy in round 3 |
| 3 | `avg_ms_r1` | Average response time, round 1 (ms) |
| 4 | `avg_ms_r2` | Average response time, round 2 |
| 5 | `avg_ms_r3` | Average response time, round 3 |
| 6 | `time_drift` | `avg_ms_r3 − avg_ms_r1` — fatigue slope across rounds |
| 7 | `acc_drift` | `acc_r3 − acc_r1` — cognitive degradation across rounds |
| 8 | `response_variance` | σ across all 9 individual response times |

### Output

A single `float` in the range 0 – 100. Higher means more alert.

### Score bands

| Score | Verdict |
|---|---|
| ≥ 85 | FULLY ALERT |
| ≥ 68 | MILDLY DROWSY |
| ≥ 50 | MODERATELY SLEEPY |
| ≥ 30 | SEVERELY IMPAIRED |
| < 30 | CRITICALLY SLEEP-DEPRIVED |

### Training your own model

1. Open `SleepScorer_Train.ipynb` in Google Colab.
2. Collect labelled session data — accuracy per question, response times in ms, and ground-truth alertness labels.
3. Train the model. The notebook exports a `score_session(double* feats)` C function.
4. Replace `sleep_scorer.h` in both `main/` and `components_tests/sat/` with the exported file.
5. The function signature **must** remain `float score_session(double* feats)`.

The stub `sleep_scorer.h` included in this repo uses a hand-tuned weighted formula so the firmware compiles and runs correctly without a trained model.

---

## Web Interface

The ESP32 creates a WiFi access point at boot. Connect your phone and open the URL shown in the Serial Monitor.

```
SSID     : nodero
Password : nodero123
URL      : http://192.168.4.1
```

### Tabs

| Tab | Purpose |
|---|---|
| **SLEEP** | SleepWatch alertness test — 3 rounds, 9 questions, ML score display |
| **ALARM** | Configure timer, wake window, token stake per failure, recheck delay. Start alarm, ring now, silence, reset |
| **RADAR** | Live C4001 telemetry — distance, speed, energy, presence state, empty streak counter |
| **WATCHDOG** | GY-87 arm / disarm, tamper status, grace period countdown |

### REST API

| Method | Endpoint | Description |
|---|---|---|
| GET | `/api/status` | Full system state JSON — used by header polling |
| POST | `/api/awake` | Confirm awake → start wake window, arm watchdog |
| POST | `/api/alarm/timer` | Configure and start alarm countdown |
| POST | `/api/alarm/trigger` | Ring buzzer immediately |
| POST | `/api/reset` | Full session and state machine reset |
| POST | `/api/tokens/reset` | Reset token balance to 100 |
| POST | `/api/sw/start` | Start a new SleepWatch session |
| GET | `/api/sw/q` | Get current question text and time budget (ms) |
| POST | `/api/sw/ans` | Submit answer `{ "a": "...", "ms": 4200 }` |
| GET | `/api/sw/result` | Final ML score, verdict, per-round breakdown |
| GET | `/api/radar` | Live radar telemetry JSON |
| POST | `/api/wd/arm` | Arm watchdog — captures baseline orientation |
| POST | `/api/wd/disarm` | Disarm watchdog |
| GET | `/api/wd/status` | Armed, tamper detected, grace remaining (s) |

---

## OLED Display

The SSD1306 128×64 OLED acts as a secondary at-a-glance display. Token balance is always shown in the top status bar. The display updates every 150 ms.

| State | OLED Content |
|---|---|
| IDLE | WiFi URL, current token balance |
| TIMER | "ALARM IN" + MM:SS countdown |
| ALARM | "WAKE UP!" + open app instruction |
| WAKE WINDOW | "TAKE TEST NOW" + window countdown |
| TEST | Question number Q4/9, round number, window time remaining |
| AWAIT EXIT | Test score, "LEAVE!" + 60 s exit countdown |
| ROAMING | Token balance, recheck pending indicator |
| RECHECK | "RETURN TO STATION" + 90 s countdown |
| COMPLETE | Final score, token balance, total tokens lost |

Questions are not displayed on the OLED — they are too long for a 128×64 screen and require phone keyboard input.

---

## Setup & Flashing

### Arduino IDE settings

| Setting | Value |
|---|---|
| Board | ESP32S3 Dev Module |
| PSRAM | OPI PSRAM |
| CPU Speed | 240 MHz |
| Partition Scheme | Huge APP (3MB No OTA) |
| Upload Speed | 921600 |

### Steps

```bash
# 1. Clone the repository
git clone https://github.com/<your-username>/nodero.git

# 2. Open main/main.ino in Arduino IDE

# 3. Install libraries via Library Manager
#    ESPAsyncWebServer   AsyncTCP   Adafruit_NeoPixel
#    Adafruit_GFX   Adafruit_SSD1306   DFRobot_C4001

# 4. Ensure sleep_scorer.h is in the same folder as main.ino

# 5. Select correct board, port, and upload
```

> **Common error:** `fatal error: sleep_scorer.h: No such file or directory`  
> Fix: make sure `sleep_scorer.h` is inside the `main/` folder alongside `main.ino`, not in a parent directory.

### Validating hardware before flashing the full firmware

Test each component individually using the sketches in `components_tests/`:

| Sketch | What it validates |
|---|---|
| `buzzer/buzzer.ino` | Buzzer wiring, tone generation |
| `c4001/c4001.ino` | Radar I2C connection, distance readings, WiFi web monitor |
| `gy/gy.ino` | MPU6050 tilt + QMC5883P compass baseline capture |
| `oled/oled.ino` | OLED I2C address, text rendering |
| `sat/SleepAlertnessTester.ino` | Full SleepWatch flow in isolation, ML score output |

---

## Library Dependencies

| Library | Minimum version | Install via |
|---|---|---|
| ESPAsyncWebServer | 1.2.3 | Library Manager |
| AsyncTCP | 1.1.1 | Library Manager |
| Adafruit NeoPixel | 1.12.0 | Library Manager |
| Adafruit GFX | 1.11.0 | Library Manager |
| Adafruit SSD1306 | 2.5.0 | Library Manager |
| DFRobot_C4001 | latest | Library Manager |

---

## Configuration Reference

All tuning constants are defined near the top of `main.ino`:

| Constant | Default | Description |
|---|---|---|
| `TILT_THRESHOLD` | `4000` | MPU6050 raw ADC delta to trigger tilt tamper |
| `YAW_THRESHOLD` | `500` | Compass raw delta to trigger rotation tamper |
| `EXIT_POLLS` | `15` | Consecutive empty radar polls to confirm room exit |
| `TAMPER_GRACE_MS` | `30000` | Milliseconds to restore device after tamper (30 s) |
| `EXIT_TIMEOUT_MS` | `60000` | Milliseconds to leave room after test completes (60 s) |
| `RECHECK_TIMEOUT_MS` | `90000` | Milliseconds to return to station for recheck (90 s) |
| `WAKE_WINDOW_DEF_S` | `420` | Default wake window in seconds (7 minutes) |
| `SCORE_PASS` | `50` | Minimum SleepWatch score to avoid token penalty |
| `OLED_ADDR` | `0x3C` | Change to `0x3D` if your OLED uses that address |
| `SDA_PIN` | `8` | I2C data line |
| `SCL_PIN` | `9` | I2C clock line |
| `RGB_PIN` | `38` | WS2812B NeoPixel data pin |
| `BUZZER_PIN` | `5` | Passive buzzer pin |
