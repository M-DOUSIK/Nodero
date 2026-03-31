#include <Wire.h>
#include "DFRobot_C4001.h"
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <WebServer.h>

#define WIFI_SSID   "Nodero-Debug"
#define WIFI_PASS   "nodero123"
#define SDA_PIN     8
#define SCL_PIN     9
#define RGB_PIN     38

#define MIN_ENERGY  5000  // present = above this, gone = below this
#define EXIT_POLLS  50    // 50 × 200ms = 10 seconds to confirm exit

#define I2C_COMMUNICATION
DFRobot_C4001_I2C radar(&Wire, DEVICE_ADDR_0);

Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ800);
WebServer server(80);

int      currentTargets = 0;
float    currentSpeed   = 0.0;
float    currentDist    = 0.0;
float    currentEnergy  = 0.0;
uint32_t lastPoll       = 0;

bool     roomOccupied   = false;
int      emptyStreak    = 0;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <title>Nodero Telemetry</title>
  <style>
    body { 
      background: #080c16; color: #fff; font-family: monospace; 
      display: flex; flex-direction: column; align-items: center; justify-content: center; 
      height: 100vh; margin: 0; 
    }
    .card { 
      background: #111827; padding: 30px; border-radius: 16px; text-align: center; 
      box-shadow: 0 10px 30px rgba(0,0,0,0.8); border: 1px solid #1f2937; 
      width: 90%; max-width: 400px;
    }
    h1 { margin: 0 0 20px 0; font-size: 16px; color: #38bdf8; text-transform: uppercase; letter-spacing: 2px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; margin-bottom: 20px; }
    .box { background: #030712; border: 1px solid #1f2937; border-radius: 10px; padding: 15px 5px; }
    .val { font-size: 24px; font-weight: bold; color: #10b981; }
    .lbl { font-size: 10px; color: #6b7280; margin-top: 5px; text-transform: uppercase; }
    .footer { font-size: 14px; color: #9ca3af; background: #030712; padding: 10px; border-radius: 10px; border: 1px solid #1f2937; }
    .active { color: #10b981; font-weight: bold; }
  </style>
</head>
<body>
  <div class="card">
    <h1>Radar Telemetry</h1>
    <div class="grid">
      <div class="box"><div class="val" id="dist">0.00</div><div class="lbl">Dist (m)</div></div>
      <div class="box"><div class="val" id="speed">0.00</div><div class="lbl">Speed (m/s)</div></div>
      <div class="box"><div class="val" id="energy">0</div><div class="lbl">Energy</div></div>
    </div>
    <div class="footer" id="targets">Targets Detected: 0</div>
  </div>
  <script>
    setInterval(() => {
      fetch('/status').then(r => r.json()).then(data => {
        document.getElementById('dist').innerText   = data.dist.toFixed(2);
        document.getElementById('speed').innerText  = data.speed.toFixed(3);
        document.getElementById('energy').innerText = data.energy.toFixed(0);
        const tgt = document.getElementById('targets');
        tgt.innerText = "Targets Detected: " + data.targets;
        tgt.className = data.targets > 0 ? "footer active" : "footer";
      }).catch(err => console.log("Connection lost"));
    }, 200);
  </script>
</body>
</html>
)rawliteral";

void handleRoot()   { server.send(200, "text/html", index_html); }
void handleStatus() {
  String json = "{\"targets\":"  + String(currentTargets)    +
                ",\"dist\":"     + String(currentDist,    2) +
                ",\"speed\":"    + String(currentSpeed,   3) +
                ",\"energy\":"   + String(currentEnergy,  0) + "}";
  server.send(200, "application/json", json);
}

void setup()
{
  Serial.begin(115200);

  pixels.begin();
  pixels.setBrightness(80);
  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.show();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  server.on("/",       handleRoot);
  server.on("/status", handleStatus);
  server.begin();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(2000);

  Serial.println("\n--- Nodero: Factory Speed Mode ---");

  while(!radar.begin()){
    Serial.println("NO Devices !");
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); pixels.show();
    delay(1000);
  }
  Serial.println("Device connected!");

  radar.setSensorMode(eSpeedMode);

  sSensorStatus_t data = radar.getStatus();
  Serial.print("work status  = "); Serial.println(data.workStatus);
  Serial.print("work mode    = "); Serial.println(data.workMode);
  Serial.print("init status  = "); Serial.println(data.initStatus);

  if(radar.setDetectThres(/*min*/30, /*max*/1200, /*thres*/10))
    Serial.println("set detect threshold successfully");

  radar.setFrettingDetection(eON);

  Serial.print("min range          = "); Serial.println(radar.getTMinRange());
  Serial.print("max range          = "); Serial.println(radar.getTMaxRange());
  Serial.print("threshold range    = "); Serial.println(radar.getThresRange());
  Serial.print("fretting detection = "); Serial.println(radar.getFrettingDetection());

  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.show();
}

void loop()
{
  server.handleClient();

  if(millis() - lastPoll >= 200) {
    lastPoll = millis();

    int   rawTargets = radar.getTargetNumber();
    float rawSpeed   = 0, rawDist = 0, rawEnergy = 0;

    if(rawTargets > 0) {
      rawSpeed  = radar.getTargetSpeed();
      rawDist   = radar.getTargetRange();
      rawEnergy = radar.getTargetEnergy();
    }

    bool validTarget = (rawTargets > 0) && (rawEnergy >= MIN_ENERGY);

    if(validTarget) {
      // Energy high — present, instant confirm, reset counter
      roomOccupied   = true;
      emptyStreak    = 0;
      currentTargets = rawTargets;
      currentSpeed   = rawSpeed;
      currentDist    = rawDist;
      currentEnergy  = rawEnergy;
      pixels.setPixelColor(0, pixels.Color(0, 255, 255)); // Cyan — tracking

      Serial.print("target number = "); Serial.println(currentTargets);
      Serial.print("target speed  = "); Serial.print(currentSpeed);  Serial.println(" m/s");
      Serial.print("target range  = "); Serial.print(currentDist);   Serial.println(" m");
      Serial.print("target energy = "); Serial.println(currentEnergy);
      Serial.println();

    } else if(roomOccupied) {
      // Energy dropped — start exit countdown
      emptyStreak++;
      Serial.print("exit countdown: "); Serial.print(emptyStreak);
      Serial.print(" / "); Serial.println(EXIT_POLLS);

      if(emptyStreak >= EXIT_POLLS) {
        roomOccupied   = false;
        emptyStreak    = 0;
        currentTargets = 0;
        currentSpeed   = 0;
        currentDist    = 0;
        currentEnergy  = 0;
        pixels.setPixelColor(0, pixels.Color(20, 0, 0));
        Serial.println(">> Room EMPTY");
      }
      // Still counting — last known telemetry stays on web UI

    } else {
      // Already empty
      currentTargets = 0;
      currentSpeed   = 0;
      currentDist    = 0;
      currentEnergy  = 0;
      pixels.setPixelColor(0, pixels.Color(20, 0, 0));
    }

    pixels.show();
  }
}