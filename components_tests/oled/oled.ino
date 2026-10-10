/*
 * ═══════════════════════════════════════════════════════════════════
 * STANDALONE OLED TEST — Dousik Cat Edition
 * * WIRING for ESP32-S3:
 * SDA -> GPIO 8
 * SCL -> GPIO 9
 * VCC -> 3.3V or 5V
 * GND -> GND
 * ═══════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SDA_PIN    8
#define SCL_PIN    9
#define OLED_W     128
#define OLED_H     64
#define OLED_ADDR  0x3C  // Default I2C address for most 0.96" OLEDs

// Initialize the OLED display object
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, -1);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[OLED Test] Booting...");

  // Start the I2C bus on the custom pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100kHz for stability

  // Attempt to initialize the OLED
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[ERROR] SSD1306 allocation failed. Check wiring!");
    while (true); // Freeze here if the screen isn't found
  }

  Serial.println("[OLED Test] Screen found and initialized!");

  // Boot Splash Screen
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(15, 25);
  oled.print("OLED OK!");
  oled.display();
  delay(2000);
}

void loop() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  // ── Top bar ───────────────────────────────────────────────
  oled.setTextSize(1);
  oled.setCursor(0, 0);   
  oled.print("DOUSIK =^..^="); 
  
  // Blinking "Alive" indicator in the top right
  if ((millis() / 500) % 2 == 0) {
    oled.fillCircle(120, 4, 2, SSD1306_WHITE);
  }
  
  oled.drawLine(0, 9, 127, 9, SSD1306_WHITE);

  // ── ASCII Cat ─────────────────────────────────────────────
  oled.setCursor(25, 14); oled.print("Hello dousik!");
  oled.setCursor(43, 28); oled.print(" /\\_/\\ ");
  oled.setCursor(43, 38); oled.print("( o.o )");
  oled.setCursor(43, 48); oled.print(" > ^ < ");

  // Push the buffer to the screen
  oled.display();

  // Short delay to prevent flickering
  delay(50);
}