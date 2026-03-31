#include <Wire.h>

// ESP32-S3 Pins
#define I2C_SDA 8
#define I2C_SCL 9

// Nodero Hardware Addresses
#define MPU_ADDR 0x68       // Accelerometer (Tilt)
#define COMPASS_ADDR 0x2C   // The new QMC5883P Compass (Rotation)

int16_t baseTiltX, baseTiltY;
int16_t baseYawX, baseYawY; 

void setup() {
  Serial.begin(115200);
  delay(2000); 
  
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("\n--- Nodero Zero-Trust Watchdog Calibration ---");
  
  // 1. Wake the MPU6050 Accelerometer
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0x00); 
  Wire.endTransmission();
  
  // 2. Open the I2C Bypass Gate for the Compass
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x37); 
  Wire.write(0x02); 
  Wire.endTransmission();
  delay(100);

  // 3. WAKE THE QMC5883P (HP5883) COMPASS - The New Boot Sequence
  Wire.beginTransmission(COMPASS_ADDR);
  Wire.write(0x29); Wire.write(0x06);
  Wire.endTransmission();
  
  Wire.beginTransmission(COMPASS_ADDR);
  Wire.write(0x0B); Wire.write(0x08);
  Wire.endTransmission();
  
  Wire.beginTransmission(COMPASS_ADDR);
  Wire.write(0x0A); Wire.write(0xCD); // 0xCD = Continuous Measurement Mode
  Wire.endTransmission();
  
  delay(500); // Give the sensors half a second to stabilize

  Serial.println("Calibrating Baseline... DO NOT TOUCH THE CLOCK.");
  delay(2000); // Simulate the user taking their hands off the device
  
  // --- ESTABLISH THE BASELINE ---
  
  // Read MPU6050 (Tilt)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)4, true);
  baseTiltX = Wire.read() << 8 | Wire.read();
  baseTiltY = Wire.read() << 8 | Wire.read(); 

  // Read QMC5883P (Rotation)
  Wire.beginTransmission(COMPASS_ADDR);
  Wire.write(0x01); // Start at X-axis LSB
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)COMPASS_ADDR, (uint8_t)4, true);
  
  // Safe Byte Reading (LSB first, then MSB)
  uint8_t xL = Wire.read(); uint8_t xM = Wire.read();
  uint8_t yL = Wire.read(); uint8_t yM = Wire.read();
  baseYawX = xL | (xM << 8); 
  baseYawY = yL | (yM << 8);

  Serial.println("BASELINE LOCKED. Watchdog is armed.");
}

void loop() {
  // --- POLL CURRENT TILT ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)4, true);
  int16_t curTiltX = Wire.read() << 8 | Wire.read(); 
  int16_t curTiltY = Wire.read() << 8 | Wire.read(); 

  // --- POLL CURRENT YAW (ROTATION) ---
  Wire.beginTransmission(COMPASS_ADDR);
  Wire.write(0x01); 
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)COMPASS_ADDR, (uint8_t)4, true);
  
  uint8_t cxL = Wire.read(); uint8_t cxM = Wire.read();
  uint8_t cyL = Wire.read(); uint8_t cyM = Wire.read();
  int16_t curYawX = cxL | (cxM << 8);
  int16_t curYawY = cyL | (cyM << 8);

  // --- THE ANTI-TAMPER MATH ---
  int diffTiltX = abs(curTiltX - baseTiltX);
  int diffTiltY = abs(curTiltY - baseTiltY);
  
  // Magnetic fields shift across X and Y when rotated flat
  int diffYawX = abs(curYawX - baseYawX);
  int diffYawY = abs(curYawY - baseYawY);

  if (diffTiltX > 4000 || diffTiltY > 4000) {
    Serial.println("🚨 TAMPER DETECTED: Clock was TILTED!");
  } 
  // Magnetic sensitivity set to 500 to catch a slow rotation
  else if (diffYawX > 500 || diffYawY > 500) {
    Serial.println("🚨 TAMPER DETECTED: Clock was SPUN!");
  } 
  else {
    Serial.println("✅ Secure. No movement.");
  }

  delay(500); 
}