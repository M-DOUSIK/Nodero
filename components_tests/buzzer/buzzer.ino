#define BUZZER_PIN 19

void setup() {
  // Set the buzzer pin as an output
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  // Play a 1000 Hz tone
  tone(BUZZER_PIN, 1000);
  delay(500);
  
  // Stop the tone
  noTone(BUZZER_PIN);
  delay(500);
}