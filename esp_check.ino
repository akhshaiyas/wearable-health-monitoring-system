#define LED_PIN 2  // Built-in LED on ESP32

void setup() {
  Serial.begin(115200);  
  pinMode(LED_PIN, OUTPUT);
  Serial.println("ESP32 is working!");
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
  Serial.println("ESP32 is working!");
}