void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(A2); // Read the analog value from pin A0
  float sensorVoltage = sensorValue * (5.0 / 1023.0); // Convert to voltage

  // Convert sensor voltage to UV index
  float uvIndex = map(sensorVoltage, 0.99, 2.8, 0.0, 15.0);

  // Ensure UV index is within a realistic range
  if (uvIndex < 0) uvIndex = 0;
  if (uvIndex > 15) uvIndex = 15;

  // Display the results
  Serial.print("Sensor Voltage: ");
  Serial.print(sensorVoltage);
  Serial.print(" V, UV Index: ");
  Serial.println(uvIndex);

  // Provide protection recommendations based on UV index
  if (uvIndex < 3) {
    Serial.println("UV Level: Low");
    Serial.println("Protection: Minimal protection required.");
  } else if (uvIndex < 6) {
    Serial.println("UV Level: Moderate");
    Serial.println("Protection: Wear sunglasses and use SPF 30+ sunscreen.");
  } else if (uvIndex < 8) {
    Serial.println("UV Level: High");
    Serial.println("Protection: Wear protective clothing, a wide-brimmed hat, and SPF 30+ sunscreen.");
  } else if (uvIndex < 11) {
    Serial.println("UV Level: Very High");
    Serial.println("Protection: Apply SPF 30+ sunscreen every 2 hours, seek shade, and avoid sun exposure during midday hours.");
  } else {
    Serial.println("UV Level: Extreme");
    Serial.println("Protection: Avoid sun exposure, wear protective clothing, and apply SPF 50+ sunscreen.");
  }

  Serial.println("----------------------------");
  delay(1000); // Wait for 1 second before the next reading
}