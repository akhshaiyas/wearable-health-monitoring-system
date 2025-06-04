#include <Wire.h>
#include <MAX30100_PulseOximeter.h>

#define REPORTING_PERIOD_MS 1000  // Time interval for printing data

PulseOximeter pox;
uint32_t lastReportTime = 0;

void onBeatDetected() {
    Serial.println("💓 Heartbeat detected!");
}

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing MAX30100...");

    if (!pox.begin()) {
        Serial.println("FAILED to initialize MAX30100. Check wiring!");
        while (1);
    } else {
        Serial.println("MAX30100 Initialized Successfully!");
    }

    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
    pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop() {
    pox.update();

    if (millis() - lastReportTime > REPORTING_PERIOD_MS) {
        Serial.print("Heart Rate: ");
        Serial.print(pox.getHeartRate());
        Serial.print(" BPM | SpO2: ");
        Serial.print(pox.getSpO2());
        Serial.println(" %");

        lastReportTime = millis();
    }
}