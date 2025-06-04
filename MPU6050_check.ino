#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Variables for step counting 
int stepCount = 0;
float threshold = 1.5; // Adjust this threshold based on your testing
float previousAccelerationZ = 0;
bool stepDetected = false;

void setup() {
    Serial.begin(115200);  // Initialize serial communication
    while (!Serial) delay(10); // Wait for Serial Monitor to open

    Serial.println("Adafruit MPU6050 Step Counter Test!");

    // Initialize the MPU6050
    Wire.begin(21, 22);  // Set SDA to GPIO 21 and SCL to GPIO 22 for ESP32
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    // Configure settings
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("");
    delay(1000);
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Print accelerometer values
    Serial.print("Acceleration Z: ");
    Serial.println(a.acceleration.z);

    // Step detection logic
    if (a.acceleration.z > threshold && !stepDetected) {
        stepDetected = true; // A step has been detected
        stepCount++; // Increment step count
        Serial.print("Step Count: ");
        Serial.println(stepCount);
    } else if (a.acceleration.z < threshold) {
        stepDetected = false; // Reset step detection
    }

    delay(1000); // Shorter delay for responsiveness (you can adjust this for stability)
}