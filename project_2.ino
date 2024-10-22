#define BLYNK_TEMPLATE_ID "TMPL3nQSE7qb8"
#define BLYNK_TEMPLATE_NAME "fall detection"
#define BLYNK_AUTH_TOKEN "BDxgkvOs_yCrV_rodFQcA1_4LGboEoYp"

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h" // For BPM calculations
#include "spo2_algorithm.h" // Include SpO2 algorithm header
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

MAX30105 particleSensor;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE]; 
byte rateSpot = 0;
long lastBeat = 0; 
float beatsPerMinute;
int beatAvg;

uint32_t irBuffer[100]; 
uint32_t redBuffer[100]; 
int bufferLength = 100; 
int32_t spo2; 
int8_t validSPO2; 
int32_t heartRate; 
int8_t validHeartRate; 

// Wi-Fi credentials
const char* ssid = "pinky9"; // Your SSID
const char* password = "sunny123"; // Your Password
char auth[] = BLYNK_AUTH_TOKEN;

#define REPORTING_PERIOD_MS 1000
uint32_t tsLastReport = 0; 

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1);
  }

  Serial.println("Place your index finger on the sensor with steady pressure.");
  particleSensor.setup(); 
  particleSensor.setPulseAmplitudeRed(0x1F); 
  particleSensor.setPulseAmplitudeIR(0x1F); 

  // Initialize accelerometer
  if (!accel.begin()) {
    Serial.println("No valid sensor found");
    while (1);
  }

  // Set up WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi Connected");

  // Set up Blynk
  Blynk.begin(auth, ssid, password);
}

void loop() {
  // Collect 100 samples of both IR and Red LED
  for (int i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false) // Wait for new data
      particleSensor.check();
      
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); 
  }

  // Calculate heart rate and SpO2
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Display results in Serial Monitor
  Serial.print("Heart Rate: ");
  if (validHeartRate) {
    Serial.print(heartRate);
  } else {
    Serial.print("Invalid");
  }
  
  Serial.print(" bpm, SpO2: ");
  if (validSPO2) {
    Serial.print(spo2);
  } else {
    Serial.print("Invalid");
  }
  Serial.println();

  // Report to Blynk periodically
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    // Send Heart Rate and SpO2 to Blynk
    if (validHeartRate) {
      Blynk.virtualWrite(V3, heartRate); // Send Heart Rate to Virtual Pin 0
    }
    if (validSPO2) {
      Blynk.virtualWrite(V4, spo2); // Send SpO2 to Virtual Pin 1
    }
    
    // Log event for low heart rate
    if (heartRate >120) {
      Blynk.logEvent("high_heart_rate", "he/she having symptoms of seizures, please consult doctor ");
    }

    // Read accelerometer data
    sensors_event_t event;
    accel.getEvent(&event);

    // Send x, y, z values to Blynk
    Blynk.virtualWrite(V0, event.acceleration.x); // Send X to Virtual Pin 2
    Blynk.virtualWrite(V1, event.acceleration.y); // Send Y to Virtual Pin 3
    Blynk.virtualWrite(V2, event.acceleration.z); // Send Z to Virtual Pin 4000000000000

    tsLastReport = millis();-



}

  Blynk.run();
  delay(1000); // Delay for next measurement
}
