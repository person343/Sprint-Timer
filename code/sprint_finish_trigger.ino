#include <esp_now.h>                          // ESP-NOW communication library
#include <WiFi.h>                             // Required for setting WiFi mode
#include <Wire.h>                             // I2C communication library
#include "SparkFun_VL53L1X.h"                 // VL53L1X distance sensor library

SFEVL53L1X distanceSensor;                    // Create instance of distance sensor

uint8_t startMac[] = {0x38, 0x18, 0x2B, 0x89, 0xEE, 0xBC}; // MAC address of starting ESP32

typedef struct struct_message {
  char msgType[10];                           // Message type: "START" or "STOP"
  unsigned long timestamp;                    
} struct_message;

bool objectDetected = false;                  // Flag to prevent multiple triggers

void setup() {
  Serial.begin(115200);                       // Start Serial Monitor
  Wire.begin();                               // Initialize I2C communication

  if (distanceSensor.begin() != 0) {          // Initialize sensor
    Serial.println("Sensor fail");            // Print error if initialization fails
    while (1);                                
  }

  distanceSensor.setDistanceModeLong();       // Set long range mode
  distanceSensor.setTimingBudgetInMs(33);
  distanceSensor.setIntermeasurementPeriod(33);
  distanceSensor.startRanging();              // Start distance measurements

  WiFi.mode(WIFI_STA);                        // Set WiFi mode to Station
  WiFi.disconnect();                          // Disconnects from any previous WiFi connections

  if (esp_now_init() != ESP_OK) {             // Initialize ESP-NOW
    Serial.println("ESP-NOW failed");         // If it fails, print error message and stop
    return;
  }

  esp_now_peer_info_t peerInfo = {};          // Create peer info struct
  memcpy(peerInfo.peer_addr, startMac, 6);    // Set peer address (start ESP32)
  peerInfo.channel = 0;                       // Default channel
  peerInfo.encrypt = false;                   

  esp_now_add_peer(&peerInfo);                // Add peer (start ESP32)
}

void loop() {
  if (distanceSensor.checkForDataReady()) {           // Check if new distance data is available
    uint16_t distance = distanceSensor.getDistance(); // Read distance
    distanceSensor.clearInterrupt();                  // Clear interrupt for next reading

    Serial.print("Distance: ");               
    Serial.println(distance);                         // Print distance to Serial

    if (distance  >= 40 && distance < 300 && !objectDetected) {          // 40mm to 300mm threshold to detect an object
      struct_message stopMsg;                         // Create STOP message
      strcpy(stopMsg.msgType, "STOP");                // Set message type
      stopMsg.timestamp = millis();                   // Optional timestamp

      // Send STOP signal back to start ESP32
      esp_err_t result = esp_now_send(startMac, (uint8_t *)&stopMsg, sizeof(stopMsg));
      if (result == ESP_OK) {
        Serial.println("Stop signal sent");           // Confirm send
      } else {
        Serial.print("Send failed: ");                // Reports failure
        Serial.println(result);
      }

      objectDetected = true;                          // Mark object as detected
    } else if (distance > 300) {                      // If object is out of range (Won't be declared an object)
      objectDetected = false;                         // Reset detection flag
    }
  }
}
