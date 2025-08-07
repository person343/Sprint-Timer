#include <esp_now.h>                                  // Library for ESP-NOW communication
#include <WiFi.h>                             
#include <Wire.h>                                     // I2C communication library
#include <LiquidCrystal_I2C.h>                        // LCD I2C library
#include "SparkFun_VL53L1X.h"                         // Sparkfun VL53L1X distance sensor library

LiquidCrystal_I2C lcd(0x27, 16, 2);                   // Initialize 16x2 LCD. I2C Address: 0x27
SFEVL53L1X distanceSensor;                            // Create instance of distance sensor

uint8_t finishMac[] = {0x00, 0x4B, 0x12, 0x2F, 0xBD, 0x30}; // MAC address of finishing ESP32

typedef struct struct_message {
  char msgType[10];                                   // Message says either "START" or "STOP"
  unsigned long timestamp;                            // Timestamp in milliseconds
} struct_message;

struct_message messageToSend;                         // Message to send to finish ESP32
bool timingStarted = false;                           // Indicates if timing has started
unsigned long startTime = 0;                          // Variable to store start time

// Callback function when data is received from another ESP32
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  struct_message received;                            // Temporary structure to hold incoming data
  memcpy(&received, incomingData, sizeof(received));  // Copy received data into structure

  // If STOP message is received and timing was started
  if (strcmp(received.msgType, "STOP") == 0 && timingStarted) {
    unsigned long endTime = millis();                 // Record current time as end time
    unsigned long elapsed = endTime - startTime;      // Calculate elapsed time

    lcd.clear();                                      // Clear LCD
    lcd.setCursor(0, 0);                              
    lcd.print("Time:");                               
    lcd.setCursor(6, 0);                              
    lcd.print(elapsed / 1000.0, 2);                   // Display elapsed time in seconds (2 decimals)
    lcd.print(" sec");                                // Units

    Serial.print("Elapsed Time: ");                   // Prints elapsed time to Serial Monitor
    Serial.println(elapsed);

    timingStarted = false;                            // Resets timing
  }
}

void setup() {
  Serial.begin(115200);                               // Start Serial Monitor
  Wire.begin();                                       // Initialize I2C communication
  lcd.init();                                         // Initialize LCD screen
  lcd.backlight();                                    // Turn on LCD backlight
  lcd.setCursor(0, 0);                        
  lcd.print("Ready...");                      

  if (distanceSensor.begin() != 0) {                  // Initialize distance sensor
    Serial.println("Sensor fail");                    // If fails, prints an error message
    while (1);                                        
  }

  distanceSensor.setDistanceModeLong();               // Set sensor to long-distance mode
  distanceSensor.setTimingBudgetInMs(33);
  distanceSensor.setIntermeasurementPeriod(33);
  distanceSensor.startRanging();                      // Start measuring distance

  WiFi.mode(WIFI_STA);                                // Set WiFi to Station mode (required for ESP-NOW)
  WiFi.disconnect();                                  // Disconnect from any previous WiFi

  if (esp_now_init() != ESP_OK) {                     // Initialize ESP-NOW
    Serial.println("ESP-NOW failed");                 // Print error if failed
    return;                                   
  }

  esp_now_register_recv_cb(OnDataRecv);               // Register callback for receiving data

  esp_now_peer_info_t peerInfo = {};                  // Create peer info struct
  memcpy(peerInfo.peer_addr, finishMac, 6);           // Set peer MAC address
  peerInfo.channel = 0;                               
  peerInfo.encrypt = false;                           

  if (!esp_now_add_peer(&peerInfo)) {                 // Add the peer
    Serial.println("Peer added");                     // Confirm peer was added
  }
}

void loop() {
  // If timing hasn't started and new distance data is ready
  if (!timingStarted && distanceSensor.checkForDataReady()) {
    uint16_t distance = distanceSensor.getDistance(); // Read distance in mm
    distanceSensor.clearInterrupt();                  // Clear sensor interrupt

    Serial.print("Distance: ");                       
    Serial.println(distance);                         // Debug: print distance

    if (distance >= 40 && distance < 300) {           // Threshold of 40 to 300mm (Won't start timing unless an object makes it within the threshold)
      startTime = millis();                           // Records time
      messageToSend.timestamp = startTime;            // Include timestamp in message
      strcpy(messageToSend.msgType, "START");         

      esp_now_send(finishMac, (uint8_t *)&messageToSend, sizeof(messageToSend)); // Send start message

      lcd.clear();                                    
      lcd.setCursor(0, 0);
      lcd.print("Timing...");                         // Display "Timing..." on LCD. Indicates that timer is running
      Serial.println("Start detected");              
      timingStarted = true;                           // Flag that timing has started
    }
  }
}
