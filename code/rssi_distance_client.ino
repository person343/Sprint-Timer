#include <WiFi.h>

// Target network to measure distance to
const char* targetSSID = "ESP32_DISTANCE_AP";
const char* password = "12345678";
const int measuredPower = -40;

// Smoothing parameters
const int BUFFER_SIZE = 10;
int rssiBuffer[BUFFER_SIZE];
float distanceBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

// Exponential Moving Average
float rssiEMA = 0;
float distanceEMA = 0;
const float alpha = 0.3; // Smoothing factor (0.1 = heavy smoothing, 0.9 = light smoothing)

// Outlier detection parameters
float lastValidDistance = 0;
const float MAX_CHANGE_PERCENT = 50.0; // Reject changes > 50%
const float MIN_DISTANCE = 0.1;        // Minimum realistic distance (10cm)
const float MAX_DISTANCE = 50.0;       // Maximum realistic distance (50m)
const int MIN_RSSI = -100;             // Minimum realistic RSSI
const int MAX_RSSI = -10;              // Maximum realistic RSSI

void setup() {
  Serial.begin(115200);
  delay(1000);
 
  Serial.println("ESP32 Distance Measurement - Receiver");
  Serial.println("Connecting to target AP...");
 
  WiFi.mode(WIFI_STA);
  WiFi.begin(targetSSID, password);
 
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.println("\nConnected! Starting RSSI monitoring...");
  delay(2000); // Give time to switch to Serial Plotter
}

void loop() {
  // Get RSSI of connected network - much faster than scanning!
  if (WiFi.status() == WL_CONNECTED) {
    int rssi = WiFi.RSSI();
   
    float rawDistance = distanceCalculation(rssi);
   
    // Check if this reading should be rejected
    bool isValid = isValidReading(rssi, rawDistance);
   
    // Apply smoothing algorithms (only if valid, otherwise use last valid)
    float movingAvgDistance = getMovingAverage(rssi, rawDistance);
    float emaDistance = getExponentialMovingAverage(rssi, rawDistance);
    float kalmanDistance = isValid ? getKalmanFiltered(rawDistance) : getKalmanFiltered(lastValidDistance);
   
    // Option 1: Show all for comparison with outlier detection status
    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.print(" ,Raw:");
    Serial.print(rawDistance);
    Serial.print(",MovingAvg:");
    Serial.print(movingAvgDistance);
    Serial.print(",EMA:");
    Serial.print(emaDistance);
    Serial.print(",Kalman:");
    Serial.print(kalmanDistance);
    Serial.print(",Valid:");
    Serial.println(isValid ? 1 : 0); // 1 = valid, 0 = outlier rejected
   
    // Option 2: Show just cleaned data (uncomment preferred)
    // if (isValid) {  // Only print valid readings
    //   Serial.print("RSSI:");
    //   Serial.print(rssi);
    //   Serial.print(",Distance:");
    //   Serial.println(emaDistance);
    // }
   
  } else {
    Serial.println("Disconnected");
  }
 
  delay(100);
}

float distanceCalculation(int rssi) {
  float rssi_1m = measuredPower;  // RSSI at 1 meter (calibrate this!)
  float pathLoss = 2.0; // Environment factor
 
  // Distance = 10^((RSSI_1m - RSSI) / (10 * pathLoss))
  float distance = pow(10, (rssi_1m - rssi) / (10 * pathLoss));
  return distance;
}

// Outlier Detection Functions
bool isValidRSSI(int rssi) {
  return (rssi >= MIN_RSSI && rssi <= MAX_RSSI);
}

bool isValidDistance(float distance) {
  return (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE);
}

bool isReasonableChange(float newDistance) {
  if (lastValidDistance == 0) return true; // First reading
 
  float changePercent = abs(newDistance - lastValidDistance) / lastValidDistance * 100.0;
  return (changePercent <= MAX_CHANGE_PERCENT);
}

// Combined outlier detection
bool isValidReading(int rssi, float distance) {
  return isValidRSSI(rssi) && isValidDistance(distance) && isReasonableChange(distance);
}
// Moving Average Filter (now with outlier rejection)
float getMovingAverage(int newRssi, float newDistance) {
  // Only add to buffer if it's a valid reading
  if (isValidReading(newRssi, newDistance)) {
    rssiBuffer[bufferIndex] = newRssi;
    distanceBuffer[bufferIndex] = newDistance;
   
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
    if (bufferIndex == 0) bufferFull = true;
   
    lastValidDistance = newDistance; // Update last valid reading
  }
 
  // Calculate average from valid readings only
  int count = bufferFull ? BUFFER_SIZE : bufferIndex;
  if (count == 0) return lastValidDistance; // No valid readings yet
 
  float distanceSum = 0;
  for (int i = 0; i < count; i++) {
    distanceSum += distanceBuffer[i];
  }
 
  return distanceSum / count;
}


// Exponential Moving Average Filter (with outlier rejection)
float getExponentialMovingAverage(int newRssi, float newDistance) {
  // Only update if it's a valid reading
  if (isValidReading(newRssi, newDistance)) {
    if (rssiEMA == 0) { // First valid reading
      rssiEMA = newRssi;
      distanceEMA = newDistance;
    } else {
      rssiEMA = alpha * newRssi + (1 - alpha) * rssiEMA;
      distanceEMA = alpha * newDistance + (1 - alpha) * distanceEMA;
    }
    lastValidDistance = newDistance;
  }
 
  return distanceEMA;
}

// Kalman Filter (simplified)
float kalmanGain = 0.5;
float estimatedDistance = 0;
float errorCovariance = 1;

float getKalmanFiltered(float measurement) {
  if (estimatedDistance == 0) {
    estimatedDistance = measurement;
    return estimatedDistance;
  }
 
  // Prediction step (assume no change)
  float predictedDistance = estimatedDistance;
  float predictedError = errorCovariance + 0.1; // Process noise
 
  // Update step
  kalmanGain = predictedError / (predictedError + 0.5); // Measurement noise
  estimatedDistance = predictedDistance + kalmanGain * (measurement - predictedDistance);
  errorCovariance = (1 - kalmanGain) * predictedError;
 
  return estimatedDistance;
}
