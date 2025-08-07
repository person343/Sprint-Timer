#include "WiFi.h"

void setup() {
  Serial.begin(115200);
}

void loop() {
  WiFi.mode(WIFI_STA);
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());
  while(1){}
}
