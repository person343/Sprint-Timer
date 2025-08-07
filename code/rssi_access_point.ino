// Network credentials
const char* ssid = "ESP32_DISTANCE_AP";
const char* password = "12345678";

void setup() {
  Serial.begin(115200);
  delay(1000);
 
  Serial.println("Setting up ESP32 as Access Point...");
 
  // Set WiFi mode to Access Point
  WiFi.mode(WIFI_AP);
 
  // Configure Access Point
  WiFi.softAP(ssid, password);
 
  // Print AP IP address
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
 
  Serial.println("Access Point started!");
  Serial.println("SSID: " + String(ssid));
  Serial.println("Waiting for connections...");
}

void loop() {
  // Check connected clients
  int clients = WiFi.softAPgetStationNum();
  static int lastClients = -1;
 
  if (clients != lastClients) {
    Serial.println("Connected clients: " + String(clients));
    lastClients = clients;
  }
 
  delay(2000);
}
