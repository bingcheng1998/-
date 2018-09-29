#include "WiFi_setup.h"

// ================================================================
// ===                       setup Udp                          ===
// ================================================================


void setupUdp() {
  bool udpSetup = 0;
  Serial.println("UDP connecting...");
  while (!udpSetup) {
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      Serial.println("UDP connected!!!");
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      IPAddress remoteIp = Udp.remoteIP();
      Serial.print(remoteIp);
      Serial.print(", port ");
      Serial.println(Udp.remotePort());
      // read the packet into packetBufffer
      int len = Udp.read(packetBuffer, 255);
      if (len > 0) {
        packetBuffer[len] = 0;
      }
      Serial.println("Contents:");
      Serial.println(packetBuffer);
      udpSetup = !udpSetup;
    }
  }
}

// ================================================================
// ===                  setup Wifi connection                   ===
// ================================================================
#ifndef WiFiHOTSPOTS
void WiFiSetup() {
  int k = 0;

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    k++;
    if(k%35 == 0){
      Serial.print("\n");
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
#endif