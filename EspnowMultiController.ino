#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
extern "C" {
  #include <espnow.h>
  #include <user_interface.h>
}

/**
   ESPNOW - Basic communication - Master
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
*/

typedef struct esp_now_peer_info {
    u8 peer_addr[6];    /**< ESPNOW peer MAC address that is also the MAC address of station or softap */
    uint8_t channel;                        /**< Wi-Fi channel that peer uses to send/receive ESPNOW data. If the value is 0,
                                                 use the current channel which station or softap is on. Otherwise, it must be
                                                 set as the channel that station or softap is on. */
} esp_now_peer_info_t;

// Global copy of slave
#define NUMSLAVES 20
esp_now_peer_info_t slaves[NUMSLAVES] = {};
int SlaveCnt = 0;

#define CHANNEL 1

void printMacAddress(uint8_t* macaddr) {
  Serial.print("{");
  for (int i = 0; i < 6; i++) {
    Serial.print("0x");
    Serial.print(macaddr[i], HEX);
    if (i < 5) Serial.print(',');
  }
  Serial.println("};");
}

// Init ESP Now with fallback
void InitESPNow() {
  if (esp_now_init() == 0) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
}

// Scan for slaves in AP mode
void ScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  //reset slaves
  memset(slaves, 0, sizeof(slaves));
  SlaveCnt = 0;
  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.print(i + 1); Serial.print(": ");
        Serial.print(SSID); Serial.print(" [");
        Serial.print(BSSIDstr); Serial.print("]");
        Serial.print(" ("); Serial.print(RSSI);
        Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        char macPart[4];
        macPart[0] = '0';
        macPart[1] = 'x';

        macPart[2] = (char)BSSIDstr[0];
        macPart[3] = (char)BSSIDstr[1];
        slaves[SlaveCnt].peer_addr[0] = strtol(macPart,0,16);
        macPart[2] = (char)BSSIDstr[3];
        macPart[3] = (char)BSSIDstr[4];
        slaves[SlaveCnt].peer_addr[1] = strtol(macPart,0,16);
        macPart[2] = (char)BSSIDstr[6];
        macPart[3] = (char)BSSIDstr[7];
        slaves[SlaveCnt].peer_addr[2] = strtol(macPart,0,16);
        macPart[2] = (char)BSSIDstr[9];
        macPart[3] = (char)BSSIDstr[10];
        slaves[SlaveCnt].peer_addr[3] = strtol(macPart,0,16);
        macPart[2] = (char)BSSIDstr[12];
        macPart[3] = (char)BSSIDstr[13];
        slaves[SlaveCnt].peer_addr[4] = strtol(macPart,0,16);
        macPart[2] = (char)BSSIDstr[15];
        macPart[3] = (char)BSSIDstr[16];
        slaves[SlaveCnt].peer_addr[5] = strtol(macPart,0,16);

        SlaveCnt++;
      }
    }
  }

  if (SlaveCnt > 0) {
    Serial.print(SlaveCnt); Serial.println(" Slave(s) found, processing..");
  } else {
    Serial.println("No Slave Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
void manageSlave() {
  if (SlaveCnt > 0) {
    for (int i = 0; i < SlaveCnt; i++) {
      const esp_now_peer_info_t *peer = &slaves[i];
      u8 *peer_addr = slaves[i].peer_addr;
      Serial.print("Processing: ");
      for (int ii = 0; ii < 6; ++ii ) {
        Serial.print((uint8_t) slaves[i].peer_addr[ii], HEX);
        if (ii != 5) Serial.print(":");
      }
      Serial.print(" Status: ");
      // check if the peer exists
      bool exists = esp_now_is_peer_exist((u8*)peer_addr);
      if (exists) {
        // Slave already paired.
        Serial.println("Already Paired");
      } else {
        // Slave not paired, attempt pair
        int addStatus = esp_now_add_peer((u8*)peer_addr, ESP_NOW_ROLE_CONTROLLER, CHANNEL, NULL, 0);
        if (addStatus == 0) {
          // Pair success
          Serial.println("Pair success");
        } else {
          Serial.println("Pair failed");
        }
        delay(100);
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
  }
}


uint8_t data = 0;
// send data
void sendData() {
  data++;
  for (int i = 0; i < SlaveCnt; i++) {
    u8 *peer_addr = slaves[i].peer_addr;
    if (i == 0) { // print only for first slave
      Serial.print("Sending: ");
      Serial.println(data);
    }
    int result = esp_now_send(peer_addr, &data, sizeof(data));
    Serial.print("Send Status: ");
    if (result ==0) {
      Serial.println("Success " + String(result));
    } else {
      Serial.println("Failed " + String(result));
    }
    delay(100);
  }
}

// callback when data is sent from Master to Slave
esp_now_send_cb_t OnDataSent(const uint8_t *mac_addr, u8 status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == 0 ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  Serial.println("ESPNow/Multi-Slave/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  // esp_now_register_send_cb(OnDataSent);
  esp_now_register_send_cb([](uint8_t* macaddr, uint8_t status) {
    printMacAddress(macaddr);
    static uint32_t ok = 0;
    static uint32_t fail = 0;
    if (status == 0) {
      Serial.println("ESPNOW: SEND_OK");
      ok++;
    }
    else {
      Serial.println("ESPNOW: SEND_FAILED");
      fail++;
    }
    Serial.printf("[SUCCESS] = %lu/%lu \r\n", ok, ok+fail);
  });
}

void loop() {
  // In the loop we scan for slave
  ScanForSlave();
  // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (SlaveCnt > 0) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    manageSlave();
    // pair success or already paired
    // Send data to device
    sendData();
  } else {
    // No slave found to process
  }

  // wait for 3seconds to run the logic again
  delay(1000);
}
