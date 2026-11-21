#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define ANCHOR_ID   2         // <-- 1 for the first board, 2 for the second
#define CHANNEL     1
#define PERIOD_MS   100

typedef struct __attribute__((packed)) {
  uint8_t  id;
  uint32_t seq;
} Beacon;

uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t seq = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  WiFi.setTxPower(WIFI_POWER_11dBm);       // FIXED, same as what you told the tag side
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, bcast, 6);
  peer.channel = CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.printf("Anchor %d ready, MAC %s\n", ANCHOR_ID, WiFi.macAddress().c_str());
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last >= PERIOD_MS) {
    last = millis();
    Beacon b = {ANCHOR_ID, seq++};
    esp_now_send(bcast, (uint8_t *)&b, sizeof(b));
  }
}