#include <ESP8266WiFi.h>
#include <espnow.h>
extern "C" {
  #include <user_interface.h>   // needed for wifi_set_channel on ESP8266
}

#define ANCHOR_ID  3
#define CHANNEL    1
#define PERIOD_MS  100

typedef struct __attribute__((packed)) {
  uint8_t  id;
  uint32_t seq;
} Beacon;

uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
Beacon b = {ANCHOR_ID, 0};

void setup() {
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  WiFi.setOutputPower(10.0);              // FIXED TX power (calibration absorbs the value)
  wifi_set_channel(CHANNEL);

  if (esp_now_init() != 0) { Serial.println("ESP-NOW init failed"); while (1); }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(BCAST, ESP_NOW_ROLE_SLAVE, CHANNEL, NULL, 0);

  Serial.printf("Anchor %d ready, MAC %s\n", ANCHOR_ID, WiFi.macAddress().c_str());
  delay(ANCHOR_ID * 30);
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last >= PERIOD_MS) {
    last = millis();
    b.seq++;
    esp_now_send(BCAST, (uint8_t*)&b, sizeof(b));
  }
}