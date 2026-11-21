#include <WiFi.h>
#include "esp_wifi.h"

// ===================== SETTINGS =====================
#define SESSION       1        // 1 = first session, 2 = independent validation session
#define CHANNEL       1
#define N_ANCH        3
#define N_POS         9
#define STM_RX        16
#define STM_TX        17
#define BTN_C_PIN     18       // capture next configuration
#define BTN_R_PIN     26       // redo previous (optional)
#define DEBOUNCE_MS   40
#define COUNTDOWN_S   2
#define SETTLE_MS     1000
#define COLLECT_MS    10000    // ~100 packets per anchor at 10 Hz
#define MIN_PKTS      60
#define MAX_HEAD_STD  5.0f

// Positions in WALKING ORDER (serpentine). PID = the position's real ID.
const int   PID[N_POS] = {1,    6,    2,    7,    5,    8,    3,    9,    4};
const float PX[N_POS]  = {0.40, 1.25, 2.10, 2.10, 1.25, 0.40, 0.40, 1.25, 2.10};
const float PY[N_POS]  = {0.60, 0.60, 0.60, 1.80, 1.80, 1.80, 3.30, 3.30, 3.30};

const uint8_t ANCH_MAC[N_ANCH][6] = {
  {0xE0,0x8C,0xFE,0x5D,0x66,0xC8},   // anchor 1
  {0xC0,0xCD,0xD6,0xCE,0x5D,0xD0},   // anchor 2
  {0xC8,0x2B,0x96,0x1E,0x12,0x56},   // anchor 3 (ESP8266)
};
// ====================================================

#define NCFG  (N_POS * 4)      // 9 positions x 4 headings = 36
#define MAXS  200

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
int8_t   samp[N_ANCH][MAXS];
uint16_t ns[N_ANCH];
volatile bool g_cap = false;

float    g_heading = 0;
uint32_t g_lastH = 0;
float    hS = 0, hC = 0;
uint32_t hN = 0;
int      cIdx = 0;

struct Btn { uint8_t pin; bool stable; bool lastRaw; uint32_t t; };
Btn bC = {BTN_C_PIN, HIGH, HIGH, 0};
Btn bR = {BTN_R_PIN, HIGH, HIGH, 0};

bool btnPressed(Btn& b) {
  bool raw = digitalRead(b.pin);
  uint32_t now = millis();
  if (raw != b.lastRaw) { b.lastRaw = raw; b.t = now; }
  if (now - b.t > DEBOUNCE_MS && raw != b.stable) {
    b.stable = raw;
    if (raw == LOW) return true;
  }
  return false;
}
void btnResync(Btn& b) { b.stable = b.lastRaw = digitalRead(b.pin); b.t = millis(); }

int idxFromMac(const uint8_t* m) {
  for (int i = 0; i < N_ANCH; i++) if (!memcmp(ANCH_MAC[i], m, 6)) return i;
  return -1;
}

void sniffer(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t* p = (const wifi_promiscuous_pkt_t*)buf;
  const uint8_t* d = p->payload;
  if (p->rx_ctrl.sig_len < 28) return;
  if (d[0] != 0xD0) return;
  if (d[24] != 127 || d[25] != 0x18 || d[26] != 0xFE || d[27] != 0x34) return;
  int ai = idxFromMac(d + 10);
  if (ai < 0) return;
  if (!g_cap) return;
  portENTER_CRITICAL(&mux);
  if (ns[ai] < MAXS) samp[ai][ns[ai]++] = p->rx_ctrl.rssi;
  portEXIT_CRITICAL(&mux);
}

void readSTM() {
  static char line[96]; static uint8_t n = 0;
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      line[n] = 0; n = 0;
      float h; int z, s;
      if (sscanf(line, "H,%f,%d,%d", &h, &z, &s) == 3) {
        g_heading = h; g_lastH = millis();
        if (g_cap) { hS += sinf(h * DEG_TO_RAD); hC += cosf(h * DEG_TO_RAD); hN++; }
      } else if (line[0]) Serial.printf("# STM: %s\n", line);
    } else if (c != '\r' && n < sizeof(line) - 1) line[n++] = c;
  }
}

void waitMs(uint32_t ms) {
  uint32_t t = millis();
  while (millis() - t < ms) { readSTM(); delay(2); }
}

bool stmOk() { return g_lastH != 0 && millis() - g_lastH < 2000; }

void printNext() {
  if (cIdx >= NCFG) { Serial.printf("# ALL %d CONFIGURATIONS DONE (session %d). Copy the D, lines.\n", NCFG, SESSION); return; }
  int p = cIdx / 4;
  Serial.printf("# next: cfg %d/%d  P%d (x=%.2f y=%.2f)  heading %d  -> press the button\n",
                cIdx + 1, NCFG, PID[p], PX[p], PY[p], (cIdx % 4) * 90);
}

void capture() {
  if (cIdx >= NCFG) { printNext(); return; }
  if (!stmOk()) Serial.println("# WARNING: no STM32 heading (is its calibration finished?)");
  int p = cIdx / 4, hn = (cIdx % 4) * 90;
  Serial.printf("# cfg %d/%d P%d heading %d: settle into your posture\n", cIdx + 1, NCFG, PID[p], hn);
  for (int s = COUNTDOWN_S; s > 0; s--) { Serial.printf("# %d...\n", s); waitMs(1000); }
  waitMs(SETTLE_MS);

  portENTER_CRITICAL(&mux);
  memset(ns, 0, sizeof(ns));
  portEXIT_CRITICAL(&mux);
  hS = hC = 0; hN = 0;
  g_cap = true;
  Serial.printf("# recording %d s, stay still...\n", COLLECT_MS / 1000);
  waitMs(COLLECT_MS);
  g_cap = false;

  float hm = -1;
  if (hN) { hm = atan2f(hS, hC) * RAD_TO_DEG; if (hm < 0) hm += 360; }
  float R = hN ? sqrtf(hS * hS + hC * hC) / hN : 0;
  float hsd = (R > 0.001f && R < 1.0f) ? sqrtf(-2 * logf(R)) * RAD_TO_DEG : 0;

  bool warn = (hN && hsd > MAX_HEAD_STD);
  for (int i = 0; i < N_ANCH; i++) {
    int n = ns[i];
    float mean = -127, med = -127, sd = 0; int mn = 0, mx = 0;
    if (n > 0) {
      int8_t t[MAXS]; memcpy(t, samp[i], n);
      for (int a = 1; a < n; a++) { int8_t k = t[a]; int b = a - 1; while (b >= 0 && t[b] > k) { t[b+1] = t[b]; b--; } t[b+1] = k; }
      long sum = 0; for (int a = 0; a < n; a++) sum += t[a];
      mean = (float)sum / n;
      float v = 0; for (int a = 0; a < n; a++) v += (t[a] - mean) * (t[a] - mean);
      sd = n > 1 ? sqrtf(v / (n - 1)) : 0;
      med = (n % 2) ? t[n / 2] : (t[n / 2 - 1] + t[n / 2]) / 2.0f;
      mn = t[0]; mx = t[n - 1];
    }
    if (n < MIN_PKTS) warn = true;
    // D,session,cfg,pos,x,y,head_nom,head_meas,head_std,anchor,n,mean,median,std,min,max
    Serial.printf("D,%d,%d,%d,%.2f,%.2f,%d,%.1f,%.1f,%d,%d,%.2f,%.1f,%.2f,%d,%d\n",
                  SESSION, cIdx + 1, PID[p], PX[p], PY[p], hn, hm, hsd, i + 1, n, mean, med, sd, mn, mx);
  }
  if (warn) Serial.println("# WARNING: low packets or heading moved. Redo this one (button 2 or type r).");
  else      Serial.printf("# cfg %d saved OK\n", cIdx + 1);
  cIdx++;
  printNext();
}

void handleCmd(char c) {
  if (c == 'c') capture();
  else if (c == 'r') {
    if (cIdx > 0) cIdx--;
    Serial.printf("# redo: older D, lines of cfg %d are void, keep the new ones\n", cIdx + 1);
    capture();
  }
  else if (c == 'g') { cIdx = 0; Serial.println("# restarted from cfg 1"); printNext(); }
  else Serial.println("# button 1 / c = capture next | button 2 / r = redo previous | g = restart");
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, STM_RX, STM_TX);
  pinMode(BTN_C_PIN, INPUT_PULLUP);
  pinMode(BTN_R_PIN, INPUT_PULLUP);
  WiFi.mode(WIFI_STA); WiFi.disconnect();
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  wifi_promiscuous_filter_t f = { .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT };
  esp_wifi_set_promiscuous_filter(&f);
  esp_wifi_set_promiscuous_rx_cb(&sniffer);
  esp_wifi_set_promiscuous(true);
  Serial.printf("# SESSION %d\n", SESSION);
  Serial.println("# D,session,cfg,pos,x,y,head_nom,head_meas,head_std,anchor,n,mean,median,std,min,max");
  Serial.println("# wait for the STM32 calibration to finish, then press button 1 (button 2 = redo)");
  printNext();
}

void loop() {
  readSTM();
  if (btnPressed(bC)) { handleCmd('c'); btnResync(bC); btnResync(bR); }
  if (btnPressed(bR)) { handleCmd('r'); btnResync(bC); btnResync(bR); }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;
    handleCmd(c);
    btnResync(bC); btnResync(bR);
  }
}