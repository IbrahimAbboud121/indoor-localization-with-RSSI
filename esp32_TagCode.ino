#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdarg.h>
#include "esp_wifi.h"

#define CHANNEL        1
#define N_ANCH         3
#define N_POS          5
#define STM_RX         16
#define STM_TX         17
#define BTN_PIN        18
#define SIG_RSSI       4.0f
#define RANK_W         0.7f
#define STABLE_DB      4.0f
#define STABLE_DEG     25.0f
#define NEED_BLOCKS    3     // stable seconds for the first decision
#define CONFIRM_BLOCKS 2     // stable seconds for the confirmation window
#define MIN_BLK_PKTS   5
#define MOVE_DB        6.0f
#define MOVE_DEG       40.0f

const float PX[N_POS] = {0.40, 2.10, 0.40, 2.10, 1.25};
const float PY[N_POS] = {0.60, 0.60, 3.30, 3.30, 1.80};

const float FP[N_POS][4][N_ANCH] = {
  {{-51.38,-62.35,-72.34},{-65.31,-61.76,-66.50},{-56.91,-66.53,-62.03},{-52.71,-64.81,-74.43}},
  {{-66.58,-50.53,-70.13},{-74.83,-57.33,-68.90},{-63.29,-58.68,-64.46},{-59.52,-55.62,-74.11}},
  {{-74.34,-67.88,-61.98},{-75.97,-67.57,-55.57},{-70.71,-82.20,-57.47},{-65.79,-69.77,-59.03}},
  {{-63.16,-60.99,-56.40},{-73.64,-64.99,-57.07},{-62.59,-65.57,-49.02},{-62.66,-63.11,-62.89}},
  {{-58.59,-68.71,-67.84},{-67.69,-65.99,-65.12},{-63.93,-66.78,-63.38},{-58.87,-64.63,-79.00}},
};

const uint8_t ANCH_MAC[N_ANCH][6] = {
  {0xE0,0x8C,0xFE,0x5D,0x66,0xC8},
  {0xC0,0xCD,0xD6,0xCE,0x5D,0xD0},
  {0xC8,0x2B,0x96,0x1E,0x12,0x56},
};

WiFiUDP udp;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

uint32_t bN[N_ANCH]; int32_t bSum[N_ANCH];
volatile bool g_run = false;

uint32_t wN[N_ANCH]; int32_t wSum[N_ANCH]; float wS = 0, wC = 0;

float g_heading = 0, zeroHead = 0, hS = 0, hC = 0;
uint32_t g_lastH = 0, tBlk = 0;
int   curPos = 1, stableCnt = 0, forceBlocks = 0, g_state = 2;
int   pend = -1;                                   // pending candidate (0-based), -1 = none
bool  running = false, moved = false, readyAnnounced = false, havePrev = false;
float prevM[N_ANCH], prevH = 0;
float refM[N_ANCH], refH = 0; bool haveRef = false;

void out(const char* fmt, ...) {
  char b[200]; va_list ap; va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
  Serial.print(b);
  udp.beginPacket(IPAddress(192,168,4,255), 5005);
  udp.write((const uint8_t*)b, strlen(b));
  udp.endPacket();
}

int idxFromMac(const uint8_t* m) {
  for (int i = 0; i < N_ANCH; i++) if (!memcmp(ANCH_MAC[i], m, 6)) return i;
  return -1;
}

void sniffer(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT || !g_run) return;
  const wifi_promiscuous_pkt_t* p = (const wifi_promiscuous_pkt_t*)buf;
  const uint8_t* d = p->payload;
  if (p->rx_ctrl.sig_len < 28 || d[0] != 0xD0) return;
  if (d[24] != 127 || d[25] != 0x18 || d[26] != 0xFE || d[27] != 0x34) return;
  int ai = idxFromMac(d + 10);
  if (ai < 0) return;
  portENTER_CRITICAL(&mux);
  bN[ai]++; bSum[ai] += p->rx_ctrl.rssi;
  portEXIT_CRITICAL(&mux);
}

float angDiff(float a, float b) {
  float d = fabsf(a - b);
  while (d >= 360) d -= 360;
  return d > 180 ? 360 - d : d;
}

float relHeading(float h) {
  return fmodf(zeroHead - h + 720.0f, 360.0f);     // STM32 heading is mirrored
}

void readSTM() {
  static char line[128]; static uint8_t n = 0;
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      line[n] = 0; n = 0;
      float h; int z, s;
      if (sscanf(line, "H,%f,%d,%d", &h, &z, &s) == 3) {
        g_heading = h; g_lastH = millis();
        if (g_run) { hS += sinf(h * DEG_TO_RAD); hC += cosf(h * DEG_TO_RAD); }
      } else if (line[0]) {
        out("# STM: %s\n", line);
      }
    } else if (c != '\r' && n < sizeof(line) - 1) line[n++] = c;
  }
}

void clearWindow() {
  for (int i = 0; i < N_ANCH; i++) { wN[i] = 0; wSum[i] = 0; }
  wS = wC = 0; stableCnt = 0;
}

// scores the current window; returns best position index (0-based) or -1
int evaluate(bool excludeCur, float &c1, float &c2, float &rel) {
  float m[N_ANCH]; int valid = 0;
  for (int i = 0; i < N_ANCH; i++) {
    if (wN[i] >= 10) { m[i] = (float)wSum[i] / wN[i]; valid++; } else m[i] = -999;
  }
  if (valid < 2) { out("# not enough packets (%d anchors), try again\n", valid); return -1; }
  for (int i = 0; i < N_ANCH; i++) refM[i] = m[i];
  haveRef = true;

  float hm = atan2f(wS, wC) * RAD_TO_DEG; if (hm < 0) hm += 360;
  refH = hm;
  rel = relHeading(hm);
  int bin = ((int)((rel + 45.0f) / 90.0f)) % 4;

  int nearA = 0;
  for (int i = 1; i < N_ANCH; i++) if (m[i] > m[nearA]) nearA = i;
  Serial.printf("\n=== measured RSSI: %.1f %.1f %.1f | nearest anchor: A%d | heading %.0f (bin %d)\n",
                m[0], m[1], m[2], nearA + 1, rel, bin * 90);
  Serial.println("    pos | fingerprint | rank mismatch | total");

  int best = -1; c1 = 1e9f; c2 = 1e9f;
  for (int p = 0; p < N_POS; p++) {
    if (excludeCur && p + 1 == curPos) continue;
    float fp = 0; int cnt = 0;
    for (int i = 0; i < N_ANCH; i++) {
      if (m[i] < -900) continue;
      float e = (m[i] - FP[p][bin][i]) / SIG_RSSI; fp += e * e; cnt++;
    }
    fp /= cnt;
    int mis = 0;
    for (int i = 0; i < N_ANCH; i++) for (int j = i + 1; j < N_ANCH; j++) {
      if (m[i] < -900 || m[j] < -900) continue;
      bool a = m[i] > m[j], b = FP[p][bin][i] > FP[p][bin][j];
      if (a != b) mis++;
    }
    float tot = fp + RANK_W * mis;
    Serial.printf("    P%d  |   %6.2f    |      %d        | %6.2f\n", p + 1, fp, mis, tot);
    if (tot < c1) { c2 = c1; c1 = tot; best = p; } else if (tot < c2) c2 = tot;
  }
  return best;
}

// first window: propose a candidate. Following windows: confirm or reject.
void decide() {
  float c1, c2, rel;
  bool first = (pend < 0);
  int r = evaluate(first, c1, c2, rel);            // first window excludes the current position
  if (r < 0) return;

  if (first) {
    pend = r; moved = false;
    out("# candidate P%d, waiting for confirmation\n", r + 1);
  } else if (r == pend) {                          // confirmed: commit the move
    out("P,%d,%d,%.2f,%.2f,%.1f,%.2f,%.2f\n", curPos, r + 1, PX[r], PY[r], rel, c1, c2 - c1);
    Serial.printf(">>> CONFIRMED: P%d -> P%d\n\n", curPos, r + 1);
    curPos = r + 1; pend = -1; moved = false;
  } else if (r == curPos - 1) {                    // second look says we never left
    out("# rejected P%d, still at P%d\n", pend + 1, curPos);
    pend = -1; moved = false;
  } else {                                         // a different position: new candidate
    pend = r;
    out("# new candidate P%d, waiting for confirmation\n", r + 1);
  }
}

void blockTick() {
  uint32_t n[N_ANCH]; int32_t s[N_ANCH];
  portENTER_CRITICAL(&mux);
  memcpy(n, bN, sizeof(n)); memcpy(s, bSum, sizeof(s));
  for (int i = 0; i < N_ANCH; i++) { bN[i] = 0; bSum[i] = 0; }
  portEXIT_CRITICAL(&mux);

  float m[N_ANCH]; int valid = 0;
  for (int i = 0; i < N_ANCH; i++) {
    if (n[i] >= MIN_BLK_PKTS) { m[i] = (float)s[i] / n[i]; valid++; } else m[i] = -999;
  }
  float hb = (hS != 0 || hC != 0) ? atan2f(hS, hC) * RAD_TO_DEG : g_heading;
  if (hb < 0) hb += 360;
  hS = hC = 0;

  bool stable = false;
  if (havePrev && valid >= 2) {
    stable = true;
    for (int i = 0; i < N_ANCH; i++)
      if (m[i] > -900 && prevM[i] > -900 && fabsf(m[i] - prevM[i]) > STABLE_DB) stable = false;
    if (angDiff(hb, prevH) > STABLE_DEG) stable = false;
  }
  for (int i = 0; i < N_ANCH; i++) prevM[i] = m[i];
  prevH = hb; havePrev = true;

  if (valid >= 2) {
    if (!haveRef) { for (int i = 0; i < N_ANCH; i++) refM[i] = m[i]; refH = hb; haveRef = true; }
    else {
      for (int i = 0; i < N_ANCH; i++)
        if (m[i] > -900 && refM[i] > -900 && fabsf(m[i] - refM[i]) > MOVE_DB) moved = true;
      if (angDiff(hb, refH) > MOVE_DEG) moved = true;
    }
  }

  const char* tag;
  if (forceBlocks > 0) {                           // button: measure regardless of stability
    for (int i = 0; i < N_ANCH; i++) { wN[i] += n[i]; wSum[i] += s[i]; }
    wS += sinf(hb * DEG_TO_RAD); wC += cosf(hb * DEG_TO_RAD);
    forceBlocks--; tag = "FORCED   "; g_state = 1;
    if (forceBlocks == 0) { decide(); clearWindow(); g_state = (pend >= 0) ? 1 : 2; }
  } else if (!stable) {
    clearWindow(); tag = "MOVING   "; g_state = 0;
  } else {
    for (int i = 0; i < N_ANCH; i++) { wN[i] += n[i]; wSum[i] += s[i]; }
    wS += sinf(hb * DEG_TO_RAD); wC += cosf(hb * DEG_TO_RAD);
    stableCnt++;
    bool busy = moved || pend >= 0;
    tag = busy ? "STABLE   " : "AT POS   ";
    g_state = busy ? 1 : 2;
    if (pend >= 0) {
      if (stableCnt >= CONFIRM_BLOCKS) { decide(); clearWindow(); g_state = (pend >= 0) ? 1 : 2; }
    } else if (moved && stableCnt >= NEED_BLOCKS) {
      decide(); clearWindow(); g_state = (pend >= 0) ? 1 : 2;
    }
  }
  Serial.printf("%s P%d%s heading=%.0f | rssi %.0f %.0f %.0f | stable %d\n", tag, curPos,
                pend >= 0 ? "?" : " ", relHeading(hb), m[0], m[1], m[2], stableCnt);
}

bool btnPressed() {
  static bool st = HIGH, lastRaw = HIGH; static uint32_t t = 0;
  bool raw = digitalRead(BTN_PIN); uint32_t now = millis();
  if (raw != lastRaw) { lastRaw = raw; t = now; }
  if (now - t > 40 && raw != st) { st = raw; if (raw == LOW) return true; }
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, STM_RX, STM_TX);
  pinMode(BTN_PIN, INPUT_PULLUP);
  WiFi.mode(WIFI_AP);
  WiFi.softAP("TAG", "12345678", CHANNEL, 0, 2);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  wifi_promiscuous_filter_t f = { .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT };
  esp_wifi_set_promiscuous_filter(&f);
  esp_wifi_set_promiscuous_rx_cb(&sniffer);
  esp_wifi_set_promiscuous(true);
  Serial.println("# tag ready, waiting for the STM32 calibration (messages will appear below)...");
}

void loop() {
  readSTM();
  uint32_t now = millis();
  bool stmOk = g_lastH != 0 && now - g_lastH < 2000;

  if (stmOk && !readyAnnounced) {
    readyAnnounced = true;
    if (!running)
      out("# STM32 CALIBRATION DONE -> go to position 1, face your 0 deg direction, PRESS THE BUTTON to start\n");
  }
  if (!stmOk && readyAnnounced && g_lastH != 0) {
    readyAnnounced = false; running = false; g_run = false;
    out("# STM32 heading lost (reset?). Waiting for it to finish calibrating again...\n");
  }

  if (btnPressed()) {
    if (!running) {
      if (!stmOk) out("# STM32 is still calibrating, wait for the 'CALIBRATION DONE' message\n");
      else {
        zeroHead = g_heading; running = true; curPos = 1; moved = false; pend = -1;
        havePrev = false; haveRef = false; g_state = 2;
        forceBlocks = 0; clearWindow(); hS = hC = 0; tBlk = now;
        portENTER_CRITICAL(&mux); for (int i = 0; i < N_ANCH; i++) { bN[i] = 0; bSum[i] = 0; } portEXIT_CRITICAL(&mux);
        g_run = true;
        out("# TEST STARTED at position 1, zero heading set. Walk to another position and hold still.\n");
        out("S,1\n");
      }
    } else {
      clearWindow(); forceBlocks = NEED_BLOCKS;
      out("# button: measuring the next 3 s, hold your posture still\n");
    }
  }
  if (!running) return;

  if (now - tBlk >= 1000) { tBlk += 1000; blockTick(); }

  static uint32_t tT = 0;
  if (now - tT >= 200) {
    tT = now;
    char b[80];
    int len = snprintf(b, sizeof(b), "T,%lu,%d,%d,%.2f,%.2f,%.1f\n", now, g_state, curPos,
                       PX[curPos-1], PY[curPos-1], relHeading(g_heading));
    udp.beginPacket(IPAddress(192,168,4,255), 5005);
    udp.write((const uint8_t*)b, len);
    udp.endPacket();
  }
}