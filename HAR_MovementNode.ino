/*
  HAR_MovementNode.ino — Nano 33 IoT (LSM6DS3)
  Clean BLE (LABEL JSONL + optional RAW CSV), on‑device HAR, serial commands.

  Phone / Pi usage
  - Subscribe to LABEL (…0006) → auto‑starts; emits JSONL like:
    {"t":123456,"node":1,"session":1,"label":"Walking","stepHz":1.72,"std":0.064,"mean":1.002}
  - (Optional) write "raw:on" to CTRL (…0003), then subscribe RAW (…0005) for CSV per hop:
    t_ms,node,ax,ay,az,gx,gy,gz
  - (Optional) binary IMU frames on …0002 when "bin:on" is sent (off by default).

  Commands (write text to CTRL or type in Serial Monitor):
    start | stop | rate:50|100|200 | raw:on/off | bin:on/off | serial:on/off | help

  Classifier: fs=100 Hz, window=0.8 s, hop=0.2 s, gravity calibration 0.5 s, 2‑vote debounce.
*/

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>

// ===== Build Config =====
#define NODE_ID 1  // 1=Wrist, 2=Ankle
#define DEFAULT_SAMPLE_HZ 100
#define SERIAL_BAUD 115200
#define SERIAL_LABELS 1       // print LABEL JSONL every hop
#define SERIAL_RAW_DEFAULT 0  // serial RAW CSV per hop (toggle via command)
#define RAW_NOTIFY_DEFAULT 0  // BLE RAW CSV per hop default state
#define BIN_NOTIFY_DEFAULT 0  // Binary notify default state
#define BATCH_SAMPLES 10
#define RING_CAPACITY 256
#define LED_PIN LED_BUILTIN

// ===== BLE UUIDs =====
static const char* UUID_SVC_MOV = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";       // service
static const char* UUID_CHAR_BIN_IMU = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";  // Notify (binary)
static const char* UUID_CHAR_CTRL = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";     // Write  (control)
static const char* UUID_CHAR_INFO = "6e400004-b5a3-f393-e0a9-e50e24dcca9e";     // Read   (info)
static const char* UUID_CHAR_RAW_ASC = "6e400005-b5a3-f393-e0a9-e50e24dcca9e";  // Notify (RAW CSV)
static const char* UUID_CHAR_LABEL = "6e400006-b5a3-f393-e0a9-e50e24dcca9e";    // Notify (LABEL JSONL)

// ===== Binary frame (for Pi) =====
struct __attribute__((packed)) FrameHeader {
  uint8_t node_id;
  uint8_t session_id;
  uint16_t seq;
  uint16_t nsamples;
  uint8_t flags;
};
struct __attribute__((packed)) ImuSample {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

// ===== Globals =====
bool g_streaming = false;
bool g_rateChangedFlag = false;
uint8_t g_sessionId = 1;
uint16_t g_seq = 0;
uint16_t g_sampleHz = DEFAULT_SAMPLE_HZ;
uint32_t g_samplePeriodUs = 1000000UL / DEFAULT_SAMPLE_HZ;

// runtime toggles
bool g_rawOn = RAW_NOTIFY_DEFAULT;        // BLE RAW feed
bool g_binOn = BIN_NOTIFY_DEFAULT;        // Binary feed
bool g_serialRawOn = SERIAL_RAW_DEFAULT;  // Serial RAW feed

// Binary ring
struct Ring {
  ImuSample buf[RING_CAPACITY];
  uint16_t head = 0, tail = 0;
  bool overflow = false;
} g_ring;

// HAR windowing
#define FS_HZ DEFAULT_SAMPLE_HZ
#define WIN_SEC 0.8f
#define WIN_N (uint16_t)(FS_HZ * WIN_SEC)  // 80
#define HOP_N 20                           // 0.2 s hop
#define CAL_SEC 0.5f
#define CAL_N (uint16_t)(FS_HZ * CAL_SEC)

float g_ax[WIN_N], g_ay[WIN_N], g_az[WIN_N];
float g_bp[WIN_N];  // band‑passed |a|-g_ref
uint16_t g_wr = 0, g_sinceUpdate = 0;
uint32_t g_lastSample = 0;
float g_ref = 1.0f;
bool g_calDone = false;
uint16_t g_calCount = 0;
float g_calSum = 0.0f;

// One‑pole filters
struct OnePole {
  float a = 0, y = 0;
  void setAlpha(float alpha) {
    a = constrain(alpha, 0.0f, 1.0f);
  }
  float step(float x) {
    y = a * x + (1 - a) * y;
    return y;
  }
} hp, lp;

// Debounce state
const uint8_t VOTES_NEEDED = 2;  // consecutive votes to switch label
const char* g_lastLabel = "Sitting/Idle";
uint8_t g_voteCount = 0;
const char* g_voteCandidate = "Sitting/Idle";

// BLE objects
BLEService svc(UUID_SVC_MOV);
BLECharacteristic cBIN(UUID_CHAR_BIN_IMU, BLENotify, 244, true);
BLECharacteristic cCTL(UUID_CHAR_CTRL, BLEWrite, 32, true);
BLECharacteristic cINF(UUID_CHAR_INFO, BLERead, 64, true);
BLECharacteristic cRAW(UUID_CHAR_RAW_ASC, BLENotify, 180, true);
BLECharacteristic cLAB(UUID_CHAR_LABEL, BLENotify, 128, true);

// ===== Utils =====
static inline long clamp_i32(long v, long lo, long hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static inline int16_t to_i16(float v, float scale) {
  long val = lroundf(v * scale);
  return (int16_t)clamp_i32(val, -32768, 32767);
}
static inline float fsqr(float v) {
  return v * v;
}
static inline float fsqrtp(float v) {
  return sqrtf(v < 0 ? 0 : v);
}
static inline uint16_t crc16_ccitt(const uint8_t* d, size_t n, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)d[i] << 8;
    for (uint8_t j = 0; j < 8; j++) { crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1); }
  }
  return crc;
}
static inline void ring_push(const ImuSample& s) {
  uint16_t nx = (g_ring.head + 1) % RING_CAPACITY;
  if (nx == g_ring.tail) {
    g_ring.tail = (g_ring.tail + 1) % RING_CAPACITY;
    g_ring.overflow = true;
  }
  g_ring.buf[g_ring.head] = s;
  g_ring.head = nx;
}
static inline uint16_t ring_pop(ImuSample* dst, uint16_t maxN) {
  uint16_t c = 0;
  while (g_ring.tail != g_ring.head && c < maxN) {
    dst[c++] = g_ring.buf[g_ring.tail];
    g_ring.tail = (g_ring.tail + 1) % RING_CAPACITY;
  }
  return c;
}

static void updateInfo() {
  char info[64];
  snprintf(info, sizeof(info), "Node:%u;Session:%u;Rate:%uHz;FW:1.4", (unsigned)NODE_ID, (unsigned)g_sessionId, (unsigned)g_sampleHz);
  cINF.setValue((const unsigned char*)info, strlen(info));
}

// ===== Commands =====
static void applyTextCommand(const char* s) {
  // lowercase copy
  char cmd[32];
  size_t n = strnlen(s, sizeof(cmd) - 1);
  for (size_t i = 0; i < n; i++) {
    char ch = s[i];
    if (ch >= 'A' && ch <= 'Z') ch = ch - 'A' + 'a';
    cmd[i] = ch;
  }
  cmd[n] = 0;

  if (!strcmp(cmd, "start")) {
    g_streaming = true;
    Serial.println("STREAM=ON");
    return;
  }
  if (!strcmp(cmd, "stop")) {
    g_streaming = false;
    Serial.println("STREAM=OFF");
    return;
  }

  if (!strncmp(cmd, "rate:", 5)) {
    int hz = atoi(cmd + 5);
    if (hz >= 25 && hz <= 200) {
      g_sampleHz = hz;
      g_samplePeriodUs = 1000000UL / g_sampleHz;
      g_rateChangedFlag = true;
      Serial.print("RATE=");
      Serial.println(hz);
    }
    return;
  }

  if (!strcmp(cmd, "raw:on")) {
    g_rawOn = true;
    Serial.println("RAW=ON");
    return;
  }
  if (!strcmp(cmd, "raw:off")) {
    g_rawOn = false;
    Serial.println("RAW=OFF");
    return;
  }
  if (!strcmp(cmd, "bin:on")) {
    g_binOn = true;
    Serial.println("BIN=ON");
    return;
  }
  if (!strcmp(cmd, "bin:off")) {
    g_binOn = false;
    Serial.println("BIN=OFF");
    return;
  }
  if (!strcmp(cmd, "serial:on")) {
    g_serialRawOn = true;
    Serial.println("SERIAL_RAW=ON");
    return;
  }
  if (!strcmp(cmd, "serial:off")) {
    g_serialRawOn = false;
    Serial.println("SERIAL_RAW=OFF");
    return;
  }
  if (!strcmp(cmd, "help")) {
    Serial.println("CMDS: start/stop, rate:<25..200>, raw:on/off, bin:on/off, serial:on/off");
    return;
  }
}

static void handleControlWrite(BLEDevice, BLECharacteristic) {
  uint8_t buf[32];
  int n = cCTL.readValue(buf, sizeof(buf));
  if (n <= 0) return;
  // ASCII vs legacy binary
  bool ascii = true;
  for (int i = 0; i < n; i++) {
    if (buf[i] == 0) {
      ascii = false;
      break;
    }
  }
  if (ascii) {
    char tmp[32];
    int m = min(n, 31);
    memcpy(tmp, buf, m);
    tmp[m] = 0;
    applyTextCommand(tmp);
    return;
  }
  if (n >= 3) {
    uint8_t cmd = buf[0];
    uint16_t newHz = (uint16_t)(buf[1] | (buf[2] << 8));
    if (cmd == 0) {
      g_streaming = false;
    } else if (cmd == 1) {
      g_streaming = true;
    } else if (cmd == 2 && newHz >= 25 && newHz <= 200) {
      g_sampleHz = newHz;
      g_samplePeriodUs = 1000000UL / g_sampleHz;
      g_rateChangedFlag = true;
    }
  }
}

static void onSubscribed(BLEDevice, BLECharacteristic ch) {
  if (ch.uuid() == UUID_CHAR_LABEL) g_streaming = true;
}
static void onUnsubscribed(BLEDevice, BLECharacteristic ch) {
  if (ch.uuid() == UUID_CHAR_LABEL) g_streaming = false;
}

// ===== Sampling & Features =====
static void send_raw_if_enabled(uint32_t t, float ax, float ay, float az, float gx, float gy, float gz) {
  if (g_serialRawOn) {
    Serial.print(t);
    Serial.print(',');
    Serial.print((int)NODE_ID);
    Serial.print(',');
    Serial.print(ax, 4);
    Serial.print(',');
    Serial.print(ay, 4);
    Serial.print(',');
    Serial.print(az, 4);
    Serial.print(',');
    Serial.print(gx, 3);
    Serial.print(',');
    Serial.print(gy, 3);
    Serial.print(',');
    Serial.println(gz, 3);
  }
  if (g_rawOn) {
    char line[128];
    int k = snprintf(line, sizeof(line), "%lu,%u,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f", (unsigned long)t, (unsigned)NODE_ID, ax, ay, az, gx, gy, gz);
    if (k > 0) cRAW.setValue((const unsigned char*)line, (int)strlen(line));
  }
}

static void classify_and_publish(float gx, float gy, float gz) {
  // --- Tunable parameters ---
  const float RAW_STD_IDLE = 0.020f;      // g (micro-window idle snap)
  const float RAW_STD_WALK_MIN = 0.035f;  // g (helps gate walking)
  const float RAW_STD_RUN_MIN = 0.120f;   // g (very “active”)
  const float GYRO_IDLE_RMS = 3.0f;       // dps

  const float STEP_HZ_WALK_MIN = 0.8f;  // Hz
  const float STEP_HZ_WALK_MAX = 2.4f;  // Hz
  const float STEP_HZ_RUN_MIN = 2.8f;   // Hz  (gap 2.4–2.8 to avoid flaps)

  const uint16_t REFRACTORY_S = (uint16_t)(0.25f * FS_HZ);  // 250 ms in samples

  // Debounce: how many consecutive votes to adopt a class
  const uint8_t VOTE_IDLE_ENTER = 1;
  const uint8_t VOTE_WALK_ENTER = 2;
  const uint8_t VOTE_RUN_ENTER = 3;

  // --- Stats over full window (0.8 s) ---
  float rawMean = 0, rawVar = 0;
  float bp_sq_sum = 0.0f;
  uint16_t peaks = 0;
  float prev = 0, cur = 0, nxt = 0;

  // Count peaks with refractory
  int last_peak_idx = -10000;

  for (uint16_t i = 0; i < WIN_N; ++i) {
    uint16_t idx = (g_wr + i) % WIN_N;

    float x = g_ax[idx], y = g_ay[idx], z = g_az[idx];
    float rm = fsqrtp(fsqr(x) + fsqr(y) + fsqr(z));
    rawMean += rm;

    float bp = g_bp[idx];
    bp_sq_sum += bp * bp;

    if (i > 0) prev = g_bp[(idx + WIN_N - 1) % WIN_N];
    if (i < WIN_N - 1) nxt = g_bp[(idx + 1) % WIN_N];

    if (i > 0 && i < WIN_N - 1) {
      // basic local max
      if (bp > prev && bp > nxt) {
        // enforce refractory
        if ((int)i - last_peak_idx >= REFRACTORY_S) {
          // candidate; we’ll threshold after we know bp_rms
          // temporarily count; we may prune below if under dynamic threshold
          peaks++;
          last_peak_idx = i;
        }
      }
    }
  }
  rawMean /= WIN_N;

  for (uint16_t i = 0; i < WIN_N; ++i) {
    uint16_t idx = (g_wr + i) % WIN_N;
    float rm = fsqrtp(fsqr(g_ax[idx]) + fsqr(g_ay[idx]) + fsqr(g_az[idx]));
    rawVar += fsqr(rm - rawMean);
  }
  rawVar /= (WIN_N - 1);
  float rawStd = fsqrtp(rawVar);

  // Dynamic step thresholding by band-passed RMS
  float bp_rms = fsqrtp(bp_sq_sum / WIN_N);
  float dyn_peak_thr = max(0.06f, 0.5f * bp_rms);

  // Recount peaks applying dynamic threshold & refractory properly
  peaks = 0;
  last_peak_idx = -10000;
  for (uint16_t i = 1; i < WIN_N - 1; ++i) {
    uint16_t idx = (g_wr + i) % WIN_N;
    float p = g_bp[idx];
    float pPrev = g_bp[(idx + WIN_N - 1) % WIN_N];
    float pNext = g_bp[(idx + 1) % WIN_N];

    if (p > pPrev && p > pNext && p > dyn_peak_thr) {
      if ((int)i - last_peak_idx >= REFRACTORY_S) {
        peaks++;
        last_peak_idx = i;
      }
    }
  }

  float stepHz = (float)peaks / WIN_SEC;

  // Gyro RMS from latest sample trio (cheap proxy for turning/shaking)
  float gyro_rms = fsqrtp((fsqr(gx) + fsqr(gy) + fsqr(gz)) / 3.0f);

  // --- Micro-window idle snap (0.3 s) ---
  // compute std(|a|) and gyro RMS over last ~0.3 s quickly
  const uint16_t MICRO_N = (uint16_t)(0.3f * FS_HZ);  // 30 samples at 100 Hz
  float microMean = 0, microVar = 0, microGyro = 0;
  for (uint16_t k = 0; k < MICRO_N; ++k) {
    uint16_t idx = (g_wr + WIN_N - 1 - k + WIN_N) % WIN_N;
    float x = g_ax[idx], y = g_ay[idx], z = g_az[idx];
    float rm = fsqrtp(fsqr(x) + fsqr(y) + fsqr(z));
    microMean += rm;
  }
  microMean /= MICRO_N;
  for (uint16_t k = 0; k < MICRO_N; ++k) {
    uint16_t idx = (g_wr + WIN_N - 1 - k + WIN_N) % WIN_N;
    float x = g_ax[idx], y = g_ay[idx], z = g_az[idx];
    float rm = fsqrtp(fsqr(x) + fsqr(y) + fsqr(z));
    microVar += fsqr(rm - microMean);
  }
  microVar /= (MICRO_N - 1);
  float microStd = fsqrtp(microVar);

  // use the latest gyro_rms (already computed) as micro gyro proxy
  bool idle_micro = (microStd < RAW_STD_IDLE && gyro_rms < GYRO_IDLE_RMS);

  // --- Voting with hysteresis ---
  const char* vote = g_lastLabel;  // default to current, avoid flaps

  if (microStd < 0.02f && gyro_rms < 3.0f) {
    vote = "Sitting/Idle";
  } else {
    // Use mean/std bands first
    if (rawMean >= 1.45f && rawStd >= 0.65f) {
      vote = "Running";
    } else if (rawMean >= 1.15f && rawStd >= 0.25f) {
      vote = "Walking";
    } else {
      // borderline / transitional zones
      if (stepHz > 2.5f) vote = "Running";
      else if (stepHz > 0.8f) vote = "Walking";
      else vote = "Sitting/Idle";
    }
  }

  // Hysteresis (same as before)
  uint8_t need = (!strcmp(vote, "Sitting/Idle")) ? 1 : (!strcmp(vote, "Walking")) ? 2
                                                                                  : 3;
  if (vote == g_voteCandidate) {
    if (g_voteCount < 255) g_voteCount++;
  } else {
    g_voteCandidate = vote;
    g_voteCount = 1;
  }
  if (g_voteCandidate != g_lastLabel && g_voteCount >= need) {
    g_lastLabel = g_voteCandidate;
  }

  // Emit LABEL JSONL
  uint32_t t = millis();
  char json[160];
  int k = snprintf(
    json, sizeof(json),
    "{\"t\":%lu,\"node\":%u,\"session\":%u,\"label\":\"%s\",\"stepHz\":%.2f,\"std\":%.3f,\"mean\":%.3f}",
    (unsigned long)t, (unsigned)NODE_ID, (unsigned)g_sessionId,
    g_lastLabel, stepHz, rawStd, rawMean);
  if (k > 0) {
#if SERIAL_LABELS
    Serial.println(json);
#endif
#if LABEL_NOTIFY
    cLAB.setValue((const unsigned char*)json, (int)strlen(json));
#endif
  }
}

static void sample_and_process() {
  float ax, ay, az, gx, gy, gz;
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;
  if (!IMU.readAcceleration(ax, ay, az)) return;
  if (!IMU.readGyroscope(gx, gy, gz)) return;

  float m = fsqrtp(fsqr(ax) + fsqr(ay) + fsqr(az));
  if (!g_calDone) {
    g_calSum += m;
    if (++g_calCount >= CAL_N) {
      g_ref = g_calSum / g_calCount;
      g_calDone = true;
    }
  }
  float hpOut = (m - g_ref);
  hpOut = hp.step(hpOut);
  float bp = lp.step(hpOut);
  g_ax[g_wr] = ax;
  g_ay[g_wr] = ay;
  g_az[g_wr] = az;
  g_bp[g_wr] = bp;
  g_wr = (g_wr + 1) % WIN_N;

  if (++g_sinceUpdate >= HOP_N) {
    g_sinceUpdate = 0;
    uint32_t t = millis();
    classify_and_publish(gx, gy, gz);
    send_raw_if_enabled(t, ax, ay, az, gx, gy, gz);
  }

  if (g_binOn) {
    ImuSample s;
    s.ax = to_i16(ax, 1000.0f);
    s.ay = to_i16(ay, 1000.0f);
    s.az = to_i16(az, 1000.0f);
    s.gx = to_i16(gx, 1000.0f);
    s.gy = to_i16(gy, 1000.0f);
    s.gz = to_i16(gz, 1000.0f);
    ring_push(s);
  }
}

// ===== Setup / Loop =====
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 2000) {}
  if (!IMU.begin()) {
    Serial.println("IMU init failed");
    while (1) { delay(1000); }
  }
  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1) { delay(1000); }
  }

  const char* name = (NODE_ID == 1) ? "AMS-Wrist" : "AMS-Ankle";
  BLE.setLocalName(name);
  BLE.setDeviceName(name);
  BLE.setAdvertisedService(svc);
  svc.addCharacteristic(cBIN);
  svc.addCharacteristic(cCTL);
  svc.addCharacteristic(cINF);
  svc.addCharacteristic(cRAW);
  svc.addCharacteristic(cLAB);
  BLE.addService(svc);
  cCTL.setEventHandler(BLEWritten, handleControlWrite);
  cLAB.setEventHandler(BLESubscribed, onSubscribed);
  cLAB.setEventHandler(BLEUnsubscribed, onUnsubscribed);
  updateInfo();
  BLE.advertise();

  // Band ~0.3–5 Hz for step detection
  float alphaHP = expf(-2.0f * PI * 0.3f / DEFAULT_SAMPLE_HZ);
  float alphaLP = expf(-2.0f * PI * 5.0f / DEFAULT_SAMPLE_HZ);
  hp.setAlpha(alphaHP);
  lp.setAlpha(1.0f - alphaLP);

  g_lastSample = micros();
  g_calDone = false;
  g_calCount = 0;
  g_calSum = 0;
  Serial.println("HAR node ready — LABEL JSONL by default; send 'help' to CTRL or Serial.");
}

void loop() {
  BLE.poll();

  // Serial text command parser
  while (Serial.available()) {
    static char sbuf[32];
    static uint8_t sidx = 0;
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (sidx) {
        sbuf[sidx] = 0;  // terminate string
        applyTextCommand(sbuf);
        sidx = 0;
      }
    } else {
      if (sidx < sizeof(sbuf) - 1) {
        sbuf[sidx++] = ch;
      }
    }
  }


  static uint32_t lastInfo = 0;
  if (millis() - lastInfo > 2000) {
    updateInfo();
    lastInfo = millis();
  }

  if (!g_streaming) {
    delay(2);
    return;
  }
  uint32_t now = micros();
  if ((now - g_lastSample) >= g_samplePeriodUs) {
    g_lastSample += g_samplePeriodUs;
    digitalWrite(LED_PIN, HIGH);
    sample_and_process();
    digitalWrite(LED_PIN, LOW);
  }

  // ship binary batches when enabled
  if (g_binOn) {
    static ImuSample batch[BATCH_SAMPLES];
    uint16_t got = ring_pop(batch, BATCH_SAMPLES);
    if (got > 0) {
      FrameHeader hdr;
      hdr.node_id = NODE_ID;
      hdr.session_id = g_sessionId;
      hdr.seq = g_seq++;
      hdr.nsamples = got;
      hdr.flags = 0;
      if (g_ring.overflow) {
        hdr.flags |= 0x01;
        g_ring.overflow = false;
      }
      if (g_rateChangedFlag) {
        hdr.flags |= 0x02;
        g_rateChangedFlag = false;
      }
      const size_t payloadLen = sizeof(FrameHeader) + got * sizeof(ImuSample);
      static uint8_t buf[sizeof(FrameHeader) + BATCH_SAMPLES * sizeof(ImuSample) + 2];
      memcpy(buf, &hdr, sizeof(FrameHeader));
      memcpy(buf + sizeof(FrameHeader), batch, got * sizeof(ImuSample));
      uint16_t crc = crc16_ccitt(buf, payloadLen);
      buf[payloadLen + 0] = (uint8_t)(crc & 0xFF);
      buf[payloadLen + 1] = (uint8_t)(crc >> 8);
      cBIN.setValue(buf, payloadLen + 2);
    }
  }
}
