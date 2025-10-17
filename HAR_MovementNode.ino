/*
  HAR_MovementNode.ino — Wrist "Movement" Node
  Board: Arduino Nano 33 IoT (IMU: LSM6DS3)
  Role: Stream smoothed IMU (accel+gyro) over BLE in batched binary frames
         with sequence numbers and CRC16 for the Raspberry Pi (central) to ingest.

  Dependencies (Arduino Library Manager):
    - ArduinoBLE (by Arduino)
    - Arduino_LSM6DS3 (by Arduino)

  Notes:
    - Proposal specifies Nano 33 IoT @ 100 Hz and batched 16-bit IMU frames.
    - This sketch targets the Nano 33 IoT’s LSM6DS3 (6‑axis) IMU.
    - Configure NODE_ID below to 1 (Wrist) or 2 (Ankle). David can reuse this file.

  Frame Layout (little‑endian):
    Header (8 bytes)
      uint8_t  node_id;      // 1 = Wrist, 2 = Ankle
      uint8_t  session_id;   // set by Pi or defaults to 1
      uint16_t seq;          // increments per frame
      uint16_t nsamples;     // number of samples in payload
      uint8_t  flags;        // bit0: overflow_dropped, bit1: rate_change

    Samples (nsamples * 12 bytes)
      int16_t ax, ay, az;    // accelerometer (scaled to ±4 g range)
      int16_t gx, gy, gz;    // gyroscope    (scaled to ±500 dps nominal)

    Trailer (2 bytes)
      uint16_t crc16_ccitt;  // over header+samples

  Control Characteristic (Write, little‑endian):
    struct Ctrl { uint8_t cmd; uint16_t sample_hz; };
    cmd: 0=STOP, 1=START, 2=SET_RATE

  Author: Brandon’s HAR build — movement node
*/

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>

// ======== Build-Time Config ========
#define NODE_ID 1              // 1=Wrist, 2=Ankle
#define DEFAULT_SAMPLE_HZ 100  // proposal target
#define BATCH_SAMPLES 10       // samples per BLE notification (10 => ~10 Hz frames)
#define RING_CAPACITY 256      // samples buffered (>= 2.5 s at 100 Hz)
#define LED_PIN LED_BUILTIN

// BLE UUIDs (placeholder deterministic but unique — replace if needed)
static const char* UUID_SVC_MOV = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";    // custom service
static const char* UUID_CHAR_IMU = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";   // Notify
static const char* UUID_CHAR_CTRL = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";  // Write
static const char* UUID_CHAR_INFO = "6e400004-b5a3-f393-e0a9-e50e24dcca9e";  // Read

// ======== Data Types ========
struct __attribute__((packed)) FrameHeader {
  uint8_t node_id;
  uint8_t session_id;
  uint16_t seq;
  uint16_t nsamples;
  uint8_t flags;  // bit0: overflow_dropped, bit1: rate_change
};

struct __attribute__((packed)) ImuSample {
  int16_t ax, ay, az;  // milli-g scaled to int16
  int16_t gx, gy, gz;  // milli-dps scaled to int16
};

// ======== Globals ========
static volatile bool g_streaming = false;
static volatile bool g_rateChangedFlag = false;
static uint8_t g_sessionId = 1;
static uint16_t g_seq = 0;
static uint16_t g_sampleHz = DEFAULT_SAMPLE_HZ;
static uint32_t g_samplePeriodUs = 1000000UL / DEFAULT_SAMPLE_HZ;

// Moving average (boxcar) — light smoothing
static const uint8_t MA_WINDOW = 5;  // 3–5 recommended
struct FiltState {
  float ax[MA_WINDOW];
  float ay[MA_WINDOW];
  float az[MA_WINDOW];
  float gx[MA_WINDOW];
  float gy[MA_WINDOW];
  float gz[MA_WINDOW];
  uint8_t idx = 0;
  uint8_t fill = 0;
};
static FiltState g_filt;

// Ring buffer for samples
struct Ring {
  ImuSample buf[RING_CAPACITY];
  uint16_t head = 0;  // write index
  uint16_t tail = 0;  // read index
  bool overflow = false;
} g_ring;

// BLE objects
BLEService movementService(UUID_SVC_MOV);
BLECharacteristic charImu(UUID_CHAR_IMU, BLENotify, 244, true);  // value len adapted at runtime
BLECharacteristic charCtrl(UUID_CHAR_CTRL, BLEWrite, 8, true);
BLECharacteristic charInfo(UUID_CHAR_INFO, BLERead, 32, true);

// ======== Utility ========
static inline uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

static inline void ledBlink(uint8_t times, uint16_t on = 60, uint16_t off = 80) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(on);
    digitalWrite(LED_PIN, LOW);
    delay(off);
  }
}

// Push sample into ring; drop oldest if full
static void ring_push(const ImuSample& s) {
  uint16_t next = (g_ring.head + 1) % RING_CAPACITY;
  if (next == g_ring.tail) {
    // buffer full -> drop oldest
    g_ring.tail = (g_ring.tail + 1) % RING_CAPACITY;
    g_ring.overflow = true;
  }
  g_ring.buf[g_ring.head] = s;
  g_ring.head = next;
}

// Pop up to N samples from ring into dst, return count
static uint16_t ring_pop(ImuSample* dst, uint16_t maxN) {
  uint16_t count = 0;
  while (g_ring.tail != g_ring.head && count < maxN) {
    dst[count++] = g_ring.buf[g_ring.tail];
    g_ring.tail = (g_ring.tail + 1) % RING_CAPACITY;
  }
  return count;
}

// Simple moving average filter
static inline float ma_acc(float* win, uint8_t& idx, uint8_t& fill, float v) {
  win[idx] = v;
  idx = (idx + 1) % MA_WINDOW;
  if (fill < MA_WINDOW) fill++;
  float sum = 0.0f;
  for (uint8_t i = 0; i < fill; i++) sum += win[i];
  return sum / (float)fill;
}

// Scale floats to int16 fixed units (mg, mdps) with clamping
static inline int16_t to_i16(float v, float scale) {
  long val = lroundf(v * scale);
  if (val > 32767) val = 32767;
  else if (val < -32768) val = -32768;
  return (int16_t)val;
}

// ======== Sampling ========
static void sample_once() {
  float ax, ay, az, gx, gy, gz;
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;
  if (!IMU.readAcceleration(ax, ay, az)) return;  // g
  if (!IMU.readGyroscope(gx, gy, gz)) return;     // dps

  // Light smoothing
  float sax = ma_acc(g_filt.ax, g_filt.idx, g_filt.fill, ax);
  float say = ma_acc(g_filt.ay, g_filt.idx, g_filt.fill, ay);
  float saz = ma_acc(g_filt.az, g_filt.idx, g_filt.fill, az);
  float sgx = ma_acc(g_filt.gx, g_filt.idx, g_filt.fill, gx);
  float sgy = ma_acc(g_filt.gy, g_filt.idx, g_filt.fill, gy);
  float sgz = ma_acc(g_filt.gz, g_filt.idx, g_filt.fill, gz);

  // Convert to milli‑units and pack
  ImuSample s;
  s.ax = to_i16(sax, 1000.0f);  // g → mg
  s.ay = to_i16(say, 1000.0f);
  s.az = to_i16(saz, 1000.0f);
  s.gx = to_i16(sgx, 1000.0f);  // dps → mdps
  s.gy = to_i16(sgy, 1000.0f);
  s.gz = to_i16(sgz, 1000.0f);

  ring_push(s);
}

// ======== BLE Handling ========
static void handleControlWrite() {
  // Expect 3 bytes: cmd (u8), sample_hz (u16)
  int len = charCtrl.valueLength();
  if (len < 1) return;
  uint8_t buf[8];
  int n = charCtrl.readValue(buf, sizeof(buf));
  if (n <= 0) return;
  uint8_t cmd = buf[0];
  uint16_t newHz = g_sampleHz;
  if (n >= 3) newHz = (uint16_t)(buf[1] | (buf[2] << 8));

  switch (cmd) {
    case 0:  // STOP
      g_streaming = false;
      digitalWrite(LED_PIN, LOW);
      break;
    case 1:  // START
      g_streaming = true;
      break;
    case 2:  // SET_RATE
      if (newHz >= 25 && newHz <= 200) {
        g_sampleHz = newHz;
        g_samplePeriodUs = 1000000UL / g_sampleHz;
        g_rateChangedFlag = true;
      }
      break;
    default: break;
  }
}

static void updateInfoCharacteristic() {
  char info[32];
  snprintf(info, sizeof(info), "Node:%u;Rate:%uHz;FW:1.0", (unsigned)NODE_ID, (unsigned)g_sampleHz);
  charInfo.setValue((const unsigned char*)info, strlen(info));
}

// ======== Setup ========
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  while (!Serial && millis() < 2000) { ; }

  // IMU init (LSM6DS3 on Nano 33 IoT)
  if (!IMU.begin()) {
    Serial.println("IMU init failed");
    ledBlink(5, 80, 120);
    while (1) { delay(1000); }
  }
  // Library defaults are ~±4 g and ±2000 dps. We'll scale to mg/mdps and clamp.

  // BLE init
  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    ledBlink(10, 50, 50);
    while (1) { delay(1000); }
  }
  
  const char* localName = (NODE_ID == 1) ? "AMS-Wrist" : "AMS-Ankle";
  BLE.setLocalName(localName);
  BLE.setDeviceName(localName);

  BLE.setAdvertisedService(movementService);

  movementService.addCharacteristic(charImu);
  movementService.addCharacteristic(charCtrl);
  movementService.addCharacteristic(charInfo);
  BLE.addService(movementService);

  // Set properties
  charImu.setValue((const unsigned char*)"", 0);
  charCtrl.setEventHandler(BLEWritten, [](BLEDevice central, BLECharacteristic) {
    handleControlWrite();
  });
  updateInfoCharacteristic();

  BLE.advertise();
  Serial.println("BLE advertising; waiting for central...");
}

// ======== Loop ========
void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected: ");
    Serial.println(central.address());
    g_seq = 0;
    g_ring.head = g_ring.tail = 0;
    g_ring.overflow = false;
    g_streaming = false;

    while (central.connected()) {
      BLE.poll();  // process events (incl. control writes)

      // Update INFO every ~2s to reflect rate changes
      static uint32_t lastInfo = 0;
      if (millis() - lastInfo > 2000) {
        updateInfoCharacteristic();
        lastInfo = millis();
      }

      // START/STOP driven by control characteristic
      static uint32_t lastSampleUs = micros();
      if (g_streaming) {
        uint32_t now = micros();
        if ((now - lastSampleUs) >= g_samplePeriodUs) {
          lastSampleUs += g_samplePeriodUs;  // keep cadence
          sample_once();
        }

        // If enough samples accumulated, build and send a frame
        static ImuSample batch[BATCH_SAMPLES];
        uint16_t got = ring_pop(batch, BATCH_SAMPLES);
        if (got > 0) {
          // Build header
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

          // Compose payload in a temp buffer (header + samples + crc)
          const size_t payloadLen = sizeof(FrameHeader) + got * sizeof(ImuSample);
          static uint8_t buf[sizeof(FrameHeader) + BATCH_SAMPLES * sizeof(ImuSample) + 2];
          memcpy(buf, &hdr, sizeof(FrameHeader));
          memcpy(buf + sizeof(FrameHeader), batch, got * sizeof(ImuSample));
          uint16_t crc = crc16_ccitt(buf, payloadLen);
          buf[payloadLen + 0] = (uint8_t)(crc & 0xFF);
          buf[payloadLen + 1] = (uint8_t)(crc >> 8);

          // Notify (central must have subscribed). Keep LED pulsing when streaming
          digitalWrite(LED_PIN, HIGH);
          charImu.setValue(buf, payloadLen + 2);
          digitalWrite(LED_PIN, LOW);
        }
      } else {
        delay(5);  // idle
      }
    }

    Serial.println("Disconnected");
    BLE.advertise();
  }
}
