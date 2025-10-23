/*
  HAR_MovementNode_MINMTU.ino
  Nano 33 IoT (nRF52840) — BLE notifications trimmed to fit 20-byte MTU.

  BLE Characteristics (all under service 6e400001-...-dcca9e):
    0006 (LABEL, Notify, len<=20): "IDLE,1.01,0.03" | "WALK,1.23,0.45" | "RUN,1.58,0.78"
    0005 (RAW,   Notify, len=12) : int16 ax,ay,az (mg), gx,gy,gz (mdps) — little endian
    0003 (CTRL,  Write)         : text:  start/stop/raw:on/raw:off/rate:<25..200>
                                  binary: <cmd:1 byte, hz:le uint16>  (1=start, 0=stop, 2=set rate)
    0004 (INFO,  Read)          : short info line

  Notes:
  - Keep notifications ≤20 bytes → works on phones & Windows without MTU negotiation.
  - Serial prints JSON LABEL (optional) for debugging at 115200.
*/

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>

#define SERIAL_BAUD     115200
#define NODE_ID         1              // 1 = Wrist, 2 = Ankle
#define FS_DEFAULT      100            // Hz
#define LEDPIN          LED_BUILTIN

// ---- BLE UUIDs (Nordic-like layout kept from your previous design) ----
static const char* UUID_SVC_MOV       = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char* UUID_CHAR_BIN_IMU  = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"; // UNUSED in this minimal build
static const char* UUID_CHAR_CTRL     = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
static const char* UUID_CHAR_INFO     = "6e400004-b5a3-f393-e0a9-e50e24dcca9e";
static const char* UUID_CHAR_RAW      = "6e400005-b5a3-f393-e0a9-e50e24dcca9e"; // 12-byte binary sample
static const char* UUID_CHAR_LABEL    = "6e400006-b5a3-f393-e0a9-e50e24dcca9e"; // short ASCII <=20 bytes

BLEService        svc (UUID_SVC_MOV);
// maxLen = 20 so we never exceed default MTU
BLECharacteristic cRAW (UUID_CHAR_RAW,   BLENotify,  20, true);
BLECharacteristic cLAB (UUID_CHAR_LABEL, BLENotify,  20, true);
BLECharacteristic cCTL (UUID_CHAR_CTRL,  BLEWrite,   32, true);
BLECharacteristic cINF (UUID_CHAR_INFO,  BLERead,    64, true);

// ---- Runtime state ----
static volatile bool    g_streaming   = false;
static volatile bool    g_rawOn       = false;   // enable RAW 12-byte samples
static uint16_t         g_fs          = FS_DEFAULT;
static uint32_t         g_period_us   = 1000000UL / FS_DEFAULT;
static uint8_t          g_session     = 1;

static uint32_t         g_last_us     = 0;
static bool             g_labelSubd   = false;   // autostart when true

// ========== Simple stats / classifier (kept concise) ==========
#define WIN_SEC  0.8f
#define HOP_SEC  0.2f
#define FS_MAX   200
#define WIN_N    (uint16_t)(FS_DEFAULT*WIN_SEC)  // 80 at 100Hz
#define HOP_N    (uint16_t)(FS_DEFAULT*HOP_SEC)  // 20 at 100Hz

static float axbuf[WIN_N], aybuf[WIN_N], azbuf[WIN_N];
static uint16_t wr = 0, hopCount = 0;

static inline float fsqr(float x){ return x*x; }
static inline float fmag(float x,float y,float z){ return sqrtf(fsqr(x)+fsqr(y)+fsqr(z)); }

// thresholds tuned from your empirical notes
static const float STD_IDLE_MAX  = 0.05f;
static const float MEAN_WALK_MAX = 1.30f;
static const float STD_WALK_MIN  = 0.20f;
static const float STD_RUN_MIN   = 0.70f;
static const float MEAN_RUN_MIN  = 1.50f;

// ========== Helpers ==========
static inline int16_t to_i16(float v, float scale){ long s = lroundf(v*scale); if(s<-32768) s=-32768; if(s>32767) s=32767; return (int16_t)s; }

static void updateInfo(){
  char info[64];
  snprintf(info,sizeof(info),"Node:%u;Sess:%u;Rate:%uHz;FW:MINMTU",
           (unsigned)NODE_ID, (unsigned)g_session, (unsigned)g_fs);
  cINF.setValue((const unsigned char*)info, strlen(info));
}

// ---- CTRL handling ----
static void apply_cmd_text(const char* s){
  if(!strcmp(s,"start")) { g_streaming=true;  }
  else if(!strcmp(s,"stop"))  { g_streaming=false; }
  else if(!strcmp(s,"raw:on")){ g_rawOn=true;  }
  else if(!strcmp(s,"raw:off")){g_rawOn=false; }
  else if(!strncmp(s,"rate:",5)){
    int hz = atoi(s+5); if(hz>=25 && hz<=200){ g_fs = hz; g_period_us = 1000000UL / g_fs; }
  }
}

static void onCTRL(BLEDevice, BLECharacteristic){
  uint8_t buf[32]; int n = cCTL.readValue(buf,sizeof(buf));
  if(n<=0) return;

  // ASCII?
  bool ascii=true; for(int i=0;i<n;i++){ if(buf[i]==0){ ascii=false; break; } }
  if(ascii){
    char cmd[32]; int m = (n>31)?31:n; memcpy(cmd,buf,m); cmd[m]=0;
    // lowercase
    for(int i=0;i<m;i++){ if(cmd[i]>='A'&&cmd[i]<='Z') cmd[i] = cmd[i]-'A'+'a'; }
    apply_cmd_text(cmd);
    return;
  }
  // legacy 3-byte binary
  if(n>=3){
    uint8_t  cmd = buf[0];
    uint16_t hz  = (uint16_t)(buf[1] | (buf[2]<<8));
    if(cmd==0){ g_streaming=false; }
    else if(cmd==1){ g_streaming=true; }
    else if(cmd==2 && hz>=25 && hz<=200){ g_fs=hz; g_period_us=1000000UL/g_fs; }
  }
}

static void onLABsub(BLEDevice, BLECharacteristic ch){
  if(ch.subscribed()){
    g_labelSubd = true;
    // Auto-start streaming when LABEL is subscribed
    g_streaming = true;
  } else {
    g_labelSubd = false;
  }
}

// ========== Setup ==========
void setup(){
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  Serial.begin(SERIAL_BAUD);
  while(!Serial && millis()<1500){}

  if(!IMU.begin()){
    Serial.println("IMU init failed"); while(1){ delay(1000); }
  }
  if(!BLE.begin()){
    Serial.println("BLE init failed"); while(1){ delay(1000); }
  }

  const char* name = (NODE_ID==1) ? "AMS-Wrist" : "AMS-Ankle";
  BLE.setLocalName(name);
  BLE.setDeviceName(name);
  BLE.setAdvertisedService(svc);

  svc.addCharacteristic(cRAW);
  svc.addCharacteristic(cLAB);
  svc.addCharacteristic(cCTL);
  svc.addCharacteristic(cINF);
  BLE.addService(svc);

  cCTL.setEventHandler(BLEWritten, onCTRL);
  cLAB.setEventHandler(BLESubscribed,   onLABsub);
  cLAB.setEventHandler(BLEUnsubscribed, onLABsub);

  updateInfo();
  BLE.advertise();

  g_last_us = micros();
  Serial.println("READY: subscribe to LABEL (…0006) then send START (01 64 00).");
}

// ========== Loop ==========
void loop(){
  BLE.poll();

  // Serial mini-console (optional)
  if(Serial.available()){
    static char sb[32]; static uint8_t si=0; char ch=Serial.read();
    if(ch=='\n'||ch=='\r'){ if(si){ sb[si]=0; si=0; for(char*p=sb;*p;++p){ if(*p>='A'&&*p<='Z') *p=*p-'A'+'a'; } apply_cmd_text(sb);} }
    else if(si<sizeof(sb)-1){ sb[si++]=ch; }
  }

  if(!g_streaming){ delay(2); return; }

  // pacing at g_fs
  uint32_t now = micros();
  if((now - g_last_us) < g_period_us) return;
  g_last_us += g_period_us;

  // read IMU
  float ax,ay,az,gx,gy,gz;
  if(!(IMU.accelerationAvailable() && IMU.gyroscopeAvailable())) return;
  if(!IMU.readAcceleration(ax,ay,az)) return;
  if(!IMU.readGyroscope(gx,gy,gz))   return;

  // ring buffer for simple window stats
  axbuf[wr]=ax; aybuf[wr]=ay; azbuf[wr]=az; wr = (wr+1) % WIN_N;

  // send RAW one-sample packet if enabled & subscribed
  if(g_rawOn && cRAW.subscribed()){
    // 6*int16 (mg/mdps)
    int16_t pkt[6];
    pkt[0]=to_i16(ax,1000.0f); pkt[1]=to_i16(ay,1000.0f); pkt[2]=to_i16(az,1000.0f);
    pkt[3]=to_i16(gx,1000.0f); pkt[4]=to_i16(gy,1000.0f); pkt[5]=to_i16(gz,1000.0f);
    cRAW.setValue((uint8_t*)pkt, 12); // <=20 bytes → safe everywhere
  }

  // hop for label update
  if(++hopCount >= HOP_N){
    hopCount = 0;

    // compute quick mean/std(|a|) over window
    float sum=0, sumsq=0;
    for(uint16_t i=0;i<WIN_N;i++){
      sum += fmag(axbuf[i], aybuf[i], azbuf[i]);
    }
    float mean = sum / WIN_N;
    for(uint16_t i=0;i<WIN_N;i++){
      float m = fmag(axbuf[i], aybuf[i], azbuf[i]);
      sumsq += fsqr(m - mean);
    }
    float stdv = sqrtf(sumsq / (WIN_N - 1));

    // classify with your tuned thresholds
    const char* lbl = "IDLE";
    if(stdv >= STD_RUN_MIN && mean >= MEAN_RUN_MIN) lbl = "RUN";
    else if(stdv >= STD_WALK_MIN && mean <= MEAN_WALK_MAX) lbl = "WALK";
    else if(stdv >= STD_WALK_MIN) lbl = "WALK"; // fallback

    // ----- BLE LABEL (tiny CSV <= 20 bytes) -----
    // Format: "IDLE,1.01,0.03" etc.
    char out[20];
    // keep to 2 decimals to stay well under 20 chars
    int n = snprintf(out, sizeof(out), "%s,%.2f,%.2f", lbl, mean, stdv);
    if(n > 0 && cLAB.subscribed()){
      cLAB.setValue((const unsigned char*)out, (int)strlen(out)); // safe: <=20
    }

    // ----- Serial JSON (optional, human-friendly) -----
    // comment this out if you want silent serial
    Serial.print("{\"t\":");
    Serial.print(millis());
    Serial.print(",\"node\":"); Serial.print(NODE_ID);
    Serial.print(",\"label\":\""); Serial.print(lbl); Serial.print("\"");
    Serial.print(",\"mean\":"); Serial.print(mean,3);
    Serial.print(",\"std\":");  Serial.print(stdv,3);
    Serial.println("}");
  }
}
