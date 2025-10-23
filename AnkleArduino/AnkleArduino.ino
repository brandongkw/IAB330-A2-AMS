/*
  AnkleMovementSpeed_MINMTU.ino
  Nano 33 IoT — Ankle node publishes speed/intensity (no activity classes).

  Service: 6e400001-b5a3-f393-e0a9-e50e24dcca9e
    0003 CTRL  (Write)  : text:  start/stop/raw:on/off/rate:<25..200>
                          binary: <cmd:1, hz:le uint16> (1=start, 0=stop, 2=set-rate)
    0005 RAW   (Notify) : 12 bytes <6h> ax,ay,az,gx,gy,gz (mg/mdps)  [optional]
    0006 LABEL (Notify) : "SPD,<accMag_g>,<intensity>"   (≤20 bytes)
    0004 INFO  (Read)   : short info line

  Notes:
  - Keeps all notifies ≤20 bytes so it works on phones/Windows without MTU negotiation.
  - "Intensity" here is a simple short-window std(|a|) to reflect movement magnitude.
*/

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>

#define SERIAL_BAUD 115200
#define NODE_ID     2
#define FS_DEFAULT  100
#define LEDPIN      LED_BUILTIN

// UUIDs
static const char* UUID_SVC_MOV       = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char* UUID_CHAR_CTRL     = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
static const char* UUID_CHAR_INFO     = "6e400004-b5a3-f393-e0a9-e50e24dcca9e";
static const char* UUID_CHAR_RAW      = "6e400005-b5a3-f393-e0a9-e50e24dcca9e";
static const char* UUID_CHAR_LABEL    = "6e400006-b5a3-f393-e0a9-e50e24dcca9e";

BLEService        svc(UUID_SVC_MOV);
BLECharacteristic cCTL(UUID_CHAR_CTRL,  BLEWrite,   32, true);
BLECharacteristic cINF(UUID_CHAR_INFO,  BLERead,    64, true);
BLECharacteristic cRAW(UUID_CHAR_RAW,   BLENotify,  20, true);
BLECharacteristic cLAB(UUID_CHAR_LABEL, BLENotify,  20, true);

// state
static volatile bool g_streaming = false;
static volatile bool g_rawOn     = false;
static uint16_t      g_fs        = FS_DEFAULT;
static uint32_t      g_period_us = 1000000UL / FS_DEFAULT;
static uint8_t       g_session   = 1;
static uint32_t      g_last_us   = 0;

#define WIN_SEC  0.5f          // shorter window for responsive "intensity"
#define HOP_SEC  0.2f
#define WIN_N    (uint16_t)(FS_DEFAULT*WIN_SEC)
#define HOP_N    (uint16_t)(FS_DEFAULT*HOP_SEC)

static float amag[WIN_N];
static uint16_t wr=0, hopCount=0;

static inline float fsqr(float x){ return x*x; }
static inline float fmag(float x,float y,float z){ return sqrtf(fsqr(x)+fsqr(y)+fsqr(z)); }
static inline int16_t to_i16(float v, float scale){
  long s = lroundf(v*scale); if(s<-32768) s=-32768; if(s>32767) s=32767; return (int16_t)s;
}

static void updateInfo(){
  char info[64];
  snprintf(info, sizeof(info), "Node:%u;Sess:%u;Rate:%uHz;FW:ANK_SPD",
           (unsigned)NODE_ID,(unsigned)g_session,(unsigned)g_fs);
  cINF.setValue((const unsigned char*)info, strlen(info));
}

static void apply_cmd_text(const char* s){
  if(!strcmp(s,"start"))        { g_streaming = true;  }
  else if(!strcmp(s,"stop"))    { g_streaming = false; }
  else if(!strcmp(s,"raw:on"))  { g_rawOn = true;      }
  else if(!strcmp(s,"raw:off")) { g_rawOn = false;     }
  else if(!strncmp(s,"rate:",5)){
    int hz = atoi(s+5); if(hz>=25 && hz<=200){ g_fs = hz; g_period_us = 1000000UL/g_fs; }
  }
}

// CTRL handler: ASCII & 3-byte binary
static void onCTRL(BLEDevice, BLECharacteristic){
  uint8_t buf[32]; int n = cCTL.readValue(buf, sizeof(buf));
  if(n<=0) return;
  bool ascii=true; for(int i=0;i<n;i++){ if(buf[i]==0){ ascii=false; break; } }
  if(ascii){
    char cmd[32]; int m = (n>31)?31:n; memcpy(cmd,buf,m); cmd[m]=0;
    for(int i=0;i<m;i++){ if(cmd[i]>='A'&&cmd[i]<='Z') cmd[i]=cmd[i]-'A'+'a'; }
    apply_cmd_text(cmd);
    return;
  }
  if(n>=3){
    uint8_t cmd = buf[0]; uint16_t hz=(uint16_t)(buf[1] | (buf[2]<<8));
    if(cmd==0){ g_streaming=false; }
    else if(cmd==1){ g_streaming=true; }
    else if(cmd==2 && hz>=25 && hz<=200){ g_fs=hz; g_period_us=1000000UL/g_fs; }
  }
}

static void onRAWsub(BLEDevice, BLECharacteristic ch){
  g_rawOn = ch.subscribed(); // auto toggle RAW with subscription
}

void setup(){
  pinMode(LEDPIN, OUTPUT); digitalWrite(LEDPIN, LOW);
  Serial.begin(SERIAL_BAUD);
  while(!Serial && millis()<1500){}

  if(!IMU.begin()){ Serial.println("IMU init failed"); while(1){ delay(1000);} }
  if(!BLE.begin()){ Serial.println("BLE init failed"); while(1){ delay(1000);} }

  BLE.setLocalName("AMS-Ankle");
  BLE.setDeviceName("AMS-Ankle");
  BLE.setAdvertisedService(svc);

  svc.addCharacteristic(cRAW);
  svc.addCharacteristic(cLAB);
  svc.addCharacteristic(cCTL);
  svc.addCharacteristic(cINF);
  BLE.addService(svc);

  cCTL.setEventHandler(BLEWritten, onCTRL);
  cRAW.setEventHandler(BLESubscribed,   onRAWsub);
  cRAW.setEventHandler(BLEUnsubscribed, onRAWsub);

  updateInfo();
  BLE.advertise();

  for(uint16_t i=0;i<WIN_N;i++) amag[i]=1.0f;
  g_last_us = micros();

  Serial.println("ANKLE SPEED READY: subscribe LABEL (…0006), START (01 64 00). RAW optional on …0005.");
}

void loop(){
  BLE.poll();

  // serial console (optional)
  if(Serial.available()){
    static char sb[32]; static uint8_t si=0; char ch=Serial.read();
    if(ch=='\n'||ch=='\r'){ if(si){ sb[si]=0; si=0; for(char*p=sb;*p;++p){ if(*p>='A'&&*p<='Z') *p=*p-'A'+'a'; } apply_cmd_text(sb);} }
    else if(si<sizeof(sb)-1){ sb[si++]=ch; }
  }

  if(!g_streaming){ delay(2); return; }

  uint32_t now = micros();
  if((now - g_last_us) < g_period_us) return;
  g_last_us += g_period_us;

  float ax,ay,az,gx,gy,gz;
  if(!(IMU.accelerationAvailable() && IMU.gyroscopeAvailable())) return;
  if(!IMU.readAcceleration(ax,ay,az)) return;
  if(!IMU.readGyroscope(gx,gy,gz))   return;

  float a = fmag(ax,ay,az);
  amag[wr] = a; wr = (wr+1) % WIN_N;

  // Optional RAW sample (12 bytes) — safe everywhere
  if(g_rawOn && cRAW.subscribed()){
    int16_t pkt[6] = {
      to_i16(ax,1000.0f), to_i16(ay,1000.0f), to_i16(az,1000.0f),
      to_i16(gx,1000.0f), to_i16(gy,1000.0f), to_i16(gz,1000.0f)
    };
    cRAW.setValue((uint8_t*)pkt, 12);
  }

  if(++hopCount >= HOP_N){
    hopCount = 0;

    // Simple "movement intensity": std(|a|) over a short window
    float sum=0, sumsq=0;
    for(uint16_t i=0;i<WIN_N;i++) sum += amag[i];
    float mean = sum / WIN_N;
    for(uint16_t i=0;i<WIN_N;i++){ float d = amag[i]-mean; sumsq += d*d; }
    float intensity = sqrtf(sumsq / (WIN_N - 1));

    // LABEL payload: "SPD,<accMag_g>,<intensity>"
    // Keep to 2 decimals to fit under 20 bytes
    char out[20];
    float accMag = a; // current magnitude (g)
    int n = snprintf(out, sizeof(out), "SPD,%.2f,%.2f", accMag, intensity);
    if(n>0 && cLAB.subscribed()){
      cLAB.setValue((const unsigned char*)out, (int)strlen(out));
    }

    // optional Serial for human-readability
    Serial.print("Acceleration Magnitude: ");
    Serial.print(accMag, 2);
    Serial.print(" g\tMovement Intensity: ");
    Serial.println(intensity, 2);
  }
}
