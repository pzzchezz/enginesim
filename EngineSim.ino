/*
  ESP32 Engine Simulator  (ตัวจำลองเครื่องยนต์ ESP32)
  ---------------------------------------------------
  - จำลองเครื่องยนต์สูบเดียว: รอบเครื่อง, คันเร่ง (TPS), อุณหภูมิน้ำ, แรงดันแบต,
    MAP, องศาไฟจุดระเบิด, หัวฉีด, ตัดรอบ (rev limiter), ดับเครื่อง ฯลฯ
  - รับคำสั่งจากแอปผ่าน Bluetooth Classic (SPP) และส่งค่ากลับ 10 ครั้ง/วินาที
  - แสดงค่าบนจอ OLED SSD1306 128x64 (I2C)

  *** นี่คือข้อมูลจำลอง (Mock) ไม่ใช่โปรโตคอล ECU จริงของรถรุ่นใด ***

  บอร์ด : ESP32 รุ่นบลูทูธคลาสสิก (ESP32-WROOM / WROVER) -- ไม่ใช่ S3 / C3 / C6
  Arduino IDE : Board = "ESP32 Dev Module"
  ไลบรารี  : "Adafruit SSD1306" และ "Adafruit GFX Library" (ติดตั้งจาก Library Manager)

  โปรโตคอล (ข้อความ ASCII จบบรรทัดด้วย \n)
  แอป -> ESP32 :
    KEY,1|0        กุญแจ เปิด/ปิด
    START          กดสตาร์ท (ต้องเปิดกุญแจก่อน)
    THR,0..100     คันเร่ง %
    LOAD,0..100    ภาระเครื่อง %
    MODE,0|1       0 = คันเร่งมือ, 1 = ให้ ESP32 คุมคันเร่งเพื่อรักษารอบคงที่
    RPM,n          รอบเป้าหมายสำหรับ MODE,1
    FAULT,mask     บิต0=น้ำร้อนจัด บิต1=แบตต่ำ บิต2=TPS ผิดพลาด
    WARM           ข้ามช่วงอุ่นเครื่อง (ตั้งอุณหภูมิน้ำ 85 C)
    PING           ตอบ PONG
  ESP32 -> แอป :
    D,rpm,tps,ect,iat,vbat,map,adv,inj,limiter,state,fault
    state: 0=ดับ(กุญแจปิด) 1=กุญแจเปิด 2=กำลังสตาร์ท 3=เครื่องทำงาน

  ทดสอบโดยไม่ใช้แอปได้: เปิด Serial Monitor 115200 (ส่ง New Line) แล้วพิมพ์คำสั่งข้างบน
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth ไม่ได้เปิดใช้งาน - ต้องใช้บอร์ด ESP32 รุ่นบลูทูธคลาสสิก"
#endif

// ======================= ตั้งค่าที่แก้ได้ =======================
#define BT_NAME     "ESP32-ENGINE-SIM"

#define OLED_W      128
#define OLED_H      64
#define OLED_ADDR   0x3C
#define OLED_SDA    21     // ESP32 DevKit ทั่วไป: SDA=21 SCL=22
#define OLED_SCL    22     // Heltec WiFi Kit 32: SDA=4 SCL=15 RST=16
#define OLED_RST    -1     // WeMos/Lolin32 OLED: SDA=5 SCL=4 (RST -1)
// ===============================================================

BluetoothSerial SerialBT;
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RST);
bool oledOk = false;

// ===== ENGINE MODEL BEGIN =====
enum EngineState { ST_OFF = 0, ST_ON = 1, ST_CRANK = 2, ST_RUN = 3 };

const float IDLE_RPM         = 1500.0f;
const float REV_LIMIT        = 9000.0f;
const float REV_LIMIT_RESUME = 8800.0f;
const float STALL_RPM        = 500.0f;
const float ENGINE_K         = 550.0f;   // ความเฉื่อยเครื่อง (ยิ่งมากยิ่งไวต่อแรงบิด)
const float AMBIENT_C        = 30.0f;

// อินพุตจากแอป
bool  keyOn    = false;
bool  startReq = false;
float thrCmd   = 0.0f;     // 0..1
float loadPct  = 0.0f;     // 0..1
bool  holdMode = false;
float holdRpm  = 3000.0f;
int   faultMask = 0;

// สถานะเครื่อง
EngineState state = ST_OFF;
float rpm = 0.0f;
float tps = 0.0f;          // 0..1 (หลังหน่วงเซนเซอร์)
float ect = AMBIENT_C;
float iat = AMBIENT_C;
float vbat = 12.6f;
float mapKpa = 101.0f;
float advDeg = 0.0f;
float injMs = 0.0f;
bool  limiter = false;
float holdI = 0.3f;
uint32_t runSince = 0;
uint32_t crankSince = 0;

// ค่าที่ส่งออก
int   rpmOut = 0;
float tpsOut = 0.0f;

float torqueCurve(float r) {
  float x = (r - 6000.0f) / 6500.0f;
  float c = 1.0f - x * x;
  return c < 0.35f ? 0.35f : c;
}

void engineStep(float dt, uint32_t now) {
  // ---- กุญแจ / สถานะ ----
  if (!keyOn) {
    state = ST_OFF;
  } else if (state == ST_OFF) {
    state = (rpm >= STALL_RPM) ? ST_RUN : ST_ON;
    runSince = now;
  }
  if (startReq) {
    startReq = false;
    if (keyOn && state == ST_ON) { state = ST_CRANK; crankSince = now; }
  }

  // ---- คันเร่ง ----
  float thrIn = thrCmd;
  if (holdMode && state == ST_RUN) {
    float err = holdRpm - rpm;
    if (fabsf(err) < 600.0f) holdI += err * dt * 0.0008f;   // integrate เฉพาะใกล้เป้า (กัน windup)
    holdI = constrain(holdI, 0.0f, 1.0f);
    thrIn = constrain(err * 0.001f + holdI, 0.0f, 1.0f);
  } else {
    holdI = 0.3f;
  }
  tps += (thrIn - tps) * fminf(1.0f, dt * 15.0f);

  // ---- แรงบิด / รอบเครื่อง ----
  float tFric = 0.9f + rpm / 3000.0f;
  float tLoad = loadPct * 2.5f;

  if (state == ST_CRANK) {
    rpm += (260.0f - rpm) * fminf(1.0f, dt * 6.0f);
    if (now - crankSince >= 1200) {
      state = ST_RUN; runSince = now; limiter = false; holdI = 0.3f;
    }
  } else {
    float tEng = 0.0f;
    if (state == ST_RUN) {
      if (rpm >= REV_LIMIT) limiter = true;
      else if (rpm < REV_LIMIT_RESUME) limiter = false;
      float idleThr = constrain(0.30f + (IDLE_RPM - rpm) * 0.0015f, 0.0f, 0.8f);
      float effThr = fmaxf(tps, idleThr);
      tEng = limiter ? 0.0f : effThr * torqueCurve(rpm) * 10.0f;
    } else {
      limiter = false;
    }
    rpm += (tEng - tFric - tLoad) * ENGINE_K * dt;
    if (rpm < 0.0f) rpm = 0.0f;
    if (state == ST_RUN && (now - runSince) > 2000 && rpm < STALL_RPM) state = ST_ON;   // ดับ
  }
  bool running = (state == ST_RUN);

  // ---- อุณหภูมิน้ำ ----
  float ectTarget, tau;
  if (faultMask & 1)  { ectTarget = 118.0f; tau = 12.0f; }
  else if (running)   { ectTarget = 84.0f + loadPct * 10.0f + (rpm / 9000.0f) * 6.0f; tau = 40.0f; }
  else                { ectTarget = AMBIENT_C; tau = 300.0f; }
  ect += (ectTarget - ect) * fminf(1.0f, dt / tau);
  iat = AMBIENT_C + (ect - AMBIENT_C) * 0.12f;

  // ---- แบตเตอรี่ ----
  float vT;
  if (state == ST_OFF)        vT = 12.6f;
  else if (state == ST_ON)    vT = 12.4f;
  else if (state == ST_CRANK) vT = 10.6f;
  else                        vT = 13.1f + fminf(rpm, 4000.0f) / 4000.0f * 1.1f;
  if (faultMask & 2) vT -= 3.0f;
  vbat += (vT - vbat) * fminf(1.0f, dt * 6.0f);

  // ---- MAP ----
  float mT = 101.0f;
  if (state == ST_CRANK) mT = 85.0f;
  else if (running)      mT = 28.0f + 72.0f * powf(tps, 0.85f);
  mapKpa += (mT - mapKpa) * fminf(1.0f, dt * 12.0f);

  // ---- ไฟจุดระเบิด (องศาก่อนศูนย์ตายบน) ----
  float adv = 0.0f;
  if (running) {
    if (rpm < 6500.0f) adv = 10.0f + 22.0f * fmaxf(rpm - 1000.0f, 0.0f) / 5500.0f;
    else               adv = 32.0f - (rpm - 6500.0f) / 2500.0f * 4.0f;
    if (ect > 105.0f) adv -= 6.0f;
  }
  advDeg = adv;

  // ---- หัวฉีด (มี ตัดน้ำมันตอนปล่อยคันเร่งรอบสูง / ตัดรอบ) ----
  float inj = 0.0f;
  if (state == ST_CRANK) {
    inj = 4.0f;
  } else if (running && !limiter && !(tps < 0.02f && rpm > 3500.0f)) {
    inj = (0.9f + 2.6f * mapKpa / 100.0f) * (1.0f + fmaxf(0.0f, 60.0f - ect) / 100.0f);
  }
  injMs = inj;

  // ---- ค่าที่ส่งออก ----
  float r = rpm;
  if (running) r += (float)random(-12, 13);
  rpmOut = (int)(r < 0.0f ? 0.0f : r);
  tpsOut = (faultMask & 4) ? 100.0f : tps * 100.0f;   // TPS ผิดพลาด = ค้างที่ 100%
}
// ===== ENGINE MODEL END =====

// ======================= รับคำสั่ง =======================
#define CMD_MAX 48
char    btBuf[CMD_MAX];   uint8_t btLen = 0;
char    usbBuf[CMD_MAX];  uint8_t usbLen = 0;

void handleCommand(char *line, Print &reply) {
  char *val = strchr(line, ',');
  if (val) { *val = 0; val++; }
  float v = val ? (float)atof(val) : 0.0f;

  if      (!strcmp(line, "KEY"))   keyOn = (v > 0.5f);
  else if (!strcmp(line, "START")) startReq = true;
  else if (!strcmp(line, "THR"))   thrCmd = constrain(v, 0.0f, 100.0f) / 100.0f;
  else if (!strcmp(line, "LOAD"))  loadPct = constrain(v, 0.0f, 100.0f) / 100.0f;
  else if (!strcmp(line, "MODE"))  holdMode = (v > 0.5f);
  else if (!strcmp(line, "RPM"))   holdRpm = constrain(v, 0.0f, REV_LIMIT);
  else if (!strcmp(line, "FAULT")) faultMask = ((int)v) & 7;
  else if (!strcmp(line, "WARM"))  ect = 85.0f;
  else if (!strcmp(line, "PING"))  reply.print("PONG\n");
}

void pollStream(Stream &s, char *buf, uint8_t &len) {
  while (s.available()) {
    char c = (char)s.read();
    if (c == '\n') {
      buf[len] = 0;
      if (len > 0) handleCommand(buf, s);
      len = 0;
    } else if (c != '\r') {
      if (len < CMD_MAX - 1) buf[len++] = c; else len = 0;
    }
  }
}

// ======================= จอ OLED =======================
const char *stateName() {
  switch (state) {
    case ST_OFF:   return "OFF";
    case ST_ON:    return "KEY ON";
    case ST_CRANK: return "CRANK";
    default:       return "RUN";
  }
}

void drawScreen() {
  if (!oledOk) return;
  char b[32];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // แถวบน: สถานะ + บลูทูธ
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(stateName());
  if (limiter) display.print(" LIM");
  if (faultMask) { display.print(" F"); display.print(faultMask); }
  display.setCursor(86, 0);
  display.print(SerialBT.hasClient() ? "BT:OK" : "BT:--");

  // รอบเครื่องตัวใหญ่
  display.setTextSize(3);
  snprintf(b, sizeof(b), "%4d", rpmOut);
  display.setCursor(0, 11);
  display.print(b);

  display.setTextSize(1);
  display.setCursor(80, 12); display.print("rpm");
  snprintf(b, sizeof(b), "ADV%2.0f", advDeg);
  display.setCursor(80, 21); display.print(b);
  snprintf(b, sizeof(b), "INJ%.1f", injMs);
  display.setCursor(80, 29); display.print(b);

  // แถบรอบ + เส้นแดงไลน์ 8500
  display.drawRect(0, 38, 128, 6, SSD1306_WHITE);
  int w = (int)(constrain((float)rpmOut / 10000.0f, 0.0f, 1.0f) * 124.0f);
  display.fillRect(2, 40, w, 2, SSD1306_WHITE);
  display.drawFastVLine(108, 36, 10, SSD1306_WHITE);

  // ค่าอื่น
  snprintf(b, sizeof(b), "TPS%3.0f%% ECT%3.0fC", tpsOut, ect);
  display.setCursor(0, 47); display.print(b);
  snprintf(b, sizeof(b), "BAT%4.1fV MAP%3.0fkPa", vbat, mapKpa);
  display.setCursor(0, 56); display.print(b);

  display.display();
}

// ======================= setup / loop =======================
uint32_t tEngine = 0, tTele = 0, tDisp = 0, tDbg = 0;
bool wasConnected = false;

void setup() {
  Serial.begin(115200);
  randomSeed(micros());

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledOk) {
    Serial.println("ไม่พบจอ OLED - ตรวจ SDA/SCL/ที่อยู่ I2C (ระบบยังทำงานผ่านบลูทูธได้)");
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 8);  display.print("ENGINE SIM");
    display.setTextSize(1);
    display.setCursor(0, 34); display.print("BT: "); display.print(BT_NAME);
    display.setCursor(0, 48); display.print("Mock data only");
    display.display();
  }

  SerialBT.begin(BT_NAME);
  Serial.print("Bluetooth พร้อม ชื่ออุปกรณ์: ");
  Serial.println(BT_NAME);

  tEngine = millis();
}

void loop() {
  pollStream(SerialBT, btBuf, btLen);
  pollStream(Serial, usbBuf, usbLen);

  uint32_t now = millis();

  // ถ้าแอปหลุดการเชื่อมต่อ ให้ปล่อยคันเร่งเพื่อความปลอดภัย
  bool conn = SerialBT.hasClient();
  if (wasConnected && !conn) { thrCmd = 0.0f; holdMode = false; }
  wasConnected = conn;

  if (now - tEngine >= 10) {
    float dt = (now - tEngine) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;
    tEngine = now;
    engineStep(dt, now);
  }

  if (now - tTele >= 100) {
    tTele = now;
    if (conn) {
      char out[120];
      snprintf(out, sizeof(out), "D,%d,%.1f,%.1f,%.1f,%.2f,%.1f,%.1f,%.2f,%d,%d,%d\n",
               rpmOut, tpsOut, ect, iat, vbat, mapKpa, advDeg, injMs,
               limiter ? 1 : 0, (int)state, faultMask);
      SerialBT.print(out);
    }
  }

  if (now - tDisp >= 200) {
    tDisp = now;
    drawScreen();
  }

  if (now - tDbg >= 1000) {
    tDbg = now;
    Serial.printf("[%s] rpm=%d tps=%.0f ect=%.0f vbat=%.1f map=%.0f adv=%.0f inj=%.2f BT=%d\n",
                  stateName(), rpmOut, tpsOut, ect, vbat, mapKpa, advDeg, injMs, conn ? 1 : 0);
  }
}
