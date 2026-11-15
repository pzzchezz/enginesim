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
#include "BluetoothSerial.h"

// 0 = จอ SSD1306 (0.96")   1 = จอ SH1106 (1.3")  -- ถ้าจอเป็นขยะ/เลื่อน ลองสลับค่านี้
#define OLED_SH1106 0
#if OLED_SH1106
  #include <Adafruit_SH110X.h>
  #define COL_ON  SH110X_WHITE
  #define COL_OFF SH110X_BLACK
#else
  #include <Adafruit_SSD1306.h>
  #define COL_ON  SSD1306_WHITE
  #define COL_OFF SSD1306_BLACK
#endif

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
#define OLED_I2C_HZ 200000UL  // ถ้าจอยังมีสัญญาณรบกวน ลองลดเป็น 100000UL
// ===============================================================

BluetoothSerial SerialBT;
#if OLED_SH1106
Adafruit_SH1106G display(OLED_W, OLED_H, &Wire, OLED_RST, OLED_I2C_HZ, 100000UL);
#else
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RST, OLED_I2C_HZ, 100000UL);
#endif
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

// ---------- เกจรอบแบบครึ่งวงกลม ----------
#define GCX 36
#define GCY 38
#define GR  34

static void gpolar(float f, float r, int &x, int &y) {   // f: 0 = ซ้าย, 0.5 = บน, 1 = ขวา
  float a = PI + f * PI;
  x = GCX + (int)lroundf(r * cosf(a));
  y = GCY + (int)lroundf(r * sinf(a));
}

// rpmVal < 0 = ไม่วาดเข็ม/ตัวเลข, reveal 0..1 = เผยเกจจากซ้ายไปขวา (ใช้ตอนเปิดเครื่อง)
static void drawGauge(int rpmVal, float reveal) {
  int x, y, x2, y2;
  display.setTextSize(1);

  // เส้นโค้งนอก
  for (int i = 0; i <= 90; i++) {
    if (i / 90.0f > reveal) break;
    gpolar(i / 90.0f, GR, x, y); display.drawPixel(x, y, COL_ON);
  }
  // โซนแดง 8000-9000 (เส้นหนา)
  for (int i = 0; i <= 12; i++) {
    float f = (8000.0f + i * (1000.0f / 12.0f)) / 9000.0f;
    if (f > reveal) break;
    for (int r = GR - 1; r >= GR - 3; r--) { gpolar(f, (float)r, x, y); display.drawPixel(x, y, COL_ON); }
  }
  // ขีดทุก 1000 รอบ, ตัวเลขทุก 3000
  for (int i = 0; i <= 9; i++) {
    float f = i / 9.0f;
    if (f > reveal) break;
    bool major = (i % 3 == 0);
    gpolar(f, (float)GR, x, y);
    gpolar(f, (float)(GR - (major ? 7 : 4)), x2, y2);
    display.drawLine(x, y, x2, y2, COL_ON);
    if (major) {
      char c[2] = { (char)('0' + i), 0 };
      gpolar(f, (float)(GR - 14), x, y);
      display.setCursor(x - 2, y - 3);
      display.print(c);
    }
  }
  if (rpmVal < 0) return;

  // เข็ม
  float f = constrain((float)rpmVal, 0.0f, 9000.0f) / 9000.0f;
  gpolar(f, (float)(GR - 9), x, y);
  display.drawLine(GCX, GCY, x, y, COL_ON);
  display.drawLine(GCX + 1, GCY, x, y, COL_ON);
  display.fillCircle(GCX, GCY, 3, COL_ON);

  // รอบดิจิทัลใต้เกจ
  char b[16];
  snprintf(b, sizeof(b), "%4d", rpmVal);
  display.setTextSize(2);
  display.setCursor(12, 46);
  display.print(b);
  display.setTextSize(1);
}

void drawScreen() {
  if (!oledOk) return;
  char b[32];
  display.clearDisplay();
  display.setTextColor(COL_ON);

  drawGauge(rpmOut, 1.0f);
  display.drawFastVLine(74, 0, 64, COL_ON);

  // แถวบน: สถานะ (กลับสีเมื่อตัดรอบ/มีอาการผิดปกติ) + จุดบลูทูธ
  display.setTextSize(1);
  if (limiter || faultMask) {
    display.fillRect(78, 0, 36, 10, COL_ON);
    display.setTextColor(COL_OFF, COL_ON);
    display.setCursor(80, 1);
    display.print(limiter ? "LIMIT" : "FAULT");
    display.setTextColor(COL_ON);
  } else {
    display.setCursor(78, 1);
    display.print(stateName());
  }
  if (SerialBT.hasClient()) display.fillCircle(122, 4, 3, COL_ON);
  else                      display.drawCircle(122, 4, 3, COL_ON);

  // คันเร่ง + แถบ
  snprintf(b, sizeof(b), "THR%3.0f%%", tpsOut);
  display.setCursor(78, 12); display.print(b);
  display.drawRect(78, 21, 48, 6, COL_ON);
  int w = (int)(constrain(tpsOut, 0.0f, 100.0f) / 100.0f * 44.0f);
  display.fillRect(80, 23, w, 2, COL_ON);

  // ค่าอื่น
  snprintf(b, sizeof(b), "ECT%4.0fC", ect);   display.setCursor(78, 29); display.print(b);
  snprintf(b, sizeof(b), "BAT%4.1fV", vbat);  display.setCursor(78, 38); display.print(b);
  snprintf(b, sizeof(b), "MAP%4.0f", mapKpa); display.setCursor(78, 47); display.print(b);
  snprintf(b, sizeof(b), "INJ%4.1f", injMs);  display.setCursor(78, 56); display.print(b);

  display.display();
}

// ======================= หน้าโหลด PUXXZA (10 วินาที) =======================
#define SPLASH_MS 10000

// ตัวอักษรเส้นแบบเอียง ๆ สไตล์สปอร์ต: กริด 8x12, -1 = ยกปากกา, -2 = จบตัวอักษร
static const int8_t L_P[] = {0,12, 0,0, 6,0, 8,2, 8,5, 6,7, 0,7, -2};
static const int8_t L_U[] = {0,0, 0,10, 2,12, 6,12, 8,10, 8,0, -2};
static const int8_t L_X[] = {0,0, 8,12, -1, 8,0, 0,12, -2};
static const int8_t L_Z[] = {0,0, 8,0, 0,12, 8,12, -2};
static const int8_t L_A[] = {0,12, 0,3, 3,0, 5,0, 8,3, 8,12, -1, 0,7, 8,7, -2};
static const int8_t* const LOGO[6] = {L_P, L_U, L_X, L_X, L_Z, L_A};   // P U X X Z A

static void lpt(int ox, int oy, int gx, int gy, int &x, int &y) {
  x = ox + gx * 2 + (int)lroundf((12 - gy) * 0.6f);   // เอียงไปทางขวา
  y = oy + gy * 2;
}

static void thickLine(int x0, int y0, int x1, int y1) {
  display.drawLine(x0, y0, x1, y1, COL_ON);
  display.drawLine(x0 + 1, y0, x1 + 1, y1, COL_ON);
  display.drawLine(x0, y0 + 1, x1, y1 + 1, COL_ON);
  display.drawLine(x0 + 1, y0 + 1, x1 + 1, y1 + 1, COL_ON);
  display.drawLine(x0 + 2, y0, x1 + 2, y1, COL_ON);
}

// วาดตัวอักษรตามสัดส่วน prog (0..1) เหมือนเลเซอร์ไล่เส้น
static void drawLetter(const int8_t* L, float prog, int ox, int oy) {
  // รวมความยาวเส้นทั้งหมดก่อน
  float total = 0; int px = -100, py = 0; bool pen = false;
  for (int i = 0; L[i] != -2; ) {
    if (L[i] == -1) { pen = false; i++; continue; }
    int gx = L[i], gy = L[i + 1]; i += 2;
    if (pen) total += hypotf((float)(gx - px), (float)(gy - py));
    px = gx; py = gy; pen = true;
  }
  float lim = total * prog, done = 0;
  int tipX = 0, tipY = 0; bool haveTip = false;
  pen = false;
  for (int i = 0; L[i] != -2; ) {
    if (L[i] == -1) { pen = false; i += 1; continue; }
    int gx = L[i], gy = L[i + 1]; i += 2;
    if (pen) {
      float seg = hypotf((float)(gx - px), (float)(gy - py));
      float use = fminf(seg, lim - done);
      if (use > 0) {
        float k = use / seg;
        int x0, y0, x1, y1;
        lpt(ox, oy, px, py, x0, y0);
        lpt(ox, oy, px + (int)lroundf((gx - px) * k), py + (int)lroundf((gy - py) * k), x1, y1);
        // ใช้ทศนิยมให้เส้นสุดทางเที่ยงตรง
        float fx = px + (gx - px) * k, fy = py + (gy - py) * k;
        x1 = ox + (int)lroundf(fx * 2 + (12 - fy) * 0.6f);
        y1 = oy + (int)lroundf(fy * 2);
        thickLine(x0, y0, x1, y1);
        tipX = x1; tipY = y1; haveTip = true;
      }
      done += seg;
    }
    px = gx; py = gy; pen = true;
  }
  if (prog < 1.0f && haveTip) display.fillCircle(tipX, tipY, 2, COL_ON);   // ประกายที่ปลายเส้น
}

// เลื่อนแถบแนวนอนแบบกลิตช์ (แก้ buffer ตรง ๆ ทีละ 8 พิกเซล)
static void glitchBands(int n, int maxShift) {
  uint8_t *buf = display.getBuffer();
  for (int k = 0; k < n; k++) {
    int page = random(0, 5);                 // เฉพาะส่วนโลโก้/ข้อความ ไม่แตะแถบโหลด
    int sh = random(-maxShift, maxShift + 1);
    uint8_t *p = buf + page * OLED_W;
    if (sh > 0)      { memmove(p + sh, p, OLED_W - sh); memset(p, 0, sh); }
    else if (sh < 0) { sh = -sh; memmove(p, p + sh, OLED_W - sh); memset(p + OLED_W - sh, 0, sh); }
  }
}

static void printSpacedCenter(const char* str, int y, int pitch, int maxChars) {
  int n = (int)strlen(str);
  int w = (n - 1) * pitch + 5;
  int x = (OLED_W - w) / 2;
  for (int i = 0; i < n && i < maxChars; i++) {
    display.setCursor(x + i * pitch, y);
    display.print(str[i]);
  }
}

static void drawSplashFrame(uint32_t t) {
  display.setTextColor(COL_ON);
  display.setTextSize(1);

  // มุมกรอบสปอร์ต
  if (t > 100) {
    display.drawFastHLine(0, 0, 9, COL_ON);                    display.drawFastVLine(0, 0, 9, COL_ON);
    display.drawFastHLine(OLED_W - 9, 0, 9, COL_ON);           display.drawFastVLine(OLED_W - 1, 0, 9, COL_ON);
    display.drawFastHLine(0, OLED_H - 1, 9, COL_ON);           display.drawFastVLine(0, OLED_H - 9, 9, COL_ON);
    display.drawFastHLine(OLED_W - 9, OLED_H - 1, 9, COL_ON);  display.drawFastVLine(OLED_W - 1, OLED_H - 9, 9, COL_ON);
  }

  // เส้นขยายจากกลาง (เปิดฉาก)
  if (t < 400) {
    int w = (int)(OLED_W * (t / 400.0f));
    display.drawFastHLine((OLED_W - w) / 2, 31, w, COL_ON);
    return;
  }

  // เส้นใต้โลโก้: จุดไข่ปลา + แสงวิ่ง
  for (int x = 6; x < OLED_W - 6; x += 2) display.drawPixel(x, 31, COL_ON);
  if (t >= 2800) {
    int sx = 6 + (int)((t / 6) % 140) - 20;
    for (int x = sx; x < sx + 22; x++) {
      if (x >= 6 && x < OLED_W - 6) { display.drawPixel(x, 31, COL_ON); display.drawPixel(x, 32, COL_ON); }
    }
  }

  // โลโก้ PUXXZA ไล่เส้นทีละตัว (ตัวละ 400 ms)
  float lp = (t - 400) / 400.0f;
  for (int i = 0; i < 6; i++) {
    float pr = constrain(lp - i, 0.0f, 1.0f);
    if (pr > 0.0f) drawLetter(LOGO[i], pr, 5 + i * 19, 4);
  }
  if (t < 2800) return;

  // ข้อความรอง ไล่ทีละตัว
  int chars = (int)((t - 2800) / 55);
  printSpacedCenter("ENGINE SIM", 36, 9, chars);

  // แถบโหลด
  if (t >= 3300) {
    float p = constrain((t - 3300) / 6100.0f, 0.0f, 1.0f);
    p = p * p * (3.0f - 2.0f * p);
    const char* msg = p < 0.25f ? "INIT ECU" : p < 0.5f ? "CAL SENSORS" : p < 0.75f ? "LINK BLUETOOTH" : p < 0.995f ? "ALMOST READY" : "READY";
    display.setCursor(4, 44);
    display.print("> "); display.print(msg);
    if (p < 0.995f && ((t / 250) & 1)) display.print("_");
    char pc[8]; snprintf(pc, sizeof(pc), "%3d%%", (int)(p * 100.0f + 0.5f));
    display.setCursor(OLED_W - 4 - 24, 44); display.print(pc);

    display.drawRect(4, 53, 120, 9, COL_ON);
    int fw = (int)(116.0f * p);
    display.fillRect(6, 55, fw, 5, COL_ON);
    for (int x = 0; x < fw; x++) {                       // ลายทางวิ่งในแถบ
      for (int r = 0; r < 5; r++) {
        if ((((x - r) + (int)(t / 25)) & 7) < 2) display.drawPixel(6 + x, 55 + r, COL_OFF);
      }
    }
  }

  // ปิดฉากแบบบานเกล็ด
  if (t >= SPLASH_MS - 600) {
    int h = (int)(32.0f * (t - (SPLASH_MS - 600)) / 600.0f);
    display.fillRect(0, 0, OLED_W, h, COL_OFF);
    display.fillRect(0, OLED_H - h, OLED_W, h, COL_OFF);
    display.drawFastHLine(0, 31, OLED_W, COL_ON);
  }
}

static void flashInvert(int ms) {
  display.invertDisplay(true);  display.display();
  delay(ms);
  display.invertDisplay(false);
}

static void bootSplash() {
  if (!oledOk) { delay(SPLASH_MS); return; }
  uint32_t t0 = millis();
  bool f1 = false, f2 = false;
  for (;;) {
    uint32_t t = millis() - t0;
    if (t >= SPLASH_MS) break;
    display.clearDisplay();
    drawSplashFrame(t);
    if (t >= 2800 && t < 3500)               glitchBands(3, 12);    // กลิตช์ตอนโลโก้เสร็จ
    else if (t >= 3500 && (t % 1300) < 120)  glitchBands(2, 6);     // กลิตช์เป็นช่วง ๆ
    if (t >= 2800 && !f1) { f1 = true; flashInvert(70); }
    if (t >= SPLASH_MS - 600 && !f2) { f2 = true; flashInvert(50); }
    display.display();
  }
  display.clearDisplay();
  display.display();
}

// ======================= หน้าโหลดเข็มไมล์ (5 วินาที) =======================
#define GAUGE_MS 5000

static float easeIO(float x) { return x * x * (3.0f - 2.0f * x); }

static void drawGaugeIntroFrame(uint32_t t) {
  display.setTextColor(COL_ON);
  display.setTextSize(1);

  // ลำดับ: เผยเกจ -> เข็มสวิง 0..9000 -> ค้างโซนแดงกะพริบ -> ตกกลับ 0 -> แผงขวาไล่ขึ้น -> ALL OK
  float reveal = constrain(t / 300.0f, 0.0f, 1.0f);
  int rpmV = -1;
  if (t >= 300 && t < 1700)       rpmV = (int)(9000.0f * easeIO((t - 300) / 1400.0f));
  else if (t >= 1700 && t < 2000) rpmV = 9000;
  else if (t >= 2000 && t < 3300) rpmV = (int)(9000.0f * (1.0f - easeIO((t - 2000) / 1300.0f)));
  else if (t >= 3300)             rpmV = 0;
  drawGauge(rpmV, reveal);

  // โซนแดงกะพริบตอนเข็มสุด
  if (t >= 1700 && t < 2000 && ((t / 60) & 1)) {
    for (int i = 0; i <= 12; i++) {
      float f = (8000.0f + i * (1000.0f / 12.0f)) / 9000.0f;
      int x, y;
      gpolar(f, (float)(GR + 2), x, y); display.drawPixel(x, y, COL_ON);
      gpolar(f, (float)(GR + 3), x, y); display.drawPixel(x, y, COL_ON);
    }
  }

  // เส้นแบ่งเลื่อนลง
  if (t >= 300) {
    int h = (int)constrain((t - 300) * 64.0f / 500.0f, 0.0f, 64.0f);
    display.drawFastVLine(74, 0, h, COL_ON);
  }

  // แผงขวา: แต่ละแถวสไลด์เข้ามาทีละบรรทัด
  static const char* const names[5] = {"THR", "ECT", "BAT", "MAP", "INJ"};
  static const int ys[5] = {12, 29, 38, 47, 56};
  bool done = (t >= 4300);
  for (int k = 0; k < 6; k++) {
    uint32_t st = 2300 + k * 330;
    if (t < st) continue;
    float p = easeIO(constrain((t - st) / 280.0f, 0.0f, 1.0f));
    int off = (int)((1.0f - p) * 52.0f);
    if (k == 0) {                                   // แถวสถานะ
      display.fillRect(78 + off, 0, 36, 10, COL_ON);
      display.setTextColor(COL_OFF, COL_ON);
      display.setCursor(80 + off, 1);
      display.print(done ? "ALL OK" : "CHECK");
      display.setTextColor(COL_ON);
    } else {
      int i = k - 1;
      display.setCursor(78 + off, ys[i]);
      display.print(names[i]);
      display.print(done ? "  OK" : (((t / 150) & 1) ? "  .." : "  --"));
      if (i == 0) {                                 // แถบคันเร่ง เติมจนเต็มแล้วว่าง
        display.drawRect(78 + off, 21, 48, 6, COL_ON);
        float bp = done ? 0.0f : fabsf(sinf(t / 180.0f));
        display.fillRect(80 + off, 23, (int)(44.0f * bp), 2, COL_ON);
      }
    }
  }
  if (t >= 300) {                                   // จุดบลูทูธ
    if (SerialBT.hasClient()) display.fillCircle(122, 4, 3, COL_ON);
    else                      display.drawCircle(122, 4, 3, COL_ON);
  }

  // ปาดจอออกจากซ้ายไปขวาก่อนเข้าหน้าเข็มไมล์จริง
  if (t >= GAUGE_MS - 350) {
    int w = (int)(OLED_W * (t - (GAUGE_MS - 350)) / 350.0f);
    display.fillRect(0, 0, w, OLED_H, COL_OFF);
    display.drawFastVLine(w, 0, OLED_H, COL_ON);
  }
}

static void bootGauge() {
  if (!oledOk) return;
  uint32_t t0 = millis();
  bool f1 = false;
  for (;;) {
    uint32_t t = millis() - t0;
    if (t >= GAUGE_MS) break;
    display.clearDisplay();
    drawGaugeIntroFrame(t);
    if (t >= 4300 && !f1) { f1 = true; flashInvert(60); }
    display.display();
  }
}

// ======================= setup / loop =======================
uint32_t tEngine = 0, tTele = 0, tDisp = 0, tDbg = 0;
bool wasConnected = false;

void setup() {
  Serial.begin(115200);
  randomSeed(micros());

  Wire.begin(OLED_SDA, OLED_SCL);
#if OLED_SH1106
  oledOk = display.begin(OLED_ADDR, true);
#else
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
#endif
  if (!oledOk) Serial.println("ไม่พบจอ OLED - ตรวจ SDA/SCL/ที่อยู่ I2C (ระบบยังทำงานผ่านบลูทูธได้)");

  SerialBT.begin(BT_NAME);
  Serial.print("Bluetooth พร้อม ชื่ออุปกรณ์: ");
  Serial.println(BT_NAME);

  bootSplash();            // หน้าโหลดโลโก้ PUXXZA 10 วินาที
  bootGauge();             // หน้าโหลดเข็มไมล์ 5 วินาที

  tEngine = millis();
  tTele = tEngine; tDisp = tEngine;
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

  if (now - tDisp >= 60) {
    tDisp = now;
    drawScreen();
  }

  if (now - tDbg >= 1000) {
    tDbg = now;
    Serial.printf("[%s] rpm=%d tps=%.0f ect=%.0f vbat=%.1f map=%.0f adv=%.0f inj=%.2f BT=%d\n",
                  stateName(), rpmOut, tpsOut, ect, vbat, mapKpa, advDeg, injMs, conn ? 1 : 0);
  }
}
