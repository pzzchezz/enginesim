/*
  ESP32 WiFi Repeater (NAT) + OLED status display + วัดความเร็ว
  - ต้องใช้ Arduino-ESP32 core 3.x ขึ้นไป (ไม่ใช่ 2.0.x)
  - ไลบรารีที่ต้องติดตั้ง: U8g2 (by oliver)
  - บอร์ด: ESP32 Dev Module
  - WiFi บ้านต้องเป็น 2.4GHz

  IN  = ความเร็วที่ ESP32 รับมาจากเราเตอร์บ้าน
  OUT = ความเร็วที่ ESP32 ปล่อยให้เครื่องที่มาต่อ

  ไม่ได้ทดสอบกับบอร์ดจริง
*/

#include <WiFi.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "esp_netif.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"

// ================== แก้ตรงนี้ ==================
const char* HOME_SSID = "Pooi_2.4GHz";
const char* HOME_PASS = "phuhlxn123";
const char* AP_SSID   = "PUXXZA123";
const char* AP_PASS   = "87654321";   // อย่างน้อย 8 ตัว
const int   AP_MAX_CLIENTS = 8;

// ชนิดจอ OLED: 1 = SH1106, 0 = SSD1306
#define OLED_IS_SH1106 1

// ขา I2C ของจอ (ค่ามาตรฐาน ESP32 คือ SDA=21, SCL=22)
#define OLED_SDA 21
#define OLED_SCL 22
// ===============================================

#if OLED_IS_SH1106
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);
#else
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);
#endif

// ---------- ตัววัดความเร็ว (ดักแพ็กเก็ตที่ผ่าน lwIP) ----------
static volatile uint32_t inBytes = 0;    // ขาเข้าจากเราเตอร์บ้าน (STA rx)
static volatile uint32_t outBytes = 0;   // ขาออกไปหาเครื่องที่ต่อ (AP tx)
static netif_input_fn      staInputOrig = nullptr;
static netif_linkoutput_fn apOutputOrig = nullptr;
bool hooksOk = false;

static err_t staInputHook(struct pbuf* p, struct netif* n) {
  inBytes += p->tot_len;           // อ่านก่อนส่งต่อ เพราะ p อาจถูกคืนหลังจากนั้น
  return staInputOrig(p, n);
}

static err_t apOutputHook(struct netif* n, struct pbuf* p) {
  uint32_t len = p->tot_len;
  err_t e = apOutputOrig(n, p);
  if (e == ERR_OK) outBytes += len;
  return e;
}

void installHooks() {
  esp_netif_t* sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  esp_netif_t* ap  = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  struct netif* sn = sta ? (struct netif*)esp_netif_get_netif_impl(sta) : nullptr;
  struct netif* an = ap  ? (struct netif*)esp_netif_get_netif_impl(ap)  : nullptr;

  if (sn && !staInputOrig && sn->input) {
    staInputOrig = sn->input;
    sn->input = staInputHook;
  }
  if (an && !apOutputOrig && an->linkoutput) {
    apOutputOrig = an->linkoutput;
    an->linkoutput = apOutputHook;
  }
  hooksOk = (staInputOrig && apOutputOrig);
}

float inBps = 0, outBps = 0;           // บิตต่อวินาที (เฉลี่ยให้เนียน)
uint32_t lastIn = 0, lastOut = 0;
unsigned long lastCalc = 0;

void updateSpeed() {
  unsigned long now = millis();
  unsigned long dt = now - lastCalc;
  if (dt < 1000) return;
  uint32_t i = inBytes, o = outBytes;
  float curIn  = (uint32_t)(i - lastIn)  * 8.0f * 1000.0f / dt;
  float curOut = (uint32_t)(o - lastOut) * 8.0f * 1000.0f / dt;
  inBps  = inBps  * 0.4f + curIn  * 0.6f;
  outBps = outBps * 0.4f + curOut * 0.6f;
  lastIn = i; lastOut = o; lastCalc = now;
}

String fmtRate(float bps) {
  char buf[16];
  if (bps < 1000.0f)           snprintf(buf, sizeof(buf), "%.0f bps", bps);
  else if (bps < 1000000.0f)   snprintf(buf, sizeof(buf), "%.1f kbps", bps / 1000.0f);
  else                         snprintf(buf, sizeof(buf), "%.2f Mbps", bps / 1000000.0f);
  return String(buf);
}

// ---------- จอ ----------
bool napStarted = false;
unsigned long lastDraw = 0;
unsigned long lastReconnect = 0;

String fit(String s, size_t maxLen) {
  if (s.length() <= maxLen) return s;
  return s.substring(0, maxLen - 1) + "~";
}

String uptimeStr() {
  unsigned long s = millis() / 1000;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", s / 3600, (s / 60) % 60, s % 60);
  return String(buf);
}

void drawScreen(const char* statusLine) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);

  // หัว: เวลาที่เปิดมา + สถานะ
  u8g2.drawStr(0, 9, uptimeStr().c_str());
  int w = u8g2.getStrWidth(statusLine);
  u8g2.drawStr(128 - w, 9, statusLine);
  u8g2.drawHLine(0, 11, 128);

  bool connected = (WiFi.status() == WL_CONNECTED);
  String l;

  l = "Home: " + fit(connected ? WiFi.SSID() : String(HOME_SSID), 15);
  u8g2.drawStr(0, 22, l.c_str());

  l = "IP: " + (connected ? WiFi.localIP().toString() : String("-"));
  u8g2.drawStr(0, 32, l.c_str());

  if (hooksOk) {
    l = "IN : " + fmtRate(inBps);
    u8g2.drawStr(0, 42, l.c_str());
    l = "OUT: " + fmtRate(outBps);
    u8g2.drawStr(0, 52, l.c_str());
  } else {
    u8g2.drawStr(0, 42, "IN : n/a");
    u8g2.drawStr(0, 52, "OUT: n/a");
  }

  l = "AP:" + fit(String(AP_SSID), 7) + " Cl:" + String(WiFi.softAPgetStationNum());
  if (connected) l += " " + String(WiFi.RSSI()) + "dB";
  u8g2.drawStr(0, 62, l.c_str());

  u8g2.sendBuffer();
}

void startRepeater() {
  // ให้ AP แจก DNS เดียวกับที่ได้จากเราเตอร์บ้าน (ถ้าไม่มีใช้ 8.8.8.8)
  esp_netif_t* ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  IPAddress dnsIp = WiFi.dnsIP(0);
  esp_netif_dns_info_t dns;
  dns.ip.type = IPADDR_TYPE_V4;
  if (dnsIp == IPAddress(0, 0, 0, 0)) {
    dns.ip.u_addr.ip4.addr = ipaddr_addr("8.8.8.8");
  } else {
    dns.ip.u_addr.ip4.addr = (uint32_t)dnsIp;
  }
  uint8_t offer = 1;
  esp_netif_dhcps_stop(ap);
  esp_netif_dhcps_option(ap, ESP_NETIF_OP_SET,
                         ESP_NETIF_DOMAIN_NAME_SERVER, &offer, sizeof(offer));
  esp_netif_set_dns_info(ap, ESP_NETIF_DNS_MAIN, &dns);
  esp_netif_dhcps_start(ap);

  WiFi.AP.enableNAPT(true);
  napStarted = true;
}

void setup() {
  Serial.begin(115200);

  u8g2.begin();
  drawScreen("BOOT");

  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(HOME_SSID, HOME_PASS);

  Serial.print("Connecting");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    drawScreen("CONN..");
    if (millis() - t0 > 30000) {   // ต่อไม่ติดเกิน 30 วิ ลองใหม่
      WiFi.disconnect();
      WiFi.begin(HOME_SSID, HOME_PASS);
      t0 = millis();
    }
  }
  Serial.println();
  Serial.print("STA IP: ");
  Serial.println(WiFi.localIP());

  // AP ต้องใช้ช่องเดียวกับ WiFi บ้าน
  WiFi.softAP(AP_SSID, AP_PASS, WiFi.channel(), 0, AP_MAX_CLIENTS);

  startRepeater();
  installHooks();
  lastCalc = millis();
  Serial.printf("Repeater ready, speed hooks: %s\n", hooksOk ? "OK" : "FAILED");
}

void loop() {
  bool connected = (WiFi.status() == WL_CONNECTED);

  if (!connected && millis() - lastReconnect > 5000) {
    lastReconnect = millis();
    WiFi.reconnect();
  }

  updateSpeed();

  if (millis() - lastDraw > 1000) {
    lastDraw = millis();
    drawScreen(connected ? (napStarted ? "ONLINE" : "WAIT") : "LOST");
  }
  delay(20);
}
