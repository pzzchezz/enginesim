// ESP32 Self-Test ผ่าน BLE  (Arduino IDE / PlatformIO, ESP32 core 2.x หรือ 3.x)
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <Preferences.h>

#define SVC "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RXC "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define TXC "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLECharacteristic *tx;
volatile bool runReq = false;

void sendLine(String s) {
  s += "\n";
  for (size_t i = 0; i < s.length(); i += 20) {
    String part = s.substring(i, i + 20);
    tx->setValue((uint8_t *)part.c_str(), part.length());
    tx->notify();
    delay(12);
  }
}

void rep(const char *name, bool ok, String detail) {
  sendLine(String(name) + "|" + (ok ? "OK" : "FAIL") + "|" + detail);
}

void testFlash() {
  uint32_t sz = ESP.getFlashChipSize();
  rep("flash", sz >= 1048576, String(sz / 1048576) + " MB");
}

void testNVS() {
  Preferences p;
  bool ok = p.begin("selftest", false);
  uint32_t v = esp_random();
  if (ok) {
    p.putUInt("k", v);
    ok = (p.getUInt("k", 0) == v);
    p.end();
  }
  rep("nvs", ok, ok ? "เขียน/อ่านตรงกัน" : "เขียนหรืออ่านไม่ได้");
}

void testRAM() {
  const size_t n = 32768;
  uint8_t *b = (uint8_t *)malloc(n);
  bool ok = b != NULL;
  if (ok) {
    for (size_t i = 0; i < n; i++) b[i] = (i * 7) & 0xFF;
    for (size_t i = 0; i < n && ok; i++) ok = b[i] == ((i * 7) & 0xFF);
    free(b);
  }
  rep("ram", ok, "ว่าง " + String(ESP.getFreeHeap() / 1024) + " KB");
}

void testCPU() {
  uint32_t t = millis();
  volatile uint32_t x = 0;
  for (uint32_t i = 0; i < 200000; i++) x += i * 3;
  uint32_t dt = millis() - t;
  bool ok = (uint32_t)x == (uint32_t)(3ULL * 199999ULL * 200000ULL / 2);
  rep("cpu", ok, String(ESP.getCpuFreqMHz()) + " MHz, " + String(dt) + " ms");
}

void testTemp() {
  float t = temperatureRead();
  bool ok = !isnan(t) && t > -20 && t < 120;
  rep("temp", ok, String(t, 1) + " C");
}

void testWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  int n = WiFi.scanNetworks();
  rep("wifi", n >= 0, n >= 0 ? "พบ " + String(n) + " เครือข่าย" : "สแกนไม่ได้");
  WiFi.scanDelete();
}

void testGPIO() {
  const int pins[] = {4, 13, 14, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
  String bad = "";
  for (int p : pins) {
    pinMode(p, INPUT_PULLUP);
    delay(2);
    bool hi = digitalRead(p);
    pinMode(p, INPUT_PULLDOWN);
    delay(2);
    bool lo = !digitalRead(p);
    pinMode(p, INPUT);
    if (!hi || !lo) bad += String(p) + " ";
  }
  rep("gpio", bad == "", bad == "" ? "ผ่านทุกขา" : "ผิดปกติขา " + bad);
}

// K-line: โมดูล TX -> GPIO16 (RX2), โมดูล RX -> GPIO17 (TX2)
#define K_RX 16
#define K_TX 17

void testKLine() {
  pinMode(K_TX, OUTPUT);
  pinMode(K_RX, INPUT);
  digitalWrite(K_TX, HIGH); delay(5);
  bool idleHigh = digitalRead(K_RX);
  digitalWrite(K_TX, LOW); delay(5);
  bool lowOk = !digitalRead(K_RX);
  digitalWrite(K_TX, HIGH); delay(5);
  bool highOk = digitalRead(K_RX);

  // ส่งข้อมูลแล้วรอ echo กลับมา (K-line transceiver จะสะท้อนข้อมูลที่ส่ง)
  Serial2.begin(10400, SERIAL_8N1, K_RX, K_TX);
  while (Serial2.available()) Serial2.read();
  const uint8_t pat[] = {0x55, 0xAA, 0x33, 0xC1};
  Serial2.write(pat, 4);
  Serial2.flush();
  delay(60);
  int got = 0;
  bool match = true;
  while (Serial2.available() && got < 4) {
    if (Serial2.read() != pat[got]) match = false;
    got++;
  }
  Serial2.end();
  bool echo = (got == 4 && match);

  bool ok = idleHigh && lowOk && highOk && echo;
  String d;
  if (ok) d = "ปกติ echo ตรง 4/4 ไบต์";
  else {
    if (!idleHigh || !highOk) d += "RX ไม่ขึ้นสูง (เช็ค EN/VCC/ตัวดึงขึ้น) ";
    if (!lowOk) d += "RX ไม่ลงต่ำเมื่อ TX ต่ำ (เช็คสาย) ";
    if (!echo) d += "echo " + String(got) + "/4";
  }
  rep("kline", ok, d);
}

void runAll() {
  sendLine("info|OK|" + String(ESP.getChipModel()) + " rev" + String(ESP.getChipRevision()) +
           " " + String(ESP.getChipCores()) + " คอร์");
  testFlash(); testNVS(); testRAM(); testCPU(); testTemp(); testWiFi(); testGPIO(); testKLine();
  sendLine("DONE");
}

class RxCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    auto v = c->getValue();
    if (v.length() && v[0] == 'T') runReq = true;
  }
};

class SrvCb : public BLEServerCallbacks {
  void onConnect(BLEServer *s) {}
  void onDisconnect(BLEServer *s) { BLEDevice::startAdvertising(); }
};

void setup() {
  BLEDevice::init("ESP32-SelfTest");
  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(new SrvCb());
  BLEService *svc = srv->createService(SVC);
  tx = svc->createCharacteristic(TXC, BLECharacteristic::PROPERTY_NOTIFY);
  tx->addDescriptor(new BLE2902());
  BLECharacteristic *rx = svc->createCharacteristic(
      RXC, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCb());
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void loop() {
  if (runReq) { runReq = false; runAll(); }
  delay(20);
}
