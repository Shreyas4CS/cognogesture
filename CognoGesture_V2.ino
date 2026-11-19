/* =====================================================================
   CognoGesture robot firmware  ·  CognoQuest kit  ·  CognoSpace
   ---------------------------------------------------------------------
   Flash this ONCE. After that, students build programs with blocks in
   the Cogno Gesture app and upload them over Bluetooth. The program is
   saved on the ESP32 and stays there after power off.

   Kit: ESP32 · L298N motor driver · 2 DC motors · 20x4 I2C LCD ·
        0.96" I2C OLED (SSD1306 128x64) · battery pack

   Arduino IDE  ->  Board: "ESP32 Dev Module"
   Libraries (Library Manager):
     - LiquidCrystal I2C  (Frank de Brabander)
     - Adafruit SSD1306   (+ Adafruit GFX, installed with it)
   Works on ESP32 Arduino core 2.x and 3.x.

   Wiring (change the pins below if yours differ)
     L298N  ENA -> 14   IN1 -> 27   IN2 -> 26
            ENB -> 32   IN3 -> 25   IN4 -> 33     (remove the ENA/ENB jumpers)
     LCD + OLED share I2C:  SDA -> 21   SCL -> 22   VCC -> 5V / 3V3   GND -> GND
     Built-in LED: GPIO 2
   ===================================================================== */
#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#if __has_include("esp_arduino_version.h")
  #include "esp_arduino_version.h"
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

/* ---------- set to 0 if a display is not in your kit ---------- */
#define USE_LCD  1
#define USE_OLED 1
#if USE_LCD
  #include <LiquidCrystal_I2C.h>
#endif
#if USE_OLED
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#endif

/* ---------- SETTINGS YOU MAY WANT TO CHANGE ---------- */
const char* DEVICE_NAME = "CognoGesture-Bot";   // the app looks for names starting with "CognoGesture"
const int ENA = 14, IN1 = 27, IN2 = 26;         // left motor
const int ENB = 32, IN3 = 25, IN4 = 33;         // right motor
const int LED_PIN = 2;
const int SDA_PIN = 21, SCL_PIN = 22;
const bool INVERT_LEFT = false, INVERT_RIGHT = false, SWAP_SIDES = false;   // fix a motor that spins the wrong way
const uint32_t SIGNAL_TIMEOUT_MS = 1500;        // no message from the app for this long -> program stops (safety)
const char* FW_VERSION = "2.0";

/* ---------- Bluetooth (Nordic UART service, same as the app) ---------- */
#define SVC_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_UUID  "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define TX_UUID  "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

/* ---------- program (bytecode made by the block compiler in the app) ---------- */
enum Op { OP_END, OP_MOVE, OP_STOP, OP_WAIT, OP_SPEED, OP_JMP, OP_PUSHN, OP_LOOPN, OP_IFG, OP_WAITG, OP_CALL, OP_RET,
          OP_LCD, OP_LCDCLR, OP_OLED, OP_OLEDCLR, OP_FACE, OP_LED, OP_VSET, OP_VADD, OP_FOLLOW, OP_HALT, OP_UNTILG };
const int MAX_INS = 700, MAX_STR = 96, MAX_G = 24, MAX_TH = 24;
struct Ins { uint8_t op; int32_t a, b, c; };
struct Gest { int8_t act; int16_t spd; String name; };
struct ThDef { uint8_t type; int16_t arg; int16_t pc; };     // type 0 = when program starts, 1 = when gesture seen, 2 = when nothing seen
struct Thread { bool on; int pc; uint8_t wait; uint32_t until; int32_t warg; int lsp; int32_t ls[8]; int csp; int16_t cs[8]; };
Ins prog[MAX_INS]; int nIns = 0;
String strs[MAX_STR]; int nStr = 0;
Gest gest[MAX_G + 1]; int nG = 0;
ThDef thDef[MAX_TH]; int nTh = 0;
Thread th[MAX_TH];
String progName = "", modelName = "";
int startSpeed = 40, trim = 0;
bool hasProg = false;

/* ---------- runtime state ---------- */
bool running = false, bleOn = false, lcdOk = false, oledOk = false;
int speedPct = 40, counter = 0, gesture = 0, lastGesture = 0;
int curL = 0, curR = 0;
uint32_t lastHeard = 0, jogUntil = 0, lastTele = 0;
bool teleDue = true, idleDue = true;
BLECharacteristic* txChar = nullptr;
QueueHandle_t rxQ;
String rxLine = "", upBuf = "";
bool uploading = false;

#if USE_LCD
LiquidCrystal_I2C* lcd = nullptr;
String lcdRows[4];
#endif
#if USE_OLED
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledDirty = false; uint32_t oledFlushAt = 0;
#endif

/* =====================================================================
   MOTORS
   ===================================================================== */
const int PWM_FREQ = 1000, PWM_RES = 8;
void pwmInit(){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(ENA, PWM_FREQ, PWM_RES); ledcAttach(ENB, PWM_FREQ, PWM_RES);
#else
  ledcSetup(0, PWM_FREQ, PWM_RES); ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(ENA, 0); ledcAttachPin(ENB, 1);
#endif
}
void pwmOut(int pin, int ch, int duty){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}
void wheel(int en, int ch, int a, int b, int v){
  v = constrain(v, -100, 100);
  digitalWrite(a, v > 0 ? HIGH : LOW); digitalWrite(b, v < 0 ? HIGH : LOW);
  pwmOut(en, ch, abs(v) * 255 / 100);
}
void setMotors(int l, int r){
  l = constrain(l, -100, 100); r = constrain(r, -100, 100);
  if (l != curL || r != curR){ curL = l; curR = r; teleDue = true; }
  int L = INVERT_LEFT ? -l : l, R = INVERT_RIGHT ? -r : r;
  if (SWAP_SIDES){ int t = L; L = R; R = t; }
  wheel(ENA, 0, IN1, IN2, L); wheel(ENB, 1, IN3, IN4, R);
}
/* directions, same order as the app: forward, reverse, left, right, fwd-left, fwd-right, back-left, back-right, stop */
const int8_t LR[9][2] = {{100,100},{-100,-100},{-100,100},{100,-100},{35,100},{100,35},{-35,-100},{-100,-35},{0,0}};
void drive(int dir, int spd){
  if (dir < 0 || dir > 8){ setMotors(0, 0); return; }
  int l = LR[dir][0] * spd / 100, r = LR[dir][1] * spd / 100;
  if (l == r && l != 0){ int s = l > 0 ? 1 : -1; l += trim * s; r -= trim * s; }   // trim keeps it straight
  setMotors(l, r);
}

/* =====================================================================
   BLUETOOTH
   ===================================================================== */
void bleSend(const String& s){
  if (!bleOn || !txChar) return;
  String m = s + "\n";
  for (size_t i = 0; i < m.length(); i += 20){
    size_t e = i + 20 < m.length() ? i + 20 : m.length();
    String part = m.substring(i, e);
    txChar->setValue((uint8_t*)part.c_str(), part.length()); txChar->notify(); delay(3);
  }
}
class SrvCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override { bleOn = true; idleDue = true; }
  void onDisconnect(BLEServer* s) override { bleOn = false; idleDue = true; BLEDevice::startAdvertising(); }
};
class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    auto v = c->getValue();                                   // std::string (core 2.x) or String (core 3.x)
    for (size_t i = 0; i < v.length(); i++){ char ch = v[i]; xQueueSend(rxQ, &ch, 0); }
  }
};
void bleInit(){
  BLEDevice::init(DEVICE_NAME);
  BLEServer* srv = BLEDevice::createServer(); srv->setCallbacks(new SrvCB());
  BLEService* svc = srv->createService(SVC_UUID);
  txChar = svc->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY); txChar->addDescriptor(new BLE2902());
  BLECharacteristic* rx = svc->createCharacteristic(RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising(); adv->addServiceUUID(SVC_UUID); adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

/* =====================================================================
   DISPLAYS  (both optional, found automatically on the I2C bus)
   ===================================================================== */
bool i2cHas(uint8_t a){ Wire.beginTransmission(a); return Wire.endTransmission() == 0; }
String gName(){ return gesture > 0 && gesture <= nG ? gest[gesture].name : String("-"); }
String fmt(int si){                               // {g} gesture  {s} speed  {n} count  {m} model
  String t = si >= 0 && si < nStr ? strs[si] : String("");
  t.replace("{g}", gName()); t.replace("{s}", String(speedPct) + "%"); t.replace("{n}", String(counter)); t.replace("{m}", modelName);
  return t;
}
void lcdLine(int row, String t){
#if USE_LCD
  if (!lcdOk || row < 0 || row > 3) return;
  t = t.substring(0, 20); while (t.length() < 20) t += ' ';
  if (lcdRows[row] == t) return;
  lcdRows[row] = t; lcd->setCursor(0, row); lcd->print(t);
#endif
}
void lcdClear(){
#if USE_LCD
  if (!lcdOk) return;
  lcd->clear(); for (int i = 0; i < 4; i++) lcdRows[i] = "                    ";
#endif
}
void oledText(int line, String t, int size){
#if USE_OLED
  if (!oledOk) return;
  size = size == 2 ? 2 : 1; int h = 8 * size, rows = 64 / h; line = constrain(line, 0, rows - 1);
  oled.fillRect(0, line * h, 128, h, SSD1306_BLACK);
  oled.setTextSize(size); oled.setTextColor(SSD1306_WHITE); oled.setCursor(0, line * h); oled.print(t.substring(0, size == 2 ? 10 : 21));
  oledDirty = true;
#endif
}
void oledClear(){
#if USE_OLED
  if (!oledOk) return;
  oled.clearDisplay(); oledDirty = true;
#endif
}
/* robot eyes: 0 happy, 1 sad, 2 angry, 3 surprised, 4 sleepy, 5 love */
void oledFace(int k){
#if USE_OLED
  if (!oledOk) return;
  oled.clearDisplay();
  const int lx = 22, rx = 74, y = 10, w = 32, h = 40;
  if (k == 3){ oled.fillCircle(lx + 16, 30, 17, SSD1306_WHITE); oled.fillCircle(rx + 16, 30, 17, SSD1306_WHITE); oled.fillCircle(lx + 16, 30, 6, SSD1306_BLACK); oled.fillCircle(rx + 16, 30, 6, SSD1306_BLACK); }
  else if (k == 4){ oled.fillRoundRect(lx, 34, w, 6, 3, SSD1306_WHITE); oled.fillRoundRect(rx, 34, w, 6, 3, SSD1306_WHITE); }
  else if (k == 5){
    for (int e = 0; e < 2; e++){ int x = e ? rx + 16 : lx + 16; oled.fillCircle(x - 8, 24, 9, SSD1306_WHITE); oled.fillCircle(x + 8, 24, 9, SSD1306_WHITE); oled.fillTriangle(x - 17, 27, x + 17, 27, x, 48, SSD1306_WHITE); }
  } else {
    oled.fillRoundRect(lx, y, w, h, 10, SSD1306_WHITE); oled.fillRoundRect(rx, y, w, h, 10, SSD1306_WHITE);
    if (k == 0){ oled.fillCircle(lx + 16, y + h + 6, 22, SSD1306_BLACK); oled.fillCircle(rx + 16, y + h + 6, 22, SSD1306_BLACK); }
    if (k == 1){ oled.fillTriangle(lx, y - 1, lx + w, y - 1, lx, y + 18, SSD1306_BLACK); oled.fillTriangle(rx, y - 1, rx + w, y - 1, rx + w, y + 18, SSD1306_BLACK); }
    if (k == 2){ oled.fillTriangle(lx, y - 1, lx + w, y - 1, lx + w, y + 18, SSD1306_BLACK); oled.fillTriangle(rx, y - 1, rx + w, y - 1, rx, y + 18, SSD1306_BLACK); }
  }
  oledDirty = true;
#endif
}
void showIdle(){
  idleDue = false;
  String st = bleOn ? "APP CONNECTED" : "WAITING FOR APP";
  String pn = hasProg ? progName : String("NO PROGRAM YET");
  lcdClear(); lcdLine(0, " COGNOQUEST  ROBOT"); lcdLine(1, st); lcdLine(2, pn); lcdLine(3, hasProg ? "PRESS RUN IN APP" : "UPLOAD FROM APP");
#if USE_OLED
  if (oledOk){ oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE); oled.setTextSize(2); oled.setCursor(4, 0); oled.print("CognoQuest");
    oled.drawFastHLine(0, 19, 128, SSD1306_WHITE); oled.setTextSize(1);
    oled.setCursor(0, 26); oled.print(st); oled.setCursor(0, 40); oled.print(pn.substring(0, 21)); oled.setCursor(0, 54); oled.print(hasProg ? "Press Run in the app" : "Upload from the app"); oledDirty = true; }
#endif
}

/* =====================================================================
   PROGRAM: run / stop / events
   ===================================================================== */
void sendTele(){ bleSend("M," + String(curL) + "," + String(curR) + "," + String(speedPct) + "," + String(running ? 1 : 0)); lastTele = millis(); teleDue = false; }
void startThread(int i){ Thread& t = th[i]; t.on = true; t.pc = thDef[i].pc; t.wait = 0; t.lsp = 0; t.csp = 0; }
void runProgram(){
  if (!hasProg){ bleSend("ER,No program on the robot. Upload one first."); return; }
  running = true; speedPct = startSpeed; counter = 0; gesture = lastGesture = 0; lastHeard = millis();
  setMotors(0, 0); lcdClear(); oledClear();
  for (int i = 0; i < nTh; i++){ th[i].on = false; if (thDef[i].type == 0) startThread(i); }
  bleSend("RUN,1"); teleDue = true;
}
void stopProgram(){
  bool was = running; running = false;
  for (int i = 0; i < MAX_TH; i++) th[i].on = false;
  setMotors(0, 0); digitalWrite(LED_PIN, LOW);
  if (was){ bleSend("RUN,0"); idleDue = true; }
  teleDue = true;
}
bool seen(int g){ return g < 0 ? gesture > 0 : gesture == g; }       // -1 = any gesture, 0 = nothing
void onGesture(int g){
  lastHeard = millis();
  if (g < 0 || g > nG) g = 0;
  gesture = g;
  if (!running || g == lastGesture) { lastGesture = g; return; }
  lastGesture = g;
  for (int i = 0; i < nTh; i++)
    if ((thDef[i].type == 1 && (thDef[i].arg == g || thDef[i].arg == -1) && g > 0) || (thDef[i].type == 2 && g == 0)) startThread(i);   // events restart their script
}
/* "drive by gestures": every gesture does the move chosen for it in the app */
void follow(){
  int g = gesture;
  if (g <= 0 || g > nG){ setMotors(0, 0); return; }
  int a = gest[g].act;
  if (a == 9){ if (speedPct != gest[g].spd){ speedPct = gest[g].spd; teleDue = true; } setMotors(0, 0); }
  else drive(a, speedPct);
}
void stepThread(int i){
  Thread& t = th[i]; uint32_t now = millis();
  if (t.wait == 1 || t.wait == 3){ if ((int32_t)(now - t.until) < 0) return; if (t.wait == 3) setMotors(0, 0); t.wait = 0; }
  else if (t.wait == 2){ if (!seen(t.warg)) return; t.wait = 0; }
  else if (t.wait == 4){ follow(); return; }
  for (int n = 0; n < 64; n++){
    if (t.pc < 0 || t.pc >= nIns){ t.on = false; return; }
    Ins& I = prog[t.pc++];
    switch (I.op){
      case OP_END: t.on = false; return;
      case OP_MOVE: drive(I.a, I.c >= 0 ? I.c : speedPct); if (I.b > 0){ t.wait = 3; t.until = now + I.b; return; } break;
      case OP_STOP: setMotors(0, 0); break;
      case OP_WAIT: t.wait = 1; t.until = now + I.a; return;
      case OP_SPEED: speedPct = constrain(I.a, 0, 100); teleDue = true; break;
      case OP_JMP: { bool back = I.a < t.pc; t.pc = I.a; if (back) return; break; }      // loops give other scripts a turn
      case OP_PUSHN: if (I.a <= 0) t.pc = I.b; else if (t.lsp < 8) t.ls[t.lsp++] = I.a; break;
      case OP_LOOPN: if (t.lsp > 0 && --t.ls[t.lsp - 1] > 0){ t.pc = I.a; return; } if (t.lsp > 0) t.lsp--; break;
      case OP_IFG: if (!seen(I.a)) t.pc = I.b; break;
      case OP_WAITG: if (!seen(I.a)){ t.wait = 2; t.warg = I.a; return; } break;
      case OP_CALL: if (t.csp < 8){ t.cs[t.csp++] = t.pc; t.pc = I.a; } break;
      case OP_RET: if (t.csp > 0) t.pc = t.cs[--t.csp]; else { t.on = false; return; } break;
      case OP_LCD: lcdLine(I.a, fmt(I.b)); break;
      case OP_LCDCLR: lcdClear(); break;
      case OP_OLED: oledText(I.a, fmt(I.b), I.c); break;
      case OP_OLEDCLR: oledClear(); break;
      case OP_FACE: oledFace(I.a); break;
      case OP_LED: digitalWrite(LED_PIN, I.a ? HIGH : LOW); break;
      case OP_VSET: counter = I.a; break;
      case OP_VADD: counter += I.a; break;
      case OP_FOLLOW: t.wait = 4; follow(); return;
      case OP_HALT: stopProgram(); return;
      case OP_UNTILG: if (seen(I.a)) t.pc = I.b; break;
      default: t.on = false; return;
    }
  }
}

/* =====================================================================
   PROGRAM UPLOAD  (text lines from the app, saved to flash)
   ===================================================================== */
String field(const String& s, int idx, bool rest = false){
  int start = 0;
  for (int i = 0; i < idx; i++){ start = s.indexOf(',', start); if (start < 0) return ""; start++; }
  if (rest) return s.substring(start);
  int e = s.indexOf(',', start); return e < 0 ? s.substring(start) : s.substring(start, e);
}
int fi(const String& s, int idx){ return field(s, idx).toInt(); }
bool loadProgram(const String& text){
  stopProgram();
  int eI = -1, eS = -1, eG = -1, eT = -1, lines = 0, endN = -1;
  nIns = nStr = nG = nTh = 0; hasProg = false; progName = ""; modelName = "";
  int p = 0;
  while (p < (int)text.length()){
    int e = text.indexOf('\n', p); if (e < 0) e = text.length();
    String L = text.substring(p, e); L.trim(); p = e + 1;
    if (L.length() < 2) continue;
    String c = field(L, 0);
    if (c == "PE"){ endN = fi(L, 1); break; }
    lines++;
    if (c == "PB"){ eI = fi(L, 1); eS = fi(L, 2); eG = fi(L, 3); eT = fi(L, 4); startSpeed = constrain(fi(L, 5), 0, 100); trim = constrain(fi(L, 6), -30, 30); }
    else if (c == "PN") progName = field(L, 1, true).substring(0, 20);
    else if (c == "PM") modelName = field(L, 1, true).substring(0, 20);
    else if (c == "PG"){ int n = fi(L, 1); if (n < 1 || n > MAX_G) return false; gest[n].act = fi(L, 2); gest[n].spd = fi(L, 3); gest[n].name = field(L, 4, true).substring(0, 20); if (n > nG) nG = n; }
    else if (c == "PS"){ int i = fi(L, 1); if (i < 0 || i >= MAX_STR) return false; strs[i] = field(L, 2, true).substring(0, 40); if (i + 1 > nStr) nStr = i + 1; }
    else if (c == "PT"){ int i = fi(L, 1); if (i < 0 || i >= MAX_TH) return false; thDef[i].type = fi(L, 2); thDef[i].arg = fi(L, 3); thDef[i].pc = fi(L, 4); if (i + 1 > nTh) nTh = i + 1; }
    else if (c == "PI"){ int i = fi(L, 1); if (i < 0 || i >= MAX_INS) return false; prog[i] = {(uint8_t)fi(L, 2), fi(L, 3), fi(L, 4), fi(L, 5)}; if (i + 1 > nIns) nIns = i + 1; }
  }
  if (endN != lines || nIns != eI || nStr != eS || nG != eG || nTh != eT || nIns == 0){ nIns = 0; nTh = 0; return false; }
  hasProg = true; speedPct = startSpeed;
  return true;
}
void loadSaved(){
  File f = LittleFS.open("/prog.txt", "r"); if (!f) return;
  String t = f.readString(); f.close();
  if (!loadProgram(t)) Serial.println("Saved program could not be read");
}
void sendInfo(){ bleSend("I," + String(FW_VERSION) + "," + String(hasProg ? 1 : 0) + "," + String(lcdOk ? 1 : 0) + "," + String(oledOk ? 1 : 0) + "," + String(nIns) + "," + progName); }

/* =====================================================================
   COMMANDS FROM THE APP
     H            hello -> I,fw,hasProgram,lcd,oled,instructions,name
     PB..PE       program upload -> OK,P,n  or  ER,message
     R / X        run / stop the program
     G,n          the gesture the camera sees now (0 = nothing). Also the heartbeat.
     D,l,r / S    wiring check: drive the wheels directly (only when no program runs)
     TR,n         trim
   ===================================================================== */
void handleLine(String L){
  L.trim(); if (!L.length()) return;
  String c = field(L, 0);
  lastHeard = millis();
  if (c == "G"){ onGesture(fi(L, 1)); return; }
  if (c == "PB"){ uploading = true; upBuf = ""; stopProgram(); }
  if (uploading && c.length() == 2 && c[0] == 'P'){
    upBuf += L; upBuf += '\n';
    if (upBuf.length() > 60000){ uploading = false; upBuf = ""; bleSend("ER,Program too big"); return; }
    if (c == "PE"){
      uploading = false;
      if (loadProgram(upBuf)){
        File f = LittleFS.open("/prog.txt", "w"); if (f){ f.print(upBuf); f.close(); }
        bleSend("OK,P," + String(nIns)); idleDue = true;
      } else { bleSend("ER,Upload was not complete. Try again."); loadSaved(); idleDue = true; }
      upBuf = "";
    }
    return;
  }
  if (c == "H"){ sendInfo(); sendTele(); return; }
  if (c == "R"){ runProgram(); return; }
  if (c == "X" || c == "S"){ stopProgram(); jogUntil = 0; return; }
  if (c == "D"){ if (!running){ setMotors(fi(L, 1), fi(L, 2)); jogUntil = millis() + 600; } return; }
  if (c == "TR"){ trim = constrain(fi(L, 1), -30, 30); return; }
}

/* =====================================================================
   SETUP / LOOP
   ===================================================================== */
void setup(){
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(LED_PIN, OUTPUT);
  pwmInit(); setMotors(0, 0);
  Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(400000);
#if USE_LCD
  for (uint8_t a : {0x27, 0x3F}) if (i2cHas(a)){ lcd = new LiquidCrystal_I2C(a, 20, 4); lcd->init(); lcd->backlight(); lcdOk = true; break; }
#endif
#if USE_OLED
  for (uint8_t a : {0x3C, 0x3D}) if (i2cHas(a) && oled.begin(SSD1306_SWITCHCAPVCC, a)){ oledOk = true; break; }
#endif
  Serial.printf("LCD %s, OLED %s\n", lcdOk ? "found" : "not found", oledOk ? "found" : "not found");
  if (!LittleFS.begin(true)) Serial.println("LittleFS failed");
  loadSaved();
  rxQ = xQueueCreate(4096, 1);
  bleInit();
  showIdle();
  Serial.println("CognoGesture robot ready");
}

void loop(){
  uint32_t now = millis();
  char ch;
  while (xQueueReceive(rxQ, &ch, 0) == pdTRUE){
    if (ch == '\n'){ handleLine(rxLine); rxLine = ""; }
    else if (rxLine.length() < 300) rxLine += ch;
  }
  // safety: app gone quiet or Bluetooth lost -> stop
  if (running && (!bleOn || now - lastHeard > SIGNAL_TIMEOUT_MS)){ stopProgram(); Serial.println("Signal lost: program stopped"); }
  if (!running && jogUntil && (int32_t)(now - jogUntil) > 0){ jogUntil = 0; setMotors(0, 0); }
  if (running) for (int i = 0; i < nTh; i++) if (th[i].on) stepThread(i);
  if (!running && idleDue) showIdle();
#if USE_OLED
  if (oledOk && oledDirty && (int32_t)(now - oledFlushAt) >= 0){ oled.display(); oledDirty = false; oledFlushAt = now + 50; }
#endif
  if (bleOn && (teleDue ? now - lastTele > 80 : now - lastTele > 1000)) sendTele();
  delay(2);
}
