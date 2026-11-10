/*
  =====================================================================
  ESBE-Ventilmodul  –  Arduino UNO R4 Minima als Modbus-RTU-Slave (ID 2)
  =====================================================================

  Haengt am selben RS485-Bus wie die Waermepumpe (ID 1) und der
  ESP32-Master. Der ESP schreibt nur die Soll-Stellung, dieses Modul
  faehrt den ESBE-Motor selbststaendig (Relais-Reihenfolge, Laufzeit,
  Endlagen-Ueberlauf, Referenzfahrt) und zeigt alles auf dem OLED.

  Hardware:
    - MAX485 an Serial1 (D0 = RX/RO, D1 = TX/DI), DE+/RE an D6
    - Relais 1 (Motorstrom EIN/AUS)       an D7
    - Relais 2 (Richtung, NC=HZ / NO=WW)  an D8
    - Relais 3 (Reserve 1)                an D9
    - Relais 4 (Reserve 2)                an D10
    - OLED 0,91" 128x32 (JMD0.91A, SSD1306) an I2C (SDA/SCL), Adresse 0x3C

  Bibliotheken (Bibliotheksverwalter):
    - U8g2            (olikraus)
    - ModbusRTUSlave  (C. M. Bulliner, Version 3.x)

  Debug: Serieller Monitor 115200 Baud, "?" fuer Hilfe.

  Holding-Register (Slave 2):
     0  Soll-Stellung 0..100 %   (0 = Heizung, 100 = WW)      ESP schreibt
     1  Befehl: 1 = Referenzfahrt, 2 = Stopp                    ESP schreibt
     2  Ist-Stellung in 0,1 %   (0..1000)                       lesen
     3  Status: Bit0 faehrt->WW, Bit1 faehrt->HZ,
                Bit4 Stellung unbekannt, Bit5 Notfall aktiv,
                Bit6 Handbetrieb (serielle Konsole)             lesen
     4  Laufzeit Endlage->Endlage in s (Standard 130)           ESP schreibt
     5  Notfall nach x min ohne Lebenszeichen (0 = aus)         ESP schreibt
     6  Notfall-Stellung in %                                   ESP schreibt
     7  Lebenszeichen-Zaehler                                   ESP schreibt
     8  VL  x10 (int16, 0x8000 = ungueltig)                     ESP schreibt
     9  RL  x10                                                 ESP schreibt
    10  WW  x10                                                 ESP schreibt
    11  Flags: Bit0 WP ein, Bit1 WW-Ladung aktiv                ESP schreibt
    12  WW-Soll in C                                            ESP schreibt
    13  Kennung 0xE5BE (zum Erkennen des Moduls)                lesen
    14  Reserve-Relais: Bit0 = D9, Bit1 = D10                   ESP schreibt
    15  Reserve-Relais Rueckmeldung (tatsaechlicher Zustand)    lesen
*/

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <ModbusRTUSlave.h>
#include <EEPROM.h>

// ---------------- Einstellungen ----------------
#define SLAVE_ID          2
#define MB_BAUD           9600
#define PIN_DE            6
#define PIN_PWR           7       // Relais Motorstrom
#define PIN_DIR           8       // Relais Richtung
#define PIN_AUX1          9       // Reserve-Relais 1
#define PIN_AUX2          10      // Reserve-Relais 2
#define RELAY_ACTIVE_LOW  true    // Relaismodul schaltet bei LOW (typisch fuer 4-fach-Module); false = schaltet bei HIGH
#define DIR_INVERT        false   // true, wenn NC/NO andersherum verdrahtet sind
#define AUX_OFF_IN_FAILSAFE true  // Reserve-Relais aus, wenn der ESP sich nicht mehr meldet

// ---------------- Register ----------------
enum {
  R_TARGET = 0, R_CMD, R_POS, R_STATUS, R_TRAVEL, R_FS_MIN, R_FS_POS,
  R_HB, R_VL, R_RL, R_WW, R_FLAGS, R_WWSOLL, R_ID, R_AUX, R_AUXFB, NREG
};
uint16_t regs[NREG];

static const uint16_t NO_TARGET = 0xFFFF;
static const uint16_t T_INVALID = 0x8000;

ModbusRTUSlave modbus(Serial1, PIN_DE);
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
bool    oledOk = false;
uint8_t oledAddr = 0;

// ---------------- gespeicherter Zustand ----------------
struct Persist {
  uint32_t magic;
  float    pos;
  uint8_t  known;
  uint8_t  moving;
  uint16_t travel;
  uint16_t fsMin;
  uint16_t fsPos;
};
static const uint32_t PERSIST_MAGIC = 0xE5BE0002;
Persist ps;

void savePersist() { EEPROM.put(0, ps); }

void loadPersist() {
  EEPROM.get(0, ps);
  if (ps.magic != PERSIST_MAGIC || ps.travel < 10 || ps.travel > 600) {
    ps = {PERSIST_MAGIC, 0.0f, 0, 0, 130, 10, 0};
    savePersist();
  }
}

// ---------------- Debug ----------------
bool     verbose = false;
uint32_t mbFrames = 0;           // gueltige Modbus-Anfragen an diese ID
uint32_t lastFrameMs = 0;
int      manualTarget = -1;      // Handbetrieb ueber die Konsole (-1 = ESP bestimmt)
uint8_t  auxManual = 0xFF;       // 0xFF = Reserve-Relais folgen dem ESP

void logLine(const char* s) {
  Serial.print('['); Serial.print(millis() / 1000); Serial.print("s] "); Serial.println(s);
}

// ---------------- Ventil ----------------
enum Phase : uint8_t { PH_IDLE, PH_DIR, PH_RUN, PH_STOP };
const char* PHASE_NAME[] = {"steht", "Richtung setzen", "faehrt", "stoppt"};
Phase    phase = PH_IDLE;
uint32_t phaseMs = 0, runMs = 0;
float    pos = 0, startPos = 0;
bool     known = false, dirWW = false, interrupted = false;
int      curTarget = -1;      // Ziel der laufenden / letzten Fahrt
int      queued = -1;         // naechstes Ziel (nach Referenzfahrt / Unterbrechung)
int      lastCommanded = -1;  // zuletzt angefahrenes Ziel
bool     failsafe = false;
uint16_t lastHb = 0;
uint32_t lastHbMs = 0;
bool     everHeard = false;

void relay(uint8_t pin, bool on) { digitalWrite(pin, RELAY_ACTIVE_LOW ? !on : on); }
bool relayState(uint8_t pin)     { return RELAY_ACTIVE_LOW ? !digitalRead(pin) : digitalRead(pin); }
void dirRelay(bool towardWW)     { relay(PIN_DIR, DIR_INVERT ? !towardWW : towardWW); }

float posNow() {
  if (phase != PH_RUN) return pos;
  float d = (millis() - phaseMs) / 1000.0f / ps.travel * 100.0f;
  float p = startPos + (dirWW ? d : -d);
  return constrain(p, 0.0f, 100.0f);
}

void stopMove() {                          // laufende Fahrt sauber beenden
  if (phase == PH_RUN) pos = posNow();
  if (phase == PH_IDLE) return;
  relay(PIN_PWR, false);
  interrupted = true;
  phase = PH_STOP;
  phaseMs = millis();
}

void beginMove(int target) {
  target = constrain(target, 0, 100);
  if (phase != PH_IDLE) {                  // erst anhalten, dann neu starten
    queued = target;
    stopMove();
    return;
  }
  bool toEnd = (target == 0 || target == 100);
  if (!toEnd && !known) {                  // Mittelstellung nur von bekannter Lage
    queued = target;
    target = 0;
    toEnd = true;
  }
  float from = known ? pos : (target == 0 ? 100.0f : 0.0f);
  float dist = fabsf(target - from);
  runMs = (uint32_t)(dist / 100.0f * ps.travel * 1000.0f);
  if (toEnd) runMs += ps.travel * 100UL;   // +10 % Ueberlauf in den Endschalter
  if (!known && toEnd) runMs = ps.travel * 1100UL;

  dirWW = (target == 100) || (target > 0 && target > from);
  curTarget = target;
  startPos = from;
  interrupted = false;
  relay(PIN_PWR, false);
  dirRelay(dirWW);
  phase = PH_DIR;
  phaseMs = millis();
  ps.moving = 1; ps.pos = pos; ps.known = known; savePersist();
  char b[64];
  snprintf(b, sizeof(b), "Fahre %s auf %d %% (%lu s)", dirWW ? "Richtung WW" : "Richtung HZ", target,
           (unsigned long)(runMs / 1000));
  logLine(b);
}

void valveTick() {
  uint32_t ms = millis();
  switch (phase) {
    case PH_IDLE: break;
    case PH_DIR:
      if (ms - phaseMs >= 300) { relay(PIN_PWR, true); phase = PH_RUN; phaseMs = ms; }
      break;
    case PH_RUN:
      if (ms - phaseMs >= runMs) { relay(PIN_PWR, false); pos = curTarget; phase = PH_STOP; phaseMs = ms; }
      break;
    case PH_STOP:
      if (ms - phaseMs >= 300) {
        dirRelay(false);                   // Richtungsrelais in Ruhe
        if (!interrupted) { pos = curTarget; known = true; }
        phase = PH_IDLE;
        ps.pos = pos; ps.known = known; ps.moving = 0; savePersist();
        char b[48];
        snprintf(b, sizeof(b), "Stellung %d %%%s", (int)(pos + 0.5f), known ? "" : " (unsicher)");
        logLine(b);
        if (queued >= 0) { int q = queued; queued = -1; beginMove(q); }
      }
      break;
  }
}

// ---------------- Reserve-Relais ----------------
void auxTick() {
  uint8_t want = (auxManual != 0xFF) ? auxManual : (uint8_t)(regs[R_AUX] & 0x03);
  if (failsafe && AUX_OFF_IN_FAILSAFE && auxManual == 0xFF) want = 0;
  static uint8_t last = 0xFF;
  if (want != last) {
    relay(PIN_AUX1, want & 0x01);
    relay(PIN_AUX2, want & 0x02);
    if (last != 0xFF) {
      char b[48];
      snprintf(b, sizeof(b), "Reserve-Relais: D9 %s, D10 %s", (want & 1) ? "EIN" : "aus", (want & 2) ? "EIN" : "aus");
      logLine(b);
    }
    last = want;
  }
  regs[R_AUXFB] = (relayState(PIN_AUX1) ? 1 : 0) | (relayState(PIN_AUX2) ? 2 : 0);
}

// ---------------- Modbus-Auswertung ----------------
void handleRegisters() {
  // Lebenszeichen
  if (regs[R_HB] != lastHb) {
    lastHb = regs[R_HB]; lastHbMs = millis();
    if (!everHeard) logLine("ESP meldet sich");
    everHeard = true;
  }

  // Einstellungen uebernehmen und speichern, wenn geaendert
  uint16_t tr = constrain(regs[R_TRAVEL], 10, 600);
  uint16_t fm = min(regs[R_FS_MIN], (uint16_t)240);
  uint16_t fp = min(regs[R_FS_POS], (uint16_t)100);
  if (tr != ps.travel || fm != ps.fsMin || fp != ps.fsPos) {
    ps.travel = tr; ps.fsMin = fm; ps.fsPos = fp; savePersist();
    char b[64];
    snprintf(b, sizeof(b), "Einstellungen: Laufzeit %u s, Notfall %u min -> %u %%", tr, fm, fp);
    logLine(b);
  }

  // Befehle
  if (regs[R_CMD] == 1) {                  // Referenzfahrt
    regs[R_CMD] = 0;
    stopMove();
    known = false;
    lastCommanded = -1;
    logLine("Referenzfahrt angefordert (ESP)");
  } else if (regs[R_CMD] == 2) {           // Stopp
    regs[R_CMD] = 0;
    queued = -1;
    stopMove();
    lastCommanded = (regs[R_TARGET] <= 100) ? regs[R_TARGET] : lastCommanded;
    logLine("Stopp (ESP)");
  } else if (regs[R_CMD] != 0) {
    regs[R_CMD] = 0;
  }

  // Notfall: ESP meldet sich nicht mehr
  uint32_t silentMs = millis() - lastHbMs;
  bool fs = ps.fsMin > 0 && silentMs > ps.fsMin * 60000UL;
  if (fs != failsafe) logLine(fs ? "NOTFALL: ESP meldet sich nicht -> Notfall-Stellung" : "Notfall beendet");
  failsafe = fs;

  // gewuenschte Stellung
  int desired = -1;
  if (manualTarget >= 0) desired = manualTarget;
  else if (failsafe) desired = ps.fsPos;
  else if (regs[R_TARGET] <= 100) desired = regs[R_TARGET];

  if (desired >= 0 && phase == PH_IDLE && queued < 0 && (desired != lastCommanded || !known)) {
    lastCommanded = desired;
    if (!known || fabsf(pos - desired) >= 1.0f) beginMove(desired);
  } else if (desired >= 0 && phase != PH_IDLE && desired != curTarget && desired != queued
             && desired != lastCommanded) {
    lastCommanded = desired;               // neues Ziel waehrend der Fahrt
    beginMove(desired);
  }

  // Rueckmeldung
  regs[R_POS] = (uint16_t)lroundf(posNow() * 10.0f);
  uint16_t st = 0;
  if (phase != PH_IDLE) st |= dirWW ? 0x01 : 0x02;
  if (!known) st |= 0x10;
  if (failsafe) st |= 0x20;
  if (manualTarget >= 0) st |= 0x40;
  regs[R_STATUS] = st;
  regs[R_ID] = 0xE5BE;
}

// ---------------- Anzeige 128x32 ----------------
void fmtTemp(char* out, uint16_t raw) {
  if (raw == T_INVALID) { strcpy(out, "--.-"); return; }
  int16_t v = (int16_t)raw;
  sprintf(out, "%s%d.%d", v < 0 ? "-" : "", abs(v) / 10, abs(v) % 10);
}

void drawDisplay() {
  if (!oledOk) return;
  char buf[32], vl[8], rl[8], ww[8];
  float p = posNow();
  bool comm = everHeard && (millis() - lastHbMs < 30000UL);
  bool blink = (millis() / 500) % 2;

  u8g2.clearBuffer();

  // Zeile 1: Zustand links, Prozent/Restzeit rechts
  u8g2.setFont(u8g2_font_6x10_tf);
  const char* st;
  if (phase != PH_IDLE)  st = dirWW ? "> WW" : "> HZ";
  else if (!known)       st = "Lage ?";
  else if (p <= 1)       st = "Heizung";
  else if (p >= 99)      st = "Warmwasser";
  else                   st = "Puffer";
  u8g2.drawStr(0, 8, st);
  if (phase == PH_RUN) {
    long rest = ((long)runMs - (long)(millis() - phaseMs)) / 1000;
    if (rest < 0) rest = 0;
    sprintf(buf, "%d%% %lds", (int)(p + 0.5f), rest);
  } else {
    sprintf(buf, known ? "%d%%" : "%d%%?", (int)(p + 0.5f));
  }
  int wtxt = u8g2.getStrWidth(buf);
  // rechts daneben kleiner Status (MB / NOT / HAND / WW)
  const char* tag = manualTarget >= 0 ? "H" : failsafe ? "N" : comm ? "" : "!";
  int tagW = tag[0] ? 8 : 0;
  u8g2.drawStr(128 - wtxt - tagW, 8, buf);
  if (tag[0] && (blink || manualTarget >= 0)) {
    u8g2.drawBox(121, 0, 7, 9);
    u8g2.setDrawColor(0); u8g2.drawStr(122, 8, tag); u8g2.setDrawColor(1);
  }

  // Zeile 2: Stellungsbalken HZ ... WW
  u8g2.drawFrame(0, 11, 128, 9);
  int w = (int)(p / 100.0f * 124.0f + 0.5f);
  if (w > 0) u8g2.drawBox(2, 13, w, 5);
  if (phase == PH_RUN && blink) {          // Fahrt blinkt an der Spitze
    u8g2.setDrawColor(2);
    u8g2.drawBox(constrain(2 + w - 3, 2, 123), 13, 3, 5);
    u8g2.setDrawColor(1);
  }

  // Zeile 3: Temperaturen bzw. Hinweise
  u8g2.setFont(u8g2_font_5x8_tf);
  if (!comm && !failsafe && manualTarget < 0) {
    u8g2.drawStr(0, 31, everHeard ? "keine Verbindung zum ESP" : "warte auf ESP ...");
  } else if (failsafe && manualTarget < 0) {
    sprintf(buf, "NOTFALL: ESP fehlt -> %u%%", ps.fsPos);
    u8g2.drawStr(0, 31, buf);
  } else {
    fmtTemp(vl, regs[R_VL]); fmtTemp(rl, regs[R_RL]); fmtTemp(ww, regs[R_WW]);
    bool wwLoad = regs[R_FLAGS] & 0x02;
    if (wwLoad && (millis() / 3000) % 2) sprintf(buf, "WW laedt %s > %u C", ww, regs[R_WWSOLL]);
    else sprintf(buf, "VL%s RL%s WW%s", vl, rl, ww);
    u8g2.drawStr(0, 31, buf);
  }
  // Reserve-Relais als kleine Punkte rechts unten
  uint8_t aux = regs[R_AUXFB];
  if (aux & 1) u8g2.drawDisc(119, 28, 2);
  if (aux & 2) u8g2.drawDisc(125, 28, 2);

  u8g2.sendBuffer();
}

// ---------------- Serielle Debug-Konsole ----------------
void printHelp() {
  Serial.println(F(
    "Befehle (Serieller Monitor, Zeilenende 'Neue Zeile'):\n"
    "  ?            Hilfe\n"
    "  s            Status\n"
    "  r            alle Register\n"
    "  v            Verbose an/aus (Status alle 5 s)\n"
    "  g <0..100>   Handbetrieb: Ventil auf x % fahren (ESP wird ignoriert)\n"
    "  hz / ww      Handbetrieb: Endlage Heizung / Warmwasser\n"
    "  auto         Handbetrieb beenden, ESP bestimmt wieder\n"
    "  ref          Referenzfahrt\n"
    "  stop         Fahrt anhalten\n"
    "  t <s>        Laufzeit Endlage-Endlage setzen (10..600 s)\n"
    "  aux <1|2> <0|1>  Reserve-Relais D9/D10 von Hand (bis 'aux auto')\n"
    "  aux auto     Reserve-Relais wieder vom ESP steuern\n"
    "  test         Relais-Klicktest D8, D9, D10 (D7 Motorstrom bleibt AUS)\n"
    "  i2c          I2C-Bus scannen"));
}

void printStatus() {
  char b[96];
  snprintf(b, sizeof(b), "Ventil: %s, Stellung %d %%%s, Ziel %d, Laufzeit %u s",
           PHASE_NAME[phase], (int)(posNow() + 0.5f), known ? "" : " (unbekannt)", curTarget, ps.travel);
  Serial.println(b);
  snprintf(b, sizeof(b), "Betrieb: %s%s, Notfall nach %u min -> %u %%",
           manualTarget >= 0 ? "HAND" : "ESP", failsafe ? " (NOTFALL aktiv)" : "", ps.fsMin, ps.fsPos);
  Serial.println(b);
  snprintf(b, sizeof(b), "ESP: %s, letztes Lebenszeichen vor %lu s, Anfragen %lu (letzte vor %lu ms)",
           everHeard ? "gesehen" : "noch nie", (unsigned long)((millis() - lastHbMs) / 1000),
           (unsigned long)mbFrames, (unsigned long)(lastFrameMs ? millis() - lastFrameMs : 0));
  Serial.println(b);
  snprintf(b, sizeof(b), "Relais: D7 Strom %s, D8 Richtung %s, D9 %s, D10 %s",
           relayState(PIN_PWR) ? "EIN" : "aus", relayState(PIN_DIR) ? "WW" : "HZ",
           relayState(PIN_AUX1) ? "EIN" : "aus", relayState(PIN_AUX2) ? "EIN" : "aus");
  Serial.println(b);
  char vl[8], rl[8], ww[8];
  fmtTemp(vl, regs[R_VL]); fmtTemp(rl, regs[R_RL]); fmtTemp(ww, regs[R_WW]);
  snprintf(b, sizeof(b), "Werte vom ESP: VL %s, RL %s, WW %s, WP %s%s", vl, rl, ww,
           (regs[R_FLAGS] & 1) ? "ein" : "aus", (regs[R_FLAGS] & 2) ? ", WW-Ladung" : "");
  Serial.println(b);
  snprintf(b, sizeof(b), "OLED: %s", oledOk ? "OK" : "nicht gefunden");
  Serial.println(b);
}

void printRegs() {
  static const char* names[NREG] = {"Soll", "Befehl", "Ist x10", "Status", "Laufzeit", "Notfall min",
                                    "Notfall %", "Lebenszeichen", "VL x10", "RL x10", "WW x10", "Flags",
                                    "WW-Soll", "Kennung", "Reserve soll", "Reserve ist"};
  for (int i = 0; i < NREG; i++) {
    char b[48];
    snprintf(b, sizeof(b), "  %2d %-14s %5u  (0x%04X)", i, names[i], regs[i], regs[i]);
    Serial.println(b);
  }
}

void scanI2C() {
  Serial.print("I2C:");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.print(" 0x"); Serial.print(a, HEX); n++; }
  }
  Serial.println(n ? "" : " nichts gefunden");
}

void relayTest() {
  if (phase != PH_IDLE) { Serial.println("Ventil faehrt gerade - Test abgebrochen"); return; }
  Serial.println("Klicktest: D8, D9, D10 je 1 s (D7 bleibt AUS)");
  relay(PIN_PWR, false);
  uint8_t pins[] = {PIN_DIR, PIN_AUX1, PIN_AUX2};
  for (uint8_t p : pins) {
    bool before = relayState(p);
    Serial.print("  D"); Serial.println(p);
    relay(p, !before); delay(1000); relay(p, before); delay(300);
    modbus.poll();
  }
  Serial.println("Test fertig");
}

void handleSerial() {
  static char line[40];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { if (len < sizeof(line) - 1) line[len++] = c; continue; }
    line[len] = 0; len = 0;
    char* cmd = line;
    while (*cmd == ' ') cmd++;
    if (!*cmd) continue;
    char* arg = strchr(cmd, ' ');
    if (arg) { *arg++ = 0; while (*arg == ' ') arg++; }

    if (!strcmp(cmd, "?") || !strcmp(cmd, "help")) printHelp();
    else if (!strcmp(cmd, "s")) printStatus();
    else if (!strcmp(cmd, "r")) printRegs();
    else if (!strcmp(cmd, "v")) { verbose = !verbose; Serial.println(verbose ? "Verbose AN" : "Verbose AUS"); }
    else if (!strcmp(cmd, "g") && arg) { manualTarget = constrain(atoi(arg), 0, 100); logLine("Handbetrieb"); }
    else if (!strcmp(cmd, "hz")) { manualTarget = 0; logLine("Handbetrieb: Heizung"); }
    else if (!strcmp(cmd, "ww")) { manualTarget = 100; logLine("Handbetrieb: Warmwasser"); }
    else if (!strcmp(cmd, "auto")) { manualTarget = -1; lastCommanded = -1; logLine("Handbetrieb beendet"); }
    else if (!strcmp(cmd, "ref")) { stopMove(); known = false; lastCommanded = -1; logLine("Referenzfahrt (Konsole)"); }
    else if (!strcmp(cmd, "stop")) {
      queued = -1; stopMove();
      if (manualTarget < 0) manualTarget = (int)(posNow() + 0.5f);   // stehen bleiben
      logLine("Stopp (Konsole) -> Handbetrieb, 'auto' zum Beenden");
    }
    else if (!strcmp(cmd, "t") && arg) {
      ps.travel = constrain(atoi(arg), 10, 600); regs[R_TRAVEL] = ps.travel; savePersist();
      Serial.print("Laufzeit "); Serial.print(ps.travel); Serial.println(" s (der ESP ueberschreibt das ggf.)");
    }
    else if (!strcmp(cmd, "aux") && arg) {
      if (!strcmp(arg, "auto")) { auxManual = 0xFF; Serial.println("Reserve-Relais: ESP"); }
      else {
        int n = atoi(arg); char* v = strchr(arg, ' ');
        if ((n == 1 || n == 2) && v) {
          if (auxManual == 0xFF) auxManual = regs[R_AUXFB];
          uint8_t bit = n == 1 ? 1 : 2;
          if (atoi(v + 1)) auxManual |= bit; else auxManual &= ~bit;
        } else Serial.println("Beispiel: aux 1 1");
      }
    }
    else if (!strcmp(cmd, "test")) relayTest();
    else if (!strcmp(cmd, "i2c")) scanI2C();
    else { Serial.print("Unbekannt: "); Serial.println(cmd); printHelp(); }
  }
}

// ---------------- Setup / Loop ----------------
void setup() {
  // Relais sofort definiert AUS (Pegel vor pinMode setzen)
  uint8_t pins[] = {PIN_PWR, PIN_DIR, PIN_AUX1, PIN_AUX2};
  for (uint8_t p : pins) { digitalWrite(p, RELAY_ACTIVE_LOW ? HIGH : LOW); pinMode(p, OUTPUT); relay(p, false); }

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) delay(10);     // ohne USB nicht haengen bleiben

  loadPersist();
  pos = ps.pos;
  known = ps.known && !ps.moving;          // Stromausfall waehrend Fahrt -> unbekannt
  Serial.println();
  Serial.println(F("=== ESBE-Ventilmodul (UNO R4 Minima, Modbus-ID 2) ==="));
  Serial.println(RELAY_ACTIVE_LOW ? "Relais-Logik: LOW-aktiv (Ruhe = HIGH an D7..D10)"
                                  : "Relais-Logik: HIGH-aktiv (Ruhe = LOW an D7..D10)");
  Serial.print("Gespeicherte Stellung "); Serial.print(pos, 0);
  Serial.println(known ? " %" : " % (unbekannt)");

  for (int i = 0; i < NREG; i++) regs[i] = 0;
  regs[R_TARGET] = NO_TARGET;              // erst bewegen, wenn der ESP etwas vorgibt
  regs[R_TRAVEL] = ps.travel;
  regs[R_FS_MIN] = ps.fsMin;
  regs[R_FS_POS] = ps.fsPos;
  regs[R_VL] = regs[R_RL] = regs[R_WW] = T_INVALID;
  regs[R_ID] = 0xE5BE;
  lastHbMs = millis();

  // OLED suchen (0x3C ist Standard beim 0,91"-Modul, 0x3D bei manchen Varianten)
  Wire.begin();
  for (uint8_t a : {0x3C, 0x3D}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { oledAddr = a; break; }
  }
  if (oledAddr) {
    u8g2.setI2CAddress(oledAddr << 1);
    u8g2.setBusClock(400000);              // schnell zeichnen -> Modbus wird kaum blockiert
    oledOk = u8g2.begin();
    Serial.print("OLED an 0x"); Serial.println(oledAddr, HEX);
  } else {
    Serial.println("OLED nicht gefunden (Befehl 'i2c' zum Suchen)");
  }

  Serial1.begin(MB_BAUD, SERIAL_8N1);
  modbus.configureHoldingRegisters(regs, NREG);
  modbus.begin(SLAVE_ID, MB_BAUD, SERIAL_8N1);
  Serial.println("Bereit. '?' fuer Hilfe.");
}

void loop() {
  static uint32_t lastDraw = 0, lastVerbose = 0;
  if (modbus.poll()) { mbFrames++; lastFrameMs = millis(); }
  handleRegisters();
  valveTick();
  auxTick();
  if (modbus.poll()) { mbFrames++; lastFrameMs = millis(); }
  handleSerial();
  if (millis() - lastDraw >= 250) {
    lastDraw = millis();
    drawDisplay();
  }
  if (verbose && millis() - lastVerbose >= 5000) {
    lastVerbose = millis();
    printStatus();
  }
}
