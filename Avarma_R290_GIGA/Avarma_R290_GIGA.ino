/*
  =====================================================================
  Avarma R290 Steuerung  –  Arduino GIGA R1 WiFi + GIGA Display Shield
  =====================================================================
  Plattform-Teile: giga_compat.h, giga_ui.h, giga_fonts.h - siehe LIESMICH_GIGA.txt

  Hardware:
    - Arduino GIGA R1 WiFi + GIGA Display Shield (800x480, Touch)
    - RS485-Modul 3,3 V (MAX3485 / SP3485) an Serial1 (D0/D1, DE+RE an D2)

  Arduino IDE: Board "Arduino GIGA R1 WiFi" (Paket "Arduino Mbed OS GIGA Boards")

  Bibliotheken: siehe LIESMICH_GIGA.txt

  Architektur (wichtigste Aenderung gegenueber der alten Version):
    - Modbus laeuft in einem eigenen FreeRTOS-Task. Ein langsamer
      Webrequest oder ein Display-Refresh blockiert die Abfrage nicht mehr.
    - Display + Touch laufen in einem eigenen Task.
    - Web, Logging, SD und Serial laufen in loop().
    - Logging: ein einziger Ringpuffer im PSRAM (14 Tage @ 30 s),
      jeder Punkt mit echtem Zeitstempel. Wird zusaetzlich als
      Tages-CSV auf die SD geschrieben und beim Start von dort
      wieder eingelesen (Verlauf ueberlebt Neustarts).
    - /history liefert serverseitig verdichtete Daten (max. 720 Punkte,
      kompaktes JSON, in 2-kB-Bloecken gesendet) statt tausender
      einzelner client.print()-Aufrufe.
*/

// Arduino GIGA R1 WiFi + GIGA Display Shield: dieselbe Datei, Plattform-Teile in den
// Tabs giga_compat.h (WLAN, Web, Speicher, MQTT, Tasks) und giga_ui.h (800x480-Display)
#include "giga_compat.h"
#include <ModbusMaster.h>

// Standard-Pins je Board (koennen im Web geaendert werden)
#define DEF_SENS_PIN    3     // DHT22 an D3 (SHT4x: I2C an D20/D21)
#define DEF_RELAY_PWR   4     // Ventil-Relais Motorstrom an D4
#define DEF_RELAY_DIR   5     // Ventil-Relais Richtung an D5

// =====================================================
// Datentypen (bewusst ganz oben: die Arduino-IDE setzt ihre
// automatischen Funktions-Prototypen vor die erste Funktion)
// =====================================================
struct LiveData {
  float vorlauf = 0, ruecklauf = 0, warmwasser = 0;
  float flowLmin = 0; uint16_t flowRaw = 0; bool flowValid = false;
  uint16_t verdichterHz = 0;
  float sollwert = 0;
  uint16_t fan1Rpm = 0, fan2Rpm = 0; bool fanRpmValid = false;
  uint16_t maxVerdichterHz = 0; bool maxHzValid = false;
  uint16_t maxFanRpm = 0, maxFanRaw = 0; bool maxFanValid = false;
  uint16_t wpOnOffRaw = 0, wpModeRaw = 0;
  bool wpStateValid = false, wpModeValid = false;
  bool tempsValid = false, hzValid = false, sollValid = false;
  uint32_t errCount = 0, okCount = 0;
  uint8_t  lastErr = 0;          // letzter Modbus-Fehlercode
  uint32_t lastErrMs = 0;

  // erweiterte Werte (Blocklesen 4354..4386 und 4608..4612)
  bool  extValid = false, cntValid = false;
  float ta = 0, coil = 0, discharge = 0, suction = 0, eviIn = 0, eviOut = 0, liquid = 0;
  float evap = 0, cond = 0, ipm = 0, t6 = 0, room = 0, dT = 0;
  float hp = 0, lp = 0, currentA = 0, acV = 0, dcV = 0;
  uint16_t targetHz = 0, state = 0, outputs = 0, inputs = 0;
  uint16_t starts = 0, runtimeH = 0, pumpPct = 0, opMode = 0;
  float pEl = 0, pTh = 0;           // W: elektrisch (Quelle siehe pElSrc), thermisch (berechnet)
  float pElWp = 0;                  // W: Schaetzung aus Verdichterstrom x Spannung (wird immer berechnet)
  float pElSh = NAN;                // W: Messung Shelly (NAN = keine aktuellen Daten)
  uint8_t pElSrc = 0;               // Quelle von pEl: 0 = Schaetzung, 1 = Shelly
  double eElDay = 0, eThDay = 0;    // Wh heute
  double eElYest = 0, eThYest = 0;  // Wh gestern
  double eElTot = 0, eThTot = 0;    // Wh gesamt seit Zaehlerstart
};

// PV-Wert per MQTT (Wert + Empfangszeit)
struct PvVal { volatile float v = NAN; volatile uint32_t ms = 0; };

// Shelly 3EM (Pro 3EM, 3EM-63 Gen3 ...) per MQTT: <elTopic>/status/em:0 und /status/emdata:0
uint8_t  elSrc = 0;                    // Quelle el. Leistung: 0 = Schaetzung aus der WP, 1 = Shelly
char     elTopic[40] = "shelly-wp";    // Topic-Praefix des Shelly (wie in der Shelly-App)
volatile float    shP = NAN, shI = NAN, shETot = NAN;   // W / A / kWh (Zaehlerstand Bezug)
volatile uint32_t shMs = 0, shEMs = 0, shRx = 0;
volatile uint32_t shAny = 0, shBad = 0;   // Diagnose: Meldungen unter <elTopic>/ ueberhaupt / nicht lesbar
volatile int      shLastLen = 0;
char              shLast[48] = "";        // zuletzt empfangenes Shelly-Topic (ohne Praefix)
const uint32_t SH_MAX_AGE = 120000UL;  // so lange ohne Meldung gilt der Shelly-Wert noch
double   shCmpSh = 0, shCmpWp = 0;     // Summen fuer den Korrektur-Vorschlag (bei laufendem Verdichter)

// Raumfuehler (Definition im Heizregelungs-Teil)
extern volatile float rtIst;
bool rtValid();

// Voreinstellungen fuer ein neues Geraet (gespeicherte Einstellungen gehen immer vor)
#ifndef AV_DEF_PV
#define AV_DEF_PV      0     // 1 = PV/Energiemanagement-Teil (z. B. Victron ueber Node-RED) genutzt und eingeblendet
#endif
#ifndef AV_DEF_WWMODE
#define AV_DEF_WWMODE  1     // Warmwasser: 0 = Umschaltventil vom ESP (ESBE), 1 = WW-Modus der WP
#endif

// MQTT-Verbindung (Venus OS)
struct MqttCfg {
  uint8_t  version  = 1;
  uint8_t  enabled  = 1;
  char     host[40] = "";
  uint16_t port     = 1883;
  char     user[32] = "";
  char     pass[48] = "";
  char     base[24] = "avarma";
  uint16_t interval = 10;        // s zwischen zwei Status-Meldungen
};

// Heizregelung (Heizkurve, Raumfuehler, Auto-WP)
#define HC_POINTS 7
struct HcCfg {
  uint8_t  version      = 1;
  uint8_t  enabled      = 0;     // VL-Soll aus Heizkurve schreiben
  int8_t   sensPin      = DEF_SENS_PIN;   // Raumfuehler-Pin (DHT)
  uint8_t  sensType     = 22;    // 22 = DHT22/AM2302, 11 = DHT11, 40 = SHT4x (I2C, nur GIGA)
  int8_t   sensOffset10 = 0;     // Korrektur Raumfuehler in 0,1 K
  int8_t   curveTa[HC_POINTS] = {15, 10, 5, 0, -5, -10, -15};
  uint8_t  curveVl[HC_POINTS] = {28, 32, 37, 40, 45, 50, 52};
  uint16_t rtDay10      = 210;   // Raum-Soll Tag in 0,1 C
  uint16_t rtNight10    = 190;   // Raum-Soll Nacht
  uint8_t  nightOn      = 1;
  uint16_t nightStart   = 22 * 60;
  uint16_t nightEnd     = 6 * 60;
  int8_t   nightVlShift = -3;    // K VL-Verschiebung nachts (zusaetzlich zum Raumeinfluss)
  uint8_t  roomK10      = 30;    // Raumeinfluss: K VL je K Raumabweichung (x10)
  uint8_t  roomKi10     = 0;     // I-Anteil: K VL je K*h (x10), 0 = aus
  uint8_t  roomMax      = 6;     // Raumeinfluss max. +- K
  uint8_t  vlMin        = 25;
  uint8_t  vlMax        = 55;
  uint16_t writeSec     = 600;   // VL hoechstens alle x s schreiben
  uint8_t  taTau        = 30;    // Daempfung Aussentemperatur in min
  uint8_t  autoEnable   = 0;     // WP automatisch EIN/AUS
  uint8_t  autoMode     = 1;     // 1 = Aussen, 2 = Raum, 3 = Aussen + Raum
  int8_t   taOn         = 15;    // WP an unter ... C (gedaempft)
  int8_t   taOff        = 17;    // WP aus ueber ... C
  uint8_t  rtOnBelow10  = 5;     // WP an, wenn Raum < Soll - x (0,1 K)
  uint8_t  rtOffAbove10 = 10;    // WP aus, wenn Raum > Soll + x (0,1 K)
  uint16_t minOnMin     = 30;    // Mindestlaufzeit
  uint16_t minOffMin    = 15;    // Mindestpause
  uint8_t  deadband10   = 15;    // VL erst ab 1,5 K Abweichung neu schreiben (x10)
  // Kamin-Modus: Auto-WP-Grenzen nach unten verschieben
  uint8_t  kaminTa      = 3;     // K: Aussen-Grenzen (an/aus) um x K tiefer
  uint8_t  kaminRt10    = 5;     // 0,1 K: Raum-Soll fuer Auto-WP um x tiefer
  uint8_t  kaminMaxH    = 8;     // Kamin-Modus nach x h automatisch aus (0 = nie)
  uint8_t  roomOn       = 1;     // Raumeinfluss auf den VL ein/aus
  // Wochenende: eigene Nachtzeiten (Beginn Fr + Sa abends, Ende Sa + So morgens)
  uint16_t nightWeStart = 23 * 60;
  uint16_t nightWeEnd   = 8 * 60;
  uint8_t  nightWeOn    = 0;
  uint8_t  rsvNight     = 0;     // frueheres Fuellbyte (alte Einstellungen) - unbenutzt
  // Nachtabsenkung nach Aussentemperatur: eigene Kurve fuer die VL-Verschiebung
  uint8_t  nCurveOn     = 0;     // 0 = feste Verschiebung nightVlShift, 1 = Kurve
  int8_t   nTa[HC_POINTS] = {15, 10, 5, 0, -5, -10, -15};
  int8_t   nDv[HC_POINTS] = {-5, -4, -3, -3, -2, -1, 0};   // K
  // nachts WP ganz aus, wenn es aussen warm genug ist
  uint8_t  nOffOn       = 0;
  int8_t   nOffTa       = 10;    // C (gedaempfte Aussentemperatur, 1 K Hysterese)
};

// Sperrzeiten: WP zu festen Zeiten je Wochentag komplett aus
struct LockCfg {
  uint8_t  version  = 1;
  uint8_t  enabled  = 0;
  uint16_t from[7]  = {1380, 1380, 1380, 1380, 1380, 1380, 1380};   // Minute des Tages, [0] = Montag
  uint16_t to[7]    = {240, 240, 240, 240, 240, 240, 240};         // gleich wie from = an dem Tag keine Sperre
  uint8_t  coldOn   = 1;       // Sperre aufheben, wenn es aussen kaelter ist als ...
  int8_t   coldTa   = 2;       // ... dieser Wert (gedaempfte Aussentemperatur, 1 K Hysterese)
  uint8_t  wwAllow  = 0;       // WW-Ladung nach Zeitplan / PV in der Sperrzeit erlauben
  uint16_t from2[7] = {0, 0, 0, 0, 0, 0, 0};   // zweiter Zeitraum je Tag (gleich = keiner)
  uint16_t to2[7]   = {0, 0, 0, 0, 0, 0, 0};
};

// NAS-Archiv per FTP (Einstellungen, siehe "NAS-Archiv per FTP" weiter unten)
struct FtpCfg {
  uint8_t  version = 1;
  uint8_t  enabled = 0;
  char     host[40] = "";
  uint16_t port = 21;
  char     user[32] = "";
  char     pass[48] = "";
  char     dir[48]  = "/home";
  uint16_t intervalMin = 15;     // alle x Minuten uebertragen
  uint8_t  keepDays = 10;        // so viele Tage bleiben intern fein aufgeloest
};
class FtpClient;
extern volatile bool storageReady;
extern FtpCfg ftpCfg;
extern char ftpMsg[96];
extern volatile uint32_t ftpLastOk;
extern volatile bool ftpBusy;

// Verdichter-Frequenzkurve (Max Hz = P45 aussentemperaturgefuehrt)
struct FcCfg {
  uint8_t  version      = 1;
  uint8_t  enabled      = 0;
  int8_t   ta[HC_POINTS]     = {15, 10, 5, 0, -5, -10, -15};
  uint8_t  hzHeat[HC_POINTS] = {35, 40, 45, 50, 60, 70, 80};
  uint8_t  hzWw[HC_POINTS]   = {50, 55, 60, 65, 70, 75, 80};
  uint8_t  deadband     = 3;     // erst ab dieser Abweichung (Hz) schreiben
  uint16_t minIntMin    = 15;    // hoechstens alle x min schreiben (ausser Wechsel Heizen/WW)
  uint8_t  nightCap     = 0;     // nachts hoechstens x Hz (0 = aus), nur Heizen
  // Abtau-Boost: waehrend des Abtauens Max Hz anheben, danach zuruecksetzen
  uint8_t  dfOn         = 0;
  uint8_t  dfHz         = 70;    // Max Hz waehrend des Abtauens
  uint16_t dfBackSec    = 60;    // so lange nach Abtau-Ende warten, dann zuruecksetzen
  // Sanft ausschalten: vor dem AUS Max Hz absenken
  uint8_t  offLower     = 0;
  uint8_t  offHz        = 35;
  // Frost-Bremse: im Vereisungsbereich die Frequenz senken, solange der Verdampfer kalt ist
  uint8_t  fbOn         = 0;
  int8_t   fbTaLo       = 2;     // wirkt bei Aussentemperatur von ... (C)
  int8_t   fbTaHi       = 8;     // ... bis (C)
  int8_t   fbEvap       = 0;     // bremsen, wenn Verdampfer/Coil kaelter (C)
  uint8_t  fbStep       = 5;     // Hz je Schritt
  uint8_t  fbMinHz      = 35;    // nicht tiefer als
  uint8_t  fbSrc        = 0;     // 0 = Verdampfer (4385), 1 = Coil (4358)
};

// SG-Ready / PV / Heizstab / VL per MQTT
struct PvCfg {
  uint8_t  version      = 1;
  uint8_t  pvWwEnable   = 1;     // SG3/4 -> WW-Ladung per WP
  uint8_t  pvWwSoll     = 60;    // Ziel WP bei PV-Ladung
  uint8_t  pvWwHyst     = 5;     // PV-Ladung startet erst unter pvWwSoll - Hyst
  uint8_t  heaterEnable = 0;     // Heizstab ueber Shelly
  uint8_t  heaterSoll   = 75;    // Ziel Heizstab
  uint8_t  heaterGen    = 2;     // Shelly Generation: 1 oder 2 (Plus/Pro/Gen3/Gen4)
  char     shellyIp[40] = "";
  uint16_t heaterMaxMin = 240;   // max. Heizstab-Laufzeit am Stueck
  uint16_t sgTimeoutMin = 15;    // ohne Auffrischung zurueck auf SG2
  uint8_t  sg1Block     = 1;     // SG1 schaltet WP aus
  uint8_t  vlMqtt       = 1;     // VL-Sollwert per MQTT erlaubt
  uint8_t  vlMin        = 20;
  uint8_t  vlMax        = 55;
  uint16_t vlMinSec     = 300;   // min. Abstand zwischen zwei VL-Schreibvorgaengen
  uint8_t  show         = AV_DEF_PV;   // PV/Energiemanagement nutzen und im Web anzeigen
};

enum : uint8_t {
  F_TEMPS = 0x01, F_WPON = 0x02, F_FLOW = 0x04, F_FAN = 0x08,
  F_HZ = 0x10, F_SV = 0x20, F_WPVALID = 0x40, F_MODEVALID = 0x80
};

enum : uint8_t { F2_EXT = 0x01, F2_DEFROST = 0x02, F2_WWLOAD = 0x04 };

struct LogPoint {              // 40 Byte
  uint32_t t;                  // UTC epoch
  int16_t  vl, rl, ww;         // 1/10 C
  int16_t  flow;               // 1/10 L/min
  uint16_t fan1, fan2;         // rpm
  uint8_t  hz, sv, mode, flags;
  int16_t  rt;                 // Raum 1/10 C, INT16_MIN = ungueltig
  int16_t  ta, evap;           // Aussen / Verdampfer 1/10 C, INT16_MIN = ungueltig
  int16_t  pEl, pTh;           // W elektrisch (verwendete Quelle) / thermisch
  int16_t  grid;               // W Netz (+ Bezug), INT16_MIN = ungueltig
  uint16_t pv;                 // W PV, 0xFFFF = ungueltig
  uint8_t  f2, rsv;            // F2_...
  int16_t  pElWp, pElSh;       // W Schaetzung WP / Messung Shelly, INT16_MIN = ungueltig
};

struct HistOut { uint32_t t; float vl, rl, ww, hz, flow, rt, ta, evap, sv, cop, dfr, pel, pth, wwf, fan; bool gap; };

// Summen fuer Mittelwerte (Verlauf, Stundenmittel)
struct HistAcc {
  double sT, sVl, sRl, sWw, sHz, sFl, sRt, sTa, sEv, sSv, sEl, sTh, sPv, sFan;
  uint32_t n, nFl, nRt, nTa, nSv, nPw, nDf, nPv, nWw, nFan;
};

// Tageswerte fuer die Statistik
struct DayStat {
  int32_t  day;                // JJJJMMTT
  double   el, th, elWw, thWw, pv, elG, elOwn;   // Wh
  double   runSec, taSum, rtSum;
  float    taMin, taMax;
  uint32_t n, nExt, nPv, taN, rtN, lastT;
  uint16_t starts, defrosts;
  bool     lastRun, lastDf;
};

// Hilfsstruktur beim Verdichten des Verlaufs
struct HistBuild {
  uint32_t fromT, bucket, gapLimit;
  int maxPts, n;
  int64_t curB;
  HistAcc acc;
  HistOut* out;
};

enum CmdType : uint8_t {
  CMD_SET_SOLL, CMD_POWER, CMD_MODE, CMD_MAXHZ, CMD_MAXFAN,
  CMD_READ_SOLL, CMD_READ_ALL, CMD_READ_MAXHZ, CMD_READ_MAXFAN,
  CMD_READ_FAN, CMD_READ_FLOW,
  CMD_READ_PARAMS, CMD_SET_P06, CMD_SET_P07, CMD_READ_RAW
};

struct MbCmd {
  CmdType type;
  float value;
  bool wait;    // loop() wartet auf das Ergebnis
  bool print;   // Ausgabe auf Serial (Befehl kam von Serial)
};

enum Page : uint8_t { PAGE_STATUS = 0, PAGE_CHART, PAGE_CTRL, PAGE_WP, PAGE_PV, PAGE_EXPERT };

enum BtnId : int {
  B_NONE = -1,
  B_TAB0 = 1, B_TAB1, B_TAB2, B_TAB3,
  B_RT_M, B_RT_P, B_HC_TOG, B_AUTO_TOG,
  B_H1, B_H2, B_H8, B_H24,
  B_SOLL_M, B_SOLL_P, B_SOLL_SET,
  B_HZ_M, B_HZ_P, B_HZ_SET,
  B_FAN_M, B_FAN_P, B_FAN_SET,
  B_ON, B_OFF, B_HEIZ, B_WW, B_KUEHL,
  B_WWLOAD,
  B_TAB4, B_KAMIN, B_FC_TOG, B_EXPERT, B_BACK, B_ROOM_TOG
};

struct Btn { int16_t x, y, w, h; int id; };

// ---- ESBE-Ventil / Warmwasser ----
#define SCHED_SLOTS      4
#define CTRL_CFG_VERSION 1
enum : uint8_t { ACT_NONE = 0, ACT_WW = 1, ACT_PUFFER = 2, ACT_HEIZ = 3 };
enum : uint8_t { WW_IDLE = 0, WW_MOVE, WW_HEAT, WW_COOL, WW_BACK };   // WW_BACK: Ventil zurueck auf Heizung, Verdichter laeuft weiter

struct SchedEv { uint16_t min; uint8_t act; };   // min = Minute des Tages

struct CtrlCfg {
  uint8_t  version   = CTRL_CFG_VERSION;
  bool     enabled   = false;  // erst im Web aktivieren, wenn verdrahtet
  int8_t   pinPwr    = DEF_RELAY_PWR;   // Relais 1: Motor-Versorgung
  int8_t   pinDir    = DEF_RELAY_DIR;   // Relais 2: Richtung (aus = Heizung, an = WW)
  bool     actLow    = false;  // Relaismodul schaltet bei LOW
  bool     dirInv    = false;  // Richtung vertauscht
  uint16_t travelSec = 130;    // Laufzeit Endlage -> Endlage
  uint8_t  bufPos    = 50;     // Puffer-Stellung in %
  uint8_t  defAct    = ACT_HEIZ; // Stellung, wenn kein Zeitplan-Ereignis existiert
  uint8_t  wwSoll    = 50;     // Ziel-Temperatur WW-Speicher
  uint8_t  boost     = 4;      // VL-Soll beim Laden = WW-Soll + boost
  uint16_t maxMin    = 120;    // max. Ladedauer
  uint16_t coolSec   = 60;     // Ventil nach Ladung noch so lange auf WW
  bool     wpOnForWw = true;   // WP fuer WW-Ladung einschalten, falls aus
  SchedEv  sched[7][SCHED_SLOTS];   // [0] = Montag
  // --- ab hier neu (aeltere gespeicherte Konfigurationen werden ergaenzt) ---
  uint8_t  drive     = 0;      // 0 = Relais am ESP, 1 = Ventilmodul am Modbus (Arduino)
  uint8_t  slaveId   = 2;      // Modbus-ID des Ventilmoduls
  uint8_t  fsMin     = 10;     // Modul faehrt nach x min ohne ESP auf Heizung (0 = aus)
  uint8_t  wwNoCont  = 0;      // 0 = nach der Ladung direkt weiterheizen (Verdichter laeuft durch), 1 = Nachlauf wie frueher
                               //     (liegt im frueheren Fuellbyte -> alte Einstellungen lesen hier 0)
  uint8_t  wwPullMin = 30;     // WW-Zeitplan bis zu x min vorziehen, wenn der Verdichter beim Heizen gerade abschalten wuerde (0 = aus)
  uint8_t  wwVlMax   = 60;     // VL-Soll beim Laden wird bei Bedarf bis hierhin angehoben (sonst stoppt der Verdichter am VL-Soll)
  // Puffer-Automatik: nach der WW-Ladung erst auf Puffer, wenn der Speicher abgekuehlt ist
  uint8_t  abOn      = 0;      // Puffer, wenn WW <= Heiz-VL-Soll + abK
  int8_t   abK       = 3;      // K
  uint8_t  abLow     = 0;      // Puffer, wenn WW < Heiz-VL-Soll
  uint8_t  abFixOn   = 0;      // Puffer, wenn WW <= abFixT (feste Temperatur, unabhaengig vom VL-Soll)
  uint8_t  abFixT    = 35;     // C
  uint8_t  wwMode    = AV_DEF_WWMODE;   // 0 = Umschaltventil vom ESP (ESBE), 1 = WW-Modus der WP (WP schaltet ihr Ventil selbst)
};

// =====================================================
// WLAN / Zeit
// =====================================================
// Firmware-Version: bei jeder Aenderung hochzaehlen (steht im Web, auf dem Display und in der Konsole)
#define FW_VERSION "1.36"
#define AV_PLATFORM "GIGA R1"
#define SD_NAME     "Speicher"
static const char FW_BUILD[] = __DATE__ " " __TIME__;       // Zeitpunkt des Kompilierens

// ============================================================
// >>> HIER DEINE WLAN-ZUGANGSDATEN EINTRAGEN <<<
// (Name und Passwort deines WLANs, 2,4 GHz)
// ============================================================
static const char* WIFI_SSID = "DEIN_WLAN_NAME";
static const char* WIFI_PASS = "DEIN_WLAN_PASSWORT";
static const char* HOSTNAME  = "avarma";                       // -> http://avarma.local
static const char* OTA_PASSWORD = "bitte-aendern";                      // Passwort fuer Upload per Netzwerk (Arduino-IDE)
static const char* TZ_INFO   = "CET-1CEST,M3.5.0,M10.5.0/3";   // Europa/Berlin
static const char* NTP1      = "pool.ntp.org";
static const char* NTP2      = "ptbtime1.ptb.de";

// =====================================================
// MODBUS PINS  (freie GPIOs am Erweiterungsstecker: IO2 / IO3 / IO14 / IO21)
//   ACHTUNG: ESP32-S3 ist NICHT 5V-tolerant. MAX485-Modul mit 3,3 V
//   versorgen oder einen MAX3485 / Pegelwandler verwenden.
// =====================================================
// GIGA: RS485-Modul (3,3 V!) an Serial1: RO -> D0 (RX0), DI -> D1 (TX0), DE+/RE -> D2
#define MB_RX_PIN       0
#define MB_TX_PIN       1
#define MAX485_DE_RE    2
#define MB_BAUD      9600
#define MB_SLAVE_ID     1


// =====================================================
// Einstellgrenzen
// =====================================================
static const uint16_t MAX_HZ_MIN  = 30;
static const uint16_t MAX_HZ_MAX  = 85;
static const uint16_t FAN_RPM_MIN = 300;
static const uint16_t FAN_RPM_MAX = 600;
static const int      SOLL_MIN    = 10;
static const int      SOLL_MAX    = 75;

// =====================================================
// Modbus Register
// =====================================================
static const uint16_t REG_ONOFF = 4096;
static const uint16_t REG_MODE  = 4097;
static const uint16_t REG_SOLL  = 4098;

static const uint16_t REG_RL   = 4365;
static const uint16_t REG_VL   = 4366;
static const uint16_t REG_WW   = 4367;
static const uint16_t REG_FLOW = 4368;  // raw / 10 = L/min

static const uint16_t REG_HZ       = 4373;
static const uint16_t REG_FAN1_RPM = 4374;
static const uint16_t REG_FAN2_RPM = 4375;

static const uint16_t REG_MAX_HZ      = 8235;  // P45 Max Verdichterfrequenz, 1 Hz
static const uint16_t REG_MAX_FAN_RPM = 8250;  // P60 Max Luefter, raw = rpm / 10

// =====================================================
// Abfrageintervalle
// =====================================================
const uint32_t TEMPS_INTERVAL_MS   = 3000;
const uint32_t FLOW_INTERVAL_MS    = 5000;
const uint32_t HZ_INTERVAL_MS      = 5000;
const uint32_t FANRPM_INTERVAL_MS  = 5000;
const uint32_t SOLL_INTERVAL_MS    = 7000;
const uint32_t MAXHZ_INTERVAL_MS   = 15000;
const uint32_t MAXFAN_INTERVAL_MS  = 15000;
const uint32_t WPSTATE_INTERVAL_MS = 10000;
const uint32_t WIFI_RETRY_MS       = 30000;

// =====================================================
// Logging
// =====================================================
const uint32_t LOG_INTERVAL_SEC   = 30;                 // Aufloesung
const uint32_t LOG_DAYS_PSRAM     = 14;                 // RAM-Verlauf mit PSRAM
const uint32_t LOG_DAYS_NOPSRAM   = 1;                  // Fallback ohne PSRAM
const uint32_t SD_FLUSH_MS        = 5UL * 60UL * 1000UL; // alle 5 min auf SD schreiben
const uint32_t SD_RETRY_MS        = 60UL * 1000UL;
const int      HIST_WEB_MAX       = 720;                // max. Punkte pro /history-Antwort
const int      HIST_DISP_MAX      = 256;                // max. Punkte fuer Display-Chart


// =====================================================
// Globale Objekte
// =====================================================
ModbusMaster node;
WebServer server(80);

// =====================================================
// Live-Werte (geschrieben vom Modbus-Task, gelesen von allen)
// =====================================================

// =====================================================
// Log: geht auf USB-Serial UND in einen Ringpuffer fuer die Web-Konsole
// =====================================================
static char logRing[16384];
static volatile uint32_t logRingTotal = 0;
static portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

class TeeLog : public Print {
  volatile bool lineStart = true;
  void put(const char* p, size_t n) {
    if (!n) return;
    if (Serial) Serial.write((const uint8_t*)p, n);
    portENTER_CRITICAL(&logMux);
    for (size_t i = 0; i < n; i++) { logRing[logRingTotal % sizeof(logRing)] = p[i]; logRingTotal++; }
    portEXIT_CRITICAL(&logMux);
  }
  void stamp() {                       // Zeitstempel an jedem Zeilenanfang
    char p[24];
    time_t t = time(nullptr);
    if (t > 1700000000) { struct tm tmv; localtime_r(&t, &tmv);
      snprintf(p, sizeof(p), "%02d:%02d:%02d ", tmv.tm_hour, tmv.tm_min, tmv.tm_sec); }
    else snprintf(p, sizeof(p), "[%lus] ", (unsigned long)(millis() / 1000));
    put(p, strlen(p));
  }
public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t* b, size_t n) override {
    char tmp[128]; size_t k = 0;
    for (size_t i = 0; i < n; i++) {
      if (lineStart) { put(tmp, k); k = 0; stamp(); lineStart = false; }
      tmp[k++] = (char)b[i];
      if (b[i] == '\n') lineStart = true;
      if (k == sizeof(tmp)) { put(tmp, k); k = 0; }
    }
    put(tmp, k);
    return n;
  }
  using Print::write;
  size_t printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {   // Print hat hier kein printf
    char b[256];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n < 0) return 0;
    if (n < (int)sizeof(b)) return write((const uint8_t*)b, n);
    char* big = (char*)malloc(n + 1);
    if (!big) return write((const uint8_t*)b, sizeof(b) - 1);
    va_start(ap, fmt); vsnprintf(big, n + 1, fmt, ap); va_end(ap);
    size_t r = write((const uint8_t*)big, n);
    free(big);
    return r;
  }
};
TeeLog Log;

// =====================================================
// Modbus-Einstellungen (im Flash gespeichert, ueber Weboberflaeche aenderbar)
// =====================================================
struct MbConfig {
  int8_t   rx   = MB_RX_PIN;
  int8_t   tx   = MB_TX_PIN;
  int8_t   de   = MAX485_DE_RE;
  uint8_t  mode = 0;            // 0 = DE per Software, 1 = Hardware-RS485 (UART steuert DE)
  uint32_t baud = MB_BAUD;
  uint8_t  id   = MB_SLAVE_ID;
  uint8_t  gap  = 3;            // ms Wartezeit nach dem Senden, dann Stoerbytes verwerfen
};
MbConfig mbCfg;
Preferences prefs;

void loadMbConfig() {
  prefs.begin("modbus", true);
  mbCfg.rx   = prefs.getChar("rx", MB_RX_PIN);
  mbCfg.tx   = prefs.getChar("tx", MB_TX_PIN);
  mbCfg.de   = prefs.getChar("de", MAX485_DE_RE);
  mbCfg.mode = prefs.getUChar("mode", 0);
  mbCfg.baud = prefs.getULong("baud", MB_BAUD);
  mbCfg.id   = prefs.getUChar("id", MB_SLAVE_ID);
  mbCfg.gap  = prefs.getUChar("gap", 3);
  prefs.end();
}

void saveMbConfig() {
  prefs.begin("modbus", false);
  prefs.putChar("rx", mbCfg.rx);
  prefs.putChar("tx", mbCfg.tx);
  prefs.putChar("de", mbCfg.de);
  prefs.putUChar("mode", mbCfg.mode);
  prefs.putULong("baud", mbCfg.baud);
  prefs.putUChar("id", mbCfg.id);
  prefs.putUChar("gap", mbCfg.gap);
  prefs.end();
}

const char* mbErrText(uint8_t c) {
  switch (c) {
    case 0:    return "-";
    case 1:    return "Exception: Funktion";
    case 2:    return "Exception: Adresse";
    case 3:    return "Exception: Wert";
    case 4:    return "Exception: Geraetefehler";
    case 0xE0: return "falsche Slave-ID";
    case 0xE1: return "falsche Funktion";
    case 0xE2: return "Timeout";
    case 0xE3: return "CRC falsch";
    default:   return "unbekannt";
  }
}

LiveData live;
SemaphoreHandle_t liveMutex;

static inline void liveLock()   { xSemaphoreTake(liveMutex, portMAX_DELAY); }
static inline void liveUnlock() { xSemaphoreGive(liveMutex); }

LiveData getLive() {
  liveLock();
  LiveData c = live;
  liveUnlock();
  return c;
}

// Letzte Befehlsmeldung (fuer Display + Web)
char cmdMsg[64] = "";
uint32_t cmdMsgMs = 0;

void setCmdMsg(const char* fmt, ...) {
  char tmp[64];
  va_list ap; va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);
  liveLock();
  strlcpy(cmdMsg, tmp, sizeof(cmdMsg));
  cmdMsgMs = millis();
  liveUnlock();
}

uint32_t getCmdMsg(char* out, size_t n) {   // liefert Alter in ms
  liveLock();
  strlcpy(out, cmdMsg, n);
  uint32_t age = cmdMsg[0] ? millis() - cmdMsgMs : 0xFFFFFFFF;
  liveUnlock();
  return age;
}

volatile bool verbose  = false;   // Serial: jede Abfrage ausgeben ("VERBOSE")
volatile bool cmdPrint = false;   // aktueller Befehl kam von Serial -> Ausgabe
volatile bool restartRequested = false;
bool     restartHold = false;            // Einstellungs-Import: Neustart erst am Ende (max. 60 s)
uint32_t restartHoldMs = 0;
volatile bool touchDebug = false; // Serial: Touch-Koordinaten ausgeben ("TOUCH")
bool touchChipFound = false;
int32_t lastTouchX = -1, lastTouchY = -1;
uint32_t lastTouchMs = 0;

#define MBLOG(...) do { if (verbose || cmdPrint) Log.printf(__VA_ARGS__); } while (0)

// =====================================================
// Zeit
// =====================================================
bool timeValid() { return time(nullptr) > 1700000000; }
uint32_t nowUtc() { return timeValid() ? (uint32_t)time(nullptr) : 0; }

const char* modeName(uint16_t mode) {
  if (mode == 1) return "WW";
  if (mode == 2) return "Heizen";
  if (mode == 4) return "Kuehlen";
  if (mode == 0) return "Off";
  return "Unbekannt";
}

// =====================================================
// Logging-Ringpuffer
// =====================================================

LogPoint* logBuf = nullptr;
uint32_t logCap = 0, logHead = 0, logCount = 0;
uint32_t logTotal = 0;              // Anzahl jemals hinzugefuegter Punkte
uint32_t sdFlushedTotal = 0;        // bis hierhin auf SD geschrieben
uint32_t lastLogSlot = 0;
bool historyRestored = false;
volatile bool restoringHistory = false;
SemaphoreHandle_t logMutex;

// weiter unten definiert
extern volatile uint8_t wwState;
extern PvVal pvPower, pvGrid, pvSoc;
float pvGet(const PvVal& p, uint32_t maxAgeMs);
bool sdOk = false;                  // SD-Karte bereit

static inline LogPoint& logAt(uint32_t pos) {       // pos 0 = aeltester Punkt
  return logBuf[(logHead + logCap - logCount + pos) % logCap];
}

void logPush(const LogPoint& p) {
  xSemaphoreTake(logMutex, portMAX_DELAY);
  logBuf[logHead] = p;
  logHead = (logHead + 1) % logCap;
  if (logCount < logCap) logCount++;
  logTotal++;
  xSemaphoreGive(logMutex);
}

uint32_t lastLoggedT() {
  return logCount ? logAt(logCount - 1).t : 0;
}
uint32_t firstLoggedT() {
  return logCount ? logAt(0).t : 0;
}

float logCapacityHours() { return (float)logCap * LOG_INTERVAL_SEC / 3600.0f; }

void allocLogBuffer() {
  if (psramFound()) {
    logCap = LOG_DAYS_PSRAM * 86400UL / LOG_INTERVAL_SEC;
    logBuf = (LogPoint*)ps_malloc(sizeof(LogPoint) * logCap);
  }
  if (!logBuf) {
    logCap = LOG_DAYS_NOPSRAM * 86400UL / LOG_INTERVAL_SEC;
    logBuf = (LogPoint*)malloc(sizeof(LogPoint) * logCap);
  }
  if (!logBuf) { logCap = 0; Log.println("FEHLER: kein Speicher fuer Log!"); return; }
  Log.printf("Log-Puffer: %lu Punkte (%.1f h) in %s\n",
                (unsigned long)logCap, logCapacityHours(), psramFound() ? "PSRAM" : "RAM");
}

void logPointInit(LogPoint& p) {
  memset(&p, 0, sizeof(p));
  p.rt = INT16_MIN; p.ta = INT16_MIN; p.evap = INT16_MIN; p.grid = INT16_MIN; p.pv = 0xFFFF;
  p.pElWp = INT16_MIN; p.pElSh = INT16_MIN;
}

// ---------- Mittelwerte fuer Verlauf (Rohpunkte und Stundenmittel) ----------
void accReset(HistAcc& a) { memset(&a, 0, sizeof(a)); }

void accAddPoint(HistAcc& a, const LogPoint& p) {
  if (!(p.flags & F_TEMPS)) return;
  a.sT += p.t; a.n++;
  a.sVl += p.vl / 10.0; a.sRl += p.rl / 10.0; a.sWw += p.ww / 10.0; a.sHz += p.hz;
  if (p.flags & F_FLOW)  { a.sFl += p.flow / 10.0; a.nFl++; }
  if (p.flags & F_FAN)   { a.sFan += p.fan1; a.nFan++; }
  if (p.rt != INT16_MIN) { a.sRt += p.rt / 10.0; a.nRt++; }
  if (p.ta != INT16_MIN) { a.sTa += p.ta / 10.0; a.sEv += p.evap / 10.0; a.nTa++; }
  if (p.flags & F_SV)    { a.sSv += p.sv; a.nSv++; }
  if (p.f2 & F2_EXT)     { a.sEl += p.pEl; a.sTh += p.pTh; a.nPw++; if (p.f2 & F2_DEFROST) a.nDf++;
                           if ((p.f2 & F2_WWLOAD) || ((p.flags & F_MODEVALID) && p.mode == 1)) a.nWw++; }
  if (p.pv != 0xFFFF)    { a.sPv += p.pv; a.nPv++; }
}

// Stundenzeile aus h_JJJJ-MM.csv: t;n;vl;rl;ww;hz;flow;nFl;rt;nRt;ta;evap;nTa;sv;nSv;pEl;pTh;nPw;nDf;pv;nPv[;nWw[;fan;nFan]]
bool parseHourRow(char* line, uint32_t& t, HistAcc& r) {
  if (line[0] < '0' || line[0] > '9') return false;
  double v[24]; int nf = 0;
  char* s = line;
  while (nf < 24) {
    char* sep = strchr(s, ';');
    if (sep) *sep = 0;
    v[nf++] = *s ? strtod(s, nullptr) : 0;
    if (!sep) break;
    s = sep + 1;
  }
  if (nf < 21) return false;
  accReset(r);
  t = (uint32_t)v[0];
  uint32_t n = (uint32_t)v[1];
  if (!n) return false;
  r.n = n; r.sT = (double)t * n;
  r.sVl = v[2] * n; r.sRl = v[3] * n; r.sWw = v[4] * n; r.sHz = v[5] * n;
  r.nFl = (uint32_t)v[7];  r.sFl = v[6] * r.nFl;
  r.nRt = (uint32_t)v[9];  r.sRt = v[8] * r.nRt;
  r.nTa = (uint32_t)v[12]; r.sTa = v[10] * r.nTa; r.sEv = v[11] * r.nTa;
  r.nSv = (uint32_t)v[14]; r.sSv = v[13] * r.nSv;
  r.nPw = (uint32_t)v[17]; r.sEl = v[15] * r.nPw; r.sTh = v[16] * r.nPw; r.nDf = (uint32_t)v[18];
  r.nPv = (uint32_t)v[20]; r.sPv = v[19] * r.nPv;
  r.nWw = nf > 21 ? (uint32_t)v[21] : 0;              // ab v1.15
  if (nf > 23) { r.nFan = (uint32_t)v[23]; r.sFan = v[22] * r.nFan; }   // ab v1.33
  return true;
}

void accAddAcc(HistAcc& a, const HistAcc& b) {
  a.sT += b.sT; a.n += b.n; a.sVl += b.sVl; a.sRl += b.sRl; a.sWw += b.sWw; a.sHz += b.sHz;
  a.sFl += b.sFl; a.nFl += b.nFl; a.sRt += b.sRt; a.nRt += b.nRt;
  a.sTa += b.sTa; a.sEv += b.sEv; a.nTa += b.nTa; a.sSv += b.sSv; a.nSv += b.nSv;
  a.sEl += b.sEl; a.sTh += b.sTh; a.nPw += b.nPw; a.nDf += b.nDf; a.sPv += b.sPv; a.nPv += b.nPv; a.nWw += b.nWw;
  a.sFan += b.sFan; a.nFan += b.nFan;
}

void accOut(const HistAcc& a, HistOut& o) {
  o.t    = (uint32_t)(a.sT / a.n);
  o.vl   = a.sVl / a.n; o.rl = a.sRl / a.n; o.ww = a.sWw / a.n; o.hz = a.sHz / a.n;
  o.flow = a.nFl ? a.sFl / a.nFl : NAN;
  o.rt   = a.nRt ? a.sRt / a.nRt : NAN;
  o.ta   = a.nTa ? a.sTa / a.nTa : NAN;
  o.evap = a.nTa ? a.sEv / a.nTa : NAN;
  o.sv   = a.nSv ? a.sSv / a.nSv : NAN;
  o.cop  = (a.nPw && a.sEl / a.nPw > 100) ? constrain(a.sTh / a.sEl, 0.0, 10.0) : NAN;
  o.dfr  = a.nPw ? (float)a.nDf / a.nPw : 0;
  o.pel  = a.nPw ? a.sEl / a.nPw : NAN;
  o.pth  = a.nPw ? a.sTh / a.nPw : NAN;
  o.wwf  = a.nPw ? (float)a.nWw / a.nPw : 0;
  o.fan  = a.nFan ? a.sFan / a.nFan : NAN;
  o.gap  = false;
}

// Monatsdatei mit Stundenmitteln
void monthFileName(uint32_t t, char* out, size_t n) {
  time_t tt = t; struct tm tmv; localtime_r(&tt, &tmv);
  snprintf(out, n, "/log/h_%04d-%02d.csv", tmv.tm_year + 1900, tmv.tm_mon + 1);
}

// Stundenmittel von SD lesen (fuer Zeitraeume, die nicht mehr im RAM sind)
void readHourRowsImpl(uint32_t fromT, uint32_t toT, void (*cb)(void*, uint32_t, const HistAcc&), void* ctx) {
  if (!sdOk || toT <= fromT) return;
  static char rbuf[2048];
  char line[200];
  // Monat fuer Monat (Start am 15. des Monats, damit das Weiterzaehlen sicher ist)
  time_t tt = fromT; struct tm tmv; localtime_r(&tt, &tmv);
  int y = tmv.tm_year, m = tmv.tm_mon;
  for (int guard = 0; guard < 240; guard++) {
    struct tm mt = {}; mt.tm_year = y; mt.tm_mon = m; mt.tm_mday = 1; mt.tm_isdst = -1;
    time_t ms = mktime(&mt);
    if ((uint32_t)ms > toT) break;
    char path[32];
    snprintf(path, sizeof(path), "/log/h_%04d-%02d.csv", y + 1900, m + 1);
    if (SD_MMC.exists(path)) {
      File f = SD_MMC.open(path, FILE_READ);
      if (f) {
        int ll = 0, n;
        uint32_t lastT = 0;
        while ((n = f.read((uint8_t*)rbuf, sizeof(rbuf))) > 0) {
          for (int i = 0; i < n; i++) {
            char c = rbuf[i];
            if (c == '\n') {
              line[ll] = 0; ll = 0;
              uint32_t t; HistAcc r;
              if (parseHourRow(line, t, r) && t > lastT && t >= fromT && t < toT) { lastT = t; cb(ctx, t, r); }
            } else if (c != '\r' && ll < (int)sizeof(line) - 1) line[ll++] = c;
          }
          delay(0);
        }
        f.close();
      }
    }
    if (++m > 11) { m = 0; y++; }
  }
}

// Verdichtete Ausgabe fuer Web und Display
static HistOut histWeb[HIST_WEB_MAX + 1];


void histEmit(HistBuild& h) {
  if (h.acc.n == 0 || h.n >= h.maxPts) return;
  HistOut& o = h.out[h.n];
  accOut(h.acc, o);
  o.gap = (h.n > 0) && (o.t - h.out[h.n - 1].t > h.gapLimit);
  h.n++;
}

void histFeed(HistBuild& h, uint32_t t, const HistAcc* row, const LogPoint* p) {
  int64_t b = ((int64_t)t - h.fromT) / h.bucket;
  if (b != h.curB) { histEmit(h); h.curB = b; accReset(h.acc); }
  if (row) accAddAcc(h.acc, *row); else accAddPoint(h.acc, *p);
}

void histHourCb(void* ctx, uint32_t t, const HistAcc& r) { histFeed(*(HistBuild*)ctx, t, &r, nullptr); }

int buildHistory(uint32_t fromT, uint32_t toT, int maxPts, HistOut* out, uint32_t* bucketOut, bool useSd) {
  if (!logBuf || toT <= fromT || maxPts < 2) return 0;
  uint32_t ramFirst = firstLoggedT();
  bool sdPart = useSd && sdOk && (!logCount || fromT < ramFirst);

  uint32_t span = toT - fromT;
  uint32_t bucket = (span + maxPts - 1) / maxPts;
  if (bucket < LOG_INTERVAL_SEC) bucket = LOG_INTERVAL_SEC;
  bucket = ((bucket + LOG_INTERVAL_SEC - 1) / LOG_INTERVAL_SEC) * LOG_INTERVAL_SEC;
  if (sdPart && bucket < 3600) bucket = 3600;               // Stundenmittel von SD
  if (bucketOut) *bucketOut = bucket;

  HistBuild h;
  h.fromT = fromT; h.bucket = bucket; h.maxPts = maxPts; h.n = 0; h.curB = -1; h.out = out;
  h.gapLimit = max(bucket * 3, LOG_INTERVAL_SEC * 4);
  if (sdPart) h.gapLimit = max(h.gapLimit, (uint32_t)3 * 3600);
  accReset(h.acc);

  if (sdPart) readHourRowsImpl(fromT, logCount ? min(toT, ramFirst) : toT, histHourCb, &h);

  xSemaphoreTake(logMutex, portMAX_DELAY);
  uint32_t cnt = logCount;
  uint32_t lo = 0, hi = cnt;                  // binaere Suche nach fromT
  while (lo < hi) {
    uint32_t mid = (lo + hi) / 2;
    if (logAt(mid).t < fromT) lo = mid + 1; else hi = mid;
  }
  for (uint32_t pos = lo; pos < cnt; pos++) {
    const LogPoint& p = logAt(pos);
    if (p.t > toT) break;
    if (!(p.flags & F_TEMPS)) continue;
    histFeed(h, p.t, nullptr, &p);
  }
  histEmit(h);
  xSemaphoreGive(logMutex);
  return h.n;
}

// =====================================================
// SD-Karte
// =====================================================
uint32_t lastSdTryMs = 0, lastSdFlushMs = 0, lastSdSpaceMs = 0;
uint64_t sdTotalBytes = 0, sdUsedBytes = 0;

void updateSdSpace() {
  if (!sdOk) return;
  sdTotalBytes = SD_MMC.totalBytes();
  sdUsedBytes  = SD_MMC.usedBytes();
  lastSdSpaceMs = millis();
}

bool mountSd() {
  if (!SD_MMC.begin("/sdcard", false, false, SDMMC_FREQ_DEFAULT, 5)) {
    return false;
  }
  if (SD_MMC.cardType() == CARD_NONE) { SD_MMC.end(); return false; }
  if (!SD_MMC.exists("/log")) SD_MMC.mkdir("/log");
  sdOk = true;
  updateSdSpace();
  if (SD_MMC.kind() == 1 && avDataFormatted) Log.println("Interner Speicher (QSPI-Partition 4) wurde neu eingerichtet");
  Log.printf("%s OK: %llu KB, belegt %llu KB\n", SD_MMC.kindName(), sdTotalBytes >> 10, sdUsedBytes >> 10);
  return true;
}

void sdFail(const char* why) {
  Log.printf("SD Fehler: %s -> SD deaktiviert, neuer Versuch spaeter\n", why);
  sdOk = false;
  SD_MMC.end();
  lastSdTryMs = millis();
}

void dayFileName(uint32_t t, char* out, size_t n) {
  time_t tt = t; struct tm tmv; localtime_r(&tt, &tmv);
  snprintf(out, n, "/log/%04d-%02d-%02d.csv", tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday);
}

void fmtTenth(char* out, size_t n, int v) {     // 285 -> "28,5"  (Excel-DE)
  const char* s = v < 0 ? "-" : "";
  int a = abs(v);
  snprintf(out, n, "%s%d,%d", s, a / 10, a % 10);
}

int formatCsvLine(char* out, size_t n, const LogPoint& p) {
  time_t tt = p.t; struct tm tmv; localtime_r(&tt, &tmv);
  char vl[10], rl[10], ww[10], fl[10] = "", hz[6] = "", f1[8] = "", f2[8] = "", sv[6] = "", wp[3] = "", md[4] = "", rt[10] = "";
  char ta[10] = "", ev[10] = "", pel[8] = "", pth[8] = "", df[3] = "", wl[3], pv[8] = "", gr[8] = "", pwp[8] = "", psh[8] = "";
  if (p.rt != INT16_MIN)     fmtTenth(rt, sizeof(rt), p.rt);
  fmtTenth(vl, sizeof(vl), p.vl);
  fmtTenth(rl, sizeof(rl), p.rl);
  fmtTenth(ww, sizeof(ww), p.ww);
  if (p.flags & F_FLOW)      fmtTenth(fl, sizeof(fl), p.flow);
  if (p.flags & F_HZ)        snprintf(hz, sizeof(hz), "%u", p.hz);
  if (p.flags & F_FAN)     { snprintf(f1, sizeof(f1), "%u", p.fan1); snprintf(f2, sizeof(f2), "%u", p.fan2); }
  if (p.flags & F_SV)        snprintf(sv, sizeof(sv), "%u", p.sv);
  if (p.flags & F_WPVALID)   snprintf(wp, sizeof(wp), "%u", (p.flags & F_WPON) ? 1 : 0);
  if (p.flags & F_MODEVALID) snprintf(md, sizeof(md), "%u", p.mode);
  if (p.ta != INT16_MIN)   { fmtTenth(ta, sizeof(ta), p.ta); fmtTenth(ev, sizeof(ev), p.evap); }
  if (p.f2 & F2_EXT)       { snprintf(pel, sizeof(pel), "%d", p.pEl); snprintf(pth, sizeof(pth), "%d", p.pTh);
                             snprintf(df, sizeof(df), "%u", (p.f2 & F2_DEFROST) ? 1 : 0); }
  snprintf(wl, sizeof(wl), "%u", (p.f2 & F2_WWLOAD) ? 1 : 0);
  if (p.pv != 0xFFFF)        snprintf(pv, sizeof(pv), "%u", p.pv);
  if (p.grid != INT16_MIN)   snprintf(gr, sizeof(gr), "%d", p.grid);
  if (p.pElWp != INT16_MIN)  snprintf(pwp, sizeof(pwp), "%d", p.pElWp);
  if (p.pElSh != INT16_MIN)  snprintf(psh, sizeof(psh), "%d", p.pElSh);
  return snprintf(out, n, "%lu;%02d.%02d.%04d %02d:%02d:%02d;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s\n",
                  (unsigned long)p.t, tmv.tm_mday, tmv.tm_mon + 1, tmv.tm_year + 1900,
                  tmv.tm_hour, tmv.tm_min, tmv.tm_sec,
                  vl, rl, ww, hz, fl, f1, f2, sv, wp, md, rt, ta, ev, pel, pth, df, wl, pv, gr, pwp, psh);
}

static const char CSV_HEADER[] =
  "epoch_utc;Zeit;Vorlauf;Ruecklauf;Warmwasser;Verdichter_Hz;Durchfluss_Lmin;"
  "Luefter1_rpm;Luefter2_rpm;Sollwert;WP_Ein;Modus;Raum;"
  "Aussen;Verdampfer;P_el_W;P_th_W;Abtauen;WW_Ladung;PV_W;Netz_W;P_el_WP_W;P_el_Shelly_W\n";

// Schreibt alle noch nicht gesicherten Punkte (nur aus loop() aufrufen)
void sdFlush() {
  if (!sdOk || !logBuf) return;
  uint32_t pending = logTotal - sdFlushedTotal;
  if (pending == 0) return;
  if (pending > logCount) { sdFlushedTotal = logTotal - logCount; }

  File f;
  char curPath[32] = "";
  char line[220];

  for (uint32_t k = sdFlushedTotal; k < logTotal; k++) {
    // loop() ist der einzige Schreiber des Puffers -> Lesen ohne Lock ok
    LogPoint p = logAt(logCount - (logTotal - k));
    char path[32];
    dayFileName(p.t, path, sizeof(path));
    if (strcmp(path, curPath) != 0) {
      if (f) f.close();
      f = SD_MMC.open(path, FILE_APPEND);
      if (!f) { sdFail("Datei oeffnen"); return; }
      strlcpy(curPath, path, sizeof(curPath));
      if (f.size() == 0) f.print(CSV_HEADER);
    }
    int len = formatCsvLine(line, sizeof(line), p);
    if (f.write((const uint8_t*)line, len) != (size_t)len) { f.close(); sdFail("Schreiben"); return; }
    sdFlushedTotal = k + 1;
  }
  if (f) f.close();
}

// ---- CSV-Zeile -> Messpunkt (fuer Neustart und Tagesauswertung) ----
int16_t parseTenth(char* s, bool& ok) {
  ok = (s && *s);
  if (!ok) return 0;
  for (char* c = s; *c; c++) if (*c == ',') *c = '.';
  float v = strtof(s, nullptr);
  return (int16_t)lroundf(v * 10.0f);
}

bool parseCsvLine(char* line, LogPoint& p) {
  if (line[0] < '0' || line[0] > '9') return false;   // Kopfzeile
  char* fld[23]; int nf = 0;
  char* s = line;
  while (nf < 23) {
    fld[nf++] = s;
    char* sep = strchr(s, ';');
    if (!sep) break;
    *sep = 0; s = sep + 1;
  }
  if (nf < 5) return false;

  logPointInit(p);
  p.t = strtoul(fld[0], nullptr, 10);
  bool ok1, ok2, ok3, ok;
  p.vl = parseTenth(fld[2], ok1);
  p.rl = parseTenth(fld[3], ok2);
  p.ww = parseTenth(fld[4], ok3);
  if (!(ok1 && ok2 && ok3)) return false;
  p.flags = F_TEMPS;

  if (nf > 5  && *fld[5])  { p.hz = (uint8_t)min(255L, atol(fld[5])); p.flags |= F_HZ; }
  if (nf > 6)              { p.flow = parseTenth(fld[6], ok); if (ok) p.flags |= F_FLOW; }
  if (nf > 8  && *fld[7])  { p.fan1 = atoi(fld[7]); p.fan2 = atoi(fld[8]); p.flags |= F_FAN; }
  if (nf > 9  && *fld[9])  { p.sv = (uint8_t)atoi(fld[9]); p.flags |= F_SV; }
  if (nf > 10 && *fld[10]) { p.flags |= F_WPVALID; if (atoi(fld[10])) p.flags |= F_WPON; }
  if (nf > 11 && *fld[11]) { p.mode = (uint8_t)atoi(fld[11]); p.flags |= F_MODEVALID; }
  if (nf > 12 && *fld[12]) { p.rt = parseTenth(fld[12], ok); if (!ok) p.rt = INT16_MIN; }
  if (nf > 14 && *fld[13]) { p.ta = parseTenth(fld[13], ok); p.evap = parseTenth(fld[14], ok); }
  if (nf > 17 && *fld[15]) { p.pEl = atoi(fld[15]); p.pTh = atoi(fld[16]); p.f2 |= F2_EXT;
                             if (atoi(fld[17])) p.f2 |= F2_DEFROST; }
  if (nf > 18 && atoi(fld[18])) p.f2 |= F2_WWLOAD;
  if (nf > 19 && *fld[19]) p.pv = (uint16_t)constrain(atol(fld[19]), 0L, 65000L);
  if (nf > 20 && *fld[20]) p.grid = (int16_t)constrain(atol(fld[20]), -32000L, 32000L);
  if (nf > 21 && *fld[21]) p.pElWp = (int16_t)constrain(atol(fld[21]), 0L, 32000L);
  if (nf > 22 && *fld[22] && *fld[22] != '\r' && *fld[22] != '\n') p.pElSh = (int16_t)constrain(atol(fld[22]), 0L, 32000L);
  return true;
}

void restoreLine(char* line, uint32_t minT, uint32_t maxT) {
  LogPoint p;
  if (!parseCsvLine(line, p)) return;
  if (p.t < minT || p.t > maxT || p.t <= lastLoggedT()) return;
  logPush(p);
}

void restoreHistoryFromSd() {
  if (!sdOk || !logBuf || !timeValid()) return;
  restoringHistory = true;
  uint32_t t0 = millis();
  uint32_t now = nowUtc();
  uint32_t capSec = logCap * LOG_INTERVAL_SEC;
  uint32_t minT = now - capSec;
  uint32_t days = capSec / 86400UL + 1;

  static char rbuf[4096];
  char line[240];

  for (int32_t d = days; d >= 0; d--) {
    char path[32];
    dayFileName(now - d * 86400UL, path, sizeof(path));
    if (!SD_MMC.exists(path)) continue;
    File f = SD_MMC.open(path, FILE_READ);
    if (!f) continue;
    int ll = 0;
    int n;
    while ((n = f.read((uint8_t*)rbuf, sizeof(rbuf))) > 0) {
      for (int i = 0; i < n; i++) {
        char c = rbuf[i];
        if (c == '\n') { line[ll] = 0; restoreLine(line, minT, now + 60); ll = 0; }
        else if (c != '\r' && ll < (int)sizeof(line) - 1) line[ll++] = c;
      }
      delay(0);
    }
    if (ll) { line[ll] = 0; restoreLine(line, minT, now + 60); }
    f.close();
  }
  sdFlushedTotal = logTotal;               // nicht nochmal schreiben
  lastLogSlot = lastLoggedT() / LOG_INTERVAL_SEC;
  restoringHistory = false;
  Log.printf("Verlauf von SD geladen: %lu Punkte in %lu ms\n",
                (unsigned long)logCount, (unsigned long)(millis() - t0));
}

// Wird in loop() aufgerufen
void dfFlush();
void loggingTick() {
  if (!logBuf || !timeValid()) return;
  dfFlush();

  if (!historyRestored) {
    if (!storageReady) return;                   // erst evtl. fehlende Daten vom NAS holen
    historyRestored = true;
    restoreHistoryFromSd();
  }

  uint32_t now = nowUtc();
  uint32_t slot = now / LOG_INTERVAL_SEC;        // an Uhrzeit ausgerichtet -> kein Drift
  if (slot == lastLogSlot) return;
  lastLogSlot = slot;

  LiveData d = getLive();
  if (!d.tempsValid) return;                       // keine Phantomwerte loggen

  LogPoint p;
  logPointInit(p);
  p.t     = slot * LOG_INTERVAL_SEC;
  if (p.t <= lastLoggedT()) return;
  p.vl    = (int16_t)lroundf(d.vorlauf * 10.0f);
  p.rl    = (int16_t)lroundf(d.ruecklauf * 10.0f);
  p.ww    = (int16_t)lroundf(d.warmwasser * 10.0f);
  p.flags = F_TEMPS;
  if (d.hzValid)      { p.hz = (uint8_t)min((uint16_t)255, d.verdichterHz); p.flags |= F_HZ; }
  if (d.flowValid)    { p.flow = (int16_t)lroundf(d.flowLmin * 10.0f); p.flags |= F_FLOW; }
  if (d.fanRpmValid)  { p.fan1 = d.fan1Rpm; p.fan2 = d.fan2Rpm; p.flags |= F_FAN; }
  if (d.sollValid)    { p.sv = (uint8_t)constrain((int)lroundf(d.sollwert), 0, 255); p.flags |= F_SV; }
  if (d.wpStateValid) { p.flags |= F_WPVALID; if (d.wpOnOffRaw) p.flags |= F_WPON; }
  if (d.wpModeValid)  { p.mode = (uint8_t)d.wpModeRaw; p.flags |= F_MODEVALID; }
  p.rt = rtValid() ? (int16_t)lroundf(rtIst * 10.0f) : INT16_MIN;
  if (d.extValid) {
    p.ta   = (int16_t)lroundf(d.ta * 10.0f);
    p.evap = (int16_t)lroundf(d.evap * 10.0f);
    p.pEl  = (int16_t)constrain(lroundf(d.pEl), 0L, 32000L);
    p.pElWp = (int16_t)constrain(lroundf(d.pElWp), 0L, 32000L);
    p.pTh  = (int16_t)constrain(lroundf(d.pTh), -32000L, 32000L);
    p.f2  |= F2_EXT;
    if (d.state & 16) p.f2 |= F2_DEFROST;
  }
  if (wwState == WW_MOVE || wwState == WW_HEAT) p.f2 |= F2_WWLOAD;
  float pw = pvGet(pvPower, 120000UL), gr = pvGet(pvGrid, 120000UL);
  if (!isnan(pw)) p.pv = (uint16_t)constrain(lroundf(pw), 0L, 65000L);
  if (!isnan(gr)) p.grid = (int16_t)constrain(lroundf(gr), -32000L, 32000L);
  if (shFresh()) p.pElSh = (int16_t)constrain(lroundf((float)shP), 0L, 32000L);   // Shelly immer mitloggen
  logPush(p);
}

// Konsole zusaetzlich nach /log/console.txt schreiben (alle 10 s)
static uint32_t consoleSaved = 0;
static bool     consoleBootMark = false;
static uint32_t lastConsoleMs = 0;

void consoleFlush() {
  if (!sdOk) return;
  uint32_t total;
  portENTER_CRITICAL(&logMux); total = logRingTotal; portEXIT_CRITICAL(&logMux);
  if (total == consoleSaved && consoleBootMark) return;
  File f = SD_MMC.open("/log/console.txt", FILE_APPEND);
  if (!f) return;
  if (!consoleBootMark) { f.print("\n===== Neustart =====\n"); consoleBootMark = true; }
  if (total - consoleSaved > sizeof(logRing)) {          // Teil verloren (Puffer uebergelaufen)
    f.print("\n[... Konsole uebergelaufen ...]\n");
    consoleSaved = total - sizeof(logRing);
  }
  char buf[512];
  while (consoleSaved < total) {
    uint32_t n = min((uint32_t)sizeof(buf), total - consoleSaved);
    portENTER_CRITICAL(&logMux);
    for (uint32_t i = 0; i < n; i++) buf[i] = logRing[(consoleSaved + i) % sizeof(logRing)];
    portEXIT_CRITICAL(&logMux);
    f.write((const uint8_t*)buf, n);
    consoleSaved += n;
  }
  size_t sz = f.size();
  f.close();
  if (sz > (SD_MMC.kind() == 1 ? 192UL * 1024 : 2UL * 1024 * 1024)) {   // zu gross: umbenennen, neu anfangen (intern nur 192 KB)
    SD_MMC.remove("/log/console_alt.txt");
    SD_MMC.rename("/log/console.txt", "/log/console_alt.txt");
  }
}

void sdTick() {
  uint32_t ms = millis();
  if (sdOk && ms - lastConsoleMs >= 60000) { lastConsoleMs = ms; consoleFlush(); }   // GIGA: seltener auf den Stick
  if (!sdOk) {
    if (ms - lastSdTryMs >= SD_RETRY_MS) {
      lastSdTryMs = ms;
      if (mountSd()) lastSdFlushMs = ms - SD_FLUSH_MS;   // gleich nachschreiben
    }
    return;
  }
  if (ms - lastSdFlushMs >= SD_FLUSH_MS) {
    lastSdFlushMs = ms;
    sdFlush();
  }
  if (sdOk && ms - lastSdSpaceMs >= 3600000UL) updateSdSpace();
}

// =====================================================
// Modbus (laeuft NUR im Modbus-Task)
// =====================================================
void clearSerial1Buffer() { while (Serial1.available()) Serial1.read(); }

void preTransmission() {
  if (mbCfg.mode == 0 && mbCfg.de >= 0) digitalWrite(mbCfg.de, HIGH);
  delay(1);
}

// expectId: ID des angefragten Geraets. Ein Byte mit genau dieser ID ist schon der
// Anfang der Antwort (das Ventilmodul antwortet teils nach < 3 ms) und bleibt stehen.
void postTxDiscard(uint8_t expectId) {
  Serial1.flush();                       // warten bis das letzte Byte raus ist
  delayMicroseconds(11000000UL / mbCfg.baud + 100);   // GIGA: flush() wartet nur bis zum Schieberegister
  if (mbCfg.mode == 0 && mbCfg.de >= 0) digitalWrite(mbCfg.de, LOW);
  // Beim Umschalten Senden->Empfangen entsteht oft ein Stoerbyte (-> Fehler 224).
  // Ein Byte braucht bei 9600 Baud ~1 ms, also kurz warten und DANN verwerfen.
  if (mbCfg.gap) delay(mbCfg.gap);
  uint8_t junk[8]; int nj = 0;
  while (Serial1.available()) {
    if (Serial1.peek() == expectId) break;          // Antwort beginnt -> behalten
    int c = Serial1.read(); if (nj < 8) junk[nj] = (uint8_t)c; nj++;
  }
  if (nj && verbose) {
    Log.printf("Modbus: %d Stoerbyte(s) verworfen:", nj);
    for (int i = 0; i < min(nj, 8); i++) Log.printf(" %02X", junk[i]);
    Log.println();
  }
}
void postTransmission() { postTxDiscard(mbCfg.id); }

void mbSetup() {
  if (mbCfg.de >= 0) { pinMode(mbCfg.de, OUTPUT); digitalWrite(mbCfg.de, LOW); }
  mbCfg.mode = 0;                        // GIGA: DE immer per Software (feste Pins D0/D1)
  Serial1.begin(mbCfg.baud);             // 8N1, RX mit Pull-up
  node.begin(mbCfg.id, Serial1);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  node.idle([]() { vTaskDelay(1); });    // beim Warten auf Antwort CPU freigeben
  Log.printf("Modbus: RX=IO%d TX=IO%d DE=IO%d, %s, %lu Baud, Slave %u, Pause %u ms\n",
             mbCfg.rx, mbCfg.tx, mbCfg.de, mbCfg.mode ? "Hardware-RS485" : "DE per Software",
             (unsigned long)mbCfg.baud, mbCfg.id, mbCfg.gap);
}

volatile uint8_t mbFailStreak = 0;     // aufeinanderfolgende Fehlversuche (WP nicht erreichbar?)

uint8_t readRegsRetry(uint16_t startReg, uint8_t count) {
  uint8_t result = 255;
  uint8_t tries = mbFailStreak >= 3 ? 1 : 3;   // offline: nicht jede Abfrage 3x 2 s blockieren
  for (uint8_t attempt = 0; attempt < tries; attempt++) {
    clearSerial1Buffer();
    result = node.readHoldingRegisters(startReg, count);
    if (result == node.ku8MBSuccess) { mbFailStreak = 0; return result; }
    delay(180);
  }
  if (mbFailStreak < 255) mbFailStreak++;
  liveLock(); live.lastErr = result; live.lastErrMs = millis(); liveUnlock();
  return result;
}

uint8_t writeRegRetry(uint16_t reg, uint16_t value) {
  uint8_t result = 255;
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    clearSerial1Buffer();
    result = node.writeSingleRegister(reg, value);
    if (result == node.ku8MBSuccess) return result;
    delay(250);
  }
  liveLock(); live.lastErr = result; live.lastErrMs = millis(); liveUnlock();
  return result;
}

static inline void countOk()  { liveLock(); live.okCount++;  liveUnlock(); }

// ---------- Installateur-Parameter (Block 0x2000, Adresse = 8192 + P - 2) ----------
// 8194 P105 Plattformfrequenz Verdichter (Hz), 8195 P106 Plattform-Verzoegerung,
// 8196 P06 Hysterese Heizen (0,1 K), 8197 P07 Hysterese Warmwasser (0,1 K)
static const uint16_t REG_PAR_BASE = 8194;
volatile uint16_t wpPar[4] = {0, 0, 0, 0};
volatile bool     wpParValid = false;
volatile uint32_t wpParMs = 0;
volatile uint8_t  rawReadN = 1;            // Konsole "RD": Anzahl Register
bool readWpParams();
static inline void countErr() { liveLock(); live.errCount++; liveUnlock(); }

bool readWpParams() {
  uint8_t r = readRegsRetry(REG_PAR_BASE, 4);
  if (r != node.ku8MBSuccess) {
    countErr();
    Log.printf("Fehler Parameter lesen (8194..8197): %u (%s)\n", r, mbErrText(r));
    return false;
  }
  countOk();
  bool changed = !wpParValid;
  for (int i = 0; i < 4; i++) {
    uint16_t v = node.getResponseBuffer(i);
    if (v != wpPar[i]) changed = true;
    wpPar[i] = v;
  }
  wpParValid = true; wpParMs = millis();
  if (changed) Log.printf("WP-Parameter: P105 Plattform %u Hz, P106 Verzoegerung %u, P06 Hyst. Heizen %.1f K (roh %u), P07 Hyst. WW %.1f K (roh %u)\n",
                          wpPar[0], wpPar[1], wpPar[2] / 10.0f, wpPar[2], wpPar[3] / 10.0f, wpPar[3]);
  return true;
}

bool setWpParam(uint16_t reg, uint16_t raw, const char* name) {
  Log.printf("Schreibe %s: Register %u = %u\n", name, reg, raw);
  uint8_t r = writeRegRetry(reg, raw);
  if (r != node.ku8MBSuccess) {
    countErr();
    Log.printf("Fehler %s schreiben: %u (%s)\n", name, r, mbErrText(r));
    setCmdMsg("%s schreiben FEHLER", name);
    return false;
  }
  delay(800);
  readWpParams();
  bool ok = wpParValid && wpPar[reg - REG_PAR_BASE] == raw;
  Log.printf("%s Ruecklesen: %u %s\n", name, wpPar[reg - REG_PAR_BASE], ok ? "OK" : "-> NICHT uebernommen");
  setCmdMsg("%s %s", name, ok ? "geschrieben" : "nicht uebernommen");
  return ok;
}

bool readRawRegs(uint16_t addr, uint8_t n) {
  n = constrain(n, 1, 20);
  uint8_t r = readRegsRetry(addr, n);
  if (r != node.ku8MBSuccess) { Log.printf("RD %u: Fehler %u (%s)\n", addr, r, mbErrText(r)); return false; }
  Log.printf("RD %u..%u:", addr, addr + n - 1);
  for (int i = 0; i < n; i++) Log.printf(" %u", node.getResponseBuffer(i));
  Log.println();
  return true;
}

bool readWpStateAndMode() {
  uint8_t r = readRegsRetry(REG_ONOFF, 2);
  if (r == node.ku8MBSuccess) {
    uint16_t onoff = node.getResponseBuffer(0), mode = node.getResponseBuffer(1);
    liveLock();
    live.wpOnOffRaw = onoff; live.wpModeRaw = mode;
    live.wpStateValid = live.wpModeValid = true;
    live.okCount++;
    liveUnlock();
    MBLOG("WP OK   | OnOff=%u Mode=%u %s\n", onoff, mode, modeName(mode));
    return true;
  }
  countErr();
  Log.printf("Fehler WP Status lesen (4096..4097): %u (%s)\n", r, mbErrText(r));
  return false;
}

bool readTemps() {
  uint8_t r = readRegsRetry(REG_RL, 3);
  if (r == node.ku8MBSuccess) {
    float rl = (int16_t)node.getResponseBuffer(0) / 10.0f;
    float vl = (int16_t)node.getResponseBuffer(1) / 10.0f;
    float ww = (int16_t)node.getResponseBuffer(2) / 10.0f;
    liveLock();
    live.ruecklauf = rl; live.vorlauf = vl; live.warmwasser = ww;
    live.tempsValid = true; live.okCount++;
    liveUnlock();
    MBLOG("Temps OK | RL=%.1f  VL=%.1f  WW=%.1f\n", rl, vl, ww);
    return true;
  }
  countErr();
  Log.printf("Fehler Temps (4365..4367): %u (%s)\n", r, mbErrText(r));
  return false;
}

bool readFlow() {
  uint8_t r = readRegsRetry(REG_FLOW, 1);
  if (r == node.ku8MBSuccess) {
    uint16_t raw = node.getResponseBuffer(0);
    liveLock();
    live.flowRaw = raw; live.flowLmin = raw / 10.0f; live.flowValid = true; live.okCount++;
    liveUnlock();
    MBLOG("Flow OK  | Reg 4368 raw=%u -> %.1f L/min\n", raw, raw / 10.0f);
    return true;
  }
  countErr();
  Log.printf("Fehler Flow lesen (4368): %u (%s)\n", r, mbErrText(r));
  return false;
}

bool readHz() {
  uint8_t r = readRegsRetry(REG_HZ, 1);
  if (r == node.ku8MBSuccess) {
    uint16_t hz = node.getResponseBuffer(0);
    liveLock();
    live.verdichterHz = hz; live.hzValid = true; live.okCount++;
    liveUnlock();
    MBLOG("Hz OK    | VD=%u\n", hz);
    return true;
  }
  countErr();
  Log.printf("Fehler Hz (4373): %u (%s)\n", r, mbErrText(r));
  return false;
}

bool readFanRpm() {
  uint8_t r = readRegsRetry(REG_FAN1_RPM, 2);
  if (r == node.ku8MBSuccess) {
    uint16_t f1 = node.getResponseBuffer(0), f2 = node.getResponseBuffer(1);
    liveLock();
    live.fan1Rpm = f1; live.fan2Rpm = f2; live.fanRpmValid = true; live.okCount++;
    liveUnlock();
    MBLOG("Fan OK   | Fan1=%u rpm  Fan2=%u rpm\n", f1, f2);
    return true;
  }
  countErr();
  Log.printf("Fehler Fan RPM lesen (4374..4375): %u (%s)\n", r, mbErrText(r));
  return false;
}

// ---------- Blocklesen: alle Temperaturen, Druecke, Strom ... in einer Abfrage ----------
volatile bool blockReadOk = true;      // false, wenn die WP Blocklesen ablehnt -> Einzelabfragen
uint16_t rawBlock[33];                 // Rohwerte 4354..4386 (Diagnose im Web)
uint16_t rawCnt[5];                    // Rohwerte 4608..4612
volatile bool rawBlockValid = false, rawCntValid = false;
uint8_t  gridPhases = 3;               // Netzanschluss WP: 1 oder 3 Phasen
uint16_t pElFactor = 100;              // Korrektur el. Leistung in % (Abgleich mit echtem Zaehler)
bool shFresh() { return shMs && millis() - shMs < SH_MAX_AGE && !isnan((float)shP); }
static uint32_t lastEnergyMs = 0;
static int      lastEnergyDay = -1;

void energyAccumulate(LiveData& L) {   // unter liveLock aufrufen
  uint32_t ms = millis();
  if (lastEnergyMs && ms - lastEnergyMs < 60000UL) {
    double h = (ms - lastEnergyMs) / 3600000.0;
    if (L.pEl > 0) { L.eElDay += L.pEl * h; L.eElTot += L.pEl * h; }
    if (L.pTh > 0) { L.eThDay += L.pTh * h; L.eThTot += L.pTh * h; }
  }
  lastEnergyMs = ms;
  if (timeValid()) {
    time_t tt = time(nullptr); struct tm tmv; localtime_r(&tt, &tmv);
    if (lastEnergyDay >= 0 && tmv.tm_yday != lastEnergyDay) {
      bool yesterday = (tmv.tm_yday == lastEnergyDay + 1) || (tmv.tm_yday == 0 && lastEnergyDay >= 364);
      L.eElYest = yesterday ? L.eElDay : 0; L.eThYest = yesterday ? L.eThDay : 0;
      L.eElDay = 0; L.eThDay = 0;
    }
    lastEnergyDay = tmv.tm_yday;
  }
}

bool readStatusBlock() {
  uint8_t r = readRegsRetry(4354, 33);            // 4354 .. 4386
  if (r != node.ku8MBSuccess) {
    countErr();
    Log.printf("Fehler Statusblock (4354..4386): %u (%s)\n", r, mbErrText(r));
    if (r >= 1 && r <= 4) { blockReadOk = false; Log.println("WP lehnt Blocklesen ab -> Einzelabfragen"); }
    return false;
  }
  auto S = [](uint16_t a) -> int16_t { return (int16_t)node.getResponseBuffer(a - 4354); };
  for (int i = 0; i < 33; i++) rawBlock[i] = node.getResponseBuffer(i);
  rawBlockValid = true;
  liveLock();
  LiveData& L = live;
  L.state = S(4354); L.outputs = S(4355); L.inputs = S(4356);
  L.coil = S(4358) / 10.0f;   L.discharge = S(4359) / 10.0f; L.ta = S(4360) / 10.0f;
  L.suction = S(4361) / 10.0f; L.eviIn = S(4362) / 10.0f;   L.eviOut = S(4363) / 10.0f;
  L.liquid = S(4364) / 10.0f;
  L.ruecklauf = S(4365) / 10.0f; L.vorlauf = S(4366) / 10.0f; L.warmwasser = S(4367) / 10.0f;
  L.tempsValid = true;
  L.flowRaw = (uint16_t)S(4368); L.flowLmin = L.flowRaw / 10.0f; L.flowValid = true;
  L.dT = S(4369) / 10.0f;
  L.hp = S(4371) / 100.0f; L.lp = S(4372) / 100.0f;
  L.verdichterHz = (uint16_t)S(4373); L.hzValid = true;
  L.fan1Rpm = (uint16_t)S(4374); L.fan2Rpm = (uint16_t)S(4375); L.fanRpmValid = true;
  L.targetHz = (uint16_t)S(4378);
  L.currentA = S(4379) / 10.0f; L.ipm = S(4380);    // IPM in ganzen Grad (WP-Display: TPM 17)
  L.acV = S(4381); L.dcV = S(4382);       // Volt ohne Nachkomma (Anzeige zeigte 24 statt 240)
  L.t6 = S(4383) / 10.0f; L.room = S(4384) / 10.0f;
  L.evap = S(4385) / 10.0f; L.cond = S(4386) / 10.0f;
  // el. Leistung (Schaetzung): Strom x Spannung x cos phi; bei Drehstrom x3 (L-N) bzw. x1,73 (L-L)
  float u = (L.acV > 150 && L.acV < 480) ? L.acV : 230.0f;
  float k = (gridPhases == 3) ? (u > 300 ? 1.732f : 3.0f) : 1.0f;
  L.pElWp = (L.verdichterHz > 0 && L.currentA > 0) ? k * u * L.currentA * 0.95f * pElFactor / 100.0f : 0;
  L.pElSh = shFresh() ? max(0.0f, (float)shP) : NAN;
  L.pElSrc = (elSrc == 1 && !isnan(L.pElSh)) ? 1 : 0;     // Shelly fehlt -> automatisch Schaetzung
  L.pEl = L.pElSrc ? L.pElSh : L.pElWp;
  if (!isnan(L.pElSh) && L.verdichterHz > 0 && L.pElWp > 200) {     // fuer "Vorschlag Korrektur"
    shCmpSh += L.pElSh; shCmpWp += L.pElWp * 100.0f / pElFactor;     // Schaetzung ohne Korrektur
  }
  L.pTh = (L.flowLmin > 1.0f) ? L.flowLmin / 60.0f * 4186.0f * (L.vorlauf - L.ruecklauf) : 0;
  L.extValid = true;
  energyAccumulate(L);
  L.okCount++;
  float vl = L.vorlauf, rl = L.ruecklauf, ww = L.warmwasser, ta = L.ta;
  liveUnlock();
  MBLOG("Block OK | VL=%.1f RL=%.1f WW=%.1f Aussen=%.1f\n", vl, rl, ww, ta);
  return true;
}

bool readCounterBlock() {
  uint8_t r = readRegsRetry(4608, 5);             // 4608 .. 4612
  if (r != node.ku8MBSuccess) {
    countErr();
    Log.printf("Fehler Zaehlerblock (4608..4612): %u (%s)\n", r, mbErrText(r));
    return false;
  }
  liveLock();
  for (int i = 0; i < 5; i++) rawCnt[i] = node.getResponseBuffer(i);
  rawCntValid = true;
  live.starts = node.getResponseBuffer(0);
  live.runtimeH = node.getResponseBuffer(2);
  live.pumpPct = node.getResponseBuffer(3);
  live.opMode = node.getResponseBuffer(4);
  live.cntValid = true;
  live.okCount++;
  liveUnlock();
  return true;
}

bool readSollwert() {
  uint8_t r = readRegsRetry(REG_SOLL, 1);
  if (r == node.ku8MBSuccess) {
    uint16_t raw = node.getResponseBuffer(0);
    liveLock();
    live.sollwert = raw; live.sollValid = true; live.okCount++;
    liveUnlock();
    MBLOG("Soll OK  | 4098 raw=%u  -> %.1f\n", raw, (float)raw);
    return true;
  }
  countErr();
  Log.printf("Fehler Soll lesen (4098): %u (%s)\n", r, mbErrText(r));
  return false;
}

bool readMaxHzValue(uint16_t& value) {
  uint8_t r = readRegsRetry(REG_MAX_HZ, 1);
  if (r == node.ku8MBSuccess) {
    value = node.getResponseBuffer(0);
    liveLock();
    live.maxVerdichterHz = value; live.maxHzValid = true; live.okCount++;
    liveUnlock();
    return true;
  }
  countErr();
  Log.printf("Fehler MaxHz lesen (8235/P45): %u (%s)\n", r, mbErrText(r));
  return false;
}

bool readMaxHz() {
  uint16_t v;
  if (!readMaxHzValue(v)) return false;
  MBLOG("MaxHz OK | P45=%u\n", v);
  return true;
}

bool readMaxFanValue(uint16_t& valueRpm) {
  uint8_t r = readRegsRetry(REG_MAX_FAN_RPM, 1);
  if (r == node.ku8MBSuccess) {
    uint16_t raw = node.getResponseBuffer(0);
    valueRpm = raw * 10;
    liveLock();
    live.maxFanRaw = raw; live.maxFanRpm = valueRpm; live.maxFanValid = true; live.okCount++;
    liveUnlock();
    return true;
  }
  countErr();
  Log.printf("Fehler MaxFan lesen (8250/P60): %u (%s)\n", r, mbErrText(r));
  return false;
}

bool readMaxFan() {
  uint16_t v;
  if (!readMaxFanValue(v)) return false;
  MBLOG("MaxFan OK | P60 raw=%u -> %u rpm\n", v / 10, v);
  return true;
}

// ---------- Schreiben ----------
bool setSollwert(float tempC) {
  uint16_t raw = (uint16_t)(tempC + 0.5f);
  Log.printf("Schreibe 4098 = %u (%.1f C)\n", raw, tempC);
  uint8_t r = writeRegRetry(REG_SOLL, raw);
  if (r == node.ku8MBSuccess) {
    Log.println("Schreiben OK");
    delay(500);
    readSollwert();
    setCmdMsg("Sollwert %u C geschrieben", raw);
    return true;
  }
  countErr();
  Log.printf("Fehler Soll schreiben (4098): %u (%s)\n", r, mbErrText(r));
  setCmdMsg("Sollwert schreiben FEHLER");
  return false;
}

bool setMaxHz(uint16_t hz) {
  if (hz < MAX_HZ_MIN || hz > MAX_HZ_MAX) {
    Log.printf("Ungueltig. MaxHz nur %u..%u erlaubt.\n", MAX_HZ_MIN, MAX_HZ_MAX);
    return false;
  }
  Log.printf("SetMaxHz Ziel = %u Hz\n", hz);
  for (uint8_t attempt = 1; attempt <= 5; attempt++) {
    Log.printf("MaxHz Versuch %u/5: schreibe P45 = %u\n", attempt, hz);
    uint8_t r = writeRegRetry(REG_MAX_HZ, hz);
    if (r != node.ku8MBSuccess) {
      countErr();
      Log.printf("Fehler MaxHz schreiben (8235/P45): %u (%s)\n", r, mbErrText(r));
      delay(600);
      continue;
    }
    delay(1200);
    uint16_t rb = 0;
    if (readMaxHzValue(rb)) {
      Log.printf("MaxHz Ruecklesen = %u\n", rb);
      if (rb == hz) {
        Log.println("MaxHz wirklich uebernommen");
        setCmdMsg("MaxHz %u uebernommen", hz);
        return true;
      }
    }
    delay(800);
  }
  Log.println("MaxHz NICHT sicher uebernommen");
  setCmdMsg("MaxHz NICHT uebernommen");
  return false;
}

bool setMaxFan(uint16_t rpm) {
  if (rpm < FAN_RPM_MIN || rpm > FAN_RPM_MAX) {
    Log.printf("Ungueltig. MaxFan nur %u..%u rpm erlaubt.\n", FAN_RPM_MIN, FAN_RPM_MAX);
    return false;
  }
  uint16_t raw = (rpm + 5) / 10;
  Log.printf("SetMaxFan Ziel = %u rpm, schreibe raw=%u\n", rpm, raw);
  for (uint8_t attempt = 1; attempt <= 5; attempt++) {
    Log.printf("MaxFan Versuch %u/5: schreibe P60 raw=%u\n", attempt, raw);
    uint8_t r = writeRegRetry(REG_MAX_FAN_RPM, raw);
    if (r != node.ku8MBSuccess) {
      countErr();
      Log.printf("Fehler MaxFan schreiben (8250/P60): %u (%s)\n", r, mbErrText(r));
      delay(600);
      continue;
    }
    delay(1200);
    uint16_t rbRpm = 0;
    if (readMaxFanValue(rbRpm)) {
      Log.printf("MaxFan Ruecklesen = %u rpm\n", rbRpm);
      if (rbRpm == raw * 10) {
        Log.println("MaxFan wirklich uebernommen");
        setCmdMsg("MaxFan %u rpm uebernommen", rbRpm);
        return true;
      }
    }
    delay(800);
  }
  Log.println("MaxFan NICHT sicher uebernommen");
  setCmdMsg("MaxFan NICHT uebernommen");
  return false;
}

bool setHeatpumpOnOff(bool on) {
  Log.printf("Schreibe WP %s\n", on ? "EIN" : "AUS");
  uint8_t r = writeRegRetry(REG_ONOFF, on ? 1 : 0);
  if (r == node.ku8MBSuccess) {
    Log.println("WP OnOff Schreiben OK");
    for (int i = 0; i < 3; i++) {                      // Ruecklesen, bei Abweichung erneut schreiben
      delay(800);
      if (readWpStateAndMode()) {
        LiveData d = getLive();
        if ((d.wpOnOffRaw != 0) == on) break;
        Log.printf("WP OnOff Ruecklesen: 4096 = %u statt %u -> erneut schreiben\n", d.wpOnOffRaw, on ? 1 : 0);
        writeRegRetry(REG_ONOFF, on ? 1 : 0);
      }
    }
    setCmdMsg("WP %s geschrieben", on ? "EIN" : "AUS");
    return true;
  }
  countErr();
  Log.printf("Fehler WP OnOff schreiben (4096): %u (%s)\n", r, mbErrText(r));
  setCmdMsg("WP schalten FEHLER");
  return false;
}

bool setMode(uint16_t mode) {
  if (!(mode == 1 || mode == 2 || mode == 4)) {
    Log.println("Ungueltiger Modus. Erlaubt: 1=WW, 2=Heizen, 4=Kuehlen");
    return false;
  }
  Log.printf("Schreibe Mode %u %s\n", mode, modeName(mode));
  uint8_t r = writeRegRetry(REG_MODE, mode);
  if (r == node.ku8MBSuccess) {
    Log.println("Mode Schreiben OK");
    delay(500);
    readWpStateAndMode();
    setCmdMsg("Modus %s geschrieben", modeName(mode));
    return true;
  }
  countErr();
  Log.printf("Fehler Mode schreiben (4097): %u (%s)\n", r, mbErrText(r));
  setCmdMsg("Modus schreiben FEHLER");
  return false;
}

// =====================================================
// Befehls-Queue (Web / Touch / Serial -> Modbus-Task)
// =====================================================

QueueHandle_t mbQueue;
SemaphoreHandle_t mbDoneSem;      // nur fuer wartende Aufrufe aus loop()
volatile bool mbDoneResult = false;

bool executeCmd(const MbCmd& c) {
  switch (c.type) {
    case CMD_SET_SOLL:    return setSollwert(c.value);
    case CMD_POWER:       return setHeatpumpOnOff(c.value > 0.5f);
    case CMD_MODE:        return setMode((uint16_t)c.value);
    case CMD_MAXHZ:       return setMaxHz((uint16_t)c.value);
    case CMD_MAXFAN:      return setMaxFan((uint16_t)c.value);
    case CMD_READ_SOLL:   return readSollwert();
    case CMD_READ_MAXHZ:  return readMaxHz();
    case CMD_READ_MAXFAN: return readMaxFan();
    case CMD_READ_FAN:    return readFanRpm();
    case CMD_READ_FLOW:   return readFlow();
    case CMD_READ_PARAMS: return readWpParams();
    case CMD_SET_P06:     return setWpParam(8196, (uint16_t)c.value, "P06 Hysterese Heizen");
    case CMD_SET_P07:     return setWpParam(8197, (uint16_t)c.value, "P07 Hysterese WW");
    case CMD_READ_RAW:    return readRawRegs((uint16_t)c.value, rawReadN);
    case CMD_READ_ALL: {
      bool ok = readWpStateAndMode();
      ok &= readMaxHz(); ok &= readMaxFan(); ok &= readFanRpm();
      ok &= readFlow();  ok &= readSollwert();
      return ok;
    }
  }
  return false;
}

// Aus loop() (Web-Handler): wartet auf Ergebnis inkl. Ruecklesen
bool runCmdBlocking(CmdType t, float v, uint32_t timeoutMs) {
  xSemaphoreTake(mbDoneSem, 0);                    // evtl. alten Zustand loeschen
  MbCmd c{t, v, true, false};
  if (xQueueSend(mbQueue, &c, pdMS_TO_TICKS(2000)) != pdTRUE) return false;
  if (xSemaphoreTake(mbDoneSem, pdMS_TO_TICKS(timeoutMs)) != pdTRUE) return false;
  return mbDoneResult;
}

// Von Touch / Serial: nicht warten
bool submitCmd(CmdType t, float v, bool print) {
  MbCmd c{t, v, false, print};
  return xQueueSend(mbQueue, &c, 0) == pdTRUE;
}

bool valveSlaveActive();      // im Ventil-Teil weiter unten definiert
void processSerialCmd(String cmd);
void valveSlavePoll();

void modbusTask(void*) {
  uint32_t now = millis();
  enum { J_WP, J_TEMPS, J_FLOW, J_HZ, J_FAN, J_SOLL, J_MAXHZ, J_MAXFAN, J_VALVE, J_BLOCK, J_CNT, J_PAR, NJOBS };
  struct Job { uint32_t interval; uint32_t last; void (*fn)(); };
  Job jobs[NJOBS] = {
    {WPSTATE_INTERVAL_MS, 0, []() { readWpStateAndMode(); }},
    {TEMPS_INTERVAL_MS,   0, []() { readTemps(); }},
    {FLOW_INTERVAL_MS,    0, []() { readFlow(); }},
    {HZ_INTERVAL_MS,      0, []() { readHz(); }},
    {FANRPM_INTERVAL_MS,  0, []() { readFanRpm(); }},
    {SOLL_INTERVAL_MS,    0, []() { readSollwert(); }},
    {MAXHZ_INTERVAL_MS,   0, []() { readMaxHz(); }},
    {MAXFAN_INTERVAL_MS,  0, []() { readMaxFan(); }},
    {2000,                0, []() { valveSlavePoll(); }},
    {TEMPS_INTERVAL_MS,   0, []() { readStatusBlock(); }},   // ersetzt Temps/Flow/Hz/Fan
    {30000,               0, []() { readCounterBlock(); }},
    {600000,              0, []() { readWpParams(); }},       // Installateur-Parameter alle 10 min
  };
  for (int i = 0; i < NJOBS; i++) jobs[i].last = now - jobs[i].interval;   // alles gleich zu Beginn

  for (;;) {
    MbCmd c;
    if (xQueueReceive(mbQueue, &c, pdMS_TO_TICKS(20)) == pdTRUE) {
      cmdPrint = c.print;
      bool ok = executeCmd(c);
      cmdPrint = false;
      if (c.wait) { mbDoneResult = ok; xSemaphoreGive(mbDoneSem); }
      continue;     // weitere Befehle zuerst
    }

    // Pro Durchlauf hoechstens EINE Abfrage -> Befehle werden schnell bedient.
    // Faire Auswahl: die am staerksten ueberfaellige Abfrage kommt dran, und
    // "zuletzt" wird erst NACH der Abfrage gesetzt. So kann eine langsame /
    // fehlschlagende Abfrage die anderen nicht mehr verdraengen.
    now = millis();
    int best = -1; float bestRatio = 1.0f;
    for (int i = 0; i < NJOBS; i++) {
      if (i == J_VALVE && !valveSlaveActive()) continue;
      bool single = (i == J_TEMPS || i == J_FLOW || i == J_HZ || i == J_FAN);
      if (single && blockReadOk) continue;           // per Blocklesen abgedeckt
      if (i == J_BLOCK && !blockReadOk) continue;
      float ratio = (float)(now - jobs[i].last) / jobs[i].interval;
      if (ratio >= bestRatio) { bestRatio = ratio; best = i; }
    }
    if (best >= 0) {
      jobs[best].fn();
      jobs[best].last = millis();
    }

    // Lange keine Antwort der WP -> Werte als ungueltig markieren (Anzeige "--")
    if (live.okCount > 0 || mbFailStreak) {
      static uint32_t lastOkMs = 0;
      static uint32_t lastOkCount = 0;
      liveLock();
      if (live.okCount != lastOkCount) { lastOkCount = live.okCount; lastOkMs = millis(); }
      bool stale = millis() - lastOkMs > 60000UL;
      if (stale && (live.tempsValid || live.wpStateValid)) {
        live.tempsValid = live.flowValid = live.hzValid = live.fanRpmValid = false;
        live.sollValid = live.maxHzValid = live.maxFanValid = false;
        live.wpStateValid = live.wpModeValid = false;
        live.extValid = live.cntValid = false;
        live.pEl = live.pTh = 0;
        liveUnlock();
        Log.println("Modbus: seit 60 s keine Antwort der WP -> Werte ungueltig");
      } else liveUnlock();
    }
  }
}

// =====================================================
// ESBE-Ventil + Warmwasser-Ladung (eigener Task "ctrl")
// =====================================================
//
//  Relais 1 (pinPwr): Motor-Versorgung EIN/AUS
//  Relais 2 (pinDir): Richtung. AUS (NC) = Richtung Heizung, AN (NO) = Richtung WW
//  Ventilstellung intern: 0 % = nur Heizung, 100 % = nur WW-Speicher, bufPos = Puffer
//
//  Ablauf jeder Bewegung: Versorgung aus -> Richtung setzen -> 300 ms -> Versorgung an
//  -> Laufzeit -> Versorgung aus -> 300 ms -> Richtungsrelais in Ruhe (aus)
//  Endlagen werden immer mit 10 % Ueberlauf angefahren (Endschalter im Motor),
//  dadurch kalibriert sich die Positionsschaetzung bei jeder Endlagenfahrt neu.

CtrlCfg ctrlCfg;
SemaphoreHandle_t ctrlMutex;

#define RELAY_PINS_TXT "D2..D13, D22..D53"
bool relayPinAllowed(int p) {          // D0/D1 = RS485, D14..D21 = weitere UARTs / I2C
  return p == -1 || (p >= 2 && p <= 13) || (p >= 22 && p <= 53);
}

void ctrlDefaults(CtrlCfg& c) {
  c = CtrlCfg();
  for (int d = 0; d < 7; d++) {
    for (int s = 0; s < SCHED_SLOTS; s++) { c.sched[d][s].min = 0; c.sched[d][s].act = ACT_NONE; }
    c.sched[d][0] = {18 * 60, ACT_WW};       // Beispiel: 18:00 WW laden
    c.sched[d][1] = {21 * 60, ACT_PUFFER};   //           21:00 Speicher als Puffer
  }
}

void loadCtrlCfg() {
  Preferences p;
  p.begin("ctrl", true);
  CtrlCfg tmp;                                   // neue Felder behalten ihre Standardwerte
  size_t len = p.getBytesLength("cfg");
  const size_t oldLen = (offsetof(CtrlCfg, abFixOn) + alignof(CtrlCfg) - 1) / alignof(CtrlCfg) * alignof(CtrlCfg);   // bis v1.33
  bool ok = (len == sizeof(CtrlCfg) || len == offsetof(CtrlCfg, wwMode) || len == oldLen || len == offsetof(CtrlCfg, abFixOn) || len == offsetof(CtrlCfg, abOn) || len == offsetof(CtrlCfg, wwPullMin) || len == offsetof(CtrlCfg, wwNoCont) || len == offsetof(CtrlCfg, drive)) &&
            p.getBytes("cfg", &tmp, len) == len && tmp.version == CTRL_CFG_VERSION;
  p.end();
  if (ok && len < offsetof(CtrlCfg, wwMode)) { CtrlCfg d; tmp.abFixOn = d.abFixOn; tmp.abFixT = d.abFixT; }   // Fuellbytes alter Versionen
  if (ok && len < sizeof(CtrlCfg)) tmp.wwMode = CtrlCfg().wwMode;                                         // bis v1.35
  if (ok) ctrlCfg = tmp; else ctrlDefaults(ctrlCfg);
}

void saveCtrlCfg() {
  CtrlCfg tmp;
  xSemaphoreTake(ctrlMutex, portMAX_DELAY); tmp = ctrlCfg; xSemaphoreGive(ctrlMutex);
  Preferences p;
  p.begin("ctrl", false);
  p.putBytes("cfg", &tmp, sizeof(tmp));
  p.end();
}

CtrlCfg getCtrlCfg() {
  xSemaphoreTake(ctrlMutex, portMAX_DELAY);
  CtrlCfg c = ctrlCfg;
  xSemaphoreGive(ctrlMutex);
  return c;
}

// ---------- Relais ----------
void relayWrite(int8_t pin, bool on) {
  if (pin < 0) return;
  digitalWrite(pin, ctrlCfg.actLow ? !on : on);
}

void relaysInit() {                 // sofort beim Start: beide Relais AUS
  if (ctrlCfg.drive != 0) return;   // Ventilmodul am Modbus -> keine ESP-Relais
  for (int8_t pin : {ctrlCfg.pinPwr, ctrlCfg.pinDir}) {
    if (pin < 0) continue;
    digitalWrite(pin, ctrlCfg.actLow ? HIGH : LOW);   // Pegel vor pinMode setzen -> kein Klacken
    pinMode(pin, OUTPUT);
    digitalWrite(pin, ctrlCfg.actLow ? HIGH : LOW);
  }
}

void dirRelay(bool towardWW) { relayWrite(ctrlCfg.pinDir, ctrlCfg.dirInv ? !towardWW : towardWW); }

// ---------- Ventil-Zustand ----------
enum VPhase : uint8_t { VP_IDLE, VP_DIR, VP_RUN, VP_STOP };

volatile float valvePos = 0;        // geschaetzte Stellung 0..100 %
volatile bool  valveKnown = false;  // Stellung bekannt (nach Endlagenfahrt / gespeichert)
volatile bool  valveMoving = false;
volatile int   valveTarget = -1;    // Ziel der laufenden Fahrt
volatile bool  valveDirWW = false;
static VPhase  vphase = VP_IDLE;
static uint32_t vphaseMs = 0, vRunMs = 0;
static float   vStartPos = 0;
static int     vQueued = -1;        // zweiter Schritt (z. B. erst Endlage, dann Puffer)

volatile int   manualPos = -1;      // -1 = Automatik, sonst 0 / bufPos / 100
volatile int   reqValve = -2;       // Anforderung von Web/Display: -2 nichts, -1 Auto, 0..100
volatile bool  reqWwStart = false, reqWwStop = false;
volatile uint8_t reqWwTarget = 0;      // 0 = WW-Soll aus den Einstellungen
volatile bool  reqWwPv = false;
volatile bool  reqWwManual = false;  // "Warmwasser jetzt" (Display/Web/MQTT): WP immer einschalten, danach Ventil auf Heizung halten
volatile bool  reqValveRef = false;

void saveValveState(bool moving) {
  Preferences p;
  p.begin("valve", false);
  p.putFloat("pos", valvePos);
  p.putBool("known", valveKnown);
  p.putBool("moving", moving);
  p.end();
}

void loadValveState() {
  Preferences p;
  p.begin("valve", true);
  float pos = p.getFloat("pos", 0);
  bool known = p.getBool("known", false);
  bool moving = p.getBool("moving", false);
  p.end();
  valvePos = pos;
  valveKnown = known && !moving;     // Stromausfall waehrend der Fahrt -> unbekannt
  Log.printf("ESBE: gespeicherte Stellung %.0f %% (%s)\n", pos, valveKnown ? "bekannt" : "unbekannt");
}

float valvePosNow() {                // fuer Anzeige, auch waehrend der Fahrt
  if (ctrlCfg.drive == 1 || vphase != VP_RUN) return valvePos;
  uint16_t travel = ctrlCfg.travelSec;
  float delta = (millis() - vphaseMs) / 1000.0f / travel * 100.0f;
  float p = vStartPos + (valveDirWW ? delta : -delta);
  return constrain(p, 0.0f, 100.0f);
}

// startet eine Fahrt zum Ziel (0..100)
void valveBeginMove(int target) {
  CtrlCfg c = getCtrlCfg();
  if (c.drive != 0) return;                           // Ventilmodul faehrt selbst
  if (c.pinPwr < 0 || c.pinDir < 0) return;           // Relais nicht konfiguriert
  bool toEnd = (target <= 0 || target >= 100);
  target = constrain(target, 0, 100);

  if (!toEnd && !valveKnown) {                        // Mittelstellung nur von bekannter Lage aus
    vQueued = target;
    target = 0;
    toEnd = true;
    Log.println("ESBE: Stellung unbekannt -> erst Endlage Heizung anfahren");
  }

  float from = valveKnown ? valvePos : (target == 0 ? 100.0f : 0.0f);
  float dist = fabsf(target - from);
  uint32_t runMs = (uint32_t)(dist / 100.0f * c.travelSec * 1000.0f);
  if (toEnd) runMs += c.travelSec * 100UL;            // +10 % Ueberlauf in den Endschalter
  if (!valveKnown && toEnd) runMs = c.travelSec * 1100UL;

  valveTarget = target;
  valveDirWW = target > from || (target == 100);
  if (target == 0) valveDirWW = false;
  vStartPos = from;
  vRunMs = runMs;
  relayWrite(c.pinPwr, false);
  dirRelay(valveDirWW);
  vphase = VP_DIR;
  vphaseMs = millis();
  valveMoving = true;
  saveValveState(true);
  Log.printf("ESBE: fahre %s -> %d %% (%.0f s)\n", valveDirWW ? "Richtung WW" : "Richtung Heizung",
             target, runMs / 1000.0f);
}

void valveTick() {
  uint32_t ms = millis();
  switch (vphase) {
    case VP_IDLE: break;
    case VP_DIR:
      if (ms - vphaseMs >= 300) {
        relayWrite(ctrlCfg.pinPwr, true);
        vphase = VP_RUN; vphaseMs = ms;
      }
      break;
    case VP_RUN:
      if (ms - vphaseMs >= vRunMs) {
        relayWrite(ctrlCfg.pinPwr, false);
        vphase = VP_STOP; vphaseMs = ms;
      }
      break;
    case VP_STOP:
      if (ms - vphaseMs >= 300) {
        dirRelay(false);                              // Richtungsrelais in Ruhe
        valvePos = valveTarget;
        valveKnown = true;
        valveMoving = false;
        vphase = VP_IDLE;
        saveValveState(false);
        Log.printf("ESBE: Stellung %d %% erreicht\n", valveTarget);
        if (vQueued >= 0) { int q = vQueued; vQueued = -1; valveBeginMove(q); }
      }
      break;
  }
}

// ---------- Ventilmodul am Modbus (Arduino, Slave 2) ----------
extern volatile uint8_t wwState;          // weiter unten (Warmwasser-Teil)
ModbusMaster nodeV;
volatile bool     slaveOnline = false;
volatile bool     valveFailsafe = false;
volatile uint32_t slaveOkCount = 0, slaveErrCount = 0, slaveRetry = 0;
volatile int      slaveTarget = -1;       // vom ctrl-Task gesetzt
volatile bool     slaveRefReq = false;
static uint32_t   slaveLastOkMs = 0;
static uint16_t   slaveHb = 0;

bool valveSlaveActive() { return ctrlCfg.drive == 1 && ctrlCfg.wwMode == 0; }

void valveSlaveSetup() {
  nodeV.begin(ctrlCfg.slaveId, Serial1);
  nodeV.preTransmission(preTransmission);
  nodeV.postTransmission([]() { postTxDiscard(ctrlCfg.slaveId); });
  nodeV.idle([]() { vTaskDelay(1); });
}

// laeuft im Modbus-Task (alle 2 s)
void valveSlavePoll() {
  CtrlCfg c = getCtrlCfg();
  LiveData d = getLive();
  bool wwAct = (wwState == WW_MOVE || wwState == WW_HEAT || wwState == WW_COOL);
  auto t10 = [&](float v) -> uint16_t {
    return d.tempsValid ? (uint16_t)(int16_t)lroundf(v * 10.0f) : (uint16_t)0x8000;
  };

  // 1) Laufzeit, Notfall, Lebenszeichen, Temperaturen, Flags -> Reg 4..12
  nodeV.clearTransmitBuffer();
  nodeV.setTransmitBuffer(0, c.travelSec);
  nodeV.setTransmitBuffer(1, c.fsMin);
  nodeV.setTransmitBuffer(2, 0);                         // Notfall-Stellung = Heizung
  nodeV.setTransmitBuffer(3, ++slaveHb);
  nodeV.setTransmitBuffer(4, t10(d.vorlauf));
  nodeV.setTransmitBuffer(5, t10(d.ruecklauf));
  nodeV.setTransmitBuffer(6, t10(d.warmwasser));
  nodeV.setTransmitBuffer(7, (d.wpStateValid && d.wpOnOffRaw ? 1 : 0) | (wwAct ? 2 : 0));
  nodeV.setTransmitBuffer(8, c.wwSoll);
  clearSerial1Buffer();
  uint8_t r = nodeV.writeMultipleRegisters(4, 9);
  if (r != nodeV.ku8MBSuccess) { slaveRetry++; delay(20); clearSerial1Buffer(); r = nodeV.writeMultipleRegisters(4, 9); }
  bool ok = (r == nodeV.ku8MBSuccess);

  // 2) Soll-Stellung -> Reg 0
  int t = slaveTarget;
  if (ok && t >= 0 && c.enabled) {
    clearSerial1Buffer();
    r = nodeV.writeSingleRegister(0, (uint16_t)t);
    if (r != nodeV.ku8MBSuccess) { slaveRetry++; delay(20); clearSerial1Buffer(); r = nodeV.writeSingleRegister(0, (uint16_t)t); }
    ok = (r == nodeV.ku8MBSuccess);
  }
  // 3) Referenzfahrt -> Reg 1
  if (ok && slaveRefReq) {
    clearSerial1Buffer();
    r = nodeV.writeSingleRegister(1, 1);
    ok = (r == nodeV.ku8MBSuccess);
    if (ok) { slaveRefReq = false; Log.println("Ventilmodul: Referenzfahrt gesendet"); }
  }
  // 4) Rueckmeldung <- Reg 2..3
  if (ok) {
    clearSerial1Buffer();
    r = nodeV.readHoldingRegisters(2, 2);
    if (r != nodeV.ku8MBSuccess) { slaveRetry++; delay(20); clearSerial1Buffer(); r = nodeV.readHoldingRegisters(2, 2); }
    ok = (r == nodeV.ku8MBSuccess);
    if (ok) {
      uint16_t st = nodeV.getResponseBuffer(1);
      valvePos      = nodeV.getResponseBuffer(0) / 10.0f;
      valveMoving   = (st & 0x03) != 0;
      valveDirWW    = (st & 0x01) != 0;
      valveKnown    = (st & 0x10) == 0;
      valveFailsafe = (st & 0x20) != 0;
    }
  }

  if (ok) {
    if (!slaveOnline) Log.printf("Ventilmodul (ID %u) online\n", c.slaveId);
    slaveOnline = true;
    slaveLastOkMs = millis();
    slaveOkCount++;
  } else {
    slaveErrCount++;
    if (slaveErrCount % 30 == 1 || verbose)
      Log.printf("Ventilmodul (ID %u): Fehler %u (%s)\n", c.slaveId, r, mbErrText(r));
    if (slaveOnline && millis() - slaveLastOkMs > 15000) {
      slaveOnline = false;
      valveKnown = false;
      valveMoving = false;
      Log.println("Ventilmodul: keine Verbindung");
    }
  }
}

// ---------- Zeitplan ----------
static const char* WDAY_NAME[7] = {"Mo", "Di", "Mi", "Do", "Fr", "Sa", "So"};

const char* actName(uint8_t a) {
  switch (a) {
    case ACT_WW:     return "WW laden";
    case ACT_PUFFER: return "Puffer";
    case ACT_HEIZ:   return "Heizung";
    default:         return "-";
  }
}

int actToPos(uint8_t a, const CtrlCfg& c) {
  if (a == ACT_PUFFER) return c.bufPos;
  return 0;                                           // WW laden -> danach Heizung, Heizung -> 0
}

// Stellung laut Zeitplan = letztes vergangenes Ereignis
int scheduledBase(const CtrlCfg& c, uint8_t* actOut) {
  if (actOut) *actOut = ACT_NONE;
  if (!timeValid()) return actToPos(c.defAct, c);
  time_t tt = time(nullptr); struct tm tmv; localtime_r(&tt, &tmv);
  int today = (tmv.tm_wday + 6) % 7;                  // 0 = Montag
  int nowMin = tmv.tm_hour * 60 + tmv.tm_min;
  for (int back = 0; back <= 7; back++) {
    int d = (today - back + 7) % 7;
    int limit = back == 0 ? nowMin : 1439;
    int best = -1; uint8_t act = ACT_NONE;
    for (int s = 0; s < SCHED_SLOTS; s++) {
      const SchedEv& e = c.sched[d][s];
      if (e.act != ACT_NONE && e.min <= limit && (int)e.min > best) { best = e.min; act = e.act; }
    }
    if (best >= 0) { if (actOut) *actOut = act; return actToPos(act, c); }
  }
  return actToPos(c.defAct, c);
}

void nextEventText(const CtrlCfg& c, char* out, size_t n) {
  strlcpy(out, "-", n);
  if (!timeValid()) return;
  time_t tt = time(nullptr); struct tm tmv; localtime_r(&tt, &tmv);
  int today = (tmv.tm_wday + 6) % 7;
  int nowMin = tmv.tm_hour * 60 + tmv.tm_min;
  for (int fwd = 0; fwd <= 7; fwd++) {
    int d = (today + fwd) % 7;
    int best = 2000; uint8_t act = ACT_NONE;
    for (int s = 0; s < SCHED_SLOTS; s++) {
      const SchedEv& e = c.sched[d][s];
      if (e.act == ACT_NONE) continue;
      if (fwd == 0 && (int)e.min <= nowMin) continue;
      if (e.min < best) { best = e.min; act = e.act; }
    }
    if (act != ACT_NONE) {
      snprintf(out, n, "%s %02d:%02d %s", WDAY_NAME[d], best / 60, best % 60, actName(act));
      return;
    }
  }
}

// ---------- Warmwasser-Ladung ----------
volatile uint8_t wwState = WW_IDLE;
static uint32_t wwBeginMs = 0, wwStateMs = 0, wwCheckMs = 0, wwBackArrMs = 0;
static bool     wwManual = false;      // laufende Ladung kam von "Warmwasser jetzt"
static bool     wwSawOn = false;       // WP war waehrend der Ladung schon einmal an
static uint8_t  wwPowerTries = 0, wwModeTries = 0;
static uint32_t wwGuardMs = 0;
static float    wwSavedSoll = 0;
static int      wwSollNow = 0;         // waehrend der Ladung zuletzt geschriebener VL-Soll
static uint32_t wwRaiseMs = 0;         // letzte Anhebung des VL-Solls
static bool     wwSavedOn = true;
static uint16_t wwSavedMode = 2;
static bool     wwRecover = false;
static bool     wwHzSeen = false;      // WW-Modus der WP: Verdichter lief in dieser Ladung schon
static uint32_t wwHzOffMs = 0;         // ... und steht seit
char wwInfo[48] = "";

const char* wwStateName(uint8_t s) {
  switch (s) {
    case WW_IDLE: return "aus";
    case WW_MOVE: return "Ventil faehrt auf WW";
    case WW_HEAT: return "laedt";
    case WW_COOL: return "Nachlauf";
    case WW_BACK: return "zurueck auf Heizung";
    default:      return "?";
  }
}

void saveWwRun(bool active) {
  Preferences p;
  p.begin("wwrun", false);
  p.putBool("active", active);
  p.putFloat("soll", wwSavedSoll);
  p.putBool("on", wwSavedOn);
  p.putUShort("mode", wwSavedMode);
  p.end();
}

void loadWwRun() {                   // Neustart mitten in einer Ladung -> alte Werte wiederherstellen
  Preferences p;
  p.begin("wwrun", true);
  bool active = p.getBool("active", false);
  wwSavedSoll = p.getFloat("soll", 0);
  wwSavedOn = p.getBool("on", true);
  wwSavedMode = p.getUShort("mode", 2);
  p.end();
  if (active) {
    wwRecover = true;
    Log.printf("WW: Neustart waehrend Ladung -> stelle Sollwert %.0f wieder her\n", wwSavedSoll);
  }
}

void powerCmd(bool on, const char* src);     // weiter unten (Frequenz-Teil)

volatile int8_t hcWant = -1;           // Auto-WP: aktueller Heizbedarf (-1 unbekannt / Auto aus, 0 aus, 1 an)
LockCfg lockCfg;
volatile bool lockOn = false;          // Sperrzeit wirkt gerade (WP soll aus sein)
volatile bool nightOffOn = false;      // Nacht-Aus wirkt gerade (Nachtzeit und aussen warm genug)
volatile bool lockWin = false;         // Zeitfenster laeuft (auch wenn wegen Kaelte aufgehoben)
char lockInfo[64] = "aus";
extern HcCfg hcCfg;                    // weiter unten (Heizregelung)
extern volatile int sgState;

void wwRestorePump() {               // Sollwert / EIN-AUS / Modus wie vor der Ladung
  bool wpMode = ctrlCfg.wwMode == 1;
  if (wpMode) {                      // WW-Modus der WP: nur den Modus zurueck, den Sollwert nicht anfassen
    uint16_t m = (wwSavedMode == 2 || wwSavedMode == 4) ? wwSavedMode : 2;
    submitCmd(CMD_MODE, m, false);
  } else submitCmd(CMD_SET_SOLL, wwSavedSoll, false);
  if (!wwSavedOn) {
    if (hcCfg.autoEnable && hcWant == 1 && sgState != 1)
      Log.println("WW: Ende der Ladung - WP bleibt an (Auto-WP: Heizbedarf)");
    else powerCmd(false, "Ende WW-Ladung (WP war vorher aus)");
  }
  if (!wpMode && wwSavedMode != 2 && (wwSavedMode == 1 || wwSavedMode == 4)) submitCmd(CMD_MODE, wwSavedMode, false);
}

static uint8_t wwTarget = 50;          // Ziel der laufenden Ladung (normal oder PV)
volatile int   sgState = 2;            // SG-Ready 1..4 (2 = normal)
volatile bool  pvLoad = false;         // laufende Ladung kommt von SG3/4

// Verdichter am Laufen halten: liegt der VL knapp unter dem VL-Soll, Soll anheben.
// Die WP schaltet den Verdichter ab, sobald der VL ueber dem Soll liegt - bei der WW-Ladung
// passierte das kurz vor dem Ziel (Speicher noch nicht warm, VL schon am Soll).
// Rueckgabe: neuer Soll oder 0 (nichts geschrieben)
int wwKeepRunning(const LiveData& d, int cap, const char* what) {
  if (!d.tempsValid || !d.hzValid || !d.wpStateValid || !d.wpOnOffRaw || d.verdichterHz <= 0) return 0;
  if (wwSollNow >= cap || d.vorlauf < wwSollNow - 1.5f) return 0;
  if (wwRaiseMs && millis() - wwRaiseMs < 30000UL) return 0;
  int ns = min(cap, (int)lroundf(d.vorlauf) + 3);
  if (ns <= wwSollNow) return 0;
  Log.printf("WW: VL %.1f C knapp am Soll %d C -> %s %d C (Verdichter laeuft weiter)\n", d.vorlauf, wwSollNow, what, ns);
  submitCmd(CMD_SET_SOLL, ns, false);
  wwSollNow = ns;
  wwRaiseMs = millis();
  return ns;
}

void wwBegin(const char* why, uint8_t target, bool manual) {
  CtrlCfg c = getCtrlCfg();
  LiveData d = getLive();
  if (target < 30 || target > 70) target = c.wwSoll;
  if (sgState == 1) {
    Log.println("WW: SG1 (Sperre) aktiv -> keine Ladung");
    setCmdMsg("WW: SG-Sperre aktiv");
    return;
  }
  if (!d.tempsValid || !d.sollValid || !d.wpStateValid) {
    Log.println("WW: keine Modbus-Daten -> Ladung nicht moeglich");
    setCmdMsg("WW: keine Modbus-Daten");
    return;
  }
  if (d.warmwasser >= target) {
    Log.printf("WW: %.1f C >= Soll %u C -> keine Ladung noetig\n", d.warmwasser, target);
    setCmdMsg("WW hat schon %.0f C (Soll %u)", d.warmwasser, target);
    return;
  }
  if (c.wwMode == 0 && c.drive == 1 && !slaveOnline) {
    Log.println("WW: Ventilmodul nicht erreichbar");
    setCmdMsg("WW: Ventilmodul fehlt");
    return;
  }
  if (c.wwMode == 0 && c.drive == 0 && (c.pinPwr < 0 || c.pinDir < 0)) {
    Log.println("WW: Relais-Pins nicht konfiguriert");
    setCmdMsg("WW: Relais fehlen");
    return;
  }
  if (!d.wpOnOffRaw && !c.wpOnForWw && !manual) {
    Log.println("WW: WP ist aus -> Ladung uebersprungen");
    return;
  }
  wwSavedSoll = d.sollwert;
  wwSavedOn = d.wpOnOffRaw;
  wwSavedMode = d.wpModeRaw;
  saveWwRun(true);
  // Reihenfolge: Modus, dann EIN. Ventil am ESP: Speicher wird im Heizbetrieb ueber das Ventil geladen.
  // WW-Modus der WP: die WP schaltet ihr eigenes Ventil und heizt bis zu ihrem WW-Soll.
  uint16_t wantMode = c.wwMode == 1 ? 1 : 2;
  if (d.wpModeValid && d.wpModeRaw != wantMode) submitCmd(CMD_MODE, wantMode, false);
  if (!d.wpOnOffRaw) powerCmd(true, "WW-Ladung");
  wwManual = manual;
  wwSawOn = d.wpOnOffRaw;
  wwGuardMs = millis();
  wwPowerTries = d.wpOnOffRaw ? 0 : 1;
  wwModeTries = (d.wpModeValid && d.wpModeRaw != wantMode) ? 1 : 0;
  wwHzSeen = false; wwHzOffMs = 0;
  if (manual) manualPos = -1;                  // evtl. Handstellung aufheben, Ventil faehrt auf WW
  wwTarget = target;
  wwState = c.wwMode == 1 ? WW_HEAT : WW_MOVE;
  wwBeginMs = wwStateMs = wwCheckMs = millis();
  Log.printf("WW: Ladung gestartet (%s%s), Ist %.1f C, Ziel %u C, Heiz-Soll gemerkt %.0f C\n",
             why, c.wwMode == 1 ? ", WW-Modus der WP" : "", d.warmwasser, target, wwSavedSoll);
  // laeuft der Verdichter gerade (Heizen), waehrend der Ventilfahrt nicht am Heiz-Soll abschalten lassen
  wwSollNow = lroundf(d.sollwert); wwRaiseMs = 0;
  if (c.wwMode == 0) wwKeepRunning(d, min(75, target + c.boost), "Halte-Soll waehrend Ventilfahrt");
  setCmdMsg("WW-Ladung gestartet");
}

// nach "Warmwasser jetzt": Ventil auf Heizung und dort bis zum naechsten Zeitplan-Ereignis lassen
void wwManualDone() {
  if (!wwManual) return;
  wwManual = false;
  CtrlCfg c = getCtrlCfg();
  if (c.abOn || c.abLow || c.abFixOn) {                       // Puffer-Automatik an: die entscheidet ab jetzt
    manualPos = -1;
    Log.println("WW: Ventil wieder in Automatik (Puffer-Automatik)");
    return;
  }
  manualPos = 0;
  Log.println("WW: Ventil auf Heizung (bleibt bis zum naechsten Zeitplan-Ereignis)");
}

// Ziel erreicht, Verdichter laeuft und es soll weitergeheizt werden: Ventil sofort auf Heizung,
// VL-Soll erst auf den Heizwert setzen, wenn das kalte Heizungswasser angekommen ist.
// So bleibt der Verdichter an (beim sofortigen Umstellen lag der VL weit ueber dem neuen
// Soll -> WP stoppte und lief erst nach ihrer Sperrzeit wieder an).
void wwBackStart(const char* reason) {
  Log.printf("WW: Ladung beendet (%s) -> Ventil direkt auf Heizung, Verdichter laeuft weiter\n", reason);
  setCmdMsg("WW: %s, weiter mit Heizen", reason);
  wwManualDone();                                // "Warmwasser jetzt": Ventil bleibt danach auf Heizung
  wwState = WW_BACK; wwStateMs = millis(); wwBackArrMs = 0;
}

void wwFinish(const char* reason) {
  Log.printf("WW: Ladung beendet (%s)\n", reason);
  setCmdMsg("WW: %s", reason);
  wwRestorePump();
  CtrlCfg c = getCtrlCfg();
  if (wwState == WW_HEAT && c.coolSec > 0 && c.wwMode == 0) { wwState = WW_COOL; wwStateMs = millis(); }
  else { wwState = WW_IDLE; saveWwRun(false); wwManualDone(); }
}

// WW-Modus der WP: die WP laedt selbst. Fertig, wenn das WW das Ziel erreicht oder die WP den Verdichter
// im WW-Modus 3 min lang abschaltet (dann hat sie ihren eigenen WW-Soll erreicht).
void wwHeatWp(const CtrlCfg& c, const LiveData& d, uint32_t ms) {
  if (ms - wwCheckMs > 60000) {
    wwCheckMs = ms;
    if (d.wpStateValid && !d.wpOnOffRaw && wwSawOn && ms - wwStateMs > 120000) {
      wwSavedOn = false;                         // wurde extern ausgeschaltet -> aus lassen
      wwFinish("WP wurde ausgeschaltet");
      return;
    }
  }
  bool modeOk = d.wpModeValid && d.wpModeRaw == 1;
  if (modeOk && d.hzValid && d.verdichterHz > 0) { wwHzSeen = true; wwHzOffMs = 0; }
  else if (modeOk && d.hzValid && wwHzSeen && !wwHzOffMs) wwHzOffMs = ms;
  if (d.tempsValid && d.warmwasser >= wwTarget) {
    char r[40]; snprintf(r, sizeof(r), "Ziel erreicht (%.1f C)", d.warmwasser);
    wwFinish(r);
  } else if (wwHzOffMs && ms - wwHzOffMs > 180000UL) {
    char r[48]; snprintf(r, sizeof(r), "WP fertig (Verdichter aus, WW %.1f C)", d.warmwasser);
    wwFinish(r);
  }
}

void wwTick() {
  uint32_t ms = millis();
  CtrlCfg c = getCtrlCfg();
  LiveData d = getLive();

  if (wwRecover && d.sollValid) {              // nach Neustart: Pumpe zuruecksetzen
    wwRecover = false;
    wwRestorePump();
    saveWwRun(false);
  }

  if (reqWwStop) {
    reqWwStop = false;
    if (wwState == WW_MOVE || wwState == WW_HEAT) wwFinish("abgebrochen");
    else if (wwState == WW_COOL) { wwState = WW_IDLE; saveWwRun(false); }
    else if (wwState == WW_BACK) { wwRestorePump(); wwState = WW_IDLE; saveWwRun(false); }
  }
  if (reqWwStart) {
    reqWwStart = false;
    uint8_t t = reqWwTarget; reqWwTarget = 0;
    bool pv = reqWwPv; reqWwPv = false;
    bool man = reqWwManual; reqWwManual = false;
    if (man && wwState == WW_COOL) { wwState = WW_IDLE; saveWwRun(false); }   // Nachlauf abbrechen, neu laden
    if (!man && lockOn && !lockCfg.wwAllow) {
      static uint32_t lastLockLog = 0;
      if (!lastLockLog || millis() - lastLockLog > 600000UL) {
        lastLockLog = millis();
        Log.printf("WW: Sperrzeit (%s) -> %s-Ladung uebersprungen\n", lockInfo, pv ? "PV" : "Zeitplan");
      }
    } else if (wwState == WW_IDLE) {
      wwBegin(pv ? "PV-Ueberschuss (SG)" : man ? "Warmwasser jetzt" : "Zeitplan", t, man && !pv);
      pvLoad = pv && (wwState != WW_IDLE);
    } else if (man) setCmdMsg("WW-Ladung laeuft bereits");
    // WICHTIG: wwBegin() setzt die Startzeiten mit millis() -> "ms" ist jetzt aelter.
    // Ohne Neu-Lesen ergibt "ms - wwBeginMs" einen Unterlauf (riesige Zahl)
    // und die Ladung wurde sofort mit "Zeitlimit erreicht" beendet.
    ms = millis();
    d = getLive();
  }

  // WP einschalten / Modus Heizen absichern (falls ein Schreibbefehl nicht durchkam)
  if ((wwState == WW_MOVE || wwState == WW_HEAT) && d.wpStateValid && ms - wwGuardMs > 30000) {
    wwGuardMs = ms;
    if (d.wpOnOffRaw) wwSawOn = true;
    else if (!wwSawOn) {
      if (wwPowerTries >= 5) { wwFinish("WP laesst sich nicht einschalten"); return; }
      wwPowerTries++;
      Log.printf("WW: WP noch aus -> EIN erneut senden (%u/5)\n", wwPowerTries);
      powerCmd(true, "WW-Ladung (erneut)");
    }
    uint16_t wm = c.wwMode == 1 ? 1 : 2;
    if (d.wpModeValid && d.wpModeRaw != wm && wwModeTries < 5) {
      wwModeTries++;
      Log.printf("WW: Modus %s -> %s erneut senden (%u/5)\n", modeName(d.wpModeRaw), modeName(wm), wwModeTries);
      submitCmd(CMD_MODE, wm, false);
    }
    if (wwState == WW_MOVE) {
      Log.printf("WW: warte auf Ventil (%.0f %%%s)\n", (float)valvePos,
                 !valveKnown ? ", Stellung unbekannt - Referenzfahrt?" : valveMoving ? ", faehrt" : "");
    }
  }

  if ((wwState == WW_MOVE || wwState == WW_HEAT) && ms - wwBeginMs > c.maxMin * 60000UL) {
    wwFinish("Zeitlimit erreicht");
    return;
  }

  if (wwState == WW_MOVE && ms - wwStateMs > (c.travelSec * 2UL + 60UL) * 1000UL) {
    wwFinish("Ventil erreicht WW-Stellung nicht");
    return;
  }

  switch (wwState) {
    case WW_MOVE:
      if (!valveMoving && valveKnown && valvePos >= 99) {
        int soll = max(min(75, wwTarget + c.boost), wwSollNow);
        submitCmd(CMD_SET_SOLL, soll, false);
        wwSollNow = soll; wwRaiseMs = 0;
        Log.printf("WW: Ventil auf WW, VL-Soll -> %d C\n", soll);
        wwState = WW_HEAT; wwStateMs = wwCheckMs = ms;
      } else wwKeepRunning(d, min(75, wwTarget + c.boost), "Halte-Soll waehrend Ventilfahrt");
      break;
    case WW_HEAT: {
      if (c.wwMode == 1) { wwHeatWp(c, d, ms); break; }
      int base = min(75, wwTarget + c.boost);
      int cap = max(base, min(75, (int)c.wwVlMax));          // bis hierhin darf der VL-Soll mitwandern
      if (wwKeepRunning(d, cap, "VL-Soll angehoben auf")) wwCheckMs = ms;
      if (ms - wwCheckMs > 60000) {
        wwCheckMs = ms;
        if (d.sollValid && lroundf(d.sollwert) != wwSollNow) submitCmd(CMD_SET_SOLL, wwSollNow, false);
        if (d.wpStateValid && !d.wpOnOffRaw && wwSawOn && ms - wwStateMs > 120000) {
          wwSavedOn = false;                       // wurde extern ausgeschaltet -> aus lassen
          wwFinish("WP wurde ausgeschaltet");
          break;
        }
      }
      // Grenze erreicht und VL steht trotzdem am Soll: knapp unter dem Ziel beenden,
      // bevor der Verdichter abschaltet (dann kann direkt weitergeheizt werden)
      bool atCap = d.tempsValid && wwSollNow >= cap && d.vorlauf >= cap - 1.0f && d.warmwasser >= wwTarget - 2.0f;
      if (d.tempsValid && (d.warmwasser >= wwTarget || atCap)) {
        char r[40]; snprintf(r, sizeof(r), "%s (%.1f C)", d.warmwasser >= wwTarget ? "Ziel erreicht" : "fast erreicht, VL-Grenze", d.warmwasser);
        bool runs = d.wpStateValid && d.wpOnOffRaw && d.hzValid && d.verdichterHz > 0;
        bool heatWanted = hcCfg.autoEnable ? (hcWant == 1 && !lockOn) : wwSavedOn;   // danach soll geheizt werden
        if (!c.wwNoCont && runs && heatWanted && sgState != 1) wwBackStart(r);
        else wwFinish(r);
      }
      break;
    }
    case WW_BACK: {
      // warten, bis das Ventil weg von WW ist und das kalte Heizungswasser den VL abgekuehlt hat
      bool arrived = !valveMoving && valveKnown && valvePos < 99;
      if (arrived && !wwBackArrMs) wwBackArrMs = ms;
      float heatSoll = wwSavedSoll;
      bool cooled = d.tempsValid && d.vorlauf <= heatSoll + 1.0f;
      bool stopped = d.wpStateValid && (!d.wpOnOffRaw || (d.hzValid && d.verdichterHz == 0 && ms - wwStateMs > 60000));
      if ((wwBackArrMs && (cooled || ms - wwBackArrMs > 180000UL)) || stopped ||
          ms - wwStateMs > (c.travelSec + 300UL) * 1000UL) {
        wwRestorePump();                             // Heiz-Sollwert (und Modus) wie vor der Ladung
        wwState = WW_IDLE;
        saveWwRun(false);
        Log.printf("WW: zurueck auf Heizung, VL-Soll -> %.0f C (VL %.1f C)%s\n", heatSoll, d.vorlauf,
                   stopped ? ", Verdichter stand schon" : "");
      }
      break;
    }
    case WW_COOL:
      if (ms - wwStateMs > c.coolSec * 1000UL) {
        wwState = WW_IDLE;
        saveWwRun(false);
        if (wwManual) wwManualDone();
        else Log.println("WW: Nachlauf beendet -> Ventil nach Zeitplan");
      }
      break;
    default: break;
  }

  if (wwState == WW_IDLE) { wwInfo[0] = 0; pvLoad = false; }
  else snprintf(wwInfo, sizeof(wwInfo), "%s%s, WW %.1f/%u C", pvLoad ? "PV: " : "",
                wwStateName(wwState), d.warmwasser, wwTarget);
}

// ---------- WW-Ladung vorziehen ----------
// Heizt die WP gerade und steht der VL am Soll (Verdichter schaltet gleich ab), und steht in den
// naechsten wwPullMin Minuten eine WW-Ladung im Zeitplan: jetzt laden. Der Verdichter laeuft so
// durch, statt kurz vor der Ladung fuer wenige Minuten zu stoppen. Das Plan-Ereignis entfaellt dann.
static const uint32_t KEY_NONE = 0xFFFFFFFF;
static uint32_t wwPulledKey = KEY_NONE, wwPullTryKey = KEY_NONE;
static uint32_t wwPulledMs = 0, wwPullTryMs = 0, wwPullTickMs = 0;

void wwPullTick() {
  uint32_t ms = millis();
  if (ms - wwPullTickMs < 5000UL) return;
  wwPullTickMs = ms;
  if (wwPulledKey != KEY_NONE && ms - wwPulledMs > 3UL * 3600000UL) wwPulledKey = KEY_NONE;   // Sicherheit
  if (wwPullTryKey != KEY_NONE && ms - wwPullTryMs > 3UL * 3600000UL) wwPullTryKey = KEY_NONE;
  CtrlCfg c = getCtrlCfg();
  if (!c.wwPullMin || wwState != WW_IDLE || !timeValid()) return;
  if (sgState == 1 || (lockOn && !lockCfg.wwAllow)) return;
  LiveData d = getLive();
  if (!d.tempsValid || !d.sollValid || !d.wpStateValid || !d.hzValid) return;
  if (!d.wpOnOffRaw || d.verdichterHz <= 0) return;           // nur bei laufendem Verdichter
  if (d.wpModeValid && d.wpModeRaw != 2) return;               // nur im Heizbetrieb
  if (d.vorlauf < d.sollwert - 0.5f) return;                   // VL noch nicht am Soll -> laeuft noch
  if (d.warmwasser >= c.wwSoll) return;                        // Speicher ist noch warm genug
  time_t tt = time(nullptr); struct tm tmv; localtime_r(&tt, &tmv);
  int today = (tmv.tm_wday + 6) % 7, nowMin = tmv.tm_hour * 60 + tmv.tm_min;
  for (int fwd = 0; fwd <= 1; fwd++) {
    int day = (today + fwd) % 7;
    for (int s = 0; s < SCHED_SLOTS; s++) {
      const SchedEv& e = c.sched[day][s];
      if (e.act != ACT_WW) continue;
      int delta = fwd * 1440 + (int)e.min - nowMin;
      if (delta < 1 || delta > c.wwPullMin) continue;
      uint32_t key = day * 1440 + e.min;
      if (key == wwPullTryKey || key == wwPulledKey) return;  // fuer dieses Ereignis schon versucht
      wwPullTryKey = key; wwPullTryMs = ms;
      char why[40]; snprintf(why, sizeof(why), "vorgezogen von %02d:%02d", e.min / 60, e.min % 60);
      Log.printf("WW: VL %.1f am Soll %.0f, Verdichter wuerde stoppen -> Ladung von %02d:%02d jetzt (%d min frueher)\n",
                 d.vorlauf, d.sollwert, e.min / 60, e.min % 60, delta);
      wwBegin(why, c.wwSoll, false);
      if (wwState != WW_IDLE) { wwPulledKey = key; wwPulledMs = millis(); }
      return;
    }
  }
}

// ---------- gewuenschte Ventilstellung ----------
// ---------- Puffer-Automatik ----------
// Statt nach der WW-Ladung nur nach Uhrzeit auf Puffer zu gehen: Ventil bleibt auf Heizung, bis das
// WW nur noch wenig waermer ist als der Heiz-VL-Soll (bzw. darunter liegt). Dann bleibt es bis zur
// naechsten WW-Ladung auf Puffer (Merker, ueberlebt auch einen Neustart).
// Plan-Eintraege bleiben fest: "Puffer" (z. B. 02:00) = auf jeden Fall Puffer, "Heizung" = Heizung.
// WP aus / nicht im Heizbetrieb = Heizung (Merker bleibt, danach wieder Puffer).
extern volatile int hcTarget;
static int8_t abLatched = -1;                                   // -1 = noch nicht geladen
char abInfo[72] = "";
static void abInfoSet(const char* fmt, ...) {
  va_list ap; va_start(ap, fmt); vsnprintf(abInfo, sizeof(abInfo), fmt, ap); va_end(ap);
}
static void abLatch(bool on) {
  if (abLatched == (on ? 1 : 0)) return;
  abLatched = on ? 1 : 0;
  Preferences p; p.begin("ab", false); p.putBool("on", on); p.end();
}
float abHeatSoll(const LiveData& d) {                           // aktueller Heiz-VL-Soll
  if (hcCfg.enabled && hcTarget >= 0) return hcTarget;
  return d.sollValid ? d.sollwert : NAN;
}

int valveDesired(const CtrlCfg& c, const LiveData& d) {
  if (abLatched < 0) { Preferences p; p.begin("ab", true); abLatched = p.getBool("on", false) ? 1 : 0; p.end(); }
  if (wwState == WW_MOVE || wwState == WW_HEAT || wwState == WW_COOL) {   // neue Ladung -> Merker loeschen
    if (abLatched == 1) { abLatch(false); Log.println("Puffer-Automatik: WW-Ladung -> zurueck auf Heizung bis zur naechsten Schwelle"); }
    strlcpy(abInfo, "WW-Ladung", sizeof(abInfo));
    return 100;
  }
  bool autoOn = c.abOn || c.abLow || c.abFixOn;
  if (manualPos >= 0) {                                         // Handbetrieb (bis zum naechsten Ereignis)
    if (autoOn) strlcpy(abInfo, "pausiert: Ventil von Hand (Taste Automatik oder naechstes Ereignis)", sizeof(abInfo));
    return manualPos;
  }
  if (d.wpStateValid && !d.wpOnOffRaw) {                        // WP aus -> Heizung (WW-Speicher abkoppeln)
    if (autoOn) abInfoSet(abLatched == 1 ? "Heizung (WP aus), danach wieder Puffer" : "Heizung (WP aus)");
    return 0;
  }
  uint8_t act = ACT_NONE;
  int base = scheduledBase(c, &act);
  if (!autoOn) { abInfo[0] = 0; return base; }
  if (base != 0 || act == ACT_HEIZ) {                           // fester Plan-Eintrag gilt
    abInfoSet(base ? "Puffer laut Zeitplan" : "Heizung laut Zeitplan");
    return base;
  }
  // nach "WW laden" (oder ohne Plan-Eintrag): Automatik, nur bei laufender Heizung
  bool heating = d.wpStateValid && d.wpOnOffRaw && (!d.wpModeValid || d.wpModeRaw == 2) && wwState == WW_IDLE;
  if (!heating) {
    abInfoSet(wwState == WW_BACK ? "Heizung (WW-Ladung gerade fertig)" : abLatched == 1 ? "Heizung (kein Heizbetrieb), danach wieder Puffer" : "Heizung (kein Heizbetrieb)");
    return 0;
  }
  if (abLatched == 1) { if (!strstr(abInfo, "bis zur")) abInfoSet("Puffer bis zur naechsten WW-Ladung"); return c.bufPos; }
  float vs = abHeatSoll(d);
  bool relOn = c.abOn || c.abLow;
  if (!d.tempsValid || (!c.abFixOn && isnan(vs))) { abInfoSet("Heizung (keine Messwerte)"); return 0; }
  float diff = isnan(vs) ? NAN : d.warmwasser - vs;
  bool fixHit = c.abFixOn && d.warmwasser <= c.abFixT;
  bool relHit = !isnan(diff) && ((c.abOn && diff <= c.abK) || (c.abLow && diff < 0));
  if (fixHit || relHit) {
    abLatch(true);
    if (fixHit) {
      abInfoSet("Puffer bis zur naechsten WW-Ladung (WW %.1f <= %u C)", d.warmwasser, c.abFixT);
      Log.printf("Puffer-Automatik: WW %.1f C <= %u C -> Puffer bis zur naechsten WW-Ladung\n", d.warmwasser, c.abFixT);
    } else {
      abInfoSet("Puffer bis zur naechsten WW-Ladung (WW %.1f, VL-Soll %.0f)", d.warmwasser, vs);
      Log.printf("Puffer-Automatik: WW %.1f C nur noch %+.1f K zum VL-Soll %.0f C -> Puffer bis zur naechsten WW-Ladung\n", d.warmwasser, diff, vs);
    }
    return c.bufPos;
  }
  if (c.abFixOn && (!relOn || isnan(diff))) abInfoSet("Heizung: WW %.1f C, Puffer ab %u C", d.warmwasser, c.abFixT);
  else if (c.abFixOn) abInfoSet("Heizung: WW %.1f C (Puffer ab %u C oder %+.1f K <= %d K)", d.warmwasser, c.abFixT, diff, c.abOn ? (int)c.abK : 0);
  else abInfoSet("Heizung: WW %.1f C, VL-Soll %.0f C (%+.1f K > %d K)", d.warmwasser, vs, diff, c.abOn ? (int)c.abK : 0);
  return 0;
}

static uint32_t lastEventKey = 0xFFFFFFFF;

void scheduleTick() {
  if (!timeValid()) return;
  time_t tt = time(nullptr); struct tm tmv; localtime_r(&tt, &tmv);
  int today = (tmv.tm_wday + 6) % 7;
  int nowMin = tmv.tm_hour * 60 + tmv.tm_min;
  uint32_t key = today * 1440 + nowMin;
  if (key == lastEventKey) return;
  bool first = (lastEventKey == 0xFFFFFFFF);
  lastEventKey = key;
  if (first) return;                                            // beim Start nichts nachholen
  CtrlCfg c = getCtrlCfg();
  for (int s = 0; s < SCHED_SLOTS; s++) {
    const SchedEv& e = c.sched[today][s];
    if (e.act == ACT_NONE || (int)e.min != nowMin) continue;
    Log.printf("Zeitplan %s %02d:%02d: %s\n", WDAY_NAME[today], e.min / 60, e.min % 60, actName(e.act));
    manualPos = -1;                                             // Handbetrieb endet
    if (e.act == ACT_WW) {
      uint32_t k = today * 1440 + e.min;
      if (k == wwPullTryKey) wwPullTryKey = KEY_NONE;
      if (k == wwPulledKey) { wwPulledKey = KEY_NONE; Log.println("WW: Ladung wurde schon vorgezogen -> entfaellt"); continue; }
      reqWwStart = true;
    }
  }
}

void sgTick();                  // MQTT/SG-Teil weiter unten
void hcTick();
void vlTick();
void fcTick();
void pvTick();
void offTick();

void ctrlTask(void*) {
  loadValveState();
  loadWwRun();
  uint32_t settleUntil = millis() + 20000;                      // erst Modbus-Werte abwarten
  for (;;) {
    if ((int32_t)(millis() - settleUntil) > 0) { sgTick(); hcTick(); vlTick(); fcTick(); pvTick(); offTick(); }
    if (!ctrlCfg.enabled) {                                     // Steuerung deaktiviert
      if (vphase != VP_IDLE) {
        relayWrite(ctrlCfg.pinPwr, false); dirRelay(false);
        vphase = VP_IDLE; vQueued = -1; valveMoving = false; valveKnown = false;
        saveValveState(false);
        Log.println("ESBE: Steuerung deaktiviert -> Fahrt abgebrochen");
      }
      if (wwState == WW_MOVE || wwState == WW_HEAT) wwFinish("Steuerung deaktiviert");
      if (wwState == WW_COOL) { wwState = WW_IDLE; saveWwRun(false); }
      if (wwState == WW_BACK) { wwRestorePump(); wwState = WW_IDLE; saveWwRun(false); }
      reqWwStart = false; reqWwStop = false; reqValve = -2;
      if (wwRecover) wwTick();                                  // nur Sollwert-Wiederherstellung
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }
    if (reqValve != -2) {
      int r = reqValve; reqValve = -2;
      manualPos = r;                                            // -1 = Automatik
      Log.printf("ESBE: %s\n", r < 0 ? "Automatik" : "Handbetrieb");
    }
    scheduleTick();
    if ((int32_t)(millis() - settleUntil) > 0) wwPullTick();
    wwTick();
    if (ctrlCfg.wwMode == 1) {                                  // WW-Modus der WP: kein Ventil am ESP
    } else if (ctrlCfg.drive == 1) {                            // Ventilmodul faehrt selbst
      if ((int32_t)(millis() - settleUntil) > 0) {
        CtrlCfg c = getCtrlCfg();
        LiveData d = getLive();
        int want = valveDesired(c, d);
        if (want != slaveTarget) Log.printf("Ventilmodul: Soll %d %%\n", want);
        slaveTarget = want;
        valveTarget = want;
      }
    } else {
      if (reqValveRef) { reqValveRef = false; if (vphase == VP_IDLE) { valveKnown = false; Log.println("ESBE: Referenzfahrt"); } }
      valveTick();
      if (vphase == VP_IDLE && vQueued < 0 && (int32_t)(millis() - settleUntil) > 0) {
        CtrlCfg c = getCtrlCfg();
        LiveData d = getLive();
        int want = valveDesired(c, d);
        if (!valveKnown || fabsf(valvePos - want) >= 2.0f) valveBeginMove(want);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Kurztext fuer Display-Kopfzeile
void valveShortText(char* out, size_t n) {
  if (!ctrlCfg.enabled || ctrlCfg.wwMode == 1) { out[0] = 0; return; }
  if (ctrlCfg.drive == 0 && (ctrlCfg.pinPwr < 0 || ctrlCfg.pinDir < 0)) { out[0] = 0; return; }
  if (ctrlCfg.drive == 1 && !slaveOnline) { strlcpy(out, "V !MB", n); return; }
  if (valveMoving)      snprintf(out, n, "V%s%.0f%%", valveDirWW ? ">" : "<", valvePosNow());
  else if (!valveKnown) strlcpy(out, "V ?", n);
  else if (valvePos <= 1)  strlcpy(out, "V HZ", n);
  else if (valvePos >= 99) strlcpy(out, "V WW", n);
  else snprintf(out, n, "V %.0f%%", valvePos);
}

// =====================================================
// MQTT (Venus OS), SG-Ready, PV-Warmwasser, Heizstab, VL per MQTT
// =====================================================
//
//  Status:   <base>/state    JSON, retained, alle <interval> s
//            <base>/status   "online" / "offline" (Last Will)
//  Befehle:  <base>/cmd/vl      Zahl     Heiz-VL-Sollwert (ratenbegrenzt)
//            <base>/cmd/sg      1..4     SG-Ready (muss regelmaessig wiederholt werden)
//            <base>/cmd/wwload  0 / 1 / Zahl   WW-Ladung stoppen / starten / mit Ziel-Temp starten
//            <base>/cmd/power   0 / 1    WP aus / ein
//            <base>/cmd/mode    heizen / ww / kuehlen
//            <base>/cmd/valve   heizung / puffer / ww / auto
//            <base>/cmd/heater  0 / Minuten   Heizstab von Hand (z. B. 60 = 60 min)
//            <base>/cmd/console Text     beliebiger Konsolen-Befehl (wie im Web)

MqttCfg mqttCfg;
PvCfg   pvCfg;
esp_mqtt_client_handle_t mqttClient = nullptr;
volatile bool     mqttConnected = false;
volatile uint32_t mqttRxCount = 0, mqttTxCount = 0;
char mqttLastCmd[64] = "";

// (keine Templates: der Arduino-Vorverarbeiter erzeugt dafuer fehlerhafte Prototypen)
bool loadBlobRaw(const char* ns, void* v, size_t sz, uint8_t version) {
  Preferences p;
  p.begin(ns, true);
  size_t have = p.getBytesLength("cfg");
  uint8_t buf[256];
  bool ok = have > 0 && have <= sz && sz <= sizeof(buf);     // kuerzer = aeltere Version, neue Felder bleiben Standard
  if (ok) ok = p.getBytes("cfg", buf, have) == have && buf[0] == version;
  p.end();
  if (ok) memcpy(v, buf, have);
  return ok;
}
void saveBlobRaw(const char* ns, const void* v, size_t sz) {
  Preferences p;
  p.begin(ns, false);
  p.putBytes("cfg", v, sz);
  p.end();
}
void loadMqttCfg() { loadBlobRaw("mqtt", &mqttCfg, sizeof(mqttCfg), MqttCfg().version); }
void saveMqttCfg() { saveBlobRaw("mqtt", &mqttCfg, sizeof(mqttCfg)); }
void loadPvCfg() {
  loadBlobRaw("pv", &pvCfg, sizeof(pvCfg), PvCfg().version);
  Preferences p; p.begin("pv", true);
  gridPhases = p.getUChar("phases", 3);
  pElFactor = p.getUShort("pfac", 100);
  elSrc = p.getUChar("elsrc", 0);
  if (p.getBytesLength("eltopic") == sizeof(elTopic)) p.getBytes("eltopic", elTopic, sizeof(elTopic));
  elTopic[sizeof(elTopic) - 1] = 0;
  p.end();
}
void savePvCfg() {
  saveBlobRaw("pv", &pvCfg, sizeof(pvCfg));
  Preferences p; p.begin("pv", false);
  p.putUChar("phases", gridPhases);
  p.putUShort("pfac", pElFactor);
  p.putUChar("elsrc", elSrc);
  p.putBytes("eltopic", elTopic, sizeof(elTopic));
  p.end();
}

// ---------- SG-Ready ----------
volatile uint32_t sgLastMs = 0;
static bool  sgBlocked = false, sgSavedOn = false;

const char* sgName(int s) {
  switch (s) {
    case 1: return "SG1 Sperre";
    case 2: return "SG2 normal";
    case 3: return "SG3 PV";
    case 4: return "SG4 PV max";
    default: return "?";
  }
}

void sgSet(int s, const char* src) {
  if (s < 1 || s > 4) return;
  if (!pvCfg.show) return;                       // PV/Energiemanagement ausgeschaltet -> SG-Ready ignorieren
  sgLastMs = millis();
  if (s != sgState) Log.printf("SG-Ready: %s -> %s (%s)\n", sgName(sgState), sgName(s), src);
  sgState = s;
}

// laeuft im ctrl-Task
void sgTick() {
  PvCfg p = pvCfg;
  LiveData d = getLive();
  if (!p.show && sgState != 2) sgState = 2;
  if (sgState != 2 && p.sgTimeoutMin > 0 && millis() - sgLastMs > p.sgTimeoutMin * 60000UL) {
    Log.printf("SG-Ready: %u min keine Auffrischung -> SG2\n", p.sgTimeoutMin);
    sgState = 2;
  }

  // SG1: WP sperren (max. so lange, wie SG1 aufgefrischt wird)
  if (sgState == 1 && p.sg1Block && !sgBlocked && d.wpStateValid) {
    sgBlocked = true;
    sgSavedOn = d.wpOnOffRaw;
    if (d.wpOnOffRaw) powerCmd(false, "SG1-Sperre");
  }
  if (sgState != 1 && sgBlocked) {
    sgBlocked = false;
    if (sgSavedOn) powerCmd(true, "SG1 beendet");
  }

  // SG3/4: Warmwasser mit PV-Ueberschuss per WP laden
  bool pv = (sgState == 3 || sgState == 4);
  CtrlCfg c = getCtrlCfg();
  if (pv && p.pvWwEnable && c.enabled && wwState == WW_IDLE && !reqWwStart && d.tempsValid &&
      d.warmwasser < p.pvWwSoll - p.pvWwHyst) {
    reqWwTarget = p.pvWwSoll;
    reqWwPv = true;
    reqWwStart = true;
  }
  // PV vorbei -> PV-Ladung abbrechen
  if (!pv && pvLoad && (wwState == WW_MOVE || wwState == WW_HEAT)) {
    Log.println("SG: kein PV-Ueberschuss mehr -> PV-Ladung beenden");
    reqWwStop = true;
  }
}

// ---------- Heizstab (Shelly) ----------
volatile bool     heaterOn = false;          // zuletzt erfolgreich gesetzter Zustand
volatile uint32_t heaterManualUntil = 0;     // millis()-Zeitpunkt, 0 = kein Handbetrieb
static uint32_t   heaterOnSince = 0;
static bool       heaterLockout = false;     // Max-Laufzeit erreicht, bis SG wechselt
char heaterInfo[48] = "aus";

bool shellySwitch(bool on) {
  PvCfg p = pvCfg;
  if (!p.shellyIp[0] || WiFi.status() != WL_CONNECTED) return false;
  char url[160];
  // Selbstabschaltung nach 180 s, falls der ESP haengt -> wird alle 60 s erneuert
  if (p.heaterGen == 1)
    snprintf(url, sizeof(url), "http://%s/relay/0?turn=%s%s", p.shellyIp, on ? "on" : "off", on ? "&timer=180" : "");
  else
    snprintf(url, sizeof(url), "http://%s/rpc/Switch.Set?id=0&on=%s%s", p.shellyIp, on ? "true" : "false",
             on ? "&toggle_after=180" : "");
  HTTPClient http;
  http.setConnectTimeout(2000);
  http.setTimeout(2500);
  bool ok = false;
  if (http.begin(url)) {
    int code = http.GET();
    ok = (code == 200);
    if (!ok) Log.printf("Heizstab: Shelly antwortet %d\n", code);
    http.end();
  }
  return ok;
}

void heaterTask(void*) {
  uint32_t lastSend = 0;
  int lastSg = sgState;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    PvCfg p = pvCfg;
    LiveData d = getLive();
    uint32_t ms = millis();
    if (sgState != lastSg) { lastSg = sgState; heaterLockout = false; }

    bool manual = heaterManualUntil && (int32_t)(heaterManualUntil - ms) > 0;
    if (heaterManualUntil && !manual) heaterManualUntil = 0;

    bool want = false;
    const char* why = "aus";
    if (!p.heaterEnable || !p.shellyIp[0]) { why = "deaktiviert"; }
    else if (!d.tempsValid) { why = "keine WW-Temperatur"; }
    else if (heaterLockout) { why = "Max-Laufzeit erreicht"; }
    else {
      float lim = p.heaterSoll;
      bool below = heaterOn ? (d.warmwasser < lim) : (d.warmwasser < lim - 3.0f);
      bool wpDone = (wwState == WW_IDLE) && (d.warmwasser >= p.pvWwSoll - 1.0f || !getCtrlCfg().enabled);
      if (!below) why = "Ziel erreicht";
      else if (manual) { want = true; why = "Hand"; }
      else if (sgState == 4) { want = true; why = "SG4"; }
      else if (sgState == 3 && wpDone) { want = true; why = "SG3 nach WP"; }
      else if (sgState == 3) why = "wartet auf WP";
    }
    if (want && heaterOn && heaterOnSince && ms - heaterOnSince > p.heaterMaxMin * 60000UL) {
      want = false; heaterLockout = true; why = "Max-Laufzeit erreicht";
      Log.println("Heizstab: Max-Laufzeit erreicht -> aus");
    }

    bool changed = (want != heaterOn);
    if (changed || (want && ms - lastSend > 60000UL)) {
      if (shellySwitch(want)) {
        if (changed) {
          Log.printf("Heizstab %s (%s, WW %.1f C)\n", want ? "EIN" : "AUS", why, d.warmwasser);
          heaterOnSince = want ? ms : 0;
        }
        heaterOn = want;
        lastSend = ms;
      } else if (changed) {
        Log.println("Heizstab: Shelly nicht erreichbar");
        why = "Shelly nicht erreichbar";
        if (!want) heaterOn = false;       // beim Ausschalten greift spaetestens der 180-s-Timer
      }
    }
    snprintf(heaterInfo, sizeof(heaterInfo), "%s (%s)", heaterOn ? "EIN" : "aus", why);
  }
}

// ---------- VL-Sollwert von MQTT oder Heizkurve ----------
// Grundsaetze (schont das EEPROM der WP):
//  - geschrieben wird nur, wenn sich der gelesene Ist-Sollwert um mindestens die Totzone unterscheidet
//  - nur bei eingeschalteter WP; ist sie aus, wird der Wert beim Einschalten geschrieben
//  - hoechstens alle minSec Sekunden (ausser direkt nach dem Einschalten)
volatile float    vlWantF = NAN;          // gewuenschter VL (Kommazahl), NAN = keine Vorgabe
volatile float    vlDeadband = 1.0f;
volatile bool     vlOneShot = true;       // MQTT: einmal setzen; Heizkurve: dauerhaft vorgeben
volatile uint16_t vlPendSec = 300;
volatile uint8_t  vlPendMin = 20, vlPendMax = 55;
const char* volatile vlPendSrc = "MQTT";
static uint32_t   vlLastWriteMs = 0;
static bool       vlWritten = false;
static int        vlLastWpOn = -1;
static uint32_t   vlWpOnSince = 0;
volatile uint16_t vlWritesToday = 0, hzWritesToday = 0, dfCountToday = 0;
static int        writesDay = -1;
char vlInfo[48] = "keine Vorgabe";

void countWrite(volatile uint16_t& cnt) {
  if (timeValid()) {
    time_t tt = time(nullptr); struct tm tmv; localtime_r(&tt, &tmv);
    if (tmv.tm_yday != writesDay) { writesDay = tmv.tm_yday; vlWritesToday = 0; hzWritesToday = 0; dfCountToday = 0; }
  }
  cnt++;
}

void vlRequestF(float v, float deadband, bool oneShot, uint16_t minSec, uint8_t lo, uint8_t hi, const char* src) {
  vlPendSec = minSec; vlPendMin = lo; vlPendMax = hi; vlPendSrc = src;
  vlDeadband = deadband; vlOneShot = oneShot;
  vlWantF = v;
}
void vlRequest(int v, uint16_t minSec, uint8_t lo, uint8_t hi, const char* src) {
  vlRequestF(v, 0.5f, true, minSec, lo, hi, src);
}
void vlClear(const char* src) {
  if (!isnan(vlWantF) && !strcmp(vlPendSrc, src)) { vlWantF = NAN; strcpy(vlInfo, "keine Vorgabe"); }
}

void vlTick() {                       // im ctrl-Task
  LiveData d = getLive();
  int wpOn = (d.wpStateValid && d.wpOnOffRaw) ? 1 : 0;
  if (d.wpStateValid && wpOn != vlLastWpOn) { if (wpOn) vlWpOnSince = millis(); vlLastWpOn = wpOn; }

  float vf = vlWantF;
  if (isnan(vf)) return;
  vf = constrain(vf, (float)vlPendMin, (float)vlPendMax);
  int v = lroundf(vf);
  const char* src = vlPendSrc;

  if (wwState == WW_MOVE || wwState == WW_HEAT || wwState == WW_BACK) {   // waehrend WW: nach der Ladung uebernehmen
    if (fabsf(wwSavedSoll - vf) >= vlDeadband && lroundf(wwSavedSoll) != v) {
      wwSavedSoll = v;
      Log.printf("VL (%s): %d C wird nach der WW-Ladung gesetzt\n", src, v);
    }
    snprintf(vlInfo, sizeof(vlInfo), "%d C nach WW-Ladung", v);
    if (vlOneShot) vlWantF = NAN;
    return;
  }
  if (wwState == WW_COOL) {                             // Nachlauf: Ventil noch auf WW -> abwarten
    snprintf(vlInfo, sizeof(vlInfo), "%d C nach WW-Nachlauf", v);
    return;
  }
  if (!d.sollValid || !d.wpStateValid) return;
  if (fabsf(vf - d.sollwert) < vlDeadband || lroundf(d.sollwert) == v) {   // passt schon
    snprintf(vlInfo, sizeof(vlInfo), "passt (%d C)", (int)lroundf(d.sollwert));
    if (vlOneShot) vlWantF = NAN;
    return;
  }
  if (!wpOn) { snprintf(vlInfo, sizeof(vlInfo), "%d C beim Einschalten der WP", v); return; }
  if (vlWritten && millis() - vlLastWriteMs < 30000UL) return;          // Ruecklesen abwarten
  // direkt nach dem Einschalten sofort (einmal), sonst Mindestabstand
  bool justOn = millis() - vlWpOnSince < 300000UL && (!vlWritten || (int32_t)(vlLastWriteMs - vlWpOnSince) < 0);
  if (!justOn && vlWritten && millis() - vlLastWriteMs < vlPendSec * 1000UL) {
    snprintf(vlInfo, sizeof(vlInfo), "%d C in %lu s", v,
             (unsigned long)((vlPendSec * 1000UL - (millis() - vlLastWriteMs)) / 1000));
    return;
  }
  submitCmd(CMD_SET_SOLL, v, false);
  countWrite(vlWritesToday);
  Log.printf("VL (%s): Sollwert %d -> %d C (Schreibvorgang %u heute)\n", src, (int)lroundf(d.sollwert), v, vlWritesToday);
  snprintf(vlInfo, sizeof(vlInfo), "geschrieben: %d C", v);
  vlLastWriteMs = millis();
  vlWritten = true;
  if (vlOneShot) vlWantF = NAN;
}

// =====================================================
// Heizregelung: Raumfuehler (DHT22), Heizkurve, Raumeinfluss,
// Nachtabsenkung, automatisches WP EIN/AUS
// =====================================================
//
//  VL-Soll = Heizkurve(Aussentemp. gedaempft)
//          + Nachtverschiebung (nur nachts)
//          + Raumeinfluss  = Kp * (Raum-Soll - Raum-Ist) + I-Anteil   (begrenzt auf +-roomMax)
//  begrenzt auf vlMin..vlMax, geschrieben hoechstens alle writeSec Sekunden.

HcCfg hcCfg;
FcCfg fcCfg;
void loadFcCfg() { loadBlobRaw("fc", &fcCfg, sizeof(fcCfg), FcCfg().version); }
void saveFcCfg() { saveBlobRaw("fc", &fcCfg, sizeof(fcCfg)); }

void loadHcCfg() { loadBlobRaw("hc", &hcCfg, sizeof(hcCfg), HcCfg().version); }
void saveHcCfg() { saveBlobRaw("hc", &hcCfg, sizeof(hcCfg)); }
void loadLockCfg() { loadBlobRaw("lock", &lockCfg, sizeof(lockCfg), LockCfg().version); }
void saveLockCfg() { saveBlobRaw("lock", &lockCfg, sizeof(lockCfg)); }

// ---------- Raumfuehler DHT22 / DHT11 ----------
volatile float   rtIst = NAN, rhIst = NAN;      // Raumtemperatur / Feuchte
volatile uint32_t rtOkMs = 0;                   // letzter gueltiger Messwert
volatile uint32_t dhtOk = 0, dhtErr = 0;
static portMUX_TYPE dhtMux = portMUX_INITIALIZER_UNLOCKED;

static bool dhtWait(int pin, int level, uint32_t timeoutUs) {
  uint32_t t0 = micros();
  while (digitalRead(pin) != level) if (micros() - t0 > timeoutUs) return false;
  return true;
}

// liest 40 Bit; nur ~5 ms mit gesperrten Interrupts auf diesem Kern
bool dhtRead(int pin, uint8_t type, float& t, float& h) {
  uint8_t d[5] = {0, 0, 0, 0, 0};
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(type == 11 ? 20 : 2);                    // Startsignal
  bool ok = true;
  portENTER_CRITICAL(&dhtMux);
  digitalWrite(pin, HIGH);
  delayMicroseconds(30);
  pinMode(pin, INPUT_PULLUP);
  if (!dhtWait(pin, LOW, 120) || !dhtWait(pin, HIGH, 120) || !dhtWait(pin, LOW, 120)) ok = false;
  for (int i = 0; ok && i < 40; i++) {
    if (!dhtWait(pin, HIGH, 100)) { ok = false; break; }
    uint32_t t0 = micros();
    if (!dhtWait(pin, LOW, 120)) { ok = false; break; }
    if (micros() - t0 > 45) d[i / 8] |= (uint8_t)(1 << (7 - (i % 8)));
  }
  portEXIT_CRITICAL(&dhtMux);
  pinMode(pin, INPUT_PULLUP);
  if (!ok) return false;
  if ((uint8_t)(d[0] + d[1] + d[2] + d[3]) != d[4]) return false;
  if (type == 11) { h = d[0] + d[1] / 10.0f; t = d[2] + (d[3] & 0x7F) / 10.0f; if (d[3] & 0x80) t = -t; }
  else {
    h = ((d[0] << 8) | d[1]) / 10.0f;
    t = (((d[2] & 0x7F) << 8) | d[3]) / 10.0f;
    if (d[2] & 0x80) t = -t;
  }
  return h >= 0 && h <= 100 && t > -30 && t < 60;
}

// Sensirion SHT40/41/45 an I2C (Wire, GIGA: SDA D20 / SCL D21), Adresse 0x44
static uint8_t shtCrc(const uint8_t* d) {
  uint8_t c = 0xFF;
  for (int i = 0; i < 2; i++) { c ^= d[i]; for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x31) : (uint8_t)(c << 1); }
  return c;
}
bool shtRead(float& t, float& h) {
  static bool wireOn = false;
  if (!wireOn) { Wire.begin(); Wire.setClock(100000); wireOn = true; }
  Wire.beginTransmission(0x44);
  Wire.write(0xFD);                                // Messung, hohe Genauigkeit
  if (Wire.endTransmission() != 0) return false;
  delay(12);
  if (Wire.requestFrom(0x44, 6) != 6) return false;
  uint8_t d[6];
  for (int i = 0; i < 6; i++) d[i] = Wire.read();
  if (shtCrc(d) != d[2] || shtCrc(d + 3) != d[5]) return false;
  t = -45.0f + 175.0f * ((d[0] << 8) | d[1]) / 65535.0f;
  h = constrain(-6.0f + 125.0f * ((d[3] << 8) | d[4]) / 65535.0f, 0.0f, 100.0f);
  return t > -30 && t < 60;
}

void dhtTask(void*) {
  vTaskDelay(pdMS_TO_TICKS(3000));
  float lastT = NAN;
  for (;;) {
    HcCfg c = hcCfg;
    bool sht = c.sensType == 40;
    bool clash = !sht && c.sensPin >= 0 && ctrlCfg.drive == 0 && (c.sensPin == ctrlCfg.pinPwr || c.sensPin == ctrlCfg.pinDir);
    if (clash) { static bool warned = false; if (!warned) { Log.println("Raumfuehler: Pin belegt durch Ventil-Relais"); warned = true; } }
    if ((sht || c.sensPin >= 0) && !clash) {
      float t, h;
      bool ok = false;
      for (int k = 0; k < 3 && !ok; k++) {         // bis zu 3 Versuche
        ok = sht ? shtRead(t, h) : dhtRead(c.sensPin, c.sensType, t, h);
        if (!ok) vTaskDelay(pdMS_TO_TICKS(2100)); // DHT braucht >= 2 s Pause
      }
      if (ok && !isnan(lastT) && fabsf(t - lastT) > 3.0f) ok = false;   // Ausreisser verwerfen
      if (ok) {
        t += c.sensOffset10 / 10.0f;
        rtIst = isnan(rtIst) ? t : rtIst * 0.7f + t * 0.3f;                // leicht glaetten
        rhIst = h;
        rtOkMs = millis();
        lastT = t - c.sensOffset10 / 10.0f;
        dhtOk++;
      } else {
        dhtErr++;
        if (dhtErr % 20 == 1) {
          if (sht) Log.println("Raumfuehler: Lesefehler (SHT4x an I2C 0x44)");
          else Log.printf("Raumfuehler: Lesefehler (Pin %d, DHT%u)\n", c.sensPin, c.sensType);
        }
        if (millis() - rtOkMs > 600000UL) { rtIst = NAN; rhIst = NAN; lastT = NAN; }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

bool rtValid() { return !isnan(rtIst) && millis() - rtOkMs < 600000UL; }

// ---------- Regler-Zustand ----------
volatile float hcTaF = NAN;          // gedaempfte Aussentemperatur
volatile float hcCurve = NAN;        // VL laut Heizkurve
volatile float hcRoomCorr = 0;       // Raumeinfluss in K
volatile float hcI = 0;              // I-Anteil
volatile int   hcTarget = -1;        // berechneter VL-Soll (ganzzahlig)
volatile float hcTargetF = NAN;      // berechneter VL-Soll (Kommazahl)
volatile bool  hcNight = false;
volatile float hcNShift = 0;         // aktuelle VL-Verschiebung nachts (K)
volatile float hcRtSoll = 21.0f;     // aktuell gueltiger Raum-Soll
volatile int8_t hcDemand = -1;       // Auto-WP: -1 unbekannt, 0 aus, 1 an
char hcAutoInfo[64] = "aus";
static uint32_t hcLastSwitchMs = 0;
static int      hcLastWpOn = -1;
static uint32_t hcLastTick = 0;
static uint32_t hcLastCmdMs = 0;
static bool     hcReached = true;
volatile bool   kaminOn = false;     // Kamin brennt -> Auto-WP-Grenzen tiefer
static uint32_t kaminSinceMs = 0;
static time_t   kaminSinceT = 0;

void kaminSet(bool on, const char* src) {
  if (on == kaminOn) return;
  kaminOn = on;
  kaminSinceMs = millis();
  kaminSinceT = timeValid() ? time(nullptr) : 0;
  Preferences p; p.begin("kamin", false); p.putBool("on", on); p.end();
  Log.printf("Kamin-Modus %s (%s)\n", on ? "EIN" : "AUS", src);
}
void kaminLoad() {
  Preferences p; p.begin("kamin", true); bool on = p.getBool("on", false); p.end();
  if (on) { kaminOn = true; kaminSinceMs = millis(); Log.println("Kamin-Modus nach Neustart weiter aktiv"); }
}

// Kurve: Punkte nach Aussentemperatur absteigend (15, 10, ... -15), linear dazwischen
float curveInterp(const int8_t* ta, const uint8_t* val, float x) {
  if (x >= ta[0]) return val[0];
  for (int i = 0; i < HC_POINTS - 1; i++) {
    float t1 = ta[i], t2 = ta[i + 1];
    if (x <= t1 && x >= t2) {
      if (t1 == t2) return val[i];
      return val[i] + (val[i + 1] - val[i]) * (t1 - x) / (t1 - t2);
    }
  }
  return val[HC_POINTS - 1];
}

float curveVl(const HcCfg& c, float ta) { return curveInterp(c.curveTa, c.curveVl, ta); }

// wie curveInterp, fuer vorzeichenbehaftete Werte (Nacht-Verschiebung in K)
float curveInterpS(const int8_t* ta, const int8_t* val, float x) {
  if (x >= ta[0]) return val[0];
  for (int i = 0; i < HC_POINTS - 1; i++) {
    float t1 = ta[i], t2 = ta[i + 1];
    if (x <= t1 && x >= t2) {
      if (t1 == t2) return val[i];
      return val[i] + (val[i + 1] - val[i]) * (t1 - x) / (t1 - t2);
    }
  }
  return val[HC_POINTS - 1];
}

// VL-Verschiebung nachts: fest oder nach Aussentemperatur
float nightShiftOf(const HcCfg& c, float ta) {
  if (c.nCurveOn && !isnan(ta)) return curveInterpS(c.nTa, c.nDv, ta);
  return c.nightVlShift;
}

// Nachtbeginn am Abend von Tag d (0 = Montag): Fr + Sa evtl. Wochenend-Zeit
uint16_t nightStartOf(const HcCfg& c, int d) { return (c.nightWeOn && (d == 4 || d == 5)) ? c.nightWeStart : c.nightStart; }
// Nachtende am Morgen von Tag d: Sa + So evtl. Wochenend-Zeit
uint16_t nightEndOf(const HcCfg& c, int d)   { return (c.nightWeOn && (d == 5 || d == 6)) ? c.nightWeEnd : c.nightEnd; }

bool isNight(const HcCfg& c) {
  if (!c.nightOn || !timeValid()) return false;
  time_t tt = time(nullptr); struct tm tmv; localtime_r(&tt, &tmv);
  int wd = (tmv.tm_wday + 6) % 7, m = tmv.tm_hour * 60 + tmv.tm_min;
  // die Nacht "gehoert" zum Abend des Vortags bzw. des heutigen Tages
  for (int k = -1; k <= 0; k++) {
    int a = (wd + 7 + k) % 7, n = (a + 1) % 7;
    int st = nightStartOf(c, a), en = nightEndOf(c, n);
    if (st == en) continue;
    int s, e;
    if (st > en)        { s = st; e = 1440 + en; }          // ueber Mitternacht (normal)
    else if (st >= 720) { s = st; e = en; }                 // nachmittags, gleicher Tag
    else                { s = 1440 + st; e = 1440 + en; }   // erst nach Mitternacht
    s += k * 1440; e += k * 1440;
    if (m >= s && m < e) return true;
  }
  return false;
}

// aktiver Raum-Soll um delta aendern (Display / Web / MQTT)
void hcAdjustSoll(float delta) {
  bool night = isNight(hcCfg);
  uint16_t& v = night ? hcCfg.rtNight10 : hcCfg.rtDay10;
  int nv = constrain((int)v + (int)lroundf(delta * 10), 100, 280);
  v = nv;
  hcRtSoll = nv / 10.0f;
  saveHcCfg();
  Log.printf("Raum-Soll %s -> %.1f C\n", night ? "Nacht" : "Tag", nv / 10.0f);
}

void hcSetSoll(float soll) {
  bool night = isNight(hcCfg);
  int nv = constrain((int)lroundf(soll * 10), 100, 280);
  (night ? hcCfg.rtNight10 : hcCfg.rtDay10) = nv;
  hcRtSoll = nv / 10.0f;
  saveHcCfg();
  Log.printf("Raum-Soll %s -> %.1f C\n", night ? "Nacht" : "Tag", nv / 10.0f);
}

void preStartWrites();         // Frequenzkurve, weiter unten

// ---------- Sperrzeiten ----------
static const char* WD_SHORT[7] = {"Mo", "Di", "Mi", "Do", "Fr", "Sa", "So"};
static bool lockSavedOn = false;        // Auto-WP aus: WP war beim Beginn der Sperre an

// berechnet lockWin / lockOn / lockInfo (alle 10 s aus hcTick)
void lockEval() {
  const LockCfg L = lockCfg;
  if (!L.enabled || !timeValid()) {
    lockWin = lockOn = false;
    strlcpy(lockInfo, L.enabled ? "Uhrzeit fehlt" : "aus", sizeof(lockInfo));
    return;
  }
  time_t tt = time(nullptr); struct tm tmv; localtime_r(&tt, &tmv);
  int wd = (tmv.tm_wday + 6) % 7, m = tmv.tm_hour * 60 + tmv.tm_min;
  bool win = false; int endMin = 0;
  auto W = [&](int d, int i, int& f, int& t) { f = i ? L.from2[d] : L.from[d]; t = i ? L.to2[d] : L.to[d]; };
  int yd = (wd + 6) % 7;
  for (int i = 0; i < 2 && !win; i++) {                // je Tag zwei Zeitraeume
    int f, t;
    W(wd, i, f, t);
    if (f != t && ((f < t && m >= f && m < t) || (f > t && m >= f))) { win = true; endMin = t; }
    W(yd, i, f, t);                                    // vom Vortag ueber Mitternacht
    if (!win && f != t && f > t && m < t) { win = true; endMin = t; }
  }
  static bool coldRelease = false;
  if (L.coldOn && !isnan(hcTaF)) {
    if (hcTaF < L.coldTa) coldRelease = true;
    else if (hcTaF > L.coldTa + 1.0f) coldRelease = false;
  } else coldRelease = false;
  lockWin = win;
  lockOn = win && !coldRelease;
  if (win && coldRelease) snprintf(lockInfo, sizeof(lockInfo), "aufgehoben: aussen %.1f < %d C", (float)hcTaF, L.coldTa);
  else if (win) snprintf(lockInfo, sizeof(lockInfo), "aktiv bis %02d:%02d", endMin / 60, endMin % 60);
  else {
    strlcpy(lockInfo, "keine Zeiten eingetragen", sizeof(lockInfo));
    for (int k = 0; k < 8; k++) {
      int d = (wd + k) % 7, bf = 9999, bt = 0;
      for (int i = 0; i < 2; i++) {
        int f, t; W(d, i, f, t);
        if (f == t || (k == 0 && f <= m)) continue;
        if (f < bf) { bf = f; bt = t; }
      }
      if (bf == 9999) continue;
      snprintf(lockInfo, sizeof(lockInfo), "naechste: %s %02d:%02d-%02d:%02d", k == 0 ? "heute" : WD_SHORT[d],
               bf / 60, bf % 60, bt / 60, bt % 60);
      break;
    }
  }
}

// laeuft im ctrl-Task (alle 10 s)
void hcTick() {
  uint32_t ms = millis();
  if (hcLastTick && ms - hcLastTick < 10000UL) return;
  float dtH = hcLastTick ? (ms - hcLastTick) / 3600000.0f : 0;
  hcLastTick = ms;

  HcCfg c = hcCfg;
  LiveData d = getLive();

  // Aussentemperatur daempfen (Tiefpass, Zeitkonstante taTau Minuten).
  // Waehrend des Abtauens und 15 min danach wird der Wert gehalten: der Fuehler sitzt am
  // Verdampfer und wird vom Abtauen (Heissgas, Luefter steht) um 1..2 K verfaelscht.
  static uint32_t taHoldUntil = 0;
  static bool taHoldMsg = false;
  if (d.extValid && (d.state & 16)) taHoldUntil = ms + 900000UL;
  bool taHold = taHoldUntil && (int32_t)(taHoldUntil - ms) > 0;
  if (taHold != taHoldMsg) {
    taHoldMsg = taHold;
    Log.printf(taHold ? "Aussentemperatur: Abtauen - gedaempfter Wert %.1f C wird gehalten\n"
                      : "Aussentemperatur: wieder normal gedaempft (%.1f C)\n", hcTaF);
  }
  if (d.extValid && !(taHold && !isnan(hcTaF))) {
    if (isnan(hcTaF)) hcTaF = d.ta;
    else {
      float a = c.taTau ? dtH * 60.0f / c.taTau : 1.0f;
      hcTaF = hcTaF + constrain(a, 0.0f, 1.0f) * (d.ta - hcTaF);
    }
  }

  hcNight = isNight(c);
  hcRtSoll = (hcNight ? c.rtNight10 : c.rtDay10) / 10.0f;
  bool rtOk = rtValid();
  hcNShift = nightShiftOf(c, hcTaF);

  // ---- Nacht-Aus: in der Nachtzeit WP ganz aus, wenn es aussen warm genug ist ----
  static bool lastNOff = false;
  if (hcNight && c.nOffOn && !isnan(hcTaF)) {
    if (hcTaF > c.nOffTa) nightOffOn = true;
    else if (hcTaF < c.nOffTa - 1.0f) nightOffOn = false;
  } else nightOffOn = false;
  if (nightOffOn != lastNOff)
    Log.printf("Nacht-Aus %s (aussen ged. %.1f C, Grenze %d C)\n", nightOffOn ? "beginnt" : "endet", (float)hcTaF, c.nOffTa);
  lastNOff = nightOffOn;

  // ---- Sperrzeit (und Nacht-Aus) ----
  static bool lastLock = false, lastOff = false, lockNeedOff = false;
  lockEval();
  bool offNow = lockOn || nightOffOn;
  if (!c.autoEnable && d.wpStateValid) {     // ohne Auto-WP schalten Sperre / Nacht-Aus selbst
    bool wwRun = (wwState == WW_MOVE || wwState == WW_HEAT || wwState == WW_COOL || wwState == WW_BACK);
    if (offNow && !lastOff) { lockSavedOn = wwRun ? wwSavedOn : d.wpOnOffRaw; lockNeedOff = true; }
    if (!offNow && lastOff) {
      lockNeedOff = false;
      if (lockSavedOn) { lockSavedOn = false; powerCmd(true, lastLock ? "Ende Sperrzeit" : "Ende Nacht-Aus"); }
    }
    if (offNow && lockNeedOff && !wwRun) {   // eine laufende WW-Ladung erst fertig laden lassen
      lockNeedOff = false;
      if (d.wpOnOffRaw) powerCmd(false, lockOn ? "Sperrzeit" : "Nacht-Aus");
    }
  } else lockNeedOff = false;
  if (lockOn != lastLock) Log.printf("Sperrzeit %s (%s)\n", lockOn ? "beginnt" : "endet", lockInfo);
  lastLock = lockOn;
  lastOff = offNow;

  // ---- VL-Sollwert ----
  if (!isnan(hcTaF)) {
    float vl = curveVl(c, hcTaF);
    hcCurve = vl;
    if (hcNight) vl += hcNShift;
    float corr = 0;
    if (rtOk && c.roomOn && c.roomK10 > 0) {
      float e = hcRtSoll - rtIst;
      bool wwNow = (wwState == WW_MOVE || wwState == WW_HEAT || wwState == WW_COOL || wwState == WW_BACK);
      bool wpRuns = d.hzValid && d.verdichterHz > 0 && !wwNow;   // waehrend WW heizt die WP nicht den Raum
      if (c.roomKi10 > 0 && wpRuns) {             // I-Anteil nur bei laufendem Verdichter
        hcI = constrain(hcI + c.roomKi10 / 10.0f * e * dtH, -(float)c.roomMax, (float)c.roomMax);
      }
      if (c.roomKi10 == 0) hcI = 0;
      corr = constrain(c.roomK10 / 10.0f * e + hcI, -(float)c.roomMax, (float)c.roomMax);
    } else hcI = 0;
    hcRoomCorr = corr;
    vl += corr;
    vl = constrain(vl, (float)c.vlMin, (float)c.vlMax);
    hcTargetF = vl;
    hcTarget = lroundf(vl);
    if (c.enabled) vlRequestF(vl, c.deadband10 / 10.0f, false, c.writeSec, c.vlMin, c.vlMax, "Heizkurve");
  }
  if (!c.enabled) vlClear("Heizkurve");

  // ---- Automatisch WP EIN/AUS ----
  if (!c.autoEnable || !d.wpStateValid) {
    snprintf(hcAutoInfo, sizeof(hcAutoInfo), "%s", c.autoEnable ? "warte auf WP-Status" : "aus");
    hcDemand = -1; hcWant = -1;
    return;
  }
  int wpOn = d.wpOnOffRaw ? 1 : 0;
  if (wpOn != hcLastWpOn) { hcLastWpOn = wpOn; hcLastSwitchMs = ms; }   // auch Handschaltung zaehlt

  // Kamin: nach Hoechstdauer automatisch aus
  if (kaminOn && c.kaminMaxH && ms - kaminSinceMs > c.kaminMaxH * 3600000UL) kaminSet(false, "Zeit abgelaufen");
  float taOn = c.taOn, taOff = c.taOff, rtRef = hcRtSoll;
  if (kaminOn) { taOn -= c.kaminTa; taOff -= c.kaminTa; rtRef -= c.kaminRt10 / 10.0f; }

  static bool taDem = false, rtDem = false;
  bool haveTa = !isnan(hcTaF);
  if (haveTa) {
    if (hcTaF < taOn) taDem = true;
    else if (hcTaF > taOff) taDem = false;
  }
  if (rtOk) {
    if (rtIst < rtRef - c.rtOnBelow10 / 10.0f) rtDem = true;
    else if (rtIst > rtRef + c.rtOffAbove10 / 10.0f) rtDem = false;
  }
  int demand = -1;
  const char* why = "";
  switch (c.autoMode) {
    case 1: if (haveTa) { demand = taDem; why = kaminOn ? "Kamin/Aussen" : "Aussen"; } break;
    case 2: if (rtOk) { demand = rtDem; why = kaminOn ? "Kamin/Raum" : "Raum"; } break;
    case 3: if (haveTa && rtOk) { demand = taDem && rtDem; why = kaminOn ? "Kamin/Aussen+Raum" : "Aussen+Raum"; }
            else if (haveTa) { demand = taDem; why = "Aussen (Raumfuehler fehlt)"; } break;
  }
  static char lockWhy[40];
  if (lockOn) {                              // Sperrzeit: WP aus, unabhaengig von der Temperatur
    demand = 0;
    snprintf(lockWhy, sizeof(lockWhy), "Sperrzeit %s", lockInfo + 6);   // "aktiv bis 04:00" -> "bis 04:00"
    why = lockWhy;
  } else if (nightOffOn) {                   // Nacht-Aus: nachts warm genug
    demand = 0;
    snprintf(lockWhy, sizeof(lockWhy), "Nacht-Aus (aussen > %d C)", c.nOffTa);
    why = lockWhy;
  }
  if (demand < 0) { snprintf(hcAutoInfo, sizeof(hcAutoInfo), "keine Messwerte"); hcWant = -1; return; }
  hcWant = demand;

  // Schaltungen durch WW-Ladung oder SG1-Sperre sind keine Hand-Schaltung:
  // danach neu entscheiden (sonst blieb die WP nach einer WW-Ladung aus)
  static bool lastBlock = false;
  bool wwBusy = (wwState == WW_MOVE || wwState == WW_HEAT || wwState == WW_COOL || wwState == WW_BACK);
  bool block = wwBusy || sgState == 1 || lockOn || nightOffOn;   // auch nach Sperrzeit / Nacht-Aus neu entscheiden
  if (lastBlock && !block) { hcDemand = -1; hcReached = false; }
  lastBlock = block;

  uint32_t since = (ms - hcLastSwitchMs) / 60000UL;
  // Warte-Zustaende veraendern hcDemand NICHT - sonst wird das spaetere Schalten
  // faelschlich fuer eine Hand-Schaltung gehalten
  if (demand == wpOn) {
    if (demand == hcDemand) hcReached = true;
    hcDemand = demand;
    snprintf(hcAutoInfo, sizeof(hcAutoInfo), "%s: WP %s (passt)", why, wpOn ? "an" : "aus");
  } else if (wwBusy) {
    snprintf(hcAutoInfo, sizeof(hcAutoInfo), "%s: WW-Ladung laeuft, danach WP %s", why, demand ? "an" : "aus");
  } else if (demand && sgState == 1) {
    snprintf(hcAutoInfo, sizeof(hcAutoInfo), "%s: an gewuenscht, SG1-Sperre", why);
  } else if (demand && since < c.minOffMin) {
    snprintf(hcAutoInfo, sizeof(hcAutoInfo), "%s: an in %lu min (Mindestpause)", why, (unsigned long)(c.minOffMin - since));
  } else if (!demand && since < c.minOnMin) {
    snprintf(hcAutoInfo, sizeof(hcAutoInfo), "%s: aus in %lu min (Mindestlaufzeit)", why, (unsigned long)(c.minOnMin - since));
  } else if (demand != hcDemand || (!hcReached && ms - hcLastCmdMs > 120000UL)) {
    // nur bei geaenderter Anforderung schalten; hat die WP schon einmal wie gewuenscht
    // geschaltet und steht jetzt anders, war das Hand-Schaltung -> bleibt bis zur naechsten Aenderung
    if (demand) preStartWrites();          // passende Werte vor dem Einschalten
    powerCmd(demand, "Auto-WP");
    hcLastCmdMs = ms;
    hcReached = false;
    Log.printf("Auto-WP (%s): WP %s (Aussen ged. %.1f, Raum %.1f/%.1f)\n", why, demand ? "EIN" : "AUS",
               haveTa ? hcTaF : NAN, rtOk ? rtIst : NAN, hcRtSoll);
    hcLastSwitchMs = ms;
    hcDemand = demand;
    snprintf(hcAutoInfo, sizeof(hcAutoInfo), "%s: schalte %s", why, demand ? "EIN" : "AUS");
  } else {
    snprintf(hcAutoInfo, sizeof(hcAutoInfo), "%s: Hand-Schaltung, warte auf Aenderung", why);
  }
}

// ---------- Abtau-Auswertung ----------
// Jede Abtauung landet mit den Bedingungen kurz davor in /log/abtau.csv: Aussentemperatur, Verdichter-Hz,
// Verdampfer und Coil (jeweils ueber ca. 4 min gemittelt) und die Verdichter-Laufzeit seit der letzten
// Abtauung. Damit laesst sich pruefen, ob weniger Hz bei gleicher Aussentemperatur seltener abtauen laesst.
struct DfEvent {
  uint32_t t = 0;                    // Beginn (UTC)
  uint16_t dur = 0;                  // s
  int16_t  ta = INT16_MIN, ev = INT16_MIN, coil = INT16_MIN;   // 0,1 C
  int16_t  runMin = -1;              // Verdichter-Laufzeit seit der letzten Abtauung (-1 = unbekannt, nach Neustart)
  uint8_t  hz = 0, maxHz = 0, cap = 0, ww = 0;
};
static DfEvent  dfCur, dfQ[4];
static volatile uint8_t dfQh = 0, dfQt = 0;   // ctrl-Task schreibt, loop() speichert
static float    dfHzF = NAN, dfEvF = NAN, dfCoilF = NAN, dfTaF = NAN;
static uint32_t dfRunMs = 0, dfStartMs = 0;
static bool     dfRunValid = false;
volatile int    fbCap = -1;                    // Frost-Bremse: aktive Grenze (-1 = keine)
static bool     fbChanged = false;             // neue Grenze -> ohne Mindestabstand schreiben
volatile int    fcCurve = -1;                  // Kurvenwert ohne Frost-Bremse

// im ctrl-Task alle 5 s: Mittelwerte und Laufzeit fuer die Auswertung
void dfSample(const LiveData& d, uint32_t dtMs) {
  if (!d.extValid || (d.state & 16)) return;            // waehrend des Abtauens nichts mitteln
  if (dtMs > 60000UL) dtMs = 60000UL;
  auto ema = [](float& f, float v) { f = isnan(f) ? v : f + (v - f) * 0.02f; };   // 5-s-Takt -> ca. 4 min
  ema(dfTaF, d.ta);
  if (d.hzValid && d.verdichterHz > 0) {
    dfRunMs += dtMs;
    ema(dfHzF, (float)d.verdichterHz); ema(dfEvF, d.evap); ema(dfCoilF, d.coil);
  }
}

// gesammelte Abtauungen auf die SD-Karte / in den Speicher (aus loop())
void dfFlush() {
  if (dfQt == dfQh || !sdOk) return;
  File f = SD_MMC.open("/log/abtau.csv", FILE_APPEND);
  if (!f) return;
  if (f.size() == 0)
    f.print("Beginn;Dauer_s;Aussen_C;Verdichter_Hz;Verdampfer_C;Coil_C;Laufzeit_seit_letzter_min;Max_Hz;Frost_Bremse_Hz;Betrieb\n");
  auto T = [](char* o, size_t n, int16_t v) { if (v == INT16_MIN) o[0] = 0; else fmtTenth(o, n, v); };
  while (dfQt != dfQh) {
    DfEvent e = dfQ[dfQt];
    dfQt = (dfQt + 1) & 3;
    if (e.t < 1600000000UL) continue;                  // ohne gueltige Uhrzeit nicht speichern
    char ts[20], ta[10], ev[10], co[10], rm[8], cp[6], row[128];
    time_t tt = e.t; struct tm tmv; localtime_r(&tt, &tmv);
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M", &tmv);
    T(ta, sizeof(ta), e.ta); T(ev, sizeof(ev), e.ev); T(co, sizeof(co), e.coil);
    if (e.runMin >= 0) snprintf(rm, sizeof(rm), "%d", e.runMin); else rm[0] = 0;
    if (e.cap) snprintf(cp, sizeof(cp), "%u", e.cap); else cp[0] = 0;
    int len = snprintf(row, sizeof(row), "%s;%u;%s;%u;%s;%s;%s;%u;%s;%s\n", ts, e.dur, ta, e.hz, ev, co, rm, e.maxHz, cp,
                       e.ww ? "WW" : "Heizen");
    f.write((const uint8_t*)row, len);
  }
  f.close();
}

// ---------- Verdichter-Frequenzkurve (Max Hz P45) ----------
// gleiche Schreibregeln wie beim VL: nur bei Abweichung >= Totzone, nur bei laufender WP,
// hoechstens alle minIntMin Minuten; sofort beim Einschalten und beim Wechsel Heizen <-> WW.
volatile int  fcTarget = -1;
volatile bool fcWw = false;
char fcInfo[56] = "aus (manuell)";
static uint32_t fcLastWriteMs = 0, fcWpOnSince = 0, fcLastTick = 0;
static bool     fcWritten = false, fcSwitchPending = false;
static int      fcLastWpOn = -1, fcLastWw = -1;

bool fcWwActive(const LiveData& d) {
  return wwState == WW_MOVE || wwState == WW_HEAT || (d.wpModeValid && d.wpModeRaw == 1);
}

int fcCompute(bool ww) {
  if (isnan(hcTaF)) return -1;
  FcCfg c = fcCfg;
  float hz = ww ? curveInterp(c.ta, c.hzWw, hcTaF) : curveInterp(c.ta, c.hzHeat, hcTaF);
  if (!ww && c.nightCap && hcNight) hz = min(hz, (float)c.nightCap);
  return constrain((int)lroundf(hz), (int)MAX_HZ_MIN, (int)MAX_HZ_MAX);
}

void fcWrite(int t, int cur, const char* why) {
  submitCmd(CMD_MAXHZ, t, false);
  countWrite(hzWritesToday);
  Log.printf("Frequenzkurve (%s): Max Hz %d -> %d (Schreibvorgang %u heute)\n", why, cur, t, hzWritesToday);
  snprintf(fcInfo, sizeof(fcInfo), "geschrieben: %d Hz", t);
  fcLastWriteMs = millis(); fcWritten = true; fcSwitchPending = false; fbChanged = false;
}

// ---------- Abtau-Boost ----------
// Erkennt das Abtau-Bit (Statusregister 4354, Bit 4) und hebt dafuer Max Hz an.
// Danach wird der vorherige Wert (bzw. der Kurvenwert) wieder geschrieben.
static bool     dfActive = false, dfRestorePend = false;
static int      dfPrevHz = -1;
static uint32_t dfEndMs = 0;
char dfInfo[48] = "";

// true = Abtau-Boost hat die Frequenz gerade im Griff (Kurve pausiert)
bool dfTick(const FcCfg& c, const LiveData& d, uint32_t ms) {
  bool df = d.extValid && (d.state & 16);
  if (df && !dfActive) {
    dfActive = true; dfRestorePend = false;
    countWrite(dfCountToday);          // setzt den Tageszaehler mit zurueck
    {                                  // Bedingungen kurz vor dem Abtauen festhalten
      auto T = [](float v) -> int16_t { return isnan(v) ? INT16_MIN : (int16_t)lroundf(v * 10.0f); };
      dfCur = DfEvent();
      dfCur.t = nowUtc(); dfStartMs = ms;
      dfCur.ta = T(dfTaF); dfCur.ev = T(dfEvF); dfCur.coil = T(dfCoilF);
      dfCur.hz = isnan(dfHzF) ? 0 : (uint8_t)constrain(lroundf(dfHzF), 0L, 255L);
      dfCur.maxHz = (uint8_t)min((uint16_t)255, d.maxVerdichterHz);
      dfCur.ww = fcWwActive(d) ? 1 : 0;
      dfCur.cap = (fbCap > 0 && !dfCur.ww) ? (uint8_t)fbCap : 0;
      dfCur.runMin = dfRunValid ? (int16_t)min(dfRunMs / 60000UL, 32000UL) : -1;
    }
    if (c.dfOn && d.maxHzValid && d.maxVerdichterHz < c.dfHz) {
      dfPrevHz = d.maxVerdichterHz;
      Preferences p; p.begin("fc", false); p.putInt("dfPrev", dfPrevHz); p.end();   // fuer Neustart waehrend des Abtauens
      fcWrite(c.dfHz, d.maxVerdichterHz, "Abtauen beginnt");
      snprintf(dfInfo, sizeof(dfInfo), "Abtauen: %u Hz (vorher %d)", c.dfHz, dfPrevHz);
    } else {
      dfPrevHz = -1;
      snprintf(dfInfo, sizeof(dfInfo), c.dfOn ? "Abtauen: Grenze schon %u Hz" : "Abtauen (Boost aus)", d.maxVerdichterHz);
    }
    Log.printf("Abtauen erkannt (Max Hz %u)\n", d.maxVerdichterHz);
  }
  if (!df && dfActive) {
    dfActive = false; dfEndMs = ms;
    dfCur.dur = (uint16_t)min((ms - dfStartMs) / 1000UL, 65535UL);
    { uint8_t nh = (dfQh + 1) & 3; if (nh != dfQt) { dfQ[dfQh] = dfCur; dfQh = nh; } }
    Log.printf("Abtauung: %u s, vorher %u Hz, Verdampfer %.1f C, nach %d min Verdichterlaufzeit\n",
               dfCur.dur, dfCur.hz, dfCur.ev == INT16_MIN ? NAN : dfCur.ev / 10.0f, (int)dfCur.runMin);
    dfRunMs = 0; dfRunValid = true;
    dfRestorePend = (dfPrevHz >= 0);
    Log.println("Abtauen beendet");
    if (!dfRestorePend) dfInfo[0] = 0;
  }
  if (dfActive) return dfPrevHz >= 0;
  if (dfRestorePend && ms - dfEndMs >= c.dfBackSec * 1000UL) {
    dfRestorePend = false;
    int back = (c.enabled && fcTarget > 0) ? fcTarget : dfPrevHz;   // Kurve aktiv -> Kurvenwert
    if (d.maxHzValid && back > 0 && back != (int)d.maxVerdichterHz) fcWrite(back, d.maxVerdichterHz, "nach dem Abtauen");
    dfPrevHz = -1; dfInfo[0] = 0;
    Preferences p; p.begin("fc", false); p.remove("dfPrev"); p.end();
    return true;
  }
  if (dfRestorePend) { snprintf(dfInfo, sizeof(dfInfo), "zurueck auf %d Hz in %lu s", dfPrevHz,
                                (unsigned long)((c.dfBackSec * 1000UL - (ms - dfEndMs)) / 1000 + 1)); return true; }
  return false;
}

// nach Neustart: war der Boost noch aktiv, Wert zuruecksetzen
void dfLoad() {
  Preferences p; p.begin("fc", true); int v = p.getInt("dfPrev", -1); p.end();
  if (v >= (int)MAX_HZ_MIN && v <= (int)MAX_HZ_MAX) { dfPrevHz = v; dfRestorePend = true; dfEndMs = millis(); }
}

// ---------- Frost-Bremse ----------
// Im Bereich, in dem der Verdampfer am staerksten vereist (etwa +2..+8 C aussen, feuchte Luft), wird die
// Frequenzgrenze beim Heizen schrittweise gesenkt, solange der Verdampfer (oder die Coil) kaelter als der
// Grenzwert ist und der Vorlauf sein Soll trotzdem erreicht. Langsamerer Verdichter -> waermerer Verdampfer
// -> weniger Eis. Wird der Verdampfer 2 K waermer als der Grenzwert oder faellt der Vorlauf mehr als 3 K
// unter Soll, geht die Grenze schrittweise wieder hoch. Nach Verdichterstart und Abtauen: 5 min nur beobachten.
// Faellt der Verdampfer schnell (>= 1,5 K in einer Minute - so kuendigt sich das Zusetzen an), sofort ein Schritt.
const uint32_t FB_STEP_MS   = 300000UL;   // normal hoechstens alle 5 min ein Schritt
const uint32_t FB_FAST_MS   = 120000UL;   // bei schnellem Abfall hoechstens alle 2 min
const uint32_t FB_SETTLE_MS = 300000UL;
static float    fbEvF = NAN;
static float    fbRing[12];               // Rohwerte der letzten Minute (5-s-Takt)
static uint8_t  fbRi = 0, fbRn = 0;
static uint32_t fbStepMs = 0, fbRunSince = 0;
char fbInfo[80] = "aus";

void fbSet(int cap, const char* why) {
  char a[10], b[10];
  if (fbCap > 0) snprintf(a, sizeof(a), "%d Hz", (int)fbCap); else strcpy(a, "aus");
  if (cap > 0) snprintf(b, sizeof(b), "%d Hz", cap); else strcpy(b, "aus");
  Log.printf("Frost-Bremse: %s -> %s (%s, Verdampfer/Coil %.1f C)\n", a, b, why, fbEvF);
  fbCap = cap; fbChanged = true; fbStepMs = millis();
}

// liefert die Grenze der Bremse (oder -1); curve = Kurvenwert Heizen
int fbTick(const FcCfg& c, const LiveData& d, uint32_t ms, int curve, bool ww) {
  if (!c.fbOn || !c.enabled) {
    if (fbCap > 0) fbSet(-1, "ausgeschaltet");
    strlcpy(fbInfo, c.fbOn ? "wirkt nur mit eingeschalteter Frequenz-Automatik" : "aus", sizeof(fbInfo));
    fbRunSince = 0; fbEvF = NAN;
    return -1;
  }
  float ta = hcTaF;
  float hys = fbCap > 0 ? 0.5f : 0.0f;
  if (isnan(ta) || curve <= 0 || ta < c.fbTaLo - hys || ta > c.fbTaHi + hys) {
    if (fbCap > 0) fbSet(-1, "Aussentemperatur ausserhalb");
    if (isnan(ta) || curve <= 0) strlcpy(fbInfo, "keine Aussentemperatur", sizeof(fbInfo));
    else snprintf(fbInfo, sizeof(fbInfo), "bereit - aussen %.1f C, wirkt bei %d..%d C", ta, c.fbTaLo, c.fbTaHi);
    fbRunSince = 0; fbEvF = NAN;
    return -1;
  }
  if (fbCap > 0 && fbCap >= curve) fbSet(-1, "Kurve liegt nicht hoeher");
  bool run = d.extValid && d.hzValid && d.verdichterHz > 0 && d.wpStateValid && d.wpOnOffRaw && !ww && !dfActive;
  if (!run) {
    fbRunSince = 0; fbEvF = NAN;
    if (fbCap > 0) snprintf(fbInfo, sizeof(fbInfo), "%d Hz gehalten (%s)", (int)fbCap, ww ? "WW" : dfActive ? "Abtauen" : "Verdichter aus");
    else strlcpy(fbInfo, "wartet auf Heizbetrieb", sizeof(fbInfo));
    return fbCap;
  }
  if (!fbRunSince) { fbRunSince = ms; fbRn = 0; }
  float ev = c.fbSrc ? d.coil : d.evap;
  fbEvF = isnan(fbEvF) ? ev : fbEvF + (ev - fbEvF) * 0.15f;      // ca. 30 s glaetten
  float ev60 = fbRn >= 12 ? fbRing[fbRi] : NAN;                   // Wert vor einer Minute
  fbRing[fbRi] = ev; fbRi = (fbRi + 1) % 12; if (fbRn < 12) fbRn++;
  bool fastFall = !isnan(ev60) && ev60 - ev >= 1.5f && ev < c.fbEvap + 2.0f;
  const char* sn = c.fbSrc ? "Coil" : "Verdampfer";
  char st[16];
  if (fbCap > 0) snprintf(st, sizeof(st), "%d Hz", (int)fbCap); else strcpy(st, "keine Bremse");
  if (ms - fbRunSince < FB_SETTLE_MS || (dfEndMs && ms - dfEndMs < FB_SETTLE_MS)) {
    snprintf(fbInfo, sizeof(fbInfo), "%s - beobachtet (%s %.1f C)", st, sn, fbEvF);
    return fbCap;
  }
  bool vlKnown = d.tempsValid && d.sollValid;
  bool vlOk  = vlKnown && d.vorlauf >= d.sollwert - 1.0f;
  bool vlLow = vlKnown && d.vorlauf <  d.sollwert - 3.0f;
  bool canStep = !fbStepMs || ms - fbStepMs >= FB_STEP_MS;
  bool canFast = !fbStepMs || ms - fbStepMs >= FB_FAST_MS;
  if (fastFall && canFast && !vlLow) {
    int hzNow = (int)d.verdichterHz;
    int base = min(fbCap > 0 ? (int)fbCap : curve, hzNow + 2);
    int n = max((int)c.fbMinHz, base - (int)c.fbStep);
    if (n < curve && (fbCap <= 0 || n < fbCap)) {
      char why[48]; snprintf(why, sizeof(why), "Verdampfer faellt schnell: %.1f -> %.1f C", ev60, ev);
      fbSet(n, why);
    }
  } else if (canStep && fbCap > 0 && (vlLow || fbEvF > c.fbEvap + 2.0f)) {
    int n = fbCap + c.fbStep;
    fbSet(n >= curve ? -1 : n, vlLow ? "Vorlauf zu weit unter Soll" : "Verdampfer waermer");
  } else if (canStep && fbEvF < c.fbEvap && vlOk) {
    // auf die tatsaechliche Frequenz beziehen - regelt die WP schon tiefer, bringt ein Schritt ueber ihr nichts
    int hzNow = isnan(dfHzF) ? (int)d.verdichterHz : (int)lroundf(dfHzF);
    int base = min(fbCap > 0 ? (int)fbCap : curve, hzNow + 2);
    int n = max((int)c.fbMinHz, base - (int)c.fbStep);
    if (n < curve && (fbCap <= 0 || n < fbCap)) fbSet(n, "Verdampfer kalt");
  }
  if (fbCap > 0) snprintf(st, sizeof(st), "%d Hz", (int)fbCap); else strcpy(st, "keine Bremse");
  snprintf(fbInfo, sizeof(fbInfo), "%s - %s %.1f C (Grenze %d C), VL %.1f/%.0f C", st, sn, fbEvF, c.fbEvap,
           d.vorlauf, d.sollwert);
  return fbCap;
}

void fcTick() {                       // im ctrl-Task
  uint32_t ms = millis();
  if (fcLastTick && ms - fcLastTick < 5000UL) return;
  uint32_t dt = fcLastTick ? ms - fcLastTick : 0;
  fcLastTick = ms;
  FcCfg c = fcCfg;
  LiveData d = getLive();
  dfSample(d, dt);
  if (offBusy()) { strcpy(fcInfo, "pausiert (Ausschalten)"); return; }
  if (d.wpStateValid && d.maxHzValid && dfTick(c, d, ms)) {
    if (dfInfo[0]) strlcpy(fcInfo, dfInfo, sizeof(fcInfo));
    return;
  }
  int wpOn = (d.wpStateValid && d.wpOnOffRaw) ? 1 : 0;
  if (d.wpStateValid && wpOn != fcLastWpOn) { if (wpOn) fcWpOnSince = ms; fcLastWpOn = wpOn; }
  bool ww = fcWwActive(d);
  if (fcLastWw >= 0 && (int)ww != fcLastWw) fcSwitchPending = true;
  fcLastWw = ww; fcWw = ww;
  int t = fcCompute(ww);
  fcCurve = t;
  int fb = fbTick(c, d, ms, ww ? fcCompute(false) : t, ww);
  if (!ww && fb > 0 && t > 0 && fb < t) t = fb;
  fcTarget = t;
  if (!c.enabled) { strcpy(fcInfo, "aus (manuell)"); fcSwitchPending = false; return; }
  if (t < 0) { strcpy(fcInfo, "keine Aussentemperatur"); return; }
  if (!d.maxHzValid || !d.wpStateValid) { strcpy(fcInfo, "warte auf Messwerte"); return; }
  int cur = d.maxVerdichterHz;
  bool force = fcSwitchPending || fbChanged;
  int db = force ? 1 : max(1, (int)c.deadband);
  if (abs(t - cur) < db) {
    snprintf(fcInfo, sizeof(fcInfo), "passt (%d Hz, %s%s)", cur, ww ? "WW" : "Heizen", (!ww && fbCap > 0) ? ", Frost-Bremse" : "");
    fcSwitchPending = false; fbChanged = false;
    return;
  }
  if (!wpOn) { snprintf(fcInfo, sizeof(fcInfo), "%d Hz beim Einschalten der WP", t); return; }
  if (fcWritten && ms - fcLastWriteMs < 30000UL) return;                   // Ruecklesen abwarten
  bool justOn = ms - fcWpOnSince < 300000UL && (!fcWritten || (int32_t)(fcLastWriteMs - fcWpOnSince) < 0);
  if (!justOn && !force && fcWritten && ms - fcLastWriteMs < c.minIntMin * 60000UL) {
    snprintf(fcInfo, sizeof(fcInfo), "%d Hz in %lu min", t,
             (unsigned long)((c.minIntMin * 60000UL - (ms - fcLastWriteMs)) / 60000UL + 1));
    return;
  }
  fcWrite(t, cur, fcSwitchPending ? (ww ? "Wechsel auf WW" : "Wechsel auf Heizen") : fbChanged ? "Frost-Bremse" : justOn ? "WP an" : ww ? "WW" : "Heizen");
}

// vor dem automatischen Einschalten: abweichende Werte jetzt schreiben (WP ist noch aus)
void preStartWrites() {
  LiveData d = getLive();
  float vf = vlWantF;
  if (!isnan(vf) && d.sollValid && wwState == WW_IDLE) {     // nie waehrend WW-Ladung/Nachlauf
    vf = constrain(vf, (float)vlPendMin, (float)vlPendMax);
    int v = lroundf(vf);
    if (fabsf(vf - d.sollwert) >= vlDeadband && lroundf(d.sollwert) != v) {
      submitCmd(CMD_SET_SOLL, v, false);
      countWrite(vlWritesToday);
      Log.printf("VL (%s): %d -> %d C vor dem Einschalten\n", (const char*)vlPendSrc, (int)lroundf(d.sollwert), v);
      vlLastWriteMs = millis(); vlWritten = true;
      if (vlOneShot) vlWantF = NAN;
    }
  }
  if (fcCfg.enabled && d.maxHzValid) {
    bool ww = fcWwActive(d);
    int t = fcCompute(ww);
    if (!ww && fbCap > 0 && t > fbCap) t = fbCap;          // Frost-Bremse gilt weiter
    if (t > 0 && abs(t - (int)d.maxVerdichterHz) >= max(1, (int)fcCfg.deadband)) fcWrite(t, d.maxVerdichterHz, "vor dem Einschalten");
  }
}

// ---------- EIN/AUS zentral (mit Quelle, sanftem Ausschalten und Kontrolle) ----------
const char* volatile pwrLastSrc = "-";
static int8_t   offSeq = 0;            // 0 nichts, 1 Frequenz wird gesenkt, 2 AUS gesendet -> beobachten
static uint32_t offSeqMs = 0, offDiagMs = 0;
static int      offPrevMax = -1;
static uint8_t  offRetries = 0;
static bool     offWasLowered = false;

bool offBusy() { return offSeq != 0; }

void offDiag(const char* why) {
  LiveData d = getLive();
  Log.printf("AUS-Kontrolle: %s | 4096=%u Modus %u | Verdichter %u Hz (Ziel %u, max %u) | Status 0x%04X Ausg 0x%04X Eing 0x%04X Betriebsart %u | "
             "VL %.1f RL %.1f WW %.1f Aussen %.1f Abtauen %s | letzte Anforderung: %s\n",
             why, d.wpOnOffRaw, d.wpModeRaw, d.verdichterHz, d.targetHz, d.maxVerdichterHz, d.state, d.outputs, d.inputs, d.opMode,
             d.vorlauf, d.ruecklauf, d.warmwasser, d.ta, (d.state & 16) ? "ja" : "nein", (const char*)pwrLastSrc);
}

void powerCmd(bool on, const char* src) {
  pwrLastSrc = src;
  LiveData d = getLive();
  Log.printf("WP %s angefordert (%s), Verdichter %u Hz\n", on ? "EIN" : "AUS", src, d.verdichterHz);
  if (on) {
    if (offSeq == 1) Log.println("Sanft ausschalten abgebrochen");
    if (offWasLowered && offPrevMax > 0) {                 // abgesenkte Grenze sofort zurueck
      FcCfg c = fcCfg;
      int back = (c.enabled && fcTarget > 0) ? fcTarget : offPrevMax;
      submitCmd(CMD_MAXHZ, back, false); countWrite(hzWritesToday);
    }
    offWasLowered = false; offPrevMax = -1; offSeq = 0;
    submitCmd(CMD_POWER, 1, false);
    return;
  }
  FcCfg c = fcCfg;
  bool runs = d.hzValid && d.verdichterHz > c.offHz + 2;
  if (c.offLower && runs && d.maxHzValid && d.maxVerdichterHz > c.offHz) {
    offPrevMax = d.maxVerdichterHz;
    offWasLowered = true;
    submitCmd(CMD_MAXHZ, c.offHz, false);
    countWrite(hzWritesToday);
    Log.printf("Sanft ausschalten: Max Hz %d -> %u, AUS folgt, sobald der Verdichter langsamer ist\n", offPrevMax, c.offHz);
    offSeq = 1; offSeqMs = millis();
    return;
  }
  submitCmd(CMD_POWER, 0, false);
  offSeq = 2; offSeqMs = offDiagMs = millis(); offRetries = 0;
}

// im ctrl-Task
void offTick() {
  if (!offSeq) return;
  uint32_t ms = millis();
  LiveData d = getLive();
  FcCfg c = fcCfg;
  if (offSeq == 1) {
    bool slow = d.hzValid && d.verdichterHz <= c.offHz + 3;
    if (slow || ms - offSeqMs > 90000UL) {
      Log.printf("Sanft ausschalten: Verdichter %u Hz -> WP AUS\n", d.verdichterHz);
      submitCmd(CMD_POWER, 0, false);
      offSeq = 2; offSeqMs = offDiagMs = ms; offRetries = 0;
    }
    return;
  }
  // offSeq == 2: beobachten, ob die WP wirklich ausgeht (max. 15 min)
  if (ms - offSeqMs < 20000UL || !d.wpStateValid) return;
  bool stopped = d.hzValid && d.verdichterHz == 0;
  if (stopped || ms - offSeqMs > 900000UL) {
    if (stopped) Log.printf("AUS-Kontrolle: Verdichter steht (nach %lu s)\n", (unsigned long)((ms - offSeqMs) / 1000));
    else offDiag("Verdichter laeuft nach 15 min noch - Kontrolle beendet");
    if (offWasLowered && offPrevMax > 0) {                 // abgesenkte Grenze wieder herstellen
      int back = (c.enabled && fcTarget > 0) ? fcTarget : offPrevMax;
      if (d.maxHzValid && back != (int)d.maxVerdichterHz) {
        submitCmd(CMD_MAXHZ, back, false); countWrite(hzWritesToday);
        Log.printf("Sanft ausschalten: Max Hz zurueck auf %d\n", back);
      }
    }
    offWasLowered = false; offPrevMax = -1; offSeq = 0;
    return;
  }
  if (d.wpOnOffRaw) {                                      // Register steht wieder auf EIN
    if (ms - offDiagMs > 60000UL && offRetries < 3) {
      offRetries++; offDiagMs = ms;
      offDiag("WP meldet wieder EIN");
      Log.printf("AUS-Kontrolle: AUS erneut senden (%u/3)\n", offRetries);
      submitCmd(CMD_POWER, 0, false);
    }
  } else if (ms - offDiagMs > 120000UL) {                  // Register AUS, Verdichter laeuft weiter
    offDiagMs = ms;
    offDiag("4096 = AUS, aber Verdichter laeuft noch");
    if (offRetries < 3) { offRetries++; Log.printf("AUS-Kontrolle: AUS erneut senden (%u/3)\n", offRetries); submitCmd(CMD_POWER, 0, false); }
  }
}

// ---------- MQTT ----------
void mqttTopic(char* out, size_t n, const char* sub) { snprintf(out, n, "%s/%s", mqttCfg.base, sub); }

void pvMqtt(const char* sub, const char* payload);   // PV-Teil weiter unten

// Shelly: Zahl hinter "key": aus einem JSON-Text
float jsonField(const char* js, const char* key) {
  char k[40]; snprintf(k, sizeof(k), "\"%s\":", key);
  const char* p = strstr(js, k);
  if (!p) return NAN;
  p += strlen(k);
  while (*p == ' ') p++;
  char* end; float v = strtof(p, &end);
  return end == p ? NAN : v;
}
// Meldungen unter <elTopic>/ : status/em:0 (Leistung), status/emdata:0 (Zaehlerstand)
// oder events/rpc (NotifyStatus mit "em:0" / "emdata:0", falls nur "RPC status notifications" an ist)
void shellyMsg(const char* sub, const char* data, int len) {
  static char buf[2100];
  static bool first = true;
  shAny++; strlcpy(shLast, sub, sizeof(shLast)); shLastLen = len;
  if (len <= 0 || len >= (int)sizeof(buf)) {
    shBad++;
    if (len > 0) Log.printf("Shelly: Meldung %s zu lang (%d Byte)\n", sub, len);
    return;
  }
  memcpy(buf, data, len); buf[len] = 0;
  const char *em = nullptr, *ed = nullptr, *sw = nullptr, *e1 = nullptr, *e1d = nullptr;
  if (!strcmp(sub, "status/em:0")) em = buf;                                   // Pro 3EM, 3EM-63 (3 Phasen)
  else if (!strcmp(sub, "status/emdata:0")) ed = buf;
  else if (!strcmp(sub, "status/switch:0") || !strcmp(sub, "status/pm1:0")) sw = buf;   // 1PM, PM Mini
  else if (!strcmp(sub, "status/em1:0")) e1 = buf;                             // EM / Pro EM (1 Phase, Zange)
  else if (!strcmp(sub, "status/em1data:0")) e1d = buf;
  else if (!strcmp(sub, "events/rpc")) {
    em = strstr(buf, "\"em:0\":"); ed = strstr(buf, "\"emdata:0\":");
    sw = strstr(buf, "\"switch:0\":"); if (!sw) sw = strstr(buf, "\"pm1:0\":");
    e1 = strstr(buf, "\"em1:0\":"); e1d = strstr(buf, "\"em1data:0\":");
  }
  else return;                                         // anderes Shelly-Topic
  bool used = false;
  if (sw) {
    float p = jsonField(sw, "apower");
    if (!isnan(p)) { shP = p; shI = jsonField(sw, "current"); shMs = millis(); shRx++; used = true; }
    const char* ae = strstr(sw, "\"aenergy\":");
    float e = ae ? jsonField(ae, "total") : NAN;          // Wh
    if (!isnan(e)) { shETot = e / 1000.0f; shEMs = millis(); used = true; }
  }
  if (e1) {
    float p = jsonField(e1, "act_power");
    if (!isnan(p)) { shP = p; shI = jsonField(e1, "current"); shMs = millis(); shRx++; used = true; }
  }
  if (e1d) {
    float e = jsonField(e1d, "total_act_energy");        // Wh
    if (!isnan(e)) { shETot = e / 1000.0f; shEMs = millis(); used = true; }
  }
  if (em) {
    float p = jsonField(em, "total_act_power");
    if (!isnan(p)) { shP = p; shI = jsonField(em, "total_current"); shMs = millis(); shRx++; used = true; }
  }
  if (ed) {
    float e = jsonField(ed, "total_act");                // Wh Bezug, alle Phasen
    if (!isnan(e)) { shETot = e / 1000.0f; shEMs = millis(); used = true; }
  }
  if (!used && (em || ed || sw || e1 || e1d || strcmp(sub, "events/rpc"))) {
    shBad++;
    if (shBad <= 3) Log.printf("Shelly: %s ohne Leistungswert (%d Byte): %.80s\n", sub, len, buf);
  }
  if (used && first) { first = false; Log.printf("Shelly: erste Daten ueber %s/%s, %.0f W\n", elTopic, sub, (float)shP); }
}
// meldet Ausfall / Rueckkehr des Shelly (in loop)
void shellyTick() {
  static int8_t last = -1;
  if (!elSrc) { last = -1; return; }
  int8_t ok = shFresh() ? 1 : 0;
  if (ok != last) {
    if (ok) Log.printf("Shelly: Daten kommen (%s) -> el. Leistung gemessen\n", elTopic);
    else Log.printf("Shelly: keine Daten von %s/status/... -> el. Leistung aus der Schaetzung\n", elTopic);
    last = ok;
  }
}
void kaminSet(bool on, const char* src);

void mqttHandle(const char* sub, const char* payload) {
  String p = payload; p.trim();
  String pl = p; pl.toLowerCase();
  float v = p.toFloat();
  bool on = (pl == "1" || pl == "on" || pl == "ein" || pl == "true");
  mqttRxCount++;
  snprintf(mqttLastCmd, sizeof(mqttLastCmd), "%s=%s", sub, p.c_str());

  if (!strcmp(sub, "sg")) {
    sgSet(p.toInt(), "MQTT");
  } else if (!strcmp(sub, "vl")) {
    if (hcCfg.enabled) { Log.println("MQTT vl ignoriert: Heizkurve im ESP ist aktiv"); return; }
    if (!pvCfg.vlMqtt) { Log.println("MQTT vl: im Web deaktiviert"); return; }
    if (v >= 10 && v <= 75) vlRequest(lroundf(v), pvCfg.vlMinSec, pvCfg.vlMin, pvCfg.vlMax, "MQTT");
  } else if (!strcmp(sub, "rtsoll")) {
    if (v >= 10 && v <= 28) hcSetSoll(v);
  } else if (!strcmp(sub, "hc")) {
    hcCfg.enabled = on; saveHcCfg(); Log.printf("Heizkurve %s (MQTT)\n", on ? "EIN" : "AUS");
  } else if (!strcmp(sub, "room")) {
    hcCfg.roomOn = on; saveHcCfg(); Log.printf("Raumeinfluss %s (MQTT)\n", on ? "EIN" : "AUS");
  } else if (!strcmp(sub, "kamin")) {
    kaminSet(on, "MQTT");
  } else if (!strcmp(sub, "fc")) {
    fcCfg.enabled = on; saveFcCfg(); Log.printf("Frequenzkurve %s (MQTT)\n", on ? "EIN" : "AUS");
  } else if (!strcmp(sub, "autowp")) {
    hcCfg.autoEnable = on; saveHcCfg(); Log.printf("Auto-WP %s (MQTT)\n", on ? "EIN" : "AUS");
  } else if (!strcmp(sub, "sperre")) {
    lockCfg.enabled = on; saveLockCfg(); Log.printf("Sperrzeiten %s (MQTT)\n", on ? "EIN" : "AUS");
  } else if (!strcmp(sub, "wwload")) {
    if (pl == "0" || pl == "stop" || pl == "off") reqWwStop = true;
    else { reqWwTarget = (v >= 30 && v <= 70) ? (uint8_t)lroundf(v) : 0; reqWwPv = false; reqWwManual = true; reqWwStart = true; }
  } else if (!strcmp(sub, "power")) {
    powerCmd(on, "MQTT");
  } else if (!strcmp(sub, "mode")) {
    if (pl == "heizen" || pl == "2") submitCmd(CMD_MODE, 2, false);
    else if (pl == "ww" || pl == "1") submitCmd(CMD_MODE, 1, false);
    else if (pl == "kuehlen" || pl == "4") submitCmd(CMD_MODE, 4, false);
  } else if (!strcmp(sub, "valve")) {
    if (pl == "heizung" || pl == "0") reqValve = 0;
    else if (pl == "puffer") reqValve = getCtrlCfg().bufPos;
    else if (pl == "ww" || pl == "100") reqValve = 100;
    else if (pl == "auto") reqValve = -1;
  } else if (!strcmp(sub, "heater")) {
    int minutes = p.toInt();
    heaterManualUntil = minutes > 0 ? millis() + (uint32_t)min(minutes, 480) * 60000UL : 0;
    Log.printf("Heizstab Hand: %d min\n", minutes > 0 ? minutes : 0);
  } else if (!strcmp(sub, "console")) {
    processSerialCmd(p);
  } else {
    Log.printf("MQTT: unbekannter Befehl %s\n", sub);
  }
}

static void mqttEvent(void*, esp_event_base_t, int32_t id, void* data) {
  esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)data;
  char t[96];
  switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
      mqttConnected = true;
      mqttTopic(t, sizeof(t), "status");
      esp_mqtt_client_publish(mqttClient, t, "online", 0, 1, 1);
      { char tp[64]; mqttTopic(tp, sizeof(tp), "pv/#"); esp_mqtt_client_subscribe_single(mqttClient, tp, 0); }
      if (elTopic[0]) {                                   // Shelly-Leistungsmessung
        char st[80]; int r1, r2, r3;
        snprintf(st, sizeof(st), "%s/status/em:0", elTopic);     r1 = esp_mqtt_client_subscribe_single(mqttClient, st, 0);
        snprintf(st, sizeof(st), "%s/status/emdata:0", elTopic); r2 = esp_mqtt_client_subscribe_single(mqttClient, st, 0);
        snprintf(st, sizeof(st), "%s/events/rpc", elTopic);      r3 = esp_mqtt_client_subscribe_single(mqttClient, st, 0);
        static const char* const SH_MORE[] = {"status/switch:0", "status/pm1:0", "status/em1:0", "status/em1data:0"};
        for (const char* m : SH_MORE) {                   // 1PM, PM Mini, EM (eine Phase)
          snprintf(st, sizeof(st), "%s/%s", elTopic, m);
          if (esp_mqtt_client_subscribe_single(mqttClient, st, 0) < 0) r3 = -1;
        }
        Log.printf("MQTT: Shelly abonniert %s/status/em:0, emdata:0, switch:0, pm1:0, em1:0, em1data:0, events/rpc%s\n", elTopic,
                   (r1 < 0 || r2 < 0 || r3 < 0) ? " - FEHLER beim Abonnieren" : "");
      }
      mqttTopic(t, sizeof(t), "cmd/#");
      esp_mqtt_client_subscribe_single(mqttClient, t, 0);
      Log.printf("MQTT verbunden mit %s:%u, Befehle auf %s\n", mqttCfg.host, mqttCfg.port, t);
      break;
    case MQTT_EVENT_DISCONNECTED:
      if (mqttConnected) Log.println("MQTT getrennt");
      mqttConnected = false;
      break;
    case MQTT_EVENT_DATA: {
      if (!e->topic || e->topic_len <= 0) break;
      char topic[96], payload[128];
      int tl = min(e->topic_len, (int)sizeof(topic) - 1);
      memcpy(topic, e->topic, tl); topic[tl] = 0;
      if (elTopic[0]) {                                   // Shelly (lange JSON-Texte, nur vollstaendige Meldungen)
        char sp[56]; snprintf(sp, sizeof(sp), "%s/", elTopic);
        size_t sl = strlen(sp);
        if (strncmp(topic, sp, sl) == 0) {
          shellyMsg(topic + sl, e->data, e->data_len);     // PubSubClient: Puffer 3072 Byte, Meldung immer vollstaendig
          break;
        }
      }
      if (e->data_len >= 120) break;
      memcpy(payload, e->data, e->data_len); payload[e->data_len] = 0;
      char prefix[48]; mqttTopic(prefix, sizeof(prefix), "cmd/");
      size_t pl = strlen(prefix);
      if (strncmp(topic, prefix, pl) == 0) { mqttHandle(topic + pl, payload); break; }
      mqttTopic(prefix, sizeof(prefix), "pv/");
      pl = strlen(prefix);
      if (strncmp(topic, prefix, pl) == 0) pvMqtt(topic + pl, payload);
      break;
    }
    default: break;
  }
}

void mqttStart() {
  if (!mqttCfg.enabled || !mqttCfg.host[0]) { Log.println("MQTT aus"); return; }
  static char uri[80], lwt[48], cid[32];
  snprintf(uri, sizeof(uri), "mqtt://%s:%u", mqttCfg.host, mqttCfg.port);
  mqttTopic(lwt, sizeof(lwt), "status");
  snprintf(cid, sizeof(cid), "avarma-%06lx", (unsigned long)(ESP.getEfuseMac() & 0xFFFFFF));
  esp_mqtt_client_config_t cfg = {};
  cfg.broker.address.uri = uri;
  cfg.credentials.client_id = cid;
  if (mqttCfg.user[0]) cfg.credentials.username = mqttCfg.user;
  if (mqttCfg.pass[0]) cfg.credentials.authentication.password = mqttCfg.pass;
  cfg.session.keepalive = 30;
  cfg.session.last_will.topic = lwt;
  cfg.session.last_will.msg = "offline";
  cfg.session.last_will.qos = 1;
  cfg.session.last_will.retain = 1;
  cfg.network.reconnect_timeout_ms = 10000;
  cfg.task.stack_size = 6144;
  mqttClient = esp_mqtt_client_init(&cfg);
  if (!mqttClient) { Log.println("MQTT: Start fehlgeschlagen"); return; }
  esp_mqtt_client_register_event(mqttClient, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqttEvent, nullptr);
  esp_mqtt_client_start(mqttClient);
  Log.printf("MQTT: verbinde mit %s\n", uri);
}

void jsonNum(String& o, const char* k, float v, int dec, bool valid) {
  o += ",\""; o += k; o += "\":";
  if (valid && !isnan(v)) o += String(v, dec); else o += "null";
}

void buildStateJson(String& o) {
  LiveData d = getLive();
  CtrlCfg c = getCtrlCfg();
  o = "{\"time\":"; o += String((unsigned long)nowUtc());
  jsonNum(o, "vl", d.vorlauf, 1, d.tempsValid);
  jsonNum(o, "rl", d.ruecklauf, 1, d.tempsValid);
  jsonNum(o, "ww", d.warmwasser, 1, d.tempsValid);
  jsonNum(o, "ta", d.ta, 1, d.extValid);
  jsonNum(o, "flow", d.flowLmin, 1, d.flowValid);
  jsonNum(o, "hz", d.verdichterHz, 0, d.hzValid);
  jsonNum(o, "hzTarget", d.targetHz, 0, d.extValid);
  jsonNum(o, "fan1", d.fan1Rpm, 0, d.fanRpmValid);
  jsonNum(o, "fan2", d.fan2Rpm, 0, d.fanRpmValid);
  jsonNum(o, "vlSoll", d.sollwert, 0, d.sollValid);
  jsonNum(o, "maxHz", d.maxVerdichterHz, 0, d.maxHzValid);
  jsonNum(o, "maxFan", d.maxFanRpm, 0, d.maxFanValid);
  o += ",\"wpOn\":"; o += d.wpStateValid ? (d.wpOnOffRaw ? "1" : "0") : "null";
  jsonNum(o, "mode", d.wpModeRaw, 0, d.wpModeValid);
  jsonNum(o, "coil", d.coil, 1, d.extValid);
  jsonNum(o, "discharge", d.discharge, 1, d.extValid);
  jsonNum(o, "suction", d.suction, 1, d.extValid);
  jsonNum(o, "liquid", d.liquid, 1, d.extValid);
  jsonNum(o, "evap", d.evap, 1, d.extValid);
  jsonNum(o, "cond", d.cond, 1, d.extValid);
  jsonNum(o, "ipm", d.ipm, 1, d.extValid);
  jsonNum(o, "hp", d.hp, 2, d.extValid);
  jsonNum(o, "lp", d.lp, 2, d.extValid);
  jsonNum(o, "currentA", d.currentA, 1, d.extValid);
  jsonNum(o, "acV", d.acV, 1, d.extValid);
  jsonNum(o, "pumpPct", d.pumpPct, 0, d.cntValid);
  jsonNum(o, "starts", d.starts, 0, d.cntValid);
  jsonNum(o, "runtimeH", d.runtimeH, 0, d.cntValid);
  o += ",\"defrost\":"; o += d.extValid ? ((d.state & 16) ? "1" : "0") : "null";
  jsonNum(o, "pEl", d.pEl, 0, d.extValid);
  jsonNum(o, "pTh", d.pTh, 0, d.extValid);
  jsonNum(o, "pElWp", d.pElWp, 0, d.extValid);
  jsonNum(o, "pElShelly", d.pElSh, 0, d.extValid && !isnan(d.pElSh));
  o += ",\"pElSrc\":\""; o += d.pElSrc ? "shelly" : "wp"; o += "\"";
  jsonNum(o, "cop", d.pEl > 100 ? d.pTh / d.pEl : NAN, 2, d.extValid);
  jsonNum(o, "eElToday", d.eElDay / 1000.0, 2, true);
  jsonNum(o, "eThToday", d.eThDay / 1000.0, 2, true);
  jsonNum(o, "eElYesterday", d.eElYest / 1000.0, 2, true);
  jsonNum(o, "eThYesterday", d.eThYest / 1000.0, 2, true);
  jsonNum(o, "eElTotal", d.eElTot / 1000.0, 1, true);
  jsonNum(o, "eThTotal", d.eThTot / 1000.0, 1, true);
  jsonNum(o, "valvePos", valvePosNow(), 0, c.enabled && valveKnown);
  o += ",\"valveMoving\":"; o += valveMoving ? "1" : "0";
  o += ",\"ww_state\":\""; o += wwStateName(wwState); o += "\"";
  o += ",\"pvLoad\":"; o += pvLoad ? "1" : "0";
  jsonNum(o, "sg", sgState, 0, true);
  o += ",\"heater\":"; o += heaterOn ? "1" : "0";
  jsonNum(o, "rt", rtIst, 1, rtValid());
  jsonNum(o, "rh", rhIst, 0, rtValid());
  jsonNum(o, "rtSoll", hcRtSoll, 1, true);
  jsonNum(o, "taDamped", hcTaF, 1, !isnan(hcTaF));
  jsonNum(o, "hcVl", hcTarget, 0, hcTarget >= 0);
  o += ",\"hcOn\":"; o += hcCfg.enabled ? "1" : "0";
  o += ",\"autoWp\":"; o += hcCfg.autoEnable ? "1" : "0";
  o += ",\"night\":"; o += hcNight ? "1" : "0";
  o += ",\"fcOn\":"; o += fcCfg.enabled ? "1" : "0";
  o += ",\"kamin\":"; o += kaminOn ? "1" : "0";
  o += ",\"roomOn\":"; o += hcCfg.roomOn ? "1" : "0";
  {                                                   // PV-Daten mitschicken (z. B. fuer die ePaper-Anzeige)
    float soc = pvGet(pvSoc, 900000UL), pw = pvGet(pvPower, 300000UL);
    jsonNum(o, "pvSoc", soc, 0, !isnan(soc));
    jsonNum(o, "pvPower", pw, 0, !isnan(pw));
  }
  jsonNum(o, "fcHz", fcTarget, 0, fcTarget > 0);
  jsonNum(o, "vlWrites", vlWritesToday, 0, true);
  jsonNum(o, "hzWrites", hzWritesToday, 0, true);
  o += ",\"fw\":\"" FW_VERSION "\",\"plat\":\"" AV_PLATFORM "\"";
  jsonNum(o, "mbOk", d.okCount, 0, true);
  jsonNum(o, "mbErr", d.errCount, 0, true);
  o += "}";
}

void mqttTick() {                     // in loop()
  static uint32_t last = 0;
  shellyTick();
  if (!mqttClient || !mqttConnected) return;
  if (millis() - last < max(2, (int)mqttCfg.interval) * 1000UL) return;
  last = millis();
  String js; js.reserve(1400);
  buildStateJson(js);
  char t[64]; mqttTopic(t, sizeof(t), "state");
  if (esp_mqtt_client_publish(mqttClient, t, js.c_str(), js.length(), 0, 1) >= 0) mqttTxCount++;
}

// ---------- PV-Daten (kommen per MQTT von Venus OS / Node-RED) ----------
// Topics <base>/pv/power (W), soc (%), bat (W, + = laden), grid (W, + = Bezug), house (W),
//        today (kWh), yesterday (kWh). Fehlen today/yesterday, rechnet der ESP selbst aus power.
PvVal pvPower, pvSoc, pvBat, pvGrid, pvHouse, pvToday, pvYest;
double pvOwnDay = 0, pvOwnYest = 0;   // Wh, selbst integriert
int    pvOwnDayIdx = -1;

bool pvFresh(const PvVal& p, uint32_t maxAgeMs) { return p.ms && !isnan(p.v) && millis() - p.ms < maxAgeMs; }
float pvGet(const PvVal& p, uint32_t maxAgeMs) { return pvFresh(p, maxAgeMs) ? p.v : NAN; }
bool pvAny() { return pvFresh(pvPower, 300000UL) || pvFresh(pvSoc, 300000UL); }
float pvTodayKwh() { return pvFresh(pvToday, 1800000UL) ? (float)pvToday.v : (pvOwnDayIdx >= 0 ? (float)(pvOwnDay / 1000.0) : NAN); }
float pvYestKwh()  { return pvFresh(pvYest, 26UL * 3600000UL) ? (float)pvYest.v : (pvOwnDayIdx >= 0 ? (float)(pvOwnYest / 1000.0) : NAN); }

void pvMqtt(const char* sub, const char* payload) {
  if (!pvCfg.show) return;                       // PV/Energiemanagement ausgeschaltet
  const char* p = payload;
  const char* vpos = strstr(payload, "\"value\"");          // Venus-Format {"value": 123}
  if (vpos) { p = strchr(vpos, ':'); if (!p) return; p++; }
  while (*p == ' ') p++;
  if (!strncmp(p, "null", 4)) return;
  char* end; float v = strtof(p, &end);
  if (end == p || isnan(v)) return;
  PvVal* t = nullptr;
  if (!strcmp(sub, "power")) t = &pvPower;
  else if (!strcmp(sub, "soc")) t = &pvSoc;
  else if (!strcmp(sub, "bat")) t = &pvBat;
  else if (!strcmp(sub, "grid")) t = &pvGrid;
  else if (!strcmp(sub, "house")) t = &pvHouse;
  else if (!strcmp(sub, "today")) t = &pvToday;
  else if (!strcmp(sub, "yesterday")) t = &pvYest;
  if (!t) return;
  t->v = v; t->ms = millis();
}

void pvTick() {                       // im ctrl-Task: PV-Ertrag selbst mitzaehlen
  static uint32_t last = 0;
  uint32_t ms = millis();
  if (last && ms - last < 10000UL) return;
  float h = last ? (ms - last) / 3600000.0f : 0;
  last = ms;
  if (!timeValid()) return;
  time_t tt = time(nullptr); struct tm tmv; localtime_r(&tt, &tmv);
  if (pvOwnDayIdx != tmv.tm_yday) {
    if (pvOwnDayIdx >= 0) {
      bool yesterday = (tmv.tm_yday == pvOwnDayIdx + 1) || (tmv.tm_yday == 0 && pvOwnDayIdx >= 364);
      pvOwnYest = yesterday ? pvOwnDay : 0;
    }
    pvOwnDay = 0; pvOwnDayIdx = tmv.tm_yday;
  }
  float pw = pvGet(pvPower, 60000UL);
  if (!isnan(pw) && pw > 0 && h < 0.02f) pvOwnDay += pw * h;
}

// =====================================================
// Tagesauswertung / Statistik (SD-Karte)
//   /log/h_JJJJ-MM.csv : Stundenmittel (fuer lange Verlaeufe)
//   /log/stats.csv     : eine Zeile je Tag (Energie, COP, Laufzeit ...)
// Ein abgeschlossener Tag wird nach Mitternacht aus seiner Tagesdatei berechnet.
// Beim ersten Start werden alle vorhandenen Tagesdateien nachberechnet.
// =====================================================
static int32_t aggLast = -2;          // zuletzt ausgewerteter Tag (JJJJMMTT), -2 = noch nicht geladen
static int32_t aggFirst = 0;          // erster Tag mit Daten
static uint32_t aggNextMs = 0;
volatile uint16_t aggBacklog = 0;     // noch nachzuberechnende Tage (Anzeige)

int32_t dayKeyOf(uint32_t t) {
  time_t tt = t; struct tm tmv; localtime_r(&tt, &tmv);
  return (tmv.tm_year + 1900) * 10000 + (tmv.tm_mon + 1) * 100 + tmv.tm_mday;
}
// Mitternacht (lokal) des Tages
uint32_t dayKeyStart(int32_t key) {
  struct tm tmv = {};
  tmv.tm_year = key / 10000 - 1900; tmv.tm_mon = (key / 100) % 100 - 1; tmv.tm_mday = key % 100;
  tmv.tm_hour = 0; tmv.tm_isdst = -1;
  return (uint32_t)mktime(&tmv);
}
int32_t dayKeyNext(int32_t key) {
  struct tm tmv = {};
  tmv.tm_year = key / 10000 - 1900; tmv.tm_mon = (key / 100) % 100 - 1; tmv.tm_mday = key % 100 + 1;
  tmv.tm_hour = 12; tmv.tm_isdst = -1;
  return dayKeyOf((uint32_t)mktime(&tmv));
}

void dayReset(DayStat& s, int32_t key) {
  memset(&s, 0, sizeof(s));
  s.day = key; s.taMin = 999; s.taMax = -999;
}

void dayAdd(DayStat& s, const LogPoint& p) {
  if (!(p.flags & F_TEMPS) || p.t <= s.lastT) return;
  double dt = s.lastT ? min((double)(p.t - s.lastT), 120.0) : (double)LOG_INTERVAL_SEC;
  bool firstPt = (s.lastT == 0);
  s.lastT = p.t;
  s.n++;
  bool run = (p.flags & F_HZ) && p.hz > 0;
  if (run) s.runSec += dt;
  if (run && !s.lastRun && !firstPt) s.starts++;
  s.lastRun = run;
  bool df = (p.f2 & F2_DEFROST) != 0;
  if (df && !s.lastDf) s.defrosts++;
  s.lastDf = df;

  double h = dt / 3600.0;
  bool ww = (p.f2 & F2_WWLOAD) || ((p.flags & F_MODEVALID) && p.mode == 1);
  double el = 0, th = 0;
  if (p.f2 & F2_EXT) { el = max(0, (int)p.pEl); th = p.pTh; s.nExt++; }
  else if (p.flags & F_FLOW) th = p.flow / 10.0 / 60.0 * 4186.0 * (p.vl - p.rl) / 10.0;   // aeltere Logs
  if (th < 0) th = 0;
  s.el += el * h; s.th += th * h;
  if (ww) { s.elWw += el * h; s.thWw += th * h; }
  if (p.pv != 0xFFFF) { s.pv += p.pv * h; s.nPv++; }
  if (el > 0 && p.grid != INT16_MIN) {             // WP-Strom aus eigener Erzeugung (PV/Batterie)
    double imp = max(0, (int)p.grid);
    s.elG += el * h;
    s.elOwn += max(0.0, el - imp) * h;
  }
  if (p.ta != INT16_MIN) {
    float ta = p.ta / 10.0f;
    s.taSum += ta; s.taN++;
    if (ta < s.taMin) s.taMin = ta;
    if (ta > s.taMax) s.taMax = ta;
  }
  if (p.rt != INT16_MIN) { s.rtSum += p.rt / 10.0; s.rtN++; }
}

// JSON-Array einer Tageszeile: [Tag, Strom, Waerme, StromWW, WaermeWW, Laufzeit_h, Starts, Abtauungen,
//   Aussen_mittel, Aussen_min, Aussen_max, Raum_mittel, PV, WP-Strom_mit_Netzdaten, davon_eigen]  (kWh)
void dayToJson(const DayStat& s, String& o) {
  char b[220];
  auto FN = [](char* out, size_t n, double v, int dec, bool ok) { if (ok) snprintf(out, n, "%.*f", dec, v); else strcpy(out, "null"); };
  char el[16], elw[16], ta[12], tmi[12], tma[12], rt[12], pv[16], eg[16], eo[16];
  bool hasEl = s.nExt > 0;
  FN(el, sizeof(el), s.el / 1000, 3, hasEl); FN(elw, sizeof(elw), s.elWw / 1000, 3, hasEl);
  FN(ta, sizeof(ta), s.taN ? s.taSum / s.taN : 0, 1, s.taN > 0);
  FN(tmi, sizeof(tmi), s.taMin, 1, s.taN > 0); FN(tma, sizeof(tma), s.taMax, 1, s.taN > 0);
  FN(rt, sizeof(rt), s.rtN ? s.rtSum / s.rtN : 0, 1, s.rtN > 0);
  FN(pv, sizeof(pv), s.pv / 1000, 3, s.nPv > 0);
  FN(eg, sizeof(eg), s.elG / 1000, 3, s.elG > 0); FN(eo, sizeof(eo), s.elOwn / 1000, 3, s.elG > 0);
  snprintf(b, sizeof(b), "[%ld,%s,%.3f,%s,%.3f,%.2f,%u,%u,%s,%s,%s,%s,%s,%s,%s]",
           (long)s.day, el, s.th / 1000, elw, s.thWw / 1000, s.runSec / 3600.0, s.starts, s.defrosts,
           ta, tmi, tma, rt, pv, eg, eo);
  o += b;
}

// CSV-Zeile fuer stats.csv (gleiche Reihenfolge, leere Felder statt null)
int dayToCsv(const DayStat& s, char* out, size_t n) {
  String j; dayToJson(s, j);
  // [a,b,...] -> a;b;... und null -> leer
  int k = 0;
  for (size_t i = 1; i + 1 < j.length() && k < (int)n - 2; i++) {
    if (j[i] == 'n') { i += 3; continue; }                 // "null" -> leer
    out[k++] = j[i] == ',' ? ';' : j[i];
  }
  out[k++] = '\n'; out[k] = 0;
  return k;
}

// Stundenmittel-Zeile schreiben
int hourToCsv(uint32_t tMid, const HistAcc& a, char* out, size_t n) {
  auto avg = [](double s, uint32_t c) { return c ? s / c : 0.0; };
  return snprintf(out, n, "%lu;%lu;%.1f;%.1f;%.1f;%.1f;%.1f;%lu;%.1f;%lu;%.1f;%.1f;%lu;%.1f;%lu;%.0f;%.0f;%lu;%lu;%.0f;%lu;%lu;%.0f;%lu\n",
    (unsigned long)tMid, (unsigned long)a.n, avg(a.sVl, a.n), avg(a.sRl, a.n), avg(a.sWw, a.n), avg(a.sHz, a.n),
    avg(a.sFl, a.nFl), (unsigned long)a.nFl, avg(a.sRt, a.nRt), (unsigned long)a.nRt,
    avg(a.sTa, a.nTa), avg(a.sEv, a.nTa), (unsigned long)a.nTa, avg(a.sSv, a.nSv), (unsigned long)a.nSv,
    avg(a.sEl, a.nPw), avg(a.sTh, a.nPw), (unsigned long)a.nPw, (unsigned long)a.nDf, avg(a.sPv, a.nPv), (unsigned long)a.nPv, (unsigned long)a.nWw,
    avg(a.sFan, a.nFan), (unsigned long)a.nFan);
}

// einen abgeschlossenen Tag auswerten (nur aus loop() aufrufen)
static HistAcc hourAcc[25];
void processDay(int32_t key) {
  char path[32];
  snprintf(path, sizeof(path), "/log/%04ld-%02ld-%02ld.csv", (long)(key / 10000), (long)((key / 100) % 100), (long)(key % 100));
  if (!SD_MMC.exists(path)) return;
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return;
  uint32_t t0 = dayKeyStart(key);
  DayStat s; dayReset(s, key);
  for (int i = 0; i < 25; i++) accReset(hourAcc[i]);
  static char rbuf[4096];
  char line[240];
  int ll = 0, n;
  while ((n = f.read((uint8_t*)rbuf, sizeof(rbuf))) > 0) {
    for (int i = 0; i < n; i++) {
      char c = rbuf[i];
      if (c == '\n') {
        line[ll] = 0; ll = 0;
        LogPoint p;
        if (parseCsvLine(line, p) && p.t >= t0 && dayKeyOf(p.t) == key) {
          dayAdd(s, p);
          accAddPoint(hourAcc[constrain((int)((p.t - t0) / 3600), 0, 24)], p);
        }
      } else if (c != '\r' && ll < (int)sizeof(line) - 1) line[ll++] = c;
    }
    delay(0);
  }
  f.close();
  if (!s.n) return;

  char mpath[32];
  monthFileName(t0 + 43200, mpath, sizeof(mpath));
  File h = SD_MMC.open(mpath, FILE_APPEND);
  if (h) {
    char row[220];
    for (int i = 0; i < 25; i++) {
      if (!hourAcc[i].n) continue;
      int len = hourToCsv(t0 + i * 3600 + 1800, hourAcc[i], row, sizeof(row));
      h.write((const uint8_t*)row, len);
    }
    h.close();
  }
  File st = SD_MMC.open("/log/stats.csv", FILE_APPEND);
  if (st) {
    if (st.size() == 0)
      st.print("Tag;Strom_kWh;Waerme_kWh;Strom_WW_kWh;Waerme_WW_kWh;Laufzeit_h;Starts;Abtauungen;"
               "Aussen_mittel;Aussen_min;Aussen_max;Raum_mittel;PV_kWh;WP_Strom_mit_Netzdaten_kWh;davon_eigen_kWh\n");
    char row[240];
    int len = dayToCsv(s, row, sizeof(row));
    st.write((const uint8_t*)row, len);
    st.close();
  }
  Log.printf("Statistik: %ld ausgewertet (%lu Punkte, Waerme %.1f kWh)\n", (long)key, (unsigned long)s.n, s.th / 1000);
}

int32_t findFirstLogDay() {
  int32_t first = 0;
  File dir = SD_MMC.open("/log");
  if (!dir) return 0;
  File e;
  while ((e = dir.openNextFile())) {
    const char* nm = e.name();
    const char* b = strrchr(nm, '/'); b = b ? b + 1 : nm;
    int y, m, d;
    if (strlen(b) == 14 && sscanf(b, "%4d-%2d-%2d.csv", &y, &m, &d) == 3) {
      int32_t k = y * 10000 + m * 100 + d;
      if (!first || k < first) first = k;
    }
    e.close();
  }
  dir.close();
  return first;
}

// in loop(): abgeschlossene Tage auswerten (einer je Durchlauf, alle 2 s)
void aggTick() {
  if (!sdOk || !timeValid() || restoringHistory || !historyRestored) return;
  if ((int32_t)(millis() - aggNextMs) < 0) return;
  aggNextMs = millis() + 2000;
  int32_t today = dayKeyOf(nowUtc());
  if (aggLast == -2) {
    Preferences p; p.begin("agg", true);
    aggLast = p.getInt("last", -1); aggFirst = p.getInt("first", 0);
    p.end();
  }
  if (aggLast < 0) {                                        // erster Start: alles nachberechnen
    int32_t first = findFirstLogDay();
    if (!first) first = today;
    aggFirst = first;
    aggLast = first;
    // Tag vor dem ersten: Startpunkt
    struct tm tmv = {}; tmv.tm_year = first / 10000 - 1900; tmv.tm_mon = (first / 100) % 100 - 1;
    tmv.tm_mday = first % 100 - 1; tmv.tm_hour = 12; tmv.tm_isdst = -1;
    aggLast = dayKeyOf((uint32_t)mktime(&tmv));
    Preferences p; p.begin("agg", false); p.putInt("first", aggFirst); p.putInt("last", aggLast); p.end();
    Log.printf("Statistik: werte Tagesdateien ab %ld aus\n", (long)aggFirst);
  }
  int32_t next = dayKeyNext(aggLast);
  if (next >= today) { aggBacklog = 0; return; }
  // Rueckstand grob schaetzen (fuer die Anzeige)
  aggBacklog = (uint16_t)min(9999UL, (unsigned long)((dayKeyStart(today) - dayKeyStart(next)) / 86400UL));
  sdFlush();                                                // Tag muss komplett auf SD sein
  processDay(next);
  aggLast = next;
  if (!aggFirst) aggFirst = next;
  Preferences p; p.begin("agg", false); p.putInt("last", aggLast); if (aggFirst) p.putInt("first", aggFirst); p.end();
}

// heutiger Tag aus dem RAM-Verlauf
void todayStat(DayStat& s) {
  int32_t key = dayKeyOf(nowUtc());
  dayReset(s, key);
  if (!logBuf) return;
  uint32_t t0 = dayKeyStart(key);
  xSemaphoreTake(logMutex, portMAX_DELAY);
  uint32_t cnt = logCount, lo = 0, hi = cnt;
  while (lo < hi) { uint32_t mid = (lo + hi) / 2; if (logAt(mid).t < t0) lo = mid + 1; else hi = mid; }
  for (uint32_t pos = lo; pos < cnt; pos++) dayAdd(s, logAt(pos));
  xSemaphoreGive(logMutex);
}

// /stats -> alle Tage aus stats.csv + heute
void handleStats() {
  server.sendHeader("Cache-Control", "no-cache");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json; charset=utf-8", "");
  String o; o.reserve(2200);
  o = "{\"ok\":true,\"sd\":"; o += sdOk ? "true" : "false";
  o += ",\"backlog\":"; o += String((unsigned)aggBacklog);
  o += ",\"days\":[";
  bool firstRow = true;
  if (sdOk && SD_MMC.exists("/log/stats.csv")) {
    File f = SD_MMC.open("/log/stats.csv", FILE_READ);
    if (f) {
      static char rbuf[2048];
      char line[240];
      int ll = 0, n;
      while ((n = f.read((uint8_t*)rbuf, sizeof(rbuf))) > 0) {
        for (int i = 0; i < n; i++) {
          char c = rbuf[i];
          if (c == '\n') {
            line[ll] = 0; ll = 0;
            if (line[0] < '0' || line[0] > '9') continue;
            if (!firstRow) o += ',';
            firstRow = false;
            o += '[';
            // Felder durchreichen, leere Felder -> null
            char* s = line; bool fst = true;
            while (true) {
              char* sep = strchr(s, ';');
              if (sep) *sep = 0;
              if (!fst) o += ',';
              fst = false;
              if (*s) o += s; else o += "null";
              if (!sep) break;
              s = sep + 1;
            }
            o += ']';
            if (o.length() > 1800) { server.sendContent(o); o = ""; }
          } else if (c != '\r' && ll < (int)sizeof(line) - 1) line[ll++] = c;
        }
        delay(0);
      }
      f.close();
    }
  }
  o += "],\"today\":";
  if (timeValid()) { DayStat s; todayStat(s); dayToJson(s, o); } else o += "null";
  o += '}';
  server.sendContent(o);
  server.sendContent("");
}

// ---------- Energiezaehler sichern ----------
void energyLoad() {
  Preferences p;
  p.begin("energy", true);
  double el = p.getDouble("elTot", 0), th = p.getDouble("thTot", 0);
  double eld = p.getDouble("elDay", 0), thd = p.getDouble("thDay", 0);
  double ely = p.getDouble("elY", 0), thy = p.getDouble("thY", 0);
  int day = p.getInt("day", -1);
  pvOwnDay = p.getDouble("pvD", 0); pvOwnYest = p.getDouble("pvY", 0); pvOwnDayIdx = p.getInt("pvDay", -1);
  p.end();
  liveLock();
  live.eElTot = el; live.eThTot = th;
  live.eElDay = eld; live.eThDay = thd;
  live.eElYest = ely; live.eThYest = thy;
  liveUnlock();
  lastEnergyDay = day;                 // anderer Tag -> wird beim ersten Lesen genullt
}

void energySaveNow() {
  LiveData d = getLive();
  Preferences p;
  p.begin("energy", false);
  p.putDouble("elTot", d.eElTot); p.putDouble("thTot", d.eThTot);
  p.putDouble("elDay", d.eElDay); p.putDouble("thDay", d.eThDay);
  p.putDouble("elY", d.eElYest); p.putDouble("thY", d.eThYest);
  p.putInt("day", lastEnergyDay);
  p.putDouble("pvD", pvOwnDay); p.putDouble("pvY", pvOwnYest); p.putInt("pvDay", pvOwnDayIdx);
  p.end();
}

void energySaveTick() {               // in loop(), alle 10 min
  static uint32_t last = 0;
  if (millis() - last < 600000UL) return;
  last = millis();
  energySaveNow();
}

#include "giga_ui.h"          // 800x480-Oberflaeche fuer das GIGA Display Shield

// =====================================================
// WLAN
// =====================================================
uint32_t lastWiFiRetryMs = 0;

void connectWiFiBlocking() {
  Log.print("Verbinde WLAN");
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);     // GIGA: blockiert, bis verbunden oder Zeitlimit

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000UL) {
    delay(500);
    Log.print(".");
  }
  Log.println();
  if (WiFi.status() == WL_CONNECTED) {
    Log.printf("WLAN verbunden\nIP: http://%s  /  http://%s.local\n",
                  WiFi.localIP().toString().c_str(), HOSTNAME);
  } else {
    Log.println("WLAN fehlgeschlagen (wird im Hintergrund weiter versucht)");
  }
}

void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWiFiRetryMs < WIFI_RETRY_MS) return;
  lastWiFiRetryMs = millis();
  Log.println("WLAN getrennt, neuer Versuch...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

// =====================================================
// Serial-Befehle (nicht-blockierend)
// =====================================================
void printHelp() {
  Log.println("Befehle:");
  Log.println("  S35      -> Vorlauf Sollwert 35 C schreiben");
  Log.println("  R        -> Sollwert 4098 lesen");
  Log.println("  ON       -> Waermepumpe ein");
  Log.println("  OFF      -> Waermepumpe aus");
  Log.println("  HEIZEN   -> Betriebsmodus Heizen");
  Log.println("  WW       -> Betriebsmodus Warmwasser");
  Log.println("  KUEHLEN  -> Betriebsmodus Kuehlen");
  Log.println("  H30      -> Max Verdichterfrequenz 30 Hz setzen");
  Log.println("  H60      -> Max Verdichterfrequenz 60 Hz setzen");
  Log.println("  F300     -> Max Luefterdrehzahl 300 rpm setzen");
  Log.println("  F400     -> Max Luefterdrehzahl 400 rpm setzen");
  Log.println("  F600     -> Max Luefterdrehzahl 600 rpm setzen");
  Log.println("  FAN      -> aktuelle Luefterdrehzahl lesen");
  Log.println("  FLOW     -> Ist-Durchfluss lesen");
  Log.println("  MFAN     -> Max Luefterdrehzahl lesen");
  Log.println("  MHZ      -> Max Verdichterfrequenz lesen");
  Log.println("  RD 8194 4 -> Register lesen (Adresse, Anzahl bis 20), nur lesen");
  Log.println("  STATUS   -> WP Status/Mode lesen");
  Log.println("  VERBOSE  -> jede Modbus-Abfrage ausgeben an/aus");
  Log.println("  TOUCH    -> Touch-Diagnose an/aus (Koordinaten + Fadenkreuz)");
  Log.println("  SWAP     -> Modbus RX/TX-Pins tauschen (speichern + Neustart)");
  Log.println("  WWLADEN  -> Warmwasser jetzt laden   WWSTOP -> Ladung abbrechen");
  Log.println("  SG1..SG4 -> SG-Ready-Zustand setzen (Test)");
  Log.println("  VHZ / VPUF / VWW -> Ventil von Hand auf Heizung / Puffer / WW");
  Log.println("  VAUTO    -> Ventil wieder nach Zeitplan");
  Log.println("  VREF     -> Ventil-Referenzfahrt (Stellung neu ermitteln)");
  Log.println("  RS485HW  -> DE/RE von der UART-Hardware schalten lassen (+ Neustart)");
  Log.println("  RS485SW  -> DE/RE per Software schalten (Standard, + Neustart)");
  Log.println("  INFO     -> System/Speicher/SD/Log-Info");
  Log.println("  SD       -> Log sofort auf SD schreiben");
}

void printInfo() {
  LiveData d = getLive();
  Log.printf("WLAN: %s  IP %s  RSSI %d dBm\n", WiFi.status() == WL_CONNECTED ? "OK" : "--",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
  Log.printf("Heap frei: %u  PSRAM frei: %u\n", ESP.getFreeHeap(), ESP.getFreePsram());
  Log.printf("Log: %lu / %lu Punkte, ungespeichert %lu\n", (unsigned long)logCount,
                (unsigned long)logCap, (unsigned long)(logTotal - sdFlushedTotal));
  Log.printf("SD: %s  %llu / %llu MB\n", sdOk ? "OK" : "--", sdUsedBytes >> 20, sdTotalBytes >> 20);
  Log.printf("Modbus: OK %lu  Fehler %lu\n", (unsigned long)d.okCount, (unsigned long)d.errCount);
}

void processSerialCmd(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("?") || cmd.equalsIgnoreCase("HELP")) { printHelp(); return; }
  if (cmd.equalsIgnoreCase("R"))       { submitCmd(CMD_READ_SOLL, 0, true); return; }
  if (cmd.equalsIgnoreCase("STATUS"))  { submitCmd(CMD_READ_ALL, 0, true); return; }
  if (cmd.equalsIgnoreCase("MHZ"))     { submitCmd(CMD_READ_MAXHZ, 0, true); return; }
  if (cmd.equalsIgnoreCase("MFAN"))    { submitCmd(CMD_READ_MAXFAN, 0, true); return; }
  if (cmd.equalsIgnoreCase("FAN"))     { submitCmd(CMD_READ_FAN, 0, true); return; }
  if (cmd.equalsIgnoreCase("FLOW"))    { submitCmd(CMD_READ_FLOW, 0, true); return; }
  if (cmd.equalsIgnoreCase("ON"))      { powerCmd(true, "Konsole"); return; }
  if (cmd.equalsIgnoreCase("OFF"))     { powerCmd(false, "Konsole"); return; }
  if (cmd.equalsIgnoreCase("HEIZEN"))  { submitCmd(CMD_MODE, 2, true); return; }
  if (cmd.equalsIgnoreCase("WW"))      { submitCmd(CMD_MODE, 1, true); return; }
  if (cmd.equalsIgnoreCase("KUEHLEN")) { submitCmd(CMD_MODE, 4, true); return; }
  if (cmd.equalsIgnoreCase("WWLADEN")) { reqWwManual = true; reqWwStart = true; return; }
  if (cmd.length() == 3 && cmd.substring(0, 2).equalsIgnoreCase("SG")) { sgSet(cmd.substring(2).toInt(), "Konsole"); return; }
  if (cmd.equalsIgnoreCase("WWSTOP"))  { reqWwStop = true; return; }
  if (cmd.equalsIgnoreCase("VHZ"))     { reqValve = 0; return; }
  if (cmd.equalsIgnoreCase("VPUF"))    { reqValve = getCtrlCfg().bufPos; return; }
  if (cmd.equalsIgnoreCase("VWW"))     { reqValve = 100; return; }
  if (cmd.equalsIgnoreCase("VAUTO"))   { reqValve = -1; return; }
  if (cmd.equalsIgnoreCase("VREF"))    { if (ctrlCfg.drive == 1) slaveRefReq = true; else reqValveRef = true; return; }
  if (cmd.equalsIgnoreCase("SWAP")) {
    int8_t t = mbCfg.rx; mbCfg.rx = mbCfg.tx; mbCfg.tx = t; saveMbConfig();
    Log.printf("RX/TX getauscht: RX=IO%d TX=IO%d -> Neustart\n", mbCfg.rx, mbCfg.tx);
    restartRequested = true; return;
  }
  if (cmd.equalsIgnoreCase("RS485HW")) { mbCfg.mode = 1; saveMbConfig(); Log.println("Hardware-RS485 an -> Neustart"); restartRequested = true; return; }
  if (cmd.equalsIgnoreCase("RS485SW")) { mbCfg.mode = 0; saveMbConfig(); Log.println("DE per Software -> Neustart"); restartRequested = true; return; }
  if (cmd.equalsIgnoreCase("TOUCH"))   { touchDebug = !touchDebug;
    Log.printf("Touch-Diagnose %s, Touch-Chip %s\n", touchDebug ? "AN" : "AUS",
                  touchChipFound ? "gefunden (0x38)" : "NICHT gefunden");
    needRedraw = true; return; }
  if (cmd.equalsIgnoreCase("VERBOSE")) { verbose = !verbose; Log.printf("Verbose %s\n", verbose ? "AN" : "AUS"); return; }
  if (cmd.equalsIgnoreCase("INFO"))    { printInfo(); return; }
  if (cmd.equalsIgnoreCase("SHELLY")) {
    Log.printf("Shelly: Quelle %s, Praefix '%s', MQTT %s, Meldungen %lu (nicht lesbar %lu), zuletzt '%s' %d Byte\n",
               elSrc ? "Shelly" : "Schaetzung", elTopic, mqttConnected ? "verbunden" : "getrennt",
               (unsigned long)shAny, (unsigned long)shBad, shLast, shLastLen);
    Log.printf("Shelly: Leistung %.0f W (vor %ld s), Zaehler %.2f kWh\n", (float)shP,
               shMs ? (long)((millis() - shMs) / 1000) : -1L, (float)shETot);
    return;
  }
  if (cmd.equalsIgnoreCase("SD"))      { sdFlush(); Log.println(sdOk ? "SD geschrieben" : "SD nicht verfuegbar"); return; }

  char c0 = toupper(cmd.charAt(0));
  if (cmd.startsWith("RD ") || cmd.startsWith("rd ")) {          // RD 8194 4  -> Register lesen
    int sp = cmd.indexOf(' ', 3);
    uint16_t addr = cmd.substring(3, sp > 0 ? sp : cmd.length()).toInt();
    rawReadN = sp > 0 ? constrain(cmd.substring(sp + 1).toInt(), 1, 20) : 1;
    submitCmd(CMD_READ_RAW, addr, true);
    return;
  }
  if (c0 == 'H') { submitCmd(CMD_MAXHZ, cmd.substring(1).toInt(), true); return; }
  if (c0 == 'F') { submitCmd(CMD_MAXFAN, cmd.substring(1).toInt(), true); return; }
  if (c0 == 'S') {
    float val = cmd.substring(1).toFloat();
    if (val >= SOLL_MIN && val <= SOLL_MAX) submitCmd(CMD_SET_SOLL, val, true);
    else Log.println("Ungueltig. Beispiel: S35 oder S45");
    return;
  }
  printHelp();
}

void handleSerial() {
  static char buf[48];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) { buf[len] = 0; processSerialCmd(String(buf)); len = 0; }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

// =====================================================
// Webseite
// =====================================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Avarma R290</title>
<style>
:root{--bg:#eef1f5;--card:#fff;--ink:#1d2430;--mut:#6b7584;--line:#e2e6ec;--acc:#2f6fdb;--acc2:#e8f0fd;
 --red:#d9473a;--blue:#3570d4;--org:#e88a1a;--grn:#2f9e5a;--yel:#c9a400;--head:#1f2733;}
*{box-sizing:border-box}
body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;background:var(--bg);color:var(--ink);font-size:15px}
header{background:var(--head);color:#fff;position:sticky;top:0;z-index:5}
.hbar{display:flex;align-items:center;gap:10px;flex-wrap:wrap;padding:10px 16px 6px}
.hbar h1{font-size:18px;margin:0 8px 0 0;font-weight:600}
.chip{font-size:12px;padding:3px 9px;border-radius:12px;background:#3a4556;color:#dfe5ee;white-space:nowrap}
.chip.on{background:var(--grn);color:#fff}.chip.warn{background:#b8860b;color:#fff}.chip.bad{background:var(--red);color:#fff}
.htime{margin-left:auto;font-size:13px;color:#b9c3d1}
nav{display:flex;gap:2px;padding:0 10px;overflow-x:auto}
nav a{color:#b9c3d1;text-decoration:none;padding:9px 14px;border-bottom:3px solid transparent;white-space:nowrap;font-size:14px}
nav a.act{color:#fff;border-bottom-color:#5b9cff}
main{max-width:1200px;margin:0 auto;padding:14px}
.tab{display:none}.tab.act{display:block}
.card{background:var(--card);border-radius:12px;padding:14px 16px;margin-bottom:14px;box-shadow:0 1px 3px rgba(0,0,0,.06)}
.card h2{font-size:15px;margin:0 0 10px;color:#3a4456;text-transform:uppercase;letter-spacing:.04em}
.card h3{font-size:14px;margin:16px 0 6px;color:#3a4456}
.ch{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:4px}
.ch h2{margin:0}
.lead{color:var(--mut);font-size:13px;margin:2px 0 10px;line-height:1.4}
.card.off .body{opacity:.45}
.card.off .ch h2::after{content:" · aus";color:var(--mut);text-transform:none;font-weight:400;letter-spacing:0}
.tiles{display:grid;grid-template-columns:repeat(auto-fill,minmax(170px,1fr));gap:10px;margin-bottom:14px}
.tile{background:var(--card);border-radius:12px;padding:10px 14px;box-shadow:0 1px 3px rgba(0,0,0,.06);border-left:4px solid var(--line)}
.tile .l{font-size:12px;color:var(--mut);text-transform:uppercase;letter-spacing:.04em}
.tile .v{font-size:26px;font-weight:600;margin-top:2px}
.tile .v small{font-size:14px;font-weight:400;color:var(--mut);margin-left:3px}
.tile .s{font-size:12px;color:var(--mut);min-height:15px}
.grid2{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:14px}
.kv{display:grid;grid-template-columns:repeat(auto-fill,minmax(200px,1fr));gap:2px 18px}
.kv div{display:flex;justify-content:space-between;gap:12px;border-bottom:1px solid var(--line);padding:5px 0;font-size:14px}
.kv b{text-align:right}
.kv div span:first-child{color:var(--mut)}
.grid4{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:4px 24px}
.form{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:4px 22px}
.form label{display:flex;align-items:center;justify-content:space-between;gap:10px;font-size:14px;border-bottom:1px solid var(--line);padding:6px 0;min-height:44px}
.form label > div em{display:block;font-style:normal;font-size:12px;color:var(--mut);margin-top:1px}
.form label > span{white-space:nowrap}
.form label.chk{justify-content:flex-start}
input,select,button{font:inherit;font-size:14px}
input,select{padding:6px 8px;border:1px solid #cdd3dc;border-radius:7px;background:#fff}
input[type=number]{width:80px}input.w{width:160px}
input[type=checkbox]{width:18px;height:18px}
input[type=time]{width:108px}
button{cursor:pointer;border:1px solid #cdd3dc;border-radius:8px;background:#f6f7f9;padding:7px 12px}
button:hover{background:#eceff3}
button.pri{background:var(--acc);border-color:var(--acc);color:#fff}
button.good{background:#e3f5ea;border-color:#9fd5b4}button.bad{background:#fbe6e4;border-color:#efb1aa}
button.sel{background:var(--acc2);border-color:#8fb1ee;color:#1d4fa8;font-weight:600}
.row{display:flex;gap:7px;flex-wrap:wrap;align-items:center;margin:6px 0}
.seg{display:inline-flex;border:1px solid #cdd3dc;border-radius:8px;overflow:hidden}
.seg button{border:0;border-radius:0;border-right:1px solid #cdd3dc}.seg button:last-child{border-right:0}
/* Schalter */
.sw{position:relative;display:inline-block;width:46px;height:26px;flex:none}
.sw input{opacity:0;width:0;height:0}
.sw span{position:absolute;inset:0;background:#c5ccd6;border-radius:13px;transition:.15s;cursor:pointer}
.sw span::before{content:"";position:absolute;width:20px;height:20px;left:3px;top:3px;background:#fff;border-radius:50%;transition:.15s;box-shadow:0 1px 2px rgba(0,0,0,.3)}
.sw input:checked + span{background:var(--grn)}
.sw input:checked + span::before{transform:translateX(20px)}
/* Schnellschalter */
.qs{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:8px;margin:8px 0}
.q{display:flex;flex-direction:column;align-items:flex-start;gap:2px;text-align:left;padding:9px 12px;border-radius:10px}
.q b{font-size:14px}.q small{font-size:12px;color:var(--mut)}
.q.on{background:#e3f5ea;border-color:#7cc79a}.q.on small{color:#2a7a48}
.q.fire.on{background:#fff0dc;border-color:#f0b060}.q.fire.on small{color:#a2620f}
.soll{display:flex;align-items:center;gap:10px;margin-top:6px}
.soll .big{font-size:26px;font-weight:600;min-width:90px;text-align:center}
.setrow{display:grid;grid-template-columns:150px 1fr auto;gap:8px;align-items:center;padding:6px 0;border-bottom:1px solid var(--line)}
.setrow .cur{color:var(--mut);font-size:13px}
.msg{font-size:13px;color:var(--mut);min-height:18px}
body:not(.giga) .gigaOnly{display:none!important}body.giga .espOnly{display:none!important}
.mut{color:var(--mut);font-size:13px}
.note{background:#f4f7fb;border-radius:8px;padding:8px 10px;font-size:13px;color:#445;margin-top:10px;line-height:1.45}
canvas{width:100%;height:380px;border-radius:8px;background:#fff}
pre{height:300px;overflow:auto;background:#141a22;color:#d6dde6;padding:10px;border-radius:8px;font-size:12px;white-space:pre-wrap;margin:8px 0}
details{border-top:1px solid var(--line);margin-top:10px;padding-top:8px}
summary{cursor:pointer;font-weight:600;color:#3a4456;padding:4px 0}
table{border-collapse:collapse;width:100%;font-size:13px}
td,th{border-bottom:1px solid var(--line);padding:4px 6px;text-align:left}th{color:var(--mut);font-weight:500}
.sched .day{display:flex;gap:8px;align-items:center;flex-wrap:wrap;padding:5px 0;border-bottom:1px solid var(--line)}
.sched .day b{width:28px}
.sched span{white-space:nowrap}
.files a{display:inline-block;min-width:150px}
.savebar{position:sticky;bottom:10px;display:none;align-items:center;gap:10px;flex-wrap:wrap;background:#fff7dc;border:1px solid #f0d27a;
 border-radius:12px;padding:10px 14px;box-shadow:0 4px 14px rgba(0,0,0,.12);margin:4px 0 14px;z-index:4}
.savebar.show{display:flex}
.savebar b{margin-right:auto}
.hide{display:none!important}
.noPv .pvOnly,.wwWp .wwV{display:none!important}
.wwWpOnly{display:none}.wwWp .wwWpOnly{display:block}
.sc{display:inline-flex;align-items:center;gap:6px;padding:5px 10px;border:1px solid #cdd3dc;border-radius:16px;font-size:13px;cursor:pointer;user-select:none;background:#fff}
.sc input{display:none}.sc i{width:12px;height:12px;border-radius:3px;border:2px solid var(--c);display:inline-block}
.sc:has(input:checked){background:#f4f7fb;border-color:var(--c)}.sc:has(input:checked) i{background:var(--c)}
.cwrap{position:relative}
.tip{position:absolute;display:none;pointer-events:none;background:rgba(31,39,51,.94);color:#fff;font-size:12px;padding:7px 10px;border-radius:8px;min-width:150px;z-index:3;line-height:1.5}
.tip i{display:inline-block;width:9px;height:9px;border-radius:2px;margin-right:6px}
.tip div{white-space:nowrap}
.vg{margin:4px 0 12px}
.vg .lb{display:flex;justify-content:space-between;align-items:baseline;font-size:13px;color:var(--mut);margin-bottom:6px}
.vg .lb b{color:var(--ink);font-size:14px}
.vg .bar{position:relative;height:22px;border-radius:11px;background:linear-gradient(90deg,#fbe3e0,#f1f3f6 50%,#fdebd3);border:1px solid #dde2e8}
.vg .fill{position:absolute;left:0;top:0;bottom:0;border-radius:11px;background:linear-gradient(90deg,#d9473a,#e88a1a);opacity:.85;transition:width .6s}
.vg .knob{position:absolute;top:50%;width:26px;height:26px;margin:-13px 0 0 -13px;border-radius:50%;background:#fff;border:3px solid #3a4456;box-shadow:0 1px 4px rgba(0,0,0,.25);transition:left .6s}
.vg .tgt{position:absolute;top:-5px;bottom:-5px;width:0;border-left:2px dashed #3570d4;display:none}
.vg .buf{position:absolute;top:100%;margin-top:2px;transform:translateX(-50%);font-size:11px;color:var(--mut);white-space:nowrap}
.vg .buf::before{content:"▲";display:block;text-align:center;font-size:9px;line-height:9px}
.vg .vsc{display:flex;justify-content:space-between;font-size:11px;color:var(--mut);margin-top:16px}
.vg.mini{margin:0}.vg.mini .bar{height:10px;border-radius:5px}.vg.mini .fill{border-radius:5px}
.vg.mini .knob{width:16px;height:16px;margin:-8px 0 0 -8px;border-width:2px}.vg.mini .vsc{margin-top:4px}
.vg.unk .knob{border-color:#b8bfc9;background:#eceff3}.vg.unk .fill{opacity:.25}
#toast{position:fixed;left:50%;bottom:24px;transform:translateX(-50%);background:#1f2733;color:#fff;padding:9px 16px;border-radius:10px;
 font-size:14px;opacity:0;transition:opacity .2s;pointer-events:none;z-index:9}
#toast.show{opacity:1}
@media(max-width:600px){header{position:static}nav a{padding:9px 10px}.setrow{grid-template-columns:1fr auto}.setrow .cur{grid-column:1/3}.tile .v{font-size:22px}.form{grid-template-columns:1fr}}
</style>
</head>
<body>
<header>
  <div class="hbar">
    <h1>Avarma R290 <span id="fwv" style="font-size:12px;font-weight:400;color:#8f9bb0"></span></h1>
    <span class="chip" id="chWp">WP -</span>
    <span class="chip" id="chMode">-</span>
    <span class="pvOnly"><span class="chip" id="chSg">SG -</span></span>
    <span class="chip" id="chWw">WW -</span>
    <span class="chip" id="chMb">Modbus -</span>
    <span class="chip" id="chMq">MQTT -</span>
    <span class="htime" id="statusTime">-</span>
  </div>
  <nav id="nav">
    <a href="#uebersicht">Übersicht</a>
    <a href="#verlauf">Verlauf</a>
    <a href="#statistik">Statistik</a>
    <a href="#heizung">Heizung</a>
    <a href="#warmwasser">Warmwasser</a>
    <a href="#verdichter">Verdichter</a>
    <a href="#pv" class="pvOnly">PV &amp; Smart Grid</a>
    <a href="#system">System</a>
  </nav>
</header>

<main>
<!-- ================= Übersicht ================= -->
<section class="tab" id="uebersicht">
  <div class="tiles">
    <div class="tile" style="border-left-color:var(--red)"><div class="l">Vorlauf</div><div class="v" id="tVl">-</div><div class="s" id="tVlS"></div></div>
    <div class="tile" style="border-left-color:var(--blue)"><div class="l">Rücklauf</div><div class="v" id="tRl">-</div><div class="s" id="tRlS"></div></div>
    <div class="tile" style="border-left-color:var(--org)"><div class="l">Warmwasser</div><div class="v" id="tWw">-</div><div class="s" id="tWwS"></div></div>
    <div class="tile" style="border-left-color:#8a4fd0"><div class="l">Raum</div><div class="v" id="tRt">-</div><div class="s" id="tRtS"></div></div>
    <div class="tile" style="border-left-color:#7a8aa0"><div class="l">Außen</div><div class="v" id="tTa">-</div><div class="s" id="tTaS"></div></div>
    <div class="tile" style="border-left-color:var(--grn)"><div class="l">Verdichter</div><div class="v" id="tHz">-</div><div class="s" id="tHzS"></div></div>
    <div class="tile" style="border-left-color:#c0504d"><div class="l">Leistung</div><div class="v" id="tPth">-</div><div class="s" id="tPthS"></div></div>
    <div class="tile" style="border-left-color:var(--yel)"><div class="l">Energie heute</div><div class="v" id="tE">-</div><div class="s" id="tES"></div></div>
    <div class="tile pvOnly" style="border-left-color:#e0b000"><div class="l">PV</div><div class="v" id="tPv">-</div><div class="s" id="tPvS"></div></div>
    <div class="tile wwV" style="border-left-color:var(--org)"><div class="l">Ventil</div>
      <div class="v" id="tVg" style="font-size:20px;margin:4px 0 6px">-</div>
      <div class="vg mini" id="tVgW"><div class="bar"><div class="fill" id="tVgFill"></div><div class="tgt" id="tVgTgt"></div><div class="knob" id="tVgKnob"></div></div>
        <div class="vsc"><span>Heizung</span><span>WW</span></div></div></div>
  </div>

  <div class="grid2">
    <div class="card">
      <h2>Bedienung</h2>
      <div class="row">
        <span class="mut" style="width:90px">Wärmepumpe</span>
        <div class="seg"><button id="bOn" onclick="setPower(1)">EIN</button><button id="bOff" onclick="setPower(0)">AUS</button></div>
      </div>
      <div class="row">
        <span class="mut" style="width:90px">Betriebsart</span>
        <div class="seg"><button id="bM2" onclick="setMode(2)">Heizen</button><button id="bM1" onclick="setMode(1)">Warmwasser</button><button id="bM4" onclick="setMode(4)">Kühlen</button></div>
      </div>
      <div class="msg" id="wpResult"></div>
      <h3>Automatik – ein Klick schaltet um</h3>
      <div class="qs">
        <button class="q" id="qHc" onclick="hcCmd('hc')"><b>Heizkurve</b><small>-</small></button>
        <button class="q" id="qRoom" onclick="hcCmd('room')"><b>Raumeinfluss</b><small>-</small></button>
        <button class="q" id="qAuto" onclick="hcCmd('auto')"><b>WP auto ein/aus</b><small>-</small></button>
        <button class="q fire" id="qKamin" onclick="hcCmd('kamin')"><b>Kamin</b><small>-</small></button>
        <button class="q" id="qFc" onclick="fcCmd('toggle')"><b>Frequenz-Automatik</b><small>-</small></button>
      </div>
      <div class="soll"><span class="mut" id="qSollL" style="width:90px">Raum-Soll</span>
        <button onclick="hcCmd('minus')">−0,5</button><span class="big" id="qSoll">-</span><button onclick="hcCmd('plus')">+0,5</button></div>
    </div>

    <div class="card">
      <h2>Warmwasser &amp; Ventil</h2>
      <div class="vg wwV" id="oVg">
        <div class="lb"><span>Heizung</span><b id="oVgTxt">-</b><span>Warmwasser</span></div>
        <div class="bar"><div class="fill" id="oVgFill"></div><div class="tgt" id="oVgTgt"></div><div class="buf" id="oVgBuf">Puffer</div><div class="knob" id="oVgKnob"></div></div>
        <div class="vsc"><span>0 %</span><span>100 %</span></div>
      </div>
      <div class="kv">
        <div class="wwV"><span>Ventil</span><b id="oValve">-</b></div>
        <div><span>WW-Ladung</span><b id="oWw">-</b></div>
        <div><span>Nächstes Ereignis</span><b id="oNext">-</b></div>
        <div class="pvOnly"><span>SG-Ready</span><b id="oSg">-</b></div>
        <div class="pvOnly"><span>Heizstab</span><b id="oHeat">-</b></div>
      </div>
      <div class="row" style="margin-top:10px">
        <button class="good" onclick="ctrlCmd('wwstart')">Warmwasser jetzt laden</button>
        <button class="bad" onclick="ctrlCmd('wwstop')">Ladung abbrechen</button>
      </div>
      <div class="msg" id="ctrlCmdMsg"></div>
    </div>
  </div>

  <div class="card">
    <h2>Messwerte</h2>
    <div class="grid4">
      <div><h3>Wasserkreis</h3><div class="kv" id="gWater"></div></div>
      <div><h3>Kältekreis</h3><div class="kv" id="gRef"></div></div>
      <div><h3>Elektrik</h3><div class="kv" id="gEl"></div></div>
      <div><h3>Zähler &amp; Energie</h3><div class="kv" id="gCnt"></div></div>
    </div>
  </div>
</section>

<!-- ================= Verlauf ================= -->
<section class="tab" id="verlauf">
  <div class="card">
    <div class="row" id="rangeBar">
      <span class="mut" style="width:70px">Zeitraum</span>
      <div class="seg">
        <button data-h="1" onclick="setWindowHours(1)">1 h</button><button data-h="2" onclick="setWindowHours(2)">2 h</button>
        <button data-h="8" onclick="setWindowHours(8)">8 h</button><button data-h="24" onclick="setWindowHours(24)">24 h</button>
        <button data-h="72" onclick="setWindowHours(72)">3 T</button><button data-h="168" onclick="setWindowHours(168)">7 T</button>
        <button data-h="336" onclick="setWindowHours(336)">14 T</button><button data-h="720" onclick="setWindowHours(720)">30 T</button>
        <button data-h="2160" onclick="setWindowHours(2160)">3 Mon</button><button data-h="8760" onclick="setWindowHours(8760)">1 Jahr</button>
        <button data-h="all" onclick="setWindowHours('all')">Alles</button>
      </div>
      <div class="seg"><button onclick="monthStep(-1)">◀</button><button id="monthBtn" onclick="monthStep(0)">Monat</button><button onclick="monthStep(1)">▶</button></div>
      <button onclick="loadHistory()">Neu laden</button>
    </div>
    <div class="row" id="serBar"></div>
    <div id="viewInfo" class="mut">-</div>
    <div class="cwrap"><canvas id="chart" width="1100" height="420" style="height:420px"></canvas><div class="tip" id="tip"></div></div>
    <div class="mut">Hellblau hinterlegt = Abtauen. Längere Zeiträume als 14 Tage kommen als Stundenmittel von der SD-Karte.</div>
  </div>
</section>

<!-- ================= Statistik ================= -->
<section class="tab" id="statistik">
  <div class="card">
    <div class="row">
      <div class="seg"><button id="stTd" onclick="stMode('today')">Heute</button><button id="stYd" onclick="stMode('yest')">Gestern</button><button id="stW" onclick="stMode('w')">Woche</button><button id="stM" onclick="stMode('m')">Monat</button><button id="stY" onclick="stMode('y')">Jahr</button><button id="stR" onclick="stMode('r')">Zeitraum</button></div>
      <div class="seg"><button onclick="stStep(-1)">◀</button><button id="stTitle" style="min-width:110px;font-weight:600">-</button><button onclick="stStep(1)">▶</button></div>
      <div id="stRangeBox" style="display:none;gap:6px;align-items:center;flex-wrap:wrap">von <input type="date" id="stFrom" onchange="stRange()"> bis <input type="date" id="stTo" onchange="stRange()"></div>
      <span class="mut" id="stInfo"></span>
    </div>
    <div class="tiles" id="stTiles" style="margin-top:10px"></div>
    <div class="cwrap"><canvas id="stChart" width="1100" height="340" style="height:340px"></canvas><div class="tip" id="stTip"></div></div>
    <div class="row mut" style="gap:16px">
      <span><i style="display:inline-block;width:12px;height:12px;background:#f0a347;border-radius:2px"></i> Wärme Heizen</span>
      <span><i style="display:inline-block;width:12px;height:12px;background:#f7cf9a;border-radius:2px"></i> Wärme Warmwasser</span>
      <span><i style="display:inline-block;width:12px;height:12px;background:#4a7fd8;border-radius:2px"></i> Strom Heizen</span>
      <span><i style="display:inline-block;width:12px;height:12px;background:#a9c3f0;border-radius:2px"></i> Strom Warmwasser</span>
      <span><i style="display:inline-block;width:12px;height:3px;background:#2f9e5a;vertical-align:middle"></i> Arbeitszahl (rechte Achse)</span>
    </div>
  </div>
  <div class="grid2">
    <div class="card">
      <h2>Heizwärme je Tag über Außentemperatur</h2>
      <canvas id="stScatter" width="560" height="300" style="height:300px"></canvas>
      <div class="mut">Jeder Punkt ist ein Tag des gewählten Jahres (ohne Warmwasser), der gewählte Monat ist orange.
        So siehst du, wie viel Wärme das Haus je Außentemperatur braucht – und ob sich das über die Zeit ändert.</div>
    </div>
    <div class="card">
      <h2>Hinweise</h2>
      <div class="form" style="grid-template-columns:1fr">
        <label><div>Strompreis für die Kostenschätzung</div><span><input id="stPrice" type="number" step="0.01" min="0" max="2" onchange="savePrice()"> €/kWh</span></label>
      </div>
      <div class="note">Strom kommt vom Shelly-Zähler oder ist geschätzt (Verdichterstrom × Spannung) – siehe System → Elektrische Leistung. Wärme ist aus
        Durchfluss und Spreizung berechnet. Ein Tag wird kurz nach Mitternacht ausgewertet; „heute“ wird laufend mitgerechnet.
        „Aus eigener Anlage“ zählt WP-Strom, der nicht aus dem Netz kam (PV oder Batterie) – dafür braucht es die Netzdaten per MQTT.
        Die Tageswerte stehen in <a href="/download?f=stats.csv">stats.csv</a> (Excel).</div>
    </div>
  </div>
  <div class="card"><h2>Tabelle</h2><div style="overflow-x:auto"><table id="stTbl"></table></div></div>
</section>

<!-- ================= Heizung ================= -->
<section class="tab" id="heizung">
  <div class="grid2">
    <div class="card">
      <h2>Raum</h2>
      <div class="tiles" style="grid-template-columns:repeat(2,1fr);margin-bottom:8px">
        <div class="tile" style="border-left-color:var(--org)"><div class="l">Raum ist</div><div class="v" id="hRt">-</div><div class="s" id="hRh"></div></div>
        <div class="tile" style="border-left-color:var(--acc)"><div class="l" id="hSollL">Raum soll</div><div class="v" id="hSoll">-</div>
          <div class="s"><button onclick="hcCmd('minus')">−0,5</button> <button onclick="hcCmd('plus')">+0,5</button></div></div>
      </div>
    </div>
    <div class="card">
      <h2>So rechnet die Regelung gerade</h2>
      <div class="kv" id="hcKv"></div>
    </div>
  </div>

  <div data-grp="hc">
  <div class="card" data-sw="hEn">
    <div class="ch"><h2>1 · Heizkurve</h2><label class="sw"><input type="checkbox" id="hEn"><span></span></label></div>
    <div class="lead">Ein: der ESP berechnet den Vorlauf-Soll aus der Außentemperatur und schreibt ihn an die WP.
      Aus: der VL-Soll bleibt, wie er an der WP steht (von Hand unten, per Display oder per MQTT).</div>
    <div class="body">
      <div class="grid2">
        <div>
          <table id="curveTbl"></table>
          <div class="mut" style="margin-top:6px">Außentemperatur von warm nach kalt. Dazwischen wird gerade verbunden,
          wärmer/kälter als der erste/letzte Punkt gilt dessen Wert.</div>
        </div>
        <canvas id="curveCv" width="520" height="300" style="height:300px"></canvas>
      </div>
      <div class="form" style="margin-top:8px">
        <label><div>Vorlauf nie unter / über<em>Grenzen für alles, was die Regelung schreibt</em></div>
          <span><input id="hVlMin" type="number" min="15" max="60"> – <input id="hVlMax" type="number" min="20" max="65"> °C</span></label>
        <label><div>Außentemperatur glätten<em>kurze Schwankungen ignorieren; beim Abtauen und 15 min danach wird der Wert gehalten</em></div>
          <span><input id="hTau" type="number" min="0" max="240"> min</span></label>
      </div>
    </div>
  </div>

  <div class="card" data-sw="hRoom">
    <div class="ch"><h2>2 · Raumeinfluss</h2><label class="sw"><input type="checkbox" id="hRoom"><span></span></label></div>
    <div class="lead">Korrigiert den VL der Heizkurve mit dem Raumfühler: ist es zu kalt, wird der VL angehoben, ist es zu warm, abgesenkt.
      Beispiel Stärke 3: Raum 0,5 K zu kalt → VL +1,5 K.</div>
    <div class="body form">
      <label><div>Raum-Soll (Tag)</div><span><input id="hDay" type="number" step="0.5" min="10" max="28"> °C</span></label>
      <label><div>Stärke<em>K Vorlauf je K Raumabweichung</em></div><span><input id="hK" type="number" step="0.5" min="0" max="10"> K/K</span></label>
      <label><div>Höchstens ±<em>maximale Korrektur des Vorlaufs</em></div><span><input id="hMax" type="number" min="0" max="15"> K</span></label>
      <label><div>Langsame Nachregelung (I-Anteil)<em>gleicht bleibende Abweichung aus, 0 = aus</em></div><span><input id="hKi" type="number" step="0.5" min="0" max="10"> K/K·h</span></label>
    </div>
  </div>

  <div class="card" data-sw="hNight">
    <div class="ch"><h2>3 · Nachtabsenkung</h2><label class="sw"><input type="checkbox" id="hNight"><span></span></label></div>
    <div class="lead">Nachts gilt ein niedrigerer Raum-Soll und der VL wird zusätzlich verschoben. Die Zeit gilt auch für die nächtliche Frequenzgrenze (Tab Verdichter).</div>
    <div class="body form">
      <label><div>Nachtzeit<em>Mo–Do abends bzw. Di–Fr morgens</em></div><span><input id="hNs" type="time"> bis <input id="hNe" type="time"></span></label>
      <label class="chk"><input type="checkbox" id="hNwOn"> Am Wochenende eigene Zeiten</label>
      <label class="nw"><div>Beginn Fr + Sa abends<em>Sonntag abends gilt wieder die normale Zeit</em></div><span><input id="hNws" type="time"></span></label>
      <label class="nw"><div>Ende Sa + So morgens<em>Montag morgens gilt wieder die normale Zeit</em></div><span><input id="hNwe" type="time"></span></label>
      <label><div>Raum-Soll nachts</div><span><input id="hNightSoll" type="number" step="0.5" min="10" max="28"> °C</span></label>
      <label class="ncOff"><div>Vorlauf nachts verschieben<em>negativ = kühler</em></div><span><input id="hNshift" type="number" min="-15" max="5"> K</span></label>
      <label class="chk"><input type="checkbox" id="hNcOn" onchange="applyDeps();drawNCurve()"> Verschiebung nach Außentemperatur (eigene Kurve statt festem Wert)</label>
    </div>
    <div class="body nc" style="margin-top:4px">
      <div class="grid2">
        <div>
          <table id="nCurveTbl"></table>
          <div class="mut" style="margin-top:6px">Verschiebung des Vorlaufs nachts in K, negativ = kühler. Außentemperatur von warm nach kalt,
          dazwischen wird gerade verbunden. Typisch: wenn es mild ist, kräftig absenken – wenn es kalt ist, wenig, damit das Haus morgens nicht auskühlt.</div>
        </div>
        <canvas id="nCurveCv" width="520" height="260" style="height:260px"></canvas>
      </div>
    </div>
    <div class="body form" style="margin-top:4px">
      <label><div><input type="checkbox" id="hNoffOn" onchange="applyDeps()"> WP nachts ganz aus, wenn außen wärmer als
        <em>geglättete Außentemperatur, 1 K Hysterese · nur in der Nachtzeit · eine laufende WW-Ladung wird fertig geladen</em></div>
        <span><input id="hNoffTa" type="number" min="-10" max="25"> °C</span></label>
    </div>
  </div>

  <div class="card" data-sw="hAuto">
    <div class="ch"><h2>4 · WP automatisch ein- und ausschalten</h2><label class="sw"><input type="checkbox" id="hAuto"><span></span></label></div>
    <div class="lead">Schaltet die WP je nach Außen- und/oder Raumtemperatur. Schaltest du von Hand, bleibt das so, bis sich die Bedingung das nächste Mal ändert.</div>
    <div class="body">
      <div class="form">
        <label><div>Entscheidet nach</div><select id="hMode" onchange="applyDeps()"><option value="1">Außentemperatur</option><option value="2">Raumtemperatur</option><option value="3">Außen und Raum</option></select></label>
        <label class="m-ta"><div>Einschalten, wenn kälter als<em>Außen, geglättet</em></div><span><input id="hTaOn" type="number" min="-20" max="25"> °C</span></label>
        <label class="m-ta"><div>Ausschalten, wenn wärmer als</div><span><input id="hTaOff" type="number" min="-20" max="30"> °C</span></label>
        <label class="m-rt"><div>Einschalten, wenn Raum kälter als Soll −</div><span><input id="hRtOn" type="number" step="0.1" min="0" max="5"> K</span></label>
        <label class="m-rt"><div>Ausschalten, wenn Raum wärmer als Soll +</div><span><input id="hRtOff" type="number" step="0.1" min="0" max="5"> K</span></label>
        <label><div>Mindestens laufen</div><span><input id="hMinOn" type="number" min="0" max="240"> min</span></label>
        <label><div>Mindestens Pause</div><span><input id="hMinOff" type="number" min="0" max="240"> min</span></label>
      </div>
      <h3>Kamin-Modus</h3>
      <div class="mut">Ein-/Ausschalten am Display, in der Übersicht oder per MQTT. Solange der Kamin an ist, gelten tiefere Grenzen:</div>
      <div class="form">
        <label class="m-ta"><div>Außen-Grenzen tiefer um</div><span><input id="hKTa" type="number" min="0" max="15"> K</span></label>
        <label class="m-rt"><div>Raum-Soll tiefer um</div><span><input id="hKRt" type="number" step="0.5" min="0" max="5"> K</span></label>
        <label><div><input type="checkbox" id="hKMaxOn" onchange="applyDeps()"> Kamin-Modus automatisch beenden nach</div><span><input id="hKMax" type="number" min="1" max="24"> h</span></label>
      </div>
    </div>
  </div>

  <div class="card" data-sw="lkEn">
    <div class="ch"><h2>5 · Sperrzeiten – WP komplett aus</h2><label class="sw"><input type="checkbox" id="lkEn"><span></span></label></div>
    <div class="lead">In diesen Zeiten bleibt die WP aus, egal wie warm oder kalt es ist – mit und ohne Auto-WP.
      Je Tag zwei Zeiträume. Über Mitternacht geht (z. B. 23:00 bis 04:00, gilt dann ab dem eingetragenen Tag). Gleiche Zeit bei „von“ und „bis“ = kein Zeitraum.
      Von Hand einschalten geht trotzdem – das gilt dann bis zum Ende der Sperre, danach entscheidet die Automatik wieder.
      Eine laufende WW-Ladung wird noch fertig geladen.</div>
    <div class="body">
      <div class="grid2">
        <div style="overflow-x:auto"><table id="lkTbl"></table>
          <div class="row" style="margin-top:6px"><button onclick="lkCopy(7)">Montag für alle Tage übernehmen</button>
            <button onclick="lkCopy(5)">Montag für Mo–Fr übernehmen</button></div></div>
        <div>
          <div class="kv"><div><span>Zustand</span><b id="lkState">-</b></div></div>
          <div class="form" style="grid-template-columns:1fr;margin-top:8px">
            <label><div><input type="checkbox" id="lkCold"> Trotzdem heizen, wenn außen kälter als<em>geglättete Außentemperatur, 1 K Hysterese</em></div>
              <span><input id="lkTa" type="number" min="-25" max="20"> °C</span></label>
            <label class="chk"><input type="checkbox" id="lkWw"> Warmwasser nach Zeitplan / PV auch in der Sperrzeit laden</label>
          </div>
          <div class="mut" style="margin-top:6px">„Warmwasser jetzt“ (Taste) lädt immer.</div>
        </div>
      </div>
    </div>
  </div>

  <div class="card">
    <details>
      <summary>Für Fortgeschrittene: Schreibschutz &amp; Raumfühler</summary>
      <div class="lead" style="margin-top:6px">Die WP speichert jeden geschriebenen Wert dauerhaft. Damit ihr Speicher lange hält, schreibt der ESP
        nur bei echter Abweichung, nur bei laufender WP (sonst direkt beim Einschalten) und nicht zu oft.</div>
      <div class="form">
        <label><div>VL erst neu schreiben ab<em>Abweichung zum Wert an der WP</em></div><span><input id="hDb" type="number" step="0.5" min="0.5" max="5"> K</span></label>
        <label><div>VL höchstens alle</div><span><input id="hWs" type="number" min="60" max="3600"> s</span></label>
        <label><div>Raumfühler an IO<em>−1 = kein Fühler</em></div><input id="hPin" type="number" min="-1" max="48"></label>
        <label><div>Fühler-Typ</div><select id="hType"><option value="22">DHT22 / AM2302</option><option value="11">DHT11</option><option value="40" class="gigaOnly">SHT4x (I2C: SDA D20, SCL D21)</option></select></label>
        <label><div>Fühler-Korrektur</div><span><input id="hOff" type="number" step="0.1" min="-5" max="5"> K</span></label>
      </div>
    </details>
    <details>
      <summary>VL-Soll von Hand an die WP schreiben</summary>
      <div class="setrow"><span>VL-Sollwert</span><span class="cur" id="curSoll">-</span>
        <span><input id="sollInput" type="number" min="10" max="75" step="1"> °C <button class="pri" onclick="setSollwert()">Setzen</button></span></div>
      <div class="msg" id="sollResult"></div>
      <div class="mut">Bei eingeschalteter Heizkurve wird der Wert beim nächsten Regelschritt wieder überschrieben.</div>
    </details>
    <div class="note">MQTT: Raum-Soll <b>avarma/cmd/rtsoll</b> · Heizkurve <b>avarma/cmd/hc</b> 0/1 · Raumeinfluss <b>avarma/cmd/room</b> 0/1 ·
      Auto-WP <b>avarma/cmd/autowp</b> 0/1 · Kamin <b>avarma/cmd/kamin</b> 0/1 · Sperrzeiten <b>avarma/cmd/sperre</b> 0/1.
      Bei eingeschalteter Heizkurve werden VL-Befehle per MQTT ignoriert.</div>
  </div>
  </div>
  <div class="savebar" id="sb_hc"><b>Heizung: Änderungen noch nicht gespeichert</b>
    <span class="msg" id="hcMsg"></span>
    <button onclick="loadHc(true)">Verwerfen</button><button class="pri" onclick="saveHc()">Speichern</button></div>
</section>

<!-- ================= Warmwasser ================= -->
<section class="tab" id="warmwasser">
  <div class="card">
    <h2>Jetzt</h2>
    <div class="vg wwV" id="wVg">
        <div class="lb"><span>Heizung</span><b id="wVgTxt">-</b><span>Warmwasser</span></div>
        <div class="bar"><div class="fill" id="wVgFill"></div><div class="tgt" id="wVgTgt"></div><div class="buf" id="wVgBuf">Puffer</div><div class="knob" id="wVgKnob"></div></div>
        <div class="vsc"><span>0 %</span><span>100 %</span></div>
      </div>
    <div class="kv" id="ctrlKv"></div>
    <div class="row" style="margin-top:10px">
      <button class="good" onclick="ctrlCmd('wwstart')">Warmwasser jetzt laden</button>
      <button class="bad" onclick="ctrlCmd('wwstop')">Ladung abbrechen</button>
    </div>
    <div class="row wwV">
      <span class="mut">Ventil von Hand (gilt bis zum nächsten Zeitplan-Ereignis):</span>
      <div class="seg">
        <button onclick="ctrlCmd('vheiz')">Heizung</button><button onclick="ctrlCmd('vpuf')">Puffer</button>
        <button onclick="ctrlCmd('vww')">WW</button><button onclick="ctrlCmd('vauto')">Automatik</button>
      </div>
      <button onclick="ctrlCmd('vref')">Referenzfahrt</button>
    </div>
    <div class="msg" id="ctrlCmdMsg2"></div>
  </div>

  <div data-grp="ctrl">
  <div class="card" data-sw="cEnabled">
    <div class="ch"><h2>Ventil &amp; Warmwasser-Ladung</h2><label class="sw"><input type="checkbox" id="cEnabled"><span></span></label></div>
    <div class="lead wwWpOnly">Lädt den Speicher über den <b>WW-Modus der WP</b>: Zur Ladung (Zeitplan, „Warmwasser jetzt“, PV) stellt der
      ESP die WP auf Warmwasser – die WP schaltet ihr eigenes 3-Wege-Ventil und heizt bis zu ihrem eigenen WW-Soll. Ist „Warmwasser laden bis“
      erreicht oder schaltet die WP im WW-Modus ab, stellt der ESP wieder auf Heizen. Den WW-Soll stellst du an der WP ein (mindestens so hoch wie hier).</div>
    <div class="lead wwV">Steuert das ESBE-Ventil und lädt den Speicher: Ventil auf WW, VL-Soll auf WW-Soll + Zuschlag,
      bis der Speicher warm ist. Danach wird der alte VL-Soll wiederhergestellt. Erst einschalten, wenn das Ventil angeschlossen ist.</div>
    <div class="body form">
      <label><div>Warmwasser über</div><select id="cWwMode" onchange="FEAT.ww=+this.value;applyFeat()">
        <option value="0">Umschaltventil vom ESP (ESBE)</option><option value="1">WW-Modus der WP</option></select></label>
      <label><div>Warmwasser laden bis</div><span><input id="cWwSoll" type="number" min="30" max="65"> °C</span></label>
      <label class="wwV"><div>Vorlauf beim Laden = WW-Soll +</div><span><input id="cBoost" type="number" min="0" max="15"> K</span></label>
      <label class="wwV"><div>VL-Soll beim Laden höchstens<em>wird angehoben, wenn der VL den Soll erreicht, bevor das WW warm ist – sonst schaltet der Verdichter ab</em></div><span><input id="cVlMax" type="number" min="40" max="75"> °C</span></label>
      <label><div>WW-Ladung vorziehen um bis zu<em>wenn beim Heizen der VL den Soll erreicht und der Verdichter gleich abschalten würde · 0 = aus</em></div><span><input id="cPull" type="number" min="0" max="120"> min</span></label>
      <label><div>Ladung abbrechen nach</div><span><input id="cMaxMin" type="number" min="10" max="480"> min</span></label>
      <label class="wwV"><div>Ventil nach der Ladung noch auf WW<em>Restwärme in den Speicher</em></div><span><input id="cCool" type="number" min="0" max="900"> s</span></label>
      <label class="chk"><input type="checkbox" id="cWpOn"> WP für die Ladung einschalten, wenn sie aus ist</label>
      <label class="chk wwV"><input type="checkbox" id="cCont"> Nach der Ladung direkt weiterheizen: Ventil sofort auf Heizung, Verdichter läuft durch
        (VL-Soll wird erst umgestellt, wenn das Heizungswasser angekommen ist). Gilt, wenn danach geheizt werden soll – sonst Nachlauf wie oben.</label>
      <label class="wwV"><div>Ventil, wenn kein Plan-Eintrag gilt</div><select id="cDefAct"><option value="3">Heizung</option><option value="2">Puffer</option></select></label>
    </div>
    <div class="wwV">
    <h3 style="margin:14px 0 4px">Puffer-Automatik</h3>
    <div class="lead">Statt nur nach Uhrzeit: Nach „WW laden“ bleibt das Ventil auf Heizung, bis der Speicher abgekühlt ist – dann Puffer.
      Gilt nur, wenn die Heizung läuft (WP an, Heizbetrieb). WP aus = immer Heizung, damit nichts durch den Speicher zirkuliert.
      Plan-Einträge bleiben fest: „Puffer“ (z. B. 02:00) fährt auf jeden Fall auf Puffer, „Heizung“ auf Heizung.</div>
    <div class="body form">
      <label><div><input type="checkbox" id="cAbFix" onchange="applyDeps()"> Puffer, wenn das WW auf … gefallen ist<em>feste Temperatur, unabhängig vom VL-Soll · danach Puffer bis zur nächsten WW-Ladung</em></div>
        <span><input id="cAbFixT" type="number" min="20" max="60"> °C</span></label>
      <label><div><input type="checkbox" id="cAbOn" onchange="applyDeps()"> Puffer, wenn WW höchstens … über dem Heiz-VL-Soll<em>VL-Soll aus der Heizkurve bzw. der WP · danach Puffer bis zur nächsten WW-Ladung</em></div>
        <span><input id="cAbK" type="number" min="-5" max="15"> K</span></label>
      <label class="chk"><input type="checkbox" id="cAbLow"> Puffer, sobald das WW unter den Heiz-VL-Soll fällt (Heizung wärmt dann den Speicher mit)</label>
      <div class="mut">Sind mehrere Haken gesetzt, schaltet die Bedingung, die zuerst erfüllt ist.</div>
    </div>
    </div>
  </div>

  <div class="card">
    <h2>Wochenplan</h2>
    <div class="lead"><b>WW laden</b>: Speicher auf WW-Soll laden, danach Heizung ·
      <b>Puffer</b>: Speicher puffert die Heizung · <b>Heizung</b>: Speicher abgekoppelt. Leere Zeilen werden ignoriert.</div>
    <div class="mut wwWpOnly">Mit dem WW-Modus der WP zählen nur die Einträge „WW laden“.</div>
    <div id="schedBox" class="sched"></div>
    <div class="row">
      <button onclick="copyDay(0,[1,2,3,4])">Mo → Di–Fr</button>
      <button onclick="copyDay(0,[1,2,3,4,5,6])">Mo → alle</button>
      <button onclick="copyDay(5,[6])">Sa → So</button>
    </div>
  </div>

  <div class="card wwV">
    <details>
      <summary>Ventil-Hardware</summary>
      <div class="form">
        <label><div>Ansteuerung</div><select id="cDrive" onchange="applyDeps()"><option value="0">Relais am ESP</option><option value="1">Ventilmodul am Modbus</option></select></label>
        <label><div>Laufzeit Endlage → Endlage</div><span><input id="cTravel" type="number" min="10" max="600"> s</span></label>
        <label><div>Puffer-Stellung</div><span><input id="cBuf" type="number" min="5" max="95"> %</span></label>
        <label class="d-mod"><div>Ventilmodul Modbus-ID</div><input id="cSlave" type="number" min="2" max="247"></label>
        <label class="d-mod"><div>Modul fährt auf Heizung, wenn ESP weg seit<em>0 = nie</em></div><span><input id="cFsMin" type="number" min="0" max="240"> min</span></label>
        <label class="d-rel"><div>Relais Motorstrom an IO</div><input id="cPwr" type="number"></label>
        <label class="d-rel"><div>Relais Richtung an IO</div><input id="cDir" type="number"></label>
        <label class="chk d-rel"><input type="checkbox" id="cActLow"> Relais schalten bei LOW</label>
        <label class="chk"><input type="checkbox" id="cDirInv"> Richtung vertauschen</label>
      </div>
      <div class="mut">Relais am ESP: freie IOs 3, 43, 44. Ventilmodul: Arduino mit Sketch ESBE_Slave_R4 am RS485-Bus. Änderung von Pins/Ansteuerung = Neustart.</div>
    </details>
  </div>
  </div>
  <div class="savebar" id="sb_ctrl"><b>Warmwasser: Änderungen noch nicht gespeichert</b>
    <span class="msg" id="ctrlMsg"></span>
    <button onclick="loadCtrl(true)">Verwerfen</button><button class="pri" onclick="saveCtrl()">Speichern</button></div>
</section>

<!-- ================= Verdichter ================= -->
<section class="tab" id="verdichter">
  <div class="grid2">
    <div class="card">
      <h2>Verdichterfrequenz</h2>
      <div class="tiles" style="grid-template-columns:repeat(2,1fr);margin-bottom:8px">
        <div class="tile" style="border-left-color:var(--grn)"><div class="l">Verdichter</div><div class="v" id="fHz">-</div><div class="s" id="fMax"></div></div>
        <div class="tile" style="border-left-color:var(--acc)"><div class="l" id="fTgtL">Kurve</div><div class="v" id="fTgt">-</div><div class="s" id="fInfo"></div></div>
      </div>
    </div>
    <div class="card">
      <h2>Status</h2>
      <div class="kv" id="fcKv"></div>
    </div>
  </div>

  <div data-grp="fc">
  <div class="card" data-sw="fEn">
    <div class="ch"><h2>Frequenz-Automatik</h2><label class="sw"><input type="checkbox" id="fEn"><span></span></label></div>
    <div class="lead">Begrenzt die maximale Verdichterfrequenz (P45) abhängig von der Außentemperatur – getrennt für Heizen und Warmwasser.
      Aus: die Grenze bleibt, wie sie an der WP steht (von Hand unten oder am Display).</div>
    <div class="body">
      <div class="grid2">
        <div><table id="fcTbl"></table>
          <div class="mut" style="margin-top:6px">Die WW-Spalte gilt, solange Warmwasser geladen wird oder die WP im Modus Warmwasser steht.</div></div>
        <canvas id="fcCv" width="520" height="300" style="height:300px"></canvas>
      </div>
      <div class="form" style="margin-top:8px">
        <label><div><input type="checkbox" id="fNightOn" onchange="applyDeps()"> Nachts (Heizen) höchstens<em>leiser – Nachtzeit aus Tab Heizung</em></div>
          <span><input id="fNight" type="number" min="30" max="85"> Hz</span></label>
        <label><div>Erst neu schreiben ab<em>Abweichung zum Wert an der WP</em></div><span><input id="fDb" type="number" min="1" max="15"> Hz</span></label>
        <label><div>Höchstens alle<em>außer beim Einschalten und Wechsel Heizen ↔ WW</em></div><span><input id="fInt" type="number" min="5" max="240"> min</span></label>
      </div>
    </div>
  </div>
  <div class="card" data-sw="fOffOn">
    <div class="ch"><h2>Sanft ausschalten</h2><label class="sw"><input type="checkbox" id="fOffOn"><span></span></label></div>
    <div class="lead">Läuft der Verdichter beim Ausschalten noch schneller als der Wert unten, wird zuerst die Frequenzgrenze darauf gesenkt
      und erst dann AUS geschickt (spätestens nach 90 s). Steht der Verdichter, wird die Grenze zurückgestellt. Kostet zwei Schreibvorgänge je Ausschalten.
      Unabhängig davon prüft der ESP nach jedem AUS 15 min lang, ob die WP wirklich ausgeht, und schreibt sonst Diagnosezeilen in die Konsole.</div>
    <div class="body form">
      <label><div>Vor dem Ausschalten absenken auf</div><span><input id="fOffHz" type="number" min="30" max="60"> Hz</span></label>
    </div>
  </div>
  <div class="card" data-sw="fDfOn">
    <div class="ch"><h2>Abtauen beschleunigen</h2><label class="sw"><input type="checkbox" id="fDfOn"><span></span></label></div>
    <div class="lead">Sobald die WP das Abtau-Bit meldet, wird die maximale Verdichterfrequenz auf den Wert unten angehoben und nach
      dem Abtauen wieder zurückgestellt (auf den Kurvenwert bzw. den Wert von vorher). Das kostet zwei Schreibvorgänge je Abtauung.</div>
    <div class="body form">
      <label><div>Max. Frequenz beim Abtauen</div><span><input id="fDfHz" type="number" min="30" max="85"> Hz</span></label>
      <label><div>Zurückstellen nach Abtau-Ende<em>kurz warten, falls gleich noch einmal abgetaut wird</em></div><span><input id="fDfBack" type="number" min="0" max="1800"> s</span></label>
    </div>
  </div>
  <div class="card" data-sw="fFbOn">
    <div class="ch"><h2>Frost-Bremse</h2><label class="sw"><input type="checkbox" id="fFbOn"><span></span></label></div>
    <div class="lead">Gegen häufiges Abtauen: Liegt die Außentemperatur im Bereich unten und ist der Verdampfer beim Heizen kälter als der Grenzwert,
      wird die Frequenzgrenze alle 5 min um einen Schritt gesenkt – nur solange der Vorlauf sein Soll (bis 1 K darunter) noch erreicht.
      Fällt der Verdampfer schnell (1,5 K in einer Minute), kommt der Schritt sofort.
      Ein langsamerer Verdichter hält den Verdampfer wärmer, es setzt sich weniger Eis an. Wird der Verdampfer 2 K wärmer als der Grenzwert
      oder fällt der Vorlauf mehr als 3 K unter Soll, geht die Grenze schrittweise zurück. Nach dem Verdichterstart und nach jedem Abtauen
      wird 5 min nur beobachtet. Wirkt nur beim Heizen und nur mit eingeschalteter Frequenz-Automatik; jeder Schritt ist ein Schreibvorgang.
      Ob es etwas bringt, zeigt die Abtau-Auswertung unten.</div>
    <div class="body form">
      <label><div>Wirkt bei Außentemperatur</div><span><input id="fFbLo" type="number" min="-10" max="10" style="width:60px"> bis <input id="fFbHi" type="number" min="-5" max="15" style="width:60px"> °C</span></label>
      <label><div>Bremsen, wenn kälter als<em>geglättet über ca. 30 s</em></div><span><select id="fFbSrc"><option value="0">Verdampfer</option><option value="1">Coil</option></select> <input id="fFbEv" type="number" min="-10" max="5" style="width:60px"> °C</span></label>
      <label><div>Schrittweite</div><span><input id="fFbStep" type="number" min="2" max="15"> Hz</span></label>
      <label><div>Nicht tiefer als</div><span><input id="fFbMin" type="number" min="30" max="70"> Hz</span></label>
    </div>
  </div>
  </div>
  <div class="savebar" id="sb_fc"><b>Verdichter: Änderungen noch nicht gespeichert</b>
    <span class="msg" id="fcMsg"></span>
    <button onclick="loadFc(true)">Verwerfen</button><button class="pri" onclick="saveFc()">Speichern</button></div>

  <div class="card">
    <div class="ch"><h2>Abtau-Auswertung</h2><button onclick="loadDfl()">Aktualisieren</button></div>
    <div class="lead">Jede Abtauung mit den Bedingungen kurz davor (Mittel der letzten Minuten) und der Verdichter-Laufzeit seit der letzten
      Abtauung. Längere Laufzeit zwischen zwei Abtauungen bei gleicher Außentemperatur heißt: weniger Eis. Die Rohdaten stehen in
      <a href="/download?f=abtau.csv">abtau.csv</a>.</div>
    <div class="grid2">
      <div style="overflow-x:auto"><table id="dfBin" style="white-space:nowrap"></table></div>
      <canvas id="dfCv" width="520" height="280" style="height:280px"></canvas>
    </div>
    <div style="overflow-x:auto;margin-top:10px"><table id="dfTbl" style="white-space:nowrap"></table></div>
    <div class="mut" id="dfMsg" style="margin-top:6px"></div>
  </div>

  <div class="card">
    <h2>Von Hand an die WP schreiben</h2>
    <div class="setrow"><span>Max. Verdichter</span><span class="cur" id="curHz">-</span>
      <span><input id="maxHzInput" type="number" min="30" max="85" step="1"> Hz <button class="pri" onclick="setMaxHz()">Setzen</button></span></div>
    <div class="setrow"><span>Max. Lüfter</span><span class="cur" id="curFan">-</span>
      <span><input id="maxFanInput" type="number" min="300" max="600" step="10"> rpm <button class="pri" onclick="setMaxFan()">Setzen</button></span></div>
    <div class="msg" id="setResult"></div>
    <div class="mut">Bei eingeschalteter Frequenz-Automatik wird Max. Verdichter später wieder nach der Kurve gesetzt.</div>
  </div>

  <div class="card">
    <h2>Parameter der WP</h2>
    <div class="lead">Wird alle 10 min gelesen. Die Hysterese bestimmt, wie weit die Temperatur unter den Sollwert fallen muss,
      bevor die WP den Verdichter wieder startet. Größer = seltener takten. Jeder Schreibvorgang landet im Speicher der WP – nur selten ändern.</div>
    <div class="setrow"><span>P06 Hysterese Heizen</span><span class="cur" id="parP06">-</span>
      <span><input id="p06Input" type="number" min="1" max="15" step="0.5"> K <button class="pri" onclick="setPar('p06')">Setzen</button></span></div>
    <div class="setrow"><span>P07 Hysterese Warmwasser</span><span class="cur" id="parP07">-</span>
      <span><input id="p07Input" type="number" min="1" max="15" step="0.5"> K <button class="pri" onclick="setPar('p07')">Setzen</button></span></div>
    <div class="kv" id="parKv" style="margin-top:8px"></div>
    <div class="row"><button onclick="loadPar(true)">Jetzt neu lesen</button><span class="msg" id="parMsg"></span></div>
    <div class="mut">Register: P105 = 8194, P106 = 8195, P06 = 8196, P07 = 8197 (Liste aus einer Home-Assistant-Integration, nicht vom Hersteller).
      Bitte zuerst mit dem Display der WP vergleichen: passt der Rohwert ÷ 10 zu P06/P07? Beliebige Register lesen: Konsole <b>RD 8194 4</b>.</div>
  </div>
</section>

<!-- ================= PV & Smart Grid ================= -->
<section class="tab" id="pv">
  <div class="tiles" id="pvTiles"></div>
  <div class="card">
    <h2>Jetzt</h2>
    <div class="kv" id="pvKv"></div>
    <div class="row" style="margin-top:10px">
      <span class="mut">SG-Ready von Hand:</span>
      <div class="seg">
        <button id="sg1" onclick="pvCmd('sg1')">1 Sperre</button><button id="sg2" onclick="pvCmd('sg2')">2 Normal</button>
        <button id="sg3" onclick="pvCmd('sg3')">3 PV</button><button id="sg4" onclick="pvCmd('sg4')">4 PV max</button>
      </div>
      <button onclick="pvCmd('heater60')">Heizstab 60 min</button>
      <button onclick="pvCmd('heater0')">Heizstab aus</button>
    </div>
  </div>

  <div data-grp="pv">
  <div class="card" data-sw="pWw">
    <div class="ch"><h2>PV-Überschuss → Warmwasser</h2><label class="sw"><input type="checkbox" id="pWw"><span></span></label></div>
    <div class="lead">Meldet Node-RED SG3 oder SG4 (PV-Überschuss), lädt die WP den Speicher höher als sonst.</div>
    <div class="body form">
      <label><div>WP lädt bis</div><span><input id="pWwSoll" type="number" min="40" max="65"> °C</span></label>
      <label><div>Nur starten, wenn Speicher kälter als Ziel −</div><span><input id="pWwHyst" type="number" min="1" max="20"> K</span></label>
    </div>
  </div>

  <div class="card" data-sw="pHeat">
    <div class="ch"><h2>Heizstab (Shelly)</h2><label class="sw"><input type="checkbox" id="pHeat"><span></span></label></div>
    <div class="lead">Bei SG3 heizt der Heizstab nach der WP weiter, bei SG4 gleichzeitig. Der Shelly schaltet nach 3 min von selbst ab, falls der ESP ausfällt.</div>
    <div class="body form">
      <label><div>Heizstab heizt bis</div><span><input id="pHeatSoll" type="number" min="40" max="85"> °C</span></label>
      <label><div>Shelly-IP</div><input id="pShelly" class="w"></label>
      <label><div>Shelly-Typ</div><select id="pGen"><option value="1">Gen1</option><option value="2">Plus / Pro / Gen3 / Gen4</option></select></label>
      <label><div>Höchstens am Stück</div><span><input id="pHeatMax" type="number" min="10" max="720"> min</span></label>
    </div>
  </div>

  <div class="card">
    <h2>SG-Ready</h2>
    <div class="form">
      <label class="chk"><input type="checkbox" id="pSg1"> SG1 (Sperre vom Netzbetreiber) schaltet die WP aus</label>
      <label><div>Zurück auf SG2, wenn keine neue Meldung seit<em>Sicherheit, falls Node-RED ausfällt; 0 = nie</em></div><span><input id="pSgTo" type="number" min="0" max="240"> min</span></label>
    </div>
  </div>

  <div class="card" data-sw="pVl">
    <div class="ch"><h2>VL-Sollwert per MQTT</h2><label class="sw"><input type="checkbox" id="pVl"><span></span></label></div>
    <div class="lead">Für eine Raumregelung außerhalb des ESP (z. B. Node-RED). Wird ignoriert, solange die Heizkurve im ESP eingeschaltet ist.</div>
    <div class="body form">
      <label><div>Erlaubter Bereich</div><span><input id="pVlMin" type="number" min="15" max="60"> – <input id="pVlMax" type="number" min="20" max="65"> °C</span></label>
      <label><div>Höchstens alle</div><span><input id="pVlSec" type="number" min="30" max="3600"> s</span></label>
    </div>
  </div>
  </div>
  <div class="savebar" id="sb_pv"><b>PV: Änderungen noch nicht gespeichert</b>
    <span class="msg" id="pvMsg"></span>
    <button onclick="loadPv(true)">Verwerfen</button><button class="pri" onclick="savePv()">Speichern</button></div>
</section>

<!-- ================= System ================= -->
<section class="tab" id="system">
  <div class="grid2">
    <div class="card">
      <h2>System</h2>
      <div class="kv" id="sysKv"></div>
      <div class="row espOnly" style="margin-top:10px"><a href="/update"><button>Firmware-Update (.bin)</button></a></div>
      <div class="mut gigaOnly" style="margin-top:10px">GIGA R1: Firmware per USB-Kabel mit der Arduino-IDE hochladen – die Einstellungen bleiben erhalten.</div>
      <div class="mut espOnly">Arduino-IDE: Sketch → Kompilierte Binärdatei exportieren, dann <b>Avarma_R290_S3.ino.bin</b> hochladen. Netzwerk-Upload: Passwort wie OTA_PASSWORD im Sketch.</div>
    </div>
    <div class="card" data-grp="sys">
      <h2>Elektrische Leistung der WP</h2>
      <div class="lead">Gilt für COP, Energiezähler, Statistik und Log. Mit Shelly wird gemessen (inkl. Pumpe, Lüfter, Elektronik);
        sendet der Shelly 2 Minuten lang nichts, wird automatisch die Schätzung genommen. Im Log stehen immer beide Werte.</div>
      <div class="form">
        <label><div>Quelle</div><select id="pElSrc"><option value="0">Schätzung aus der WP</option><option value="1">Shelly per MQTT (Pro 3EM, EM, 1PM, PM Mini)</option></select></label>
        <label><div>Shelly Topic-Präfix<em>wie in der Shelly-App unter MQTT</em></div><input id="pElTopic" class="w" placeholder="shelly-wp"></label>
      </div>
      <div class="kv" id="elKv" style="margin:8px 0"></div>
      <div class="lead">Schätzung: Phasen × U × I × 0,95 × Korrektur (die WP liefert nur Verdichterstrom und Spannung).</div>
      <div class="form">
        <label><div>Netzanschluss WP</div><select id="pPhases"><option value="3">Drehstrom (3 Phasen)</option><option value="1">1 Phase</option></select></label>
        <label><div>Korrektur Schätzung</div><span><input id="pPfac" type="number" min="20" max="300"> %</span></label>
      </div>
      <div class="note">Shelly: MQTT einschalten, gleicher Broker wie unten, „Generic status update over MQTT“ an.
        Gelesen werden &lt;Präfix&gt;/status/em:0 + emdata:0 (Pro 3EM, 3 Phasen), em1:0 + em1data:0 (EM, 1 Phase)
        oder switch:0 / pm1:0 (1PM, PM Mini). Shelly der Generation 2, 3 oder 4 (Plus, Pro, Gen3 …).</div>
    </div>
  </div>

  <div class="card">
    <h2>Funktionen</h2>
    <div class="form">
      <label class="chk"><input type="checkbox" id="fePv" onchange="saveFeat()"> PV-Anlage / Energiemanagement (z. B. Victron über Node-RED):
        Tab „PV &amp; Smart Grid“, PV-Kachel, SG-Ready, PV-Warmwasser, Heizstab</label>
    </div>
    <div class="mut">Aus = alles davon ausgeblendet; PV-Werte und SG-Ready-Befehle per MQTT werden ignoriert.
      Ob Warmwasser über ein Umschaltventil oder den WW-Modus der WP läuft, stellst du im Tab Warmwasser ein.</div>
  </div>

  <div class="card">
    <h2>Einstellungen sichern / übertragen</h2>
    <div class="lead">Alle Einstellungen (Heizung inkl. Zeiten &amp; Sperrzeiten, Warmwasser &amp; Ventil inkl. Zeitplan, Verdichter, PV/MQTT/Shelly, NAS, Modbus, Strompreis) als Datei speichern
      und hier oder auf dem anderen Gerät (ESP32-S3 ↔ GIGA R1) wieder einspielen. Pin-Nummern werden nur übernommen,
      wenn die Datei vom gleichen Board-Typ stammt. Danach startet das Gerät neu.</div>
    <div class="row">
      <button class="pri" onclick="cfgExport()">Einstellungen herunterladen</button>
      <button onclick="$('cfgFile').click()">Datei einspielen …</button>
      <input type="file" id="cfgFile" accept=".json,application/json" style="display:none" onchange="cfgImport(this.files[0]);this.value=''">
      <span class="msg" id="cfgMsg"></span>
    </div>
    <div class="row" style="margin-top:8px">
      <button onclick="nasSave(false)">Aufs NAS sichern</button>
      <button onclick="nasList()">Vom NAS einspielen …</button>
      <select id="cfgNasSel" style="display:none;max-width:100%"></select>
      <button id="cfgNasGo" class="pri" style="display:none" onclick="nasRestore()">Einspielen</button>
    </div>
    <div class="mut" id="cfgNasInfo">Mit eingeschalteter NAS-Ablage (unten) wird nach jeder Änderung automatisch gesichert:
      Ordner <b>einstellungen</b>, eine Datei je Tag und <b>aktuell.json</b>. Auf einem neuen Gerät erst die NAS-Ablage eintragen, dann „Vom NAS einspielen“.</div>
  </div>

  <div class="card" data-grp="ftp" data-sw="nasEn">
    <div class="ch"><h2>Datenablage auf dem NAS (FTP)</h2><label class="sw"><input type="checkbox" id="nasEn"><span></span></label></div>
    <div class="lead">Intern speichert die Steuerung die letzten Tage fein aufgelöst, die Stundenwerte und die Statistik.
      Das NAS bekommt alles als Archiv: <b>&lt;Ordner&gt;/JJJJ-MM/JJJJ-MM-TT.csv</b> (alle 30 s ein Wert), im selben Monatsordner
      die Stundenwerte <b>h_JJJJ-MM.csv</b>, dazu <b>stats.csv</b>. Übertragen wird nur, was neu ist.
      Fehlen intern Dateien (z. B. nach dem Neuaufsetzen), holt die Steuerung sie beim Start vom NAS zurück.</div>
    <div class="body">
      <div class="form">
        <label><div>Server (IP)</div><input id="nasHost" class="w"></label>
        <label><div>Port</div><input id="nasPort" type="number" min="1" max="65535"></label>
        <label><div>Benutzer</div><input id="nasUser" class="w"></label>
        <label><div>Passwort</div><input id="nasPass" type="password" class="w"></label>
        <label><div>Ordner auf dem NAS<em>Synology: /home = Home-Ordner des Benutzers</em></div><input id="nasDir" class="w"></label>
        <label><div>Übertragen alle</div><span><input id="nasInt" type="number" min="1" max="1440"> min</span></label>
        <label><div>Tage fein im internen Speicher<em>ältere werden intern gelöscht, sobald sie auf dem NAS sind</em></div><span><input id="nasKeep" type="number" min="2" max="60"> Tage</span></label>
      </div>
      <div class="kv" id="ftpKv" style="margin:8px 0"></div>
      <div class="row"><button onclick="ftpCmd('test')">Verbindung testen</button><button onclick="ftpCmd('sync')">Jetzt übertragen</button>
        <button onclick="if(confirm('Fehlende Dateien vom NAS in den internen Speicher holen? Danach startet die Steuerung neu.'))ftpCmd('restore')">Fehlende Daten vom NAS holen</button></div>
    </div>
  </div>
  <div class="savebar" id="sb_ftp"><b>NAS: Änderungen noch nicht gespeichert</b>
    <span class="msg" id="ftpMsg"></span>
    <button onclick="loadFtp(true)">Verwerfen</button><button class="pri" onclick="saveFtp()">Speichern</button></div>

  <div class="card" data-grp="sys" data-sw="mEnabled">
    <div class="ch"><h2>MQTT (Venus OS / Node-RED)</h2><label class="sw"><input type="checkbox" id="mEnabled"><span></span></label></div>
    <div class="body">
      <div class="form">
        <label><div>Broker</div><input id="mHost" class="w"></label>
        <label><div>Port</div><input id="mPort" type="number"></label>
        <label><div>Benutzer</div><input id="mUser" class="w"></label>
        <label><div>Passwort</div><input id="mPass" type="password" class="w"></label>
        <label><div>Topic-Anfang</div><input id="mBase" class="w"></label>
        <label><div>Status senden alle</div><span><input id="mInt" type="number" min="2" max="300"> s</span></label>
      </div>
      <div class="note" id="pvTopics"></div>
    </div>
  </div>
  <div class="savebar" id="sb_sys"><b>System: Änderungen noch nicht gespeichert</b>
    <span class="msg" id="sysMsg"></span>
    <button onclick="loadPv(true)">Verwerfen</button><button class="pri" onclick="savePv()">Speichern</button></div>

  <div class="card">
    <h2>Konsole</h2>
    <div class="mut" id="mbDiag">Modbus: -</div>
    <pre id="console"></pre>
    <div class="row">
      <input id="cmdInput" class="w" placeholder="Befehl, z. B. STATUS" onkeydown="if(event.key==='Enter')sendCmd()">
      <button class="pri" onclick="sendCmd()">Senden</button>
      <button onclick="sendCmd('STATUS')">STATUS</button>
      <button onclick="sendCmd('VERBOSE')">VERBOSE</button>
      <button onclick="sendCmd('INFO')">INFO</button>
      <button onclick="sendCmd('HELP')">HELP</button>
      <button onclick="consoleEl.textContent=''">Leeren</button>
    </div>
  </div>

  <div class="grid2">
    <div class="card">
      <h2>Modbus-Schnittstelle</h2>
      <div class="form">
        <label class="espOnly"><div>RX-Pin</div><input id="mbRx" type="number"></label>
        <label class="espOnly"><div>TX-Pin</div><input id="mbTx" type="number"></label>
        <label class="gigaOnly"><div>Schnittstelle</div><span>Serial1: RX0 = D0 (RO), TX0 = D1 (DI)</span></label>
        <label><div>DE/RE-Pin</div><input id="mbDe" type="number"></label>
        <label class="espOnly"><div>Richtung</div><select id="mbMode"><option value="0">DE per Software</option><option value="1">Hardware-RS485</option></select></label>
        <label><div>Baudrate</div><input id="mbBaud" type="number"></label>
        <label><div>Modbus-ID der WP</div><input id="mbId" type="number"></label>
        <label><div>Pause nach Senden<em>gegen Störbytes beim Umschalten</em></div><span><input id="mbGap" type="number" min="0" max="20"> ms</span></label>
      </div>
      <div class="row">
        <button class="pri" onclick="saveMb()">Speichern &amp; Neustart</button>
        <button class="espOnly" onclick="sendCmd('SWAP')">RX/TX tauschen &amp; Neustart</button>
      </div>
    </div>
    <div class="card">
      <h2>Logdateien (SD-Karte)</h2>
      <div class="row"><button onclick="loadFiles()">Aktualisieren</button></div>
      <div id="files" class="files mut">-</div>
    </div>
  </div>

  <div class="card">
    <h2>Rohwerte der WP</h2>
    <div class="mut">Zum Prüfen der Skalierung: Wert hier mit dem Display der WP vergleichen.</div>
    <div class="row"><button onclick="loadRaw()">Aktualisieren</button></div>
    <table id="rawTbl"></table>
  </div>
</section>
</main>
<div id="toast"></div>

<script>
let hist = {from:0,to:0,step:30,p:[]};
const $ = id => document.getElementById(id);
const canvas = $('chart');
const ctx = canvas.getContext('2d');
const consoleEl = $('console');
let busy = {};
['sollInput','maxHzInput','maxFanInput'].forEach(id => {
  $(id).addEventListener('focus', () => busy[id] = true);
  $(id).addEventListener('blur', () => busy[id] = false);
});

function pad2(v){ return String(v).padStart(2,'0'); }
// Board-Typ (ESP32-S3 / GIGA R1): Begriffe und Pin-Felder anpassen
let PLAT = '', SDN = 'SD-Karte';
function applyPlat() {
  if (PLAT !== 'GIGA R1') return;
  document.body.classList.add('giga');
  SDN = 'Speicher';
  const rep = [['SD-Karte', 'Speicher'], ['Relais am ESP: freie IOs 3, 43, 44', 'Relais am GIGA: freie Pins D2…D13, D22…D53'],
               ['Relais am ESP', 'Relais am GIGA'], [' an IO', ' an Pin D'], ['schreibt der ESP', 'schreibt der Controller'], ['Der ESP sendet', 'Der GIGA sendet']];
  const w = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT);
  for (let n = w.nextNode(); n; n = w.nextNode()) {
    let t = n.nodeValue, o = t;
    for (const [a, b] of rep) t = t.split(a).join(b);
    if (t !== o) n.nodeValue = t;
  }
  document.querySelectorAll('option').forEach(o => { for (const [a, b] of rep) o.textContent = o.textContent.split(a).join(b); });
}
function f(v, d) { return (v === null || v === undefined) ? '–' : Number(v).toFixed(d); }
function modeName(m) { return m === 1 ? 'Warmwasser' : m === 2 ? 'Heizen' : m === 4 ? 'Kühlen' : m === 0 ? 'Aus' : '?'; }
function setV(id, val, unit) { $(id).innerHTML = val + (unit ? '<small>' + unit + '</small>' : ''); }
function chip(id, txt, cls) { const e = $(id); e.textContent = txt; e.className = 'chip' + (cls ? ' ' + cls : ''); }
function kv(id, rows) {
  $(id).innerHTML = rows.filter(r => r).map(r => '<div><span>' + r[0] + '</span><b>' + r[1] + '</b></div>').join('');
}
async function getJson(url) {
  const r = await fetch(url, {cache:'no-store'});
  const j = await r.json();
  if (/[?&]save=1|^\/ctrlset\?/.test(url) && j && j.ok !== false && typeof nasAuto === 'function') nasAuto();   // Einstellung geaendert -> aufs NAS
  return j;
}

// ---------- Ungespeicherte Änderungen ----------
const dirty = {};
function markDirty(g, on) {
  dirty[g] = on;
  const b = $('sb_' + g); if (b) b.classList.toggle('show', on);
}
document.querySelectorAll('[data-grp]').forEach(sec => {
  const h = e => { if (e.target.matches('input,select')) { markDirty(sec.dataset.grp, true); applyDeps(); } };
  sec.addEventListener('input', h); sec.addEventListener('change', h);
});
window.addEventListener('beforeunload', e => { if (Object.values(dirty).some(v => v)) { e.preventDefault(); e.returnValue = ''; } });
let toastT = 0;
function toast(t) { const e = $('toast'); e.textContent = t; e.classList.add('show'); clearTimeout(toastT); toastT = setTimeout(() => e.classList.remove('show'), 2500); }
// Karten mit Schalter abblenden, Felder je nach Auswahl ein-/ausblenden
function applyDeps() {
  document.querySelectorAll('[data-sw]').forEach(c => { const s = $(c.dataset.sw); if (s) c.classList.toggle('off', !s.checked); });
  const m = Number($('hMode').value);
  document.querySelectorAll('.m-ta').forEach(e => e.classList.toggle('hide', m === 2));
  document.querySelectorAll('.m-rt').forEach(e => e.classList.toggle('hide', m === 1));
  $('hKMax').disabled = !$('hKMaxOn').checked;
  $('lkTa').disabled = !$('lkCold').checked;
  $('cAbK').disabled = !$('cAbOn').checked;
  $('cAbFixT').disabled = !$('cAbFix').checked;
  document.querySelectorAll('.nc').forEach(e => e.classList.toggle('hide', !$('hNcOn').checked));
  document.querySelectorAll('.ncOff').forEach(e => e.classList.toggle('hide', $('hNcOn').checked));
  $('hNoffTa').disabled = !$('hNoffOn').checked;
  document.querySelectorAll('.nw').forEach(e => e.classList.toggle('hide', !$('hNwOn').checked));
  $('fNight').disabled = !$('fNightOn').checked;
  const mod = $('cDrive').value === '1';
  document.querySelectorAll('.d-mod').forEach(e => e.classList.toggle('hide', !mod));
  document.querySelectorAll('.d-rel').forEach(e => e.classList.toggle('hide', mod));
}
function q(id, on, txt) { const e = $(id); e.classList.toggle('on', !!on); e.querySelector('small').textContent = txt; }

// ---------- Tabs ----------
function showTab() {
  const id = (location.hash || '#uebersicht').slice(1);
  document.querySelectorAll('.tab').forEach(t => t.classList.toggle('act', t.id === id));
  window.scrollTo(0, 0);
  document.querySelectorAll('#nav a').forEach(a => a.classList.toggle('act', a.getAttribute('href') === '#' + id));
  if (id === 'verlauf') render();
  if (id === 'statistik') loadStats();
  if (id === 'system') { loadFiles(); loadRaw(); }
}
window.addEventListener('hashchange', showTab);

// ---------- Aktuelle Werte ----------
let last = null;
async function loadCurrent() {
  let s;
  try { s = await getJson('/current'); } catch(e) { chip('chMb', 'keine Verbindung', 'bad'); return; }
  last = s;
  const x = s.x || {};
  if (s.timeValid && s.nowUtc) {
    const d = new Date(s.nowUtc * 1000);
    $('statusTime').textContent = d.toLocaleDateString('de-DE') + ' ' + d.toLocaleTimeString('de-DE');
  } else $('statusTime').textContent = 'Zeit nicht synchron';

  chip('chWp', 'WP ' + (s.wpOn === null ? '–' : s.wpOn ? 'EIN' : 'AUS'), s.wpOn ? 'on' : '');
  chip('chMode', s.mode === null ? '–' : modeName(Number(s.mode)), '');
  chip('chSg', 'SG' + s.sg, s.sg >= 3 ? 'on' : s.sg === 1 ? 'warn' : '');
  chip('chMb', 'Modbus ' + (s.mbErr && s.mbErrAge < 60 ? 'Fehler' : 'OK'), s.mbErr && s.mbErrAge < 60 ? 'bad' : '');
  chip('chMq', 'MQTT ' + (s.mqtt === null ? 'aus' : s.mqtt ? 'OK' : 'getrennt'), s.mqtt === false ? 'warn' : '');
  ['bOn','bOff'].forEach((b,i) => $(b).classList.toggle('sel', s.wpOn !== null && s.wpOn === (i === 0)));
  [1,2,4].forEach(m => $('bM' + m).classList.toggle('sel', Number(s.mode) === m));

  setV('tVl', f(s.vl, 1), '°C');  $('tVlS').textContent = s.sv !== null ? 'Soll ' + f(s.sv, 0) + ' °C' : '';
  setV('tRl', f(s.rl, 1), '°C');  $('tRlS').textContent = 'ΔT ' + f(s.vl - s.rl, 1) + ' K';
  setV('tWw', f(s.ww, 1), '°C');  $('tWwS').textContent = '';
  setV('tTa', f(x.ta, 1), '°C');  $('tTaS').textContent = x.defrost ? 'Abtauen' : '';
  setV('tHz', s.hz === null ? '–' : s.hz, 'Hz');
  $('tHzS').textContent = (x.hzT !== null && x.hzT !== undefined ? 'Soll ' + x.hzT + ' · ' : '') + 'max ' + (s.maxHz ?? '–') + ' Hz';
  setV('tPth', x.pTh !== null && x.pTh !== undefined ? (x.pTh / 1000).toFixed(2) : '–', 'kW');
  $('tPthS').textContent = (x.pEl ? 'Strom ' + (x.pElSrc ? '' : '≈ ') + (x.pEl / 1000).toFixed(2) + ' kW' : '') + (x.cop ? ' · COP ' + x.cop.toFixed(1) : '');
  setV('tE', f(x.eThDay, 1), 'kWh');
  const cop = (th, el) => el > 0.05 ? (th / el).toFixed(1) : '–';
  $('tES').textContent = 'Strom ' + f(x.eElDay, 1) + ' · COP ' + cop(x.eThDay, x.eElDay) + ' (gestern ' + cop(x.eThY, x.eElY) + ')';

  $('curSoll').textContent = 'aktuell ' + (s.sv === null ? '–' : f(s.sv, 0) + ' °C');
  $('curHz').textContent = 'aktuell ' + (s.maxHz ?? '–') + ' Hz';
  $('curFan').textContent = 'aktuell ' + (s.maxFan ?? '–') + ' rpm';
  if (!busy.sollInput && s.sv !== null && !$('sollInput').value) $('sollInput').value = Math.round(s.sv);
  if (!busy.maxHzInput && s.maxHz !== null) $('maxHzInput').value = s.maxHz;
  if (!busy.maxFanInput && s.maxFan !== null) $('maxFanInput').value = s.maxFan;

  kv('gWater', [['Vorlauf', f(s.vl,1) + ' °C'], ['Rücklauf', f(s.rl,1) + ' °C'], ['Warmwasser', f(s.ww,1) + ' °C'],
    ['Durchfluss', f(s.flow,1) + ' l/min'], ['Pumpe', (x.pump ?? '–') + ' %'], ['Außen', f(x.ta,1) + ' °C']]);
  kv('gRef', [['Hochdruck', f(x.hp,2) + ' MPa'], ['Niederdruck', f(x.lp,2) + ' MPa'], ['Heißgas', f(x.discharge,1) + ' °C'],
    ['Sauggas', f(x.suction,1) + ' °C'], ['Verdampfer', f(x.evap,1) + ' °C'], ['Kondensator', f(x.cond,1) + ' °C'],
    ['Coil', f(x.coil,1) + ' °C'], ['Flüssigleitung', f(x.liquid,1) + ' °C'],
    ['Lüfter', (s.fan1 ?? '–') + (s.fan2 ? ' / ' + s.fan2 : '') + ' rpm'], ['Abtauen', x.defrost === null ? '–' : x.defrost ? 'ja' : 'nein']]);
  kv('gEl', [['Verdichterstrom', f(x.amp,1) + ' A'], ['Netzspannung', f(x.acV,0) + ' V' + (x.phases === 3 ? ' (1 Phase)' : '')],
    ['Zwischenkreis', f(x.dcV,0) + ' V'], ['IPM-Temperatur', f(x.ipm,0) + ' °C'],
    ['Leistung el. (' + (x.pElSrc ? 'Shelly' : 'gesch.') + ')', f(x.pEl,0) + ' W' + (x.pElSrc && x.pElWp !== null ? ' <small>(Schätzung ' + f(x.pElWp,0) + ' W)</small>' : '')],
    ['Leistung th.', f(x.pTh,0) + ' W'], ['COP', x.cop ? x.cop.toFixed(2) : '–']]);
  const elT = x.pElSrc ? '' : ' (gesch.)';
  kv('gCnt', [['Wärme heute', f(x.eThDay,2) + ' kWh'], ['Strom heute' + elT, f(x.eElDay,2) + ' kWh'],
    ['COP heute', cop(x.eThDay, x.eElDay)], ['COP gestern', cop(x.eThY, x.eElY) + ' (' + f(x.eThY,1) + ' / ' + f(x.eElY,1) + ' kWh)'],
    ['Wärme gesamt', f(x.eThTot,1) + ' kWh'], ['Strom gesamt' + elT, f(x.eElTot,1) + ' kWh'],
    ['Verdichterstarts', x.starts ?? '–'], ['Laufzeit', (x.runH ?? '–') + ' h']]);

  $('mbDiag').textContent = 'Modbus: OK ' + s.ok + ', Fehler ' + s.err +
    (s.mbErr ? ' · letzter Fehler ' + s.mbErr + ' (' + s.mbErrTxt + ') vor ' + s.mbErrAge + ' s' : '');
  if (s.fw) $('fwv').textContent = 'v' + s.fw;
  if (s.plat && s.plat !== PLAT) { PLAT = s.plat; applyPlat(); }
  kv('sysKv', [['Firmware', 'v' + s.fw + ' (' + s.fwBuild + ')'], ['WLAN', s.rssi + ' dBm'], ['Laufzeit', (s.uptime / 3600).toFixed(1) + ' h'],
    [SDN, s.sdOk ? (s.sdFreeMB >= 1024 ? (s.sdFreeMB / 1024).toFixed(1) + ' GB' : s.sdFreeMB + ' MB') + ' frei' : 'nicht verfügbar'],
    ['Verlauf', s.logPts + ' Punkte (max ' + (s.logCapH / 24).toFixed(0) + ' Tage)'],
    ['Modbus', 'OK ' + s.ok + ' · Fehler ' + s.err], ['MQTT', s.mqtt === null ? 'aus' : s.mqtt ? 'verbunden' : 'getrennt'],
    s.cmdMsg ? ['Letzte Meldung', s.cmdMsg] : null]);
  $('oSg').textContent = ['', 'SG1 Sperre', 'SG2 normal', 'SG3 PV', 'SG4 PV max'][s.sg] || '–';
  const pv = s.pv || {}, kw = v => v === null || v === undefined ? '–' : (v / 1000).toFixed(2);
  setV('tPv', pv.any ? kw(pv.power) : '–', pv.any ? 'kW' : '');
  $('tPvS').textContent = pv.any ? (pv.soc !== null ? 'Batterie ' + pv.soc + ' % · ' : '') + 'heute ' + f(pv.today, 1) + ' kWh' : 'keine Daten (MQTT)';
  if (!pv.any) $('pvTiles').innerHTML = '<div class="tile" style="grid-column:1/-1;border-left-color:#e0b000"><div class="l">PV-Anlage</div><div class="v" style="font-size:16px;font-weight:400">' +
     'Keine PV-Daten. Node-RED muss an <b>avarma/pv/power</b>, <b>soc</b>, <b>bat</b>, <b>grid</b>, <b>house</b> (und optional <b>today</b>/<b>yesterday</b>) senden.</div></div>';
  else {
    const wpShare = pv.house > 100 && x.pEl !== null && x.pEl !== undefined ? ' · davon WP ' + Math.round(Math.min(100, x.pEl / pv.house * 100)) + ' %' : '';
    const T = (l, v, u, sub, c) => '<div class="tile" style="border-left-color:' + c + '"><div class="l">' + l + '</div><div class="v">' + v + '<small>' + u + '</small></div><div class="s">' + sub + '</div></div>';
    $('pvTiles').innerHTML =
      T('PV-Leistung', kw(pv.power), 'kW', 'Daten vor ' + pv.age + ' s', '#e0b000') +
      T('Batterie', pv.soc === null ? '–' : pv.soc, '%', pv.bat === null ? '' : (pv.bat >= 0 ? 'lädt ' : 'entlädt ') + kw(Math.abs(pv.bat)) + ' kW', '#2f9e5a') +
      T('Netz', pv.grid === null ? '–' : kw(Math.abs(pv.grid)), 'kW', pv.grid === null ? '' : pv.grid > 0 ? 'Bezug' : 'Einspeisung', pv.grid > 50 ? '#d9473a' : '#2f9e5a') +
      T('Haus', kw(pv.house), 'kW', 'Verbrauch' + wpShare, '#3570d4') +
      T('PV heute', f(pv.today, 1), 'kWh', 'gestern ' + f(pv.yest, 1) + ' kWh', '#c9a400');
  }
  $('oHeat').textContent = s.heater;
}

// ---------- Befehle WP ----------
async function wpCall(url, el, okTxt) {
  $(el).textContent = 'Schreibe...';
  try {
    const d = await getJson(url);
    $(el).textContent = d.ok ? okTxt(d) : 'Fehlgeschlagen' + (d.readback !== undefined ? ' (Rücklesen: ' + (d.readback ?? '–') + ')' : '');
  } catch(e) { $(el).textContent = 'Keine Antwort'; }
  loadCurrent();
}
function setPower(on) { wpCall('/power?state=' + (on ? 'on' : 'off'), 'wpResult', () => 'WP ' + (on ? 'EIN' : 'AUS') + ' geschrieben'); }
function setMode(m)   { wpCall('/mode?value=' + m, 'wpResult', () => 'Betriebsart ' + modeName(m) + ' geschrieben'); }
function setSollwert() { wpCall('/set?soll=' + encodeURIComponent($('sollInput').value), 'sollResult', d => 'VL-Soll ' + f(d.readback, 0) + ' °C übernommen'); }
function setMaxHz() {
  const v = Number($('maxHzInput').value);
  if (v < 30 || v > 85) { $('setResult').textContent = 'Erlaubt: 30–85 Hz'; return; }
  wpCall('/setmaxhz?hz=' + v, 'setResult', d => 'Max. Verdichter ' + d.readback + ' Hz übernommen');
}
function setMaxFan() {
  const v = Number($('maxFanInput').value);
  if (v < 300 || v > 600) { $('setResult').textContent = 'Erlaubt: 300–600 rpm'; return; }
  wpCall('/setmaxfan?rpm=' + v, 'setResult', d => 'Max. Lüfter ' + d.readback + ' rpm übernommen');
}

// ---------- Verlauf ----------
const SER = [
  {k:'vl',  i:1,  n:'Vorlauf',    c:'#d9473a', ax:'t',   on:1},
  {k:'rl',  i:2,  n:'Rücklauf',   c:'#3570d4', ax:'t',   on:1},
  {k:'ww',  i:3,  n:'Warmwasser', c:'#e88a1a', ax:'t',   on:1},
  {k:'sv',  i:9,  n:'VL-Soll',    c:'#e8968e', ax:'t',   on:0, dash:1},
  {k:'rt',  i:6,  n:'Raum',       c:'#8a4fd0', ax:'t',   on:1},
  {k:'ta',  i:7,  n:'Außen',      c:'#5f7488', ax:'t',   on:1},
  {k:'ev',  i:8,  n:'Verdampfer', c:'#17a2b8', ax:'t',   on:0},
  {k:'hz',  i:4,  n:'Verdichter', c:'#2f9e5a', ax:'hz',  on:1},
  {k:'fan', i:15, n:'Lüfter',     c:'#6d4c9f', ax:'hz',  on:0, sc:0.1, raw:1, unit:'rpm', dash:1},
  {k:'fl',  i:5,  n:'Durchfluss', c:'#9a7b2f', ax:'t',   on:0, unit:'l/min'},
  {k:'cop', i:10, n:'COP',        c:'#b8860b', ax:'cop', on:0},
  {k:'pe',  i:12, n:'Leistung el.', c:'#c2185b', ax:'cop', on:0, sc:0.001, unit:'kW'},
  {k:'pt',  i:13, n:'Leistung th.', c:'#8d6e63', ax:'cop', on:0, sc:0.001, unit:'kW'},
  {k:'df',  i:11, n:'Abtauen',    c:'#7fb2ff', ax:'band',on:1},
];
let view = {type:'h', h:1};
let monthSel = null;
function lsGet(k, d) { try { const v = localStorage.getItem(k); return v === null ? d : JSON.parse(v); } catch(e) { return d; } }
function lsSet(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch(e) {} }
function buildSerBar() {
  const saved = lsGet('avSer', null);
  if (saved) SER.forEach(s => { if (s.k in saved) s.on = saved[s.k]; });
  $('serBar').innerHTML = SER.map(s => '<label class="sc" style="--c:' + s.c + '"><input type="checkbox" id="ser_' + s.k + '"' +
    (s.on ? ' checked' : '') + ' onchange="serToggle(\'' + s.k + '\')"><i></i>' + s.n + '</label>').join('');
}
function serToggle(k) {
  const s = SER.find(x => x.k === k); s.on = $('ser_' + k).checked ? 1 : 0;
  const o = {}; SER.forEach(x => o[x.k] = x.on); lsSet('avSer', o);
  render();
}
const MONTHS = ['Jan','Feb','Mär','Apr','Mai','Jun','Jul','Aug','Sep','Okt','Nov','Dez'];
function markRange() {
  document.querySelectorAll('#rangeBar button[data-h]').forEach(b =>
    b.classList.toggle('sel', view.type === 'h' ? String(view.h) === b.dataset.h : view.type === 'all' && b.dataset.h === 'all'));
  $('monthBtn').classList.toggle('sel', view.type === 'm');
  const m = monthSel || (() => { const d = new Date(); return {y: d.getFullYear(), m: d.getMonth()}; })();
  $('monthBtn').textContent = MONTHS[m.m] + ' ' + m.y;
}
function setWindowHours(h) { view = h === 'all' ? {type:'all'} : {type:'h', h:Number(h)}; markRange(); loadHistory(); }
function monthStep(d) {
  if (!monthSel) { const n = new Date(); monthSel = {y: n.getFullYear(), m: n.getMonth()}; }
  if (d) { monthSel.m += d; if (monthSel.m < 0) { monthSel.m = 11; monthSel.y--; } if (monthSel.m > 11) { monthSel.m = 0; monthSel.y++; } }
  view = {type:'m'}; markRange(); loadHistory();
}
async function loadHistory() {
  let url;
  if (view.type === 'all') url = '/history?all=1';
  else if (view.type === 'm') {
    const f = new Date(monthSel.y, monthSel.m, 1), t = new Date(monthSel.y, monthSel.m + 1, 1);
    url = '/history?from=' + Math.floor(f / 1000) + '&to=' + Math.floor(t / 1000);
  } else url = '/history?hours=' + view.h;
  $('viewInfo').textContent = 'Lade...';
  try { hist = await getJson(url); } catch(e) {}
  render();
}
function spanSec() { return (hist.to || 0) - (hist.from || 0); }
function tickLabel(t) {
  const d = new Date(t * 1000), hm = pad2(d.getHours()) + ':' + pad2(d.getMinutes()), dm = pad2(d.getDate()) + '.' + pad2(d.getMonth() + 1) + '.';
  const sp = spanSec();
  return sp > 20 * 86400 ? dm + (sp > 300 * 86400 ? String(d.getFullYear()).slice(2) : '') : sp > 86400 ? dm + ' ' + hm : hm;
}
let chartGeo = null;
function render() {
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h); ctx.fillStyle = '#fff'; ctx.fillRect(0, 0, w, h);
  const pts = hist.p || [], valid = pts.filter(p => p);
  const sp = spanSec(), step = hist.step || 0;
  $('viewInfo').textContent = (sp ? 'Zeitraum: ' + (sp >= 2 * 86400 ? (sp / 86400).toFixed(0) + ' Tage' : (sp / 3600).toFixed(0) + ' h') : '') +
    (step ? ' · ein Punkt = ' + (step >= 3600 ? (step / 3600).toFixed(step % 3600 ? 1 : 0) + ' h' : step >= 60 ? step / 60 + ' min' : step + ' s') : '') +
    (hist.first ? ' · Daten ab ' + new Date(hist.first * 1000).toLocaleDateString('de-DE') : '');
  chartGeo = null;
  if (valid.length < 2) { ctx.fillStyle = '#555'; ctx.font = '16px sans-serif'; ctx.fillText('Für diesen Zeitraum sind keine Daten vorhanden.', 20, 40); return; }
  const on = SER.filter(s => s.on);
  const showHz = on.some(s => s.ax === 'hz'), showCop = on.some(s => s.ax === 'cop');
  const L = 50, R = 14 + (showHz ? 46 : 0) + (showCop ? 40 : 0), T = 26, B = 34, pw = w - L - R, ph = h - T - B, from = hist.from, to = hist.to;
  let minT = 999, maxT = -999, maxHz = 10, maxCop = 4;
  for (const p of valid) {
    for (const s of on) if (s.ax === 't') { const v = p[s.i]; if (v === null || v === undefined) continue; minT = Math.min(minT, v); maxT = Math.max(maxT, v); }
    maxHz = Math.max(maxHz, p[4]);
    for (const s of on) if (s.ax === 'hz' && s.sc) { const v = p[s.i]; if (v !== null && v !== undefined) maxHz = Math.max(maxHz, v * s.sc); }
    for (const s of on) if (s.ax === 'cop') { const v = p[s.i]; if (v !== null && v !== undefined) maxCop = Math.max(maxCop, v * (s.sc || 1)); }
  }
  if (!(minT < maxT)) { minT = 0; maxT = 50; }
  minT = Math.floor(minT - 1); maxT = Math.ceil(maxT + 1); maxCop = Math.ceil(maxCop);
  const xF = t => L + (t - from) / (to - from) * pw, yT = v => T + (1 - (v - minT) / (maxT - minT)) * ph,
        yH = v => T + (1 - v / maxHz) * ph, yC = v => T + (1 - v / maxCop) * ph;
  // Abtauen als Hintergrund
  if (on.some(s => s.k === 'df')) {
    ctx.fillStyle = 'rgba(127,178,255,.28)';
    const bw = Math.max(2, pw * step / Math.max(1, to - from));
    for (const p of valid) if (p[11] > 0) { const a = Math.min(1, 0.25 + p[11] * 3); ctx.globalAlpha = a; ctx.fillRect(xF(p[0]) - bw / 2, T, bw, ph); }
    ctx.globalAlpha = 1;
  }
  ctx.font = '12px sans-serif'; ctx.lineWidth = 1;
  for (let i = 0; i <= 5; i++) {
    const y = T + (1 - i / 5) * ph;
    ctx.strokeStyle = '#e6e9ee'; ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(L + pw, y); ctx.stroke();
    ctx.fillStyle = '#667'; ctx.fillText((minT + i / 5 * (maxT - minT)).toFixed(0) + '°', 8, y + 4);
    if (showHz) { ctx.fillStyle = '#2f9e5a'; ctx.fillText(Math.round(i / 5 * maxHz) + ' Hz', L + pw + 6, y + 4); }
    if (showHz && i === 5 && on.some(s => s.k === 'fan')) { ctx.fillStyle = '#6d4c9f'; ctx.fillText('rpm ÷10', L + pw + 2, T - 10); }
    if (showCop) { ctx.fillStyle = '#b8860b'; ctx.fillText((i / 5 * maxCop).toFixed(1), L + pw + (showHz ? 52 : 6), y + 4); }
    if (showCop && i === 5) { ctx.fillStyle = '#b8860b'; ctx.fillText(on.some(s => s.sc && s.ax === 'cop') ? (on.some(s => s.k === 'cop') ? 'COP/kW' : 'kW') : 'COP', L + pw + (showHz ? 52 : 6), T - 10); }
  }
  ctx.fillStyle = '#667';
  for (let i = 0; i < 6; i++) {
    const t = from + (to - from) * i / 5, x = xF(t), lb = tickLabel(t), tw = ctx.measureText(lb).width;
    ctx.fillText(lb, Math.min(Math.max(x - tw / 2, 0), w - tw), T + ph + 20);
  }
  function line(s) {
    const yf = s.ax === 'hz' ? yH : s.ax === 'cop' ? yC : yT;
    ctx.strokeStyle = s.c; ctx.lineWidth = s.ax === 'hz' ? 1.5 : 2; ctx.setLineDash(s.dash ? [6, 4] : []);
    if (s.ax === 'hz') ctx.globalAlpha = .6;
    ctx.beginPath(); let st = false;
    for (const p of pts) { if (!p || p[s.i] === null || p[s.i] === undefined) { st = false; continue; } const x = xF(p[0]), y = yf(p[s.i] * (s.sc || 1)); if (!st) { ctx.moveTo(x, y); st = true; } else ctx.lineTo(x, y); }
    ctx.stroke(); ctx.lineWidth = 1; ctx.setLineDash([]); ctx.globalAlpha = 1;
  }
  for (const s of on) if (s.ax !== 'band') line(s);
  chartGeo = {L, T, pw, ph, from, to, xF};
}
function chartTip(ev) {
  const tip = $('tip');
  if (!chartGeo || !hist.p) { tip.style.display = 'none'; return; }
  const r = canvas.getBoundingClientRect(), sx = canvas.width / r.width;
  const x = (ev.clientX - r.left) * sx;
  if (x < chartGeo.L || x > chartGeo.L + chartGeo.pw) { tip.style.display = 'none'; return; }
  const t = chartGeo.from + (x - chartGeo.L) / chartGeo.pw * (chartGeo.to - chartGeo.from);
  let best = null, bd = 1e18;
  for (const p of hist.p) if (p) { const d = Math.abs(p[0] - t); if (d < bd) { bd = d; best = p; } }
  if (!best) { tip.style.display = 'none'; return; }
  const d = new Date(best[0] * 1000);
  let h = '<b>' + d.toLocaleDateString('de-DE') + ' ' + pad2(d.getHours()) + ':' + pad2(d.getMinutes()) + '</b>';
  for (const s of SER) if (s.on && s.ax !== 'band') {
    const v = best[s.i]; if (v === null || v === undefined) continue;
    const u = s.unit ? ' ' + s.unit : s.ax === 'hz' ? ' Hz' : s.ax === 'cop' ? '' : ' °C';
    h += '<div><i style="background:' + s.c + '"></i>' + s.n + ' <b>' + (s.raw ? Math.round(v) : s.sc ? (v * s.sc).toFixed(2) : s.ax === 'cop' ? v.toFixed(2) : s.ax === 'hz' ? Math.round(v) : v.toFixed(1)) + u + '</b></div>';
  }
  if (best[11] > 0) h += '<div><i style="background:#7fb2ff"></i>Abtauen ' + Math.round(best[11] * 100) + ' %</div>';
  tip.innerHTML = h; tip.style.display = 'block';
  const px = ev.clientX - r.left, wrap = canvas.parentElement.getBoundingClientRect().width;
  tip.style.left = (px > wrap / 2 ? px - tip.offsetWidth - 14 : px + 14) + 'px';
  tip.style.top = Math.max(0, ev.clientY - r.top - 30) + 'px';
}

// ---------- Statistik ----------
const _n0 = new Date();
let stData = null, stView = {mode:'y', y: _n0.getFullYear(), m: _n0.getMonth(), day: new Date(_n0.getFullYear(), _n0.getMonth(), _n0.getDate()), wk: 0, rFrom: null, rTo: null};
let stSel = null;              // Tage (JJJJMMTT) des gewaehlten Zeitraums (fuer die Streugrafik)
const dayHist = {};            // Stundenwerte je Tag (aus /history), Cache
// Spalten: 0 Tag,1 Strom,2 Waerme,3 StromWW,4 WaermeWW,5 Laufzeit,6 Starts,7 Abtau,8 TaMittel,9 TaMin,10 TaMax,11 Raum,12 PV,13 StromMitNetz,14 davonEigen
async function loadStats() {
  try { stData = await getJson('/stats'); } catch(e) { $('stInfo').textContent = 'Keine Antwort'; return; }
  renderStats();
}
function stRows() {
  const m = new Map();
  for (const r of (stData && stData.days) || []) m.set(r[0], r);
  if (stData && stData.today) m.set(stData.today[0], stData.today.concat(['heute']));
  return [...m.values()].sort((a, b) => a[0] - b[0]);
}
function stAgg(rows) {
  const a = {el:0, th:0, thE:0, elWw:0, thWw:0, run:0, starts:0, df:0, taS:0, taN:0, taMin:null, taMax:null, pv:0, pvN:0, elG:0, own:0, n:rows.length, elN:0};
  for (const r of rows) {
    a.th += r[2] || 0; a.thWw += r[4] || 0; a.run += r[5] || 0; a.starts += r[6] || 0; a.df += r[7] || 0;
    if (r[1] !== null) { a.el += r[1]; a.elWw += r[3] || 0; a.thE += r[2] || 0; a.elN++; }
    if (r[8] !== null) { a.taS += r[8]; a.taN++; a.taMin = a.taMin === null ? r[9] : Math.min(a.taMin, r[9]); a.taMax = a.taMax === null ? r[10] : Math.max(a.taMax, r[10]); }
    if (r[12] !== null) { a.pv += r[12]; a.pvN++; }
    if (r[13] !== null) { a.elG += r[13]; a.own += r[14] || 0; }
  }
  a.cop = a.el > 0.05 ? a.thE / a.el : null;
  a.ta = a.taN ? a.taS / a.taN : null;
  return a;
}
const WD = ['So','Mo','Di','Mi','Do','Fr','Sa'];
const dKey = d => d.getFullYear() * 10000 + (d.getMonth() + 1) * 100 + d.getDate();
const dIso = d => d.getFullYear() + '-' + pad2(d.getMonth() + 1) + '-' + pad2(d.getDate());
const dDE = d => pad2(d.getDate()) + '.' + pad2(d.getMonth() + 1) + '.';
function addDays(d, n) { const x = new Date(d); x.setDate(x.getDate() + n); return x; }
function mondayOf(d) { const x = new Date(d.getFullYear(), d.getMonth(), d.getDate()); x.setDate(x.getDate() - ((x.getDay() + 6) % 7)); return x; }

function stMode(m) {
  const t = new Date(); const today = new Date(t.getFullYear(), t.getMonth(), t.getDate());
  if (m === 'today') { stView.mode = 'd'; stView.day = today; }
  else if (m === 'yest') { stView.mode = 'd'; stView.day = addDays(today, -1); }
  else if (m === 'w') { stView.mode = 'w'; stView.wk = 0; }
  else if (m === 'r') {
    stView.mode = 'r';
    if (!stView.rFrom) { stView.rTo = today; stView.rFrom = addDays(today, -13); }
    $('stFrom').value = dIso(stView.rFrom); $('stTo').value = dIso(stView.rTo);
  }
  else {                                                         // Monat/Jahr: am gerade angezeigten Datum ausrichten
    const v = stView, ref = v.mode === 'd' ? v.day : v.mode === 'w' ? addDays(mondayOf(new Date()), v.wk * 7) : v.mode === 'r' ? v.rTo : null;
    if (ref) { v.y = ref.getFullYear(); v.m = ref.getMonth(); }
    v.mode = m;
  }
  renderStats();
}
function stRange() {
  const f = $('stFrom').value, t = $('stTo').value;
  if (!f || !t) return;
  let a = new Date(f + 'T00:00:00'), b = new Date(t + 'T00:00:00');
  if (b < a) [a, b] = [b, a];
  stView.mode = 'r'; stView.rFrom = a; stView.rTo = b;
  renderStats();
}
function stStep(d) {
  const v = stView;
  if (v.mode === 'y') v.y += d;
  else if (v.mode === 'm') { v.m += d; if (v.m < 0) { v.m = 11; v.y--; } if (v.m > 11) { v.m = 0; v.y++; } }
  else if (v.mode === 'd') v.day = addDays(v.day, d);
  else if (v.mode === 'w') v.wk += d;
  else if (v.mode === 'r') { const n = Math.round((v.rTo - v.rFrom) / 86400000) + 1; v.rFrom = addDays(v.rFrom, d * n); v.rTo = addDays(v.rTo, d * n);
    $('stFrom').value = dIso(v.rFrom); $('stTo').value = dIso(v.rTo); }
  renderStats();
}
const fk = (v, d) => v === null || v === undefined ? '–' : Number(v).toLocaleString('de-DE', {minimumFractionDigits: d, maximumFractionDigits: d});

// Stundenwerte eines Tages aus dem Verlauf (Energie = mittlere Leistung x Zeit)
async function loadDayHist(day) {
  const k = dKey(day);
  const isToday = k === dKey(new Date());
  if (dayHist[k] && !(isToday && Date.now() - dayHist[k].at > 60000)) return dayHist[k];
  const f = Math.floor(day / 1000), t = Math.floor(addDays(day, 1) / 1000);
  let h;
  try { h = await getJson('/history?from=' + f + '&to=' + t); } catch(e) { return null; }
  const hours = [];
  for (let i = 0; i < 24; i++) hours.push({el:0, th:0, elWw:0, thWw:0, run:0, df:0, taS:0, taN:0, taMin:null, taMax:null, hasEl:false, wasDf:false});
  const step = h.step || 120;
  for (const p of h.p || []) {
    if (!p) continue;
    const d = new Date(p[0] * 1000);
    if (dKey(d) !== k) continue;
    const H = hours[d.getHours()], dt = step / 3600;
    if (p[12] !== null && p[12] !== undefined) {
      const el = p[12] * dt / 1000, th = Math.max(0, p[13]) * dt / 1000, ww = p[14] || 0;
      H.el += el; H.th += th; H.elWw += el * ww; H.thWw += th * ww; H.hasEl = true;
    } else if (p[5] !== null) {                                  // aeltere Daten: Waerme aus Durchfluss
      const th = Math.max(0, p[5] / 60 * 4186 * (p[1] - p[2])) * dt / 1000; H.th += th;
    }
    if (p[4] > 0) H.run += dt;
    if (p[11] > 0 && !H.wasDf) H.df++;
    H.wasDf = p[11] > 0;
    if (p[7] !== null) { H.taS += p[7]; H.taN++; H.taMin = H.taMin === null ? p[7] : Math.min(H.taMin, p[7]); H.taMax = H.taMax === null ? p[7] : Math.max(H.taMax, p[7]); }
  }
  const rows = hours.map((H, i) => [k * 100 + i, H.hasEl ? H.el : null, H.th, H.elWw, H.thWw, H.run, 0, H.df,
    H.taN ? H.taS / H.taN : null, H.taMin, H.taMax, null, null, null, null]);
  dayHist[k] = {rows, at: Date.now()};
  return dayHist[k];
}

function renderStats() {
  const v = stView;
  [['stTd', v.mode === 'd' && dKey(v.day) === dKey(new Date())], ['stYd', v.mode === 'd' && dKey(v.day) === dKey(addDays(new Date(), -1))],
   ['stW', v.mode === 'w'], ['stM', v.mode === 'm'], ['stY', v.mode === 'y'], ['stR', v.mode === 'r']].forEach(([id, on]) => $(id).classList.toggle('sel', on));
  $('stRangeBox').style.display = v.mode === 'r' ? 'flex' : 'none';
  if (!stData) return;
  const all = stRows();
  let info = '';
  if (!stData.sd) info = 'Kein Speicher (' + SDN + ') – ohne ihn gibt es nur den heutigen Tag.';
  else if (stData.backlog) info = 'Ältere Tage werden noch ausgewertet (' + stData.backlog + ' Tage übrig) …';
  $('stInfo').textContent = info;

  // Zeitraum -> Balken (slots) und Tage fuer die Kacheln
  let slots = [], title, unit = 'Tag', dayRows;
  const dayRow = k => all.filter(r => r[0] === k);
  const daySlots = (a, b) => {
    for (let d = new Date(a); d <= b; d = addDays(d, 1))
      slots.push({lb: WD[d.getDay()] + ' ' + dDE(d), tip: WD[d.getDay()] + ' ' + d.toLocaleDateString('de-DE'), rows: dayRow(dKey(d)), key: dKey(d)});
  };
  if (v.mode === 'y') {
    title = String(v.y); unit = 'Monat';
    for (let m = 0; m < 12; m++) slots.push({lb: MONTHS[m], tip: MONTHS[m] + ' ' + v.y, m, rows: all.filter(r => Math.floor(r[0] / 10000) === v.y && Math.floor(r[0] / 100) % 100 === m + 1)});
  } else if (v.mode === 'm') {
    title = MONTHS[v.m] + ' ' + v.y;
    const nd = new Date(v.y, v.m + 1, 0).getDate();
    for (let d = 1; d <= nd; d++) { const k = v.y * 10000 + (v.m + 1) * 100 + d; slots.push({lb: String(d), tip: d + '. ' + MONTHS[v.m] + ' ' + v.y, rows: dayRow(k), key: k}); }
  } else if (v.mode === 'w') {
    const mo = addDays(mondayOf(new Date()), v.wk * 7), so = addDays(mo, 6);
    title = 'Woche ' + dDE(mo) + ' – ' + dDE(so);
    daySlots(mo, so);
  } else if (v.mode === 'r') {
    const a = v.rFrom, b = v.rTo, n = Math.round((b - a) / 86400000) + 1;
    title = a.toLocaleDateString('de-DE') + ' – ' + b.toLocaleDateString('de-DE');
    if (n <= 62) daySlots(a, b);
    else {                                                       // lange Zeitraeume: je Monat
      unit = 'Monat';
      const ka = dKey(a), kb = dKey(b);
      for (let d = new Date(a.getFullYear(), a.getMonth(), 1); d <= b; d = new Date(d.getFullYear(), d.getMonth() + 1, 1)) {
        const y = d.getFullYear(), m = d.getMonth();
        slots.push({lb: MONTHS[m] + ' ' + String(y).slice(2), tip: MONTHS[m] + ' ' + y,
          rows: all.filter(r => r[0] >= ka && r[0] <= kb && Math.floor(r[0] / 10000) === y && Math.floor(r[0] / 100) % 100 === m + 1)});
      }
    }
  } else {                                                       // ein Tag: Stunden
    const k = dKey(v.day);
    title = WD[v.day.getDay()] + ' ' + v.day.toLocaleDateString('de-DE') + (k === dKey(new Date()) ? ' (heute)' : '');
    unit = 'Stunde';
    const cached = dayHist[k];
    dayRows = dayRow(k);                                         // Kacheln aus der Tageswertung,
    if (!dayRows.length && cached) {                             // sonst aus den Stundenwerten
      const hs = cached.rows, a = stAgg(hs);
      if (hs.some(r => r[2] > 0 || r[8] !== null))
        dayRows = [[k, a.elN ? a.el : null, a.th, a.elWw, a.thWw, a.run, 0, a.df, a.ta, a.taMin, a.taMax, null, null, null, null]];
    }
    for (let i = 0; i < 24; i++) slots.push({lb: String(i), tip: pad2(i) + ':00–' + pad2(i + 1) + ':00', rows: cached ? [cached.rows[i]] : []});
    if (!cached || (k === dKey(new Date()) && Date.now() - cached.at > 60000))
      loadDayHist(v.day).then(r => { if (r && stView.mode === 'd' && dKey(stView.day) === k) renderStats(); });
  }
  $('stTitle').textContent = title;
  if (!dayRows) dayRows = slots.flatMap(s => s.rows);
  stSel = new Set(v.mode === 'd' ? [dKey(v.day)] : dayRows.map(r => r[0]));
  const A = stAgg(dayRows);
  const price = Number(lsGet('avPrice', 0.30));
  const tiles = [
    ['Wärme erzeugt', fk(A.th, A.th < 100 ? 1 : 0) + '<small>kWh</small>', 'davon Warmwasser ' + fk(A.thWw, 1) + ' kWh', '#e88a1a'],
    ['Strom verbraucht', fk(A.el, A.el < 100 ? 1 : 0) + '<small>kWh</small>', A.elN < A.n ? 'nur ' + A.elN + ' von ' + A.n + ' Tagen mit Stromwert' : 'Shelly bzw. Schätzung (System)', '#3570d4'],
    ['Arbeitszahl', A.cop === null ? '–' : fk(A.cop, 2), 'Wärme ÷ Strom', '#2f9e5a'],
    ['Kosten (ca.)', A.el ? fk(A.el * price, A.el * price < 20 ? 2 : 0) + '<small>€</small>' : '–', 'bei ' + fk(price, 2) + ' €/kWh' + (A.elG > 0 ? ' · Netzstrom ≈ ' + fk((A.el - A.own) * price, 2) + ' €' : ''), '#c0504d'],
    ['Laufzeit', fk(A.run, 1) + '<small>h</small>', fk(A.starts, 0) + ' Starts' + (A.starts ? ' · Ø ' + fk(A.run * 60 / A.starts, 0) + ' min' : ''), '#7a8aa0'],
    ['Außen Ø', A.ta === null ? '–' : fk(A.ta, 1) + '<small>°C</small>', A.taMin === null ? '' : 'min ' + fk(A.taMin, 1) + ' / max ' + fk(A.taMax, 1), '#5f7488'],
    !FEAT.pv ? null : ['PV-Ertrag', A.pvN ? fk(A.pv, 1) + '<small>kWh</small>' : '–', A.elG > 0 ? 'WP-Strom aus eigener Anlage ' + fk(A.own / A.elG * 100, 0) + ' %' : '', '#c9a400'],
    ['Abtauungen', fk(A.df, 0), A.n > 1 ? fk(A.df / A.n, 1) + ' je Tag' : '', '#7fb2ff'],
  ];
  $('stTiles').innerHTML = tiles.filter(t => t).map(t => '<div class="tile" style="border-left-color:' + t[3] + '"><div class="l">' + t[0] + '</div><div class="v">' + t[1] + '</div><div class="s">' + t[2] + '</div></div>').join('');
  drawStChart(slots);
  const refY = v.mode === 'd' ? v.day.getFullYear() : v.mode === 'w' ? addDays(mondayOf(new Date()), v.wk * 7).getFullYear() : v.mode === 'r' ? v.rTo.getFullYear() : v.y;
  drawStScatter(v.mode === 'r' ? all.filter(r => r[0] >= dKey(v.rFrom) - 10000 && r[0] <= dKey(v.rTo)) : all.filter(r => Math.floor(r[0] / 10000) === refY));
  // Tabelle
  let h = '<tr><th>' + unit + '</th><th>Wärme kWh</th><th>davon WW</th><th>Strom kWh</th><th>Arbeitszahl</th><th>Laufzeit h</th><th>Starts</th><th>Abtau.</th><th>Außen Ø</th><th>PV kWh</th></tr>';
  for (const s of slots) {
    if (!s.rows.length) continue;
    const a = stAgg(s.rows);
    if (v.mode === 'd' && a.th < 0.005 && a.el < 0.005 && a.ta === null) continue;
    let lb = s.lb;
    if (v.mode === 'y') lb = '<a href="#statistik" onclick="stView.mode=\'m\';stView.m=' + s.m + ';renderStats();return false">' + s.lb + '</a>';
    else if (s.key) lb = '<a href="#statistik" onclick="stView.mode=\'d\';stView.day=new Date(' + Math.floor(s.key / 10000) + ',' + (Math.floor(s.key / 100) % 100 - 1) + ',' + (s.key % 100) + ');renderStats();return false">' +
                         (v.mode === 'm' ? s.lb + '.' : s.lb) + '</a>' + (s.rows[0][15] === 'heute' ? ' (heute)' : '');
    else if (v.mode === 'd') lb = s.tip;
    h += '<tr><td>' + lb + '</td><td>' + fk(a.th, 1) + '</td><td>' + fk(a.thWw, 1) + '</td><td>' + (a.elN ? fk(a.el, 1) : '–') + '</td><td>' + (a.cop === null ? '–' : fk(a.cop, 2)) +
         '</td><td>' + fk(a.run, 1) + '</td><td>' + (v.mode === 'd' ? '–' : a.starts) + '</td><td>' + a.df + '</td><td>' + (a.ta === null ? '–' : fk(a.ta, 1)) + '</td><td>' + (a.pvN ? fk(a.pv, 1) : '–') + '</td></tr>';
  }
  if (dayRows.length) {
    h += '<tr style="font-weight:600"><td>Summe</td><td>' + fk(A.th, 1) + '</td><td>' + fk(A.thWw, 1) + '</td><td>' + (A.elN ? fk(A.el, 1) : '–') + '</td><td>' + (A.cop === null ? '–' : fk(A.cop, 2)) +
         '</td><td>' + fk(A.run, 1) + '</td><td>' + A.starts + '</td><td>' + A.df + '</td><td>' + (A.ta === null ? '–' : fk(A.ta, 1)) + '</td><td>' + (A.pvN ? fk(A.pv, 1) : '–') + '</td></tr>';
  } else h += '<tr><td colspan="10" class="mut">Keine Daten in diesem Zeitraum.</td></tr>';
  $('stTbl').innerHTML = h;
}
let stGeo = null;
function drawStChart(slots) {
  const cv = $('stChart'), g = cv.getContext('2d'), W = cv.width, H = cv.height;
  g.clearRect(0, 0, W, H); g.fillStyle = '#fff'; g.fillRect(0, 0, W, H);
  const L = 54, R = 46, T = 16, B = 30, pw = W - L - R, ph = H - T - B;
  const ag = slots.map(s => stAgg(s.rows));
  let mx = 0.5, mc = 4;
  for (const a of ag) { mx = Math.max(mx, a.th, a.el); if (a.cop !== null) mc = Math.max(mc, Math.ceil(a.cop)); }
  const nice = v => { const p = Math.pow(10, Math.floor(Math.log10(v))); for (const m of [1, 2, 2.5, 5, 10]) if (m * p >= v) return m * p; return 10 * p; };
  mx = nice(mx);
  g.font = '12px sans-serif';
  for (let i = 0; i <= 5; i++) {
    const y = T + (1 - i / 5) * ph;
    g.strokeStyle = '#e6e9ee'; g.beginPath(); g.moveTo(L, y); g.lineTo(L + pw, y); g.stroke();
    g.fillStyle = '#667'; g.fillText(fk(mx * i / 5, mx < 10 ? 1 : 0), 6, y + 4);
    g.fillStyle = '#2f9e5a'; g.fillText(fk(mc * i / 5, 1), L + pw + 8, y + 4);
  }
  g.fillStyle = '#667'; g.fillText('kWh', 6, 10); g.fillStyle = '#2f9e5a'; g.fillText('AZ', L + pw + 8, 10);
  const sw = pw / Math.max(1, slots.length), bw = Math.max(2, Math.min(28, sw * 0.36));
  const y = v => T + (1 - v / mx) * ph;
  const cops = [];
  const every = Math.ceil(slots.length / 16);
  slots.forEach((s, i) => {
    const a = ag[i], cx = L + sw * (i + 0.5);
    g.fillStyle = '#f0a347'; g.fillRect(cx - bw - 1, y(a.th - a.thWw), bw, ph + T - y(a.th - a.thWw));
    g.fillStyle = '#f7cf9a'; g.fillRect(cx - bw - 1, y(a.th), bw, y(a.th - a.thWw) - y(a.th));
    if (a.elN) {
      g.fillStyle = '#4a7fd8'; g.fillRect(cx + 1, y(a.el - a.elWw), bw, ph + T - y(a.el - a.elWw));
      g.fillStyle = '#a9c3f0'; g.fillRect(cx + 1, y(a.el), bw, y(a.el - a.elWw) - y(a.el));
    }
    if (a.cop !== null && a.el > 0.02) cops.push([cx, T + (1 - Math.min(a.cop, mc) / mc) * ph]);
    g.fillStyle = '#667';
    if (i % every === 0) { const tw = g.measureText(s.lb).width; g.fillText(s.lb, cx - tw / 2, T + ph + 18); }
  });
  g.strokeStyle = '#2f9e5a'; g.lineWidth = 2; g.beginPath();
  cops.forEach((p, i) => i ? g.lineTo(p[0], p[1]) : g.moveTo(p[0], p[1])); g.stroke(); g.lineWidth = 1;
  g.fillStyle = '#2f9e5a'; for (const p of cops) { g.beginPath(); g.arc(p[0], p[1], 3.5, 0, 7); g.fill(); }
  if (stView.mode === 'd' && !slots.some(s => s.rows.length)) { g.fillStyle = '#888'; g.fillText('Lade Stundenwerte …', L + 10, T + 20); }
  stGeo = {L, pw, slots, ag, sw};
}
function stTip(ev) {
  const tip = $('stTip'), cv = $('stChart');
  if (!stGeo) return;
  const r = cv.getBoundingClientRect(), x = (ev.clientX - r.left) * cv.width / r.width;
  const i = Math.floor((x - stGeo.L) / stGeo.sw);
  if (i < 0 || i >= stGeo.slots.length || !stGeo.slots[i].rows.length) { tip.style.display = 'none'; return; }
  const a = stGeo.ag[i], s = stGeo.slots[i];
  tip.innerHTML = '<b>' + s.tip + '</b>' +
    '<div><i style="background:#f0a347"></i>Wärme <b>' + fk(a.th, 2) + ' kWh</b> (WW ' + fk(a.thWw, 2) + ')</div>' +
    '<div><i style="background:#4a7fd8"></i>Strom <b>' + (a.elN ? fk(a.el, 2) + ' kWh' : '–') + '</b></div>' +
    '<div><i style="background:#2f9e5a"></i>Arbeitszahl <b>' + (a.cop === null ? '–' : fk(a.cop, 2)) + '</b></div>' +
    (a.ta !== null ? '<div><i style="background:#5f7488"></i>Außen Ø <b>' + fk(a.ta, 1) + ' °C</b></div>' : '');
  tip.style.display = 'block';
  const px = ev.clientX - r.left;
  tip.style.left = (px > r.width / 2 ? px - tip.offsetWidth - 14 : px + 14) + 'px';
  tip.style.top = Math.max(0, ev.clientY - r.top - 30) + 'px';
}
function drawStScatter(rows) {
  const cv = $('stScatter'), g = cv.getContext('2d'), W = cv.width, H = cv.height;
  g.clearRect(0, 0, W, H); g.fillStyle = '#fff'; g.fillRect(0, 0, W, H);
  const pts = rows.filter(r => r[8] !== null && r[2] !== null).map(r => [r[8], r[2] - (r[4] || 0), r[0]]);
  const L = 44, R = 12, T = 12, B = 30, pw = W - L - R, ph = H - T - B;
  let mx = 10; for (const p of pts) mx = Math.max(mx, p[1]); mx = Math.ceil(mx / 10) * 10;
  const tMin = -15, tMax = 25, x = t => L + (t - tMin) / (tMax - tMin) * pw, y = v => T + (1 - v / mx) * ph;
  g.font = '12px sans-serif';
  for (let t = tMin; t <= tMax; t += 5) { g.strokeStyle = '#e6e9ee'; g.beginPath(); g.moveTo(x(t), T); g.lineTo(x(t), T + ph); g.stroke(); g.fillStyle = '#667'; g.fillText(t + '°', x(t) - 8, T + ph + 16); }
  for (let i = 0; i <= 5; i++) { const v = mx * i / 5; g.strokeStyle = '#e6e9ee'; g.beginPath(); g.moveTo(L, y(v)); g.lineTo(L + pw, y(v)); g.stroke(); g.fillStyle = '#667'; g.fillText(fk(v, 0), 6, y(v) + 4); }
  if (!pts.length) { g.fillStyle = '#888'; g.fillText('Noch keine Tage mit Außentemperatur.', L + 10, T + 20); return; }
  for (const p of pts) {
    const sel = stView.mode !== 'y' && stSel && stSel.has(p[2]);
    g.fillStyle = sel ? 'rgba(232,138,26,.95)' : 'rgba(53,112,212,.45)';
    g.beginPath(); g.arc(x(Math.max(tMin, Math.min(tMax, p[0]))), y(p[1]), sel ? 4.5 : 3.5, 0, 7); g.fill();
  }
}
// ---------- Einstellungen sichern / einspielen ----------
// Nutzt die normalen Lese-/Speicher-Adressen, damit beim Einspielen dieselben Pruefungen greifen.
const CFG_SEC = [
  {k:'fc', get:'/fc', post:'/fc', keys:['en','db','int','nightCap','dfOn','dfHz','dfBack','offLower','offHz','fbOn','fbLo','fbHi','fbEv','fbStep','fbMin','fbSrc'], arr:{ta:'ta', hzHeat:'hh', hzWw:'hw'}},
  {k:'hc', get:'/hc', post:'/hc', keys:['en','roomOn','day','nightSoll','nightOn','ns','ne','nshift','nwOn','nws','nwe','k','ki','max','vlMin','vlMax','ws','db','tau',
    'auto','mode','taOn','taOff','rtOn','rtOff','minOn','minOff','kTa','kRt','kMax','type','off','lkEn','lkCold','lkTa','lkWw','ncOn','noffOn','noffTa'], pins:['pin'],
    arr:{cTa:'cTa', cVl:'cVl', lkF:'lkF', lkT:'lkT', lkF2:'lkF2', lkT2:'lkT2', nTa:'nTa', nDv:'nDv'}},
  {k:'ctrl', get:'/ctrl', post:'/ctrlset', keys:['enabled','wpOn','actLow','dirInv','travel','buf','defAct','wwSoll','boost','maxMin','cool','drive','slaveId','fsMin','cont','pull','vlMax','abOn','abK','abLow','abFix','abFixT','wwMode'],
    pins:['pinPwr','pinDir'], sched:true},
  {k:'pv', get:'/pvcfg?export=1', post:'/pvcfg', keys:['mEnabled','host','port','user','pass','base','interval','pvWw','pvWwSoll','pvWwHyst','heat','heatSoll',
    'shelly','gen','heatMax','sgTo','sg1','vl','vlMin','vlMax','vlSec','phases','pfac','elSrc','elTopic']},
  {k:'ftp', get:'/ftpcfg?export=1', post:'/ftpcfg', keys:['fEn','host','port','user','pass','dir','int','keep']},
  {k:'feat', get:'/feat', post:'/feat', keys:['pv']},
  {k:'mb', get:'/mbcfg', post:'/mbcfg', keys:['baud','id','gap'], pins:['rx','tx','de','mode']},   // zuletzt (loest Neustart aus)
];
const CFG_NAMES = {feat:'Funktionen', fc:'Verdichter', hc:'Heizung', ctrl:'Warmwasser/Ventil', pv:'PV/MQTT', ftp:'NAS', mb:'Modbus'};
// alle Einstellungen als Objekt (Format fmt 1, fuer Datei und NAS gleich)
async function cfgBuild() {
    const cur = await getJson('/current');
    const out = {avarma:'settings', fmt:1, fw:cur.fw || '', plat:cur.plat || 'ESP32-S3', date:new Date().toISOString()};
    for (const s of CFG_SEC) {
      const j = await getJson(s.get), o = {};
      for (const k of [...s.keys, ...(s.pins || []), ...Object.keys(s.arr || {})]) if (j[k] !== undefined && j[k] !== null) o[k] = j[k];
      if (s.sched && j.sched) o.sched = j.sched;
      out[s.k] = o;
    }
    try { const w = await getJson('/wppar'); if (w.valid) out.wpPar = {p105:w.p105, p106:w.p106, p06:w.p06, p07:w.p07, info:'nur zur Info, wird nicht eingespielt'}; } catch(e) {}
    out.web = {avPrice: Number(lsGet('avPrice', 0.30))};
    return out;
}
function cfgText(out) { return JSON.stringify(out, null, 1).replace(/\[\s*([-\d.,\s]+?)\s*\]/g, (x, y) => '[' + y.replace(/\s+/g, '') + ']'); }   // Zahlenlisten einzeilig
async function cfgExport() {
  const m = $('cfgMsg'); m.textContent = 'Lese Einstellungen …';
  try {
    const out = await cfgBuild();
    const d = new Date(), fn = 'avarma_einstellungen_' + d.getFullYear() + '-' + pad2(d.getMonth() + 1) + '-' + pad2(d.getDate()) + '.json';
    const a = document.createElement('a');
    const txt = cfgText(out);
    a.href = URL.createObjectURL(new Blob([txt], {type:'application/json'}));
    a.download = fn; document.body.appendChild(a); a.click(); a.remove();
    m.textContent = 'Gespeichert: ' + fn;
  } catch(e) { m.textContent = 'Fehler beim Lesen: ' + e; }
}
async function cfgImport(file) {
  if (!file) return;
  let d;
  try { d = JSON.parse(await file.text()); } catch(e) { $('cfgMsg').textContent = 'Keine gültige Datei'; return; }
  cfgApply(d, file.name);
}
// ---- Einstellungen auf dem NAS ----
let nasOn = false, nasTimer = 0, cfgApplying = false;
async function nasSave(auto) {
  if (!nasOn) { if (!auto) $('cfgMsg').textContent = 'Erst unten die NAS-Ablage einschalten und speichern.'; return; }
  if (cfgApplying) return;
  if (!auto) $('cfgMsg').textContent = 'Sichere aufs NAS …';
  try {
    const txt = cfgText(await cfgBuild());
    const r = await (await fetch('/cfgnas?a=put', {method:'POST', headers:{'Content-Type':'text/plain'}, body:txt})).json();
    if (!auto || !r.ok) $('cfgMsg').textContent = r.ok ? 'Auf dem NAS gesichert: einstellungen/' + r.name : 'NAS: ' + r.err;
  } catch(e) { if (!auto) $('cfgMsg').textContent = 'NAS: keine Antwort'; }
}
function nasAuto() { clearTimeout(nasTimer); nasTimer = setTimeout(() => nasSave(true), 4000); }   // nach Aenderungen sammeln
async function nasList() {
  const m = $('cfgMsg'); m.textContent = 'Lese Liste vom NAS …';
  try {
    const r = await getJson('/cfgnas?a=list');
    if (!r.ok) { m.textContent = 'NAS: ' + r.err; return; }
    if (!r.files.length) { m.textContent = 'Auf dem NAS liegt noch keine Sicherung (Ordner einstellungen).'; return; }
    $('cfgNasSel').innerHTML = r.files.map(f => '<option>' + f + '</option>').join('');
    $('cfgNasSel').style.display = $('cfgNasGo').style.display = '';
    m.textContent = r.files.length + ' Sicherungen gefunden – auswählen und „Einspielen“.';
  } catch(e) { m.textContent = 'NAS: keine Antwort'; }
}
async function nasRestore() {
  const f = $('cfgNasSel').value, m = $('cfgMsg');
  if (!f) return;
  m.textContent = 'Lade ' + f + ' …';
  let d;
  try { d = await getJson('/cfgnas?a=get&f=' + encodeURIComponent(f)); } catch(e) { m.textContent = 'NAS: Datei nicht lesbar'; return; }
  if (d && d.ok === false) { m.textContent = 'NAS: ' + d.err; return; }
  cfgApply(d, 'NAS: ' + f);
}
async function cfgApply(d, srcName) {
  const m = $('cfgMsg');
  if (!d || d.avarma !== 'settings') { m.textContent = 'Das ist keine Avarma-Einstellungsdatei'; return; }
  let cur = {};
  try { cur = await getJson('/current'); } catch(e) {}
  const here = cur.plat || 'ESP32-S3', from = d.plat || 'ESP32-S3', same = here === from;
  if (!confirm('Einstellungen aus „' + srcName + '“ (' + from + ', v' + d.fw + ', ' + (d.date || '').slice(0, 10) + ') einspielen?\n\n' +
               (same ? 'Gleicher Board-Typ: Pins werden mit übernommen.' : 'Anderer Board-Typ (' + here + '): Pins bleiben wie sie sind.') +
               '\nDas Gerät startet danach neu.')) return;
  const v = x => x === true ? 1 : x === false ? 0 : x;
  const errs = [], done = [];
  cfgApplying = true;
  try {
    await fetch('/cfgimport?begin=1');
    for (const s of CFG_SEC) {
      const c = d[s.k]; if (!c) continue;
      const p = new URLSearchParams(); p.set('save', '1');
      for (const k of s.keys) if (c[k] !== undefined && c[k] !== null) p.set(k, v(c[k]));
      if (same) for (const k of s.pins || []) if (c[k] !== undefined && c[k] !== null) p.set(k, v(c[k]));
      for (const [k, pk] of Object.entries(s.arr || {})) if (Array.isArray(c[k])) p.set(pk, c[k].join(','));
      if (s.sched && Array.isArray(c.sched)) p.set('sched', c.sched.map(e => e[0] + ':' + e[1]).join(','));
      m.textContent = 'Spiele ein: ' + CFG_NAMES[s.k] + ' …';
      const r = await (await fetch(s.post, {method:'POST', body:p})).json();
      if (r.ok === false) errs.push(CFG_NAMES[s.k] + ': ' + (r.err || 'Fehler')); else done.push(CFG_NAMES[s.k]);
    }
    if (d.web && d.web.avPrice) lsSet('avPrice', Number(d.web.avPrice));
    await fetch('/cfgimport?end=1&from=' + encodeURIComponent(from + ' v' + d.fw));
    m.textContent = (errs.length ? 'Teilweise übernommen. Fehler – ' + errs.join('; ') + '. ' : 'Übernommen: ' + done.join(', ') + '. ') + 'Gerät startet neu …';
    setTimeout(() => location.reload(), 12000);
  } catch(e) { m.textContent = 'Abgebrochen: ' + e; fetch('/cfgimport?end=1&from=abgebrochen'); }
}
function savePrice() { lsSet('avPrice', Number($('stPrice').value) || 0.30); renderStats(); }

// ---------- Funktionen ein-/ausblenden ----------
let FEAT = {pv: 1, ww: 0};
function applyFeat() {
  document.body.classList.toggle('noPv', !FEAT.pv);
  document.body.classList.toggle('wwWp', FEAT.ww === 1);
  if ($('fePv')) $('fePv').checked = !!FEAT.pv;
}
async function loadFeat() {
  try { const r = await getJson('/feat'); FEAT.pv = r.pv ? 1 : 0; } catch(e) {}
  applyFeat();
}
async function saveFeat() {
  FEAT.pv = $('fePv').checked ? 1 : 0; applyFeat();
  try { await getJson('/feat?save=1&pv=' + FEAT.pv); toast('Gespeichert'); } catch(e) { toast('Keine Antwort'); }
}

// ---------- Heizung ----------
let hcLast = null;
function hm(m) { return pad2(Math.floor(m / 60)) + ':' + pad2(m % 60); }
const LK_IDS = ['lkF', 'lkT', 'lkF2', 'lkT2'];
function buildLkTbl() {
  const WD = ['Mo', 'Di', 'Mi', 'Do', 'Fr', 'Sa', 'So'];
  let h = '<tr><th>Tag</th><th colspan="2">1. Zeitraum</th><th colspan="2">2. Zeitraum</th><th></th></tr>';
  for (let i = 0; i < 7; i++)
    h += '<tr><td>' + WD[i] + '</td>' + LK_IDS.map(k => '<td><input id="' + k + i + '" type="time"></td>').join('') +
         '<td><button onclick="LK_IDS.forEach(k=>$(k+' + i + ').value=\'00:00\');markDirty(\'hc\',true)">keine</button></td></tr>';
  $('lkTbl').innerHTML = h;
}
function lkCopy(n) {
  for (let i = 1; i < n; i++) LK_IDS.forEach(k => $(k + i).value = $(k + '0').value);
  markDirty('hc', true);
}
function buildCurveTbl() {
  let h = '<tr><th>Punkt</th><th>Außen °C</th><th>Vorlauf °C</th></tr>';
  for (let i = 0; i < 7; i++)
    h += '<tr><td>' + (i + 1) + '</td><td><input id="cTa' + i + '" type="number" min="-30" max="30" oninput="drawCurve()"></td>' +
         '<td><input id="cVl' + i + '" type="number" min="15" max="65" oninput="drawCurve()"></td></tr>';
  $('curveTbl').innerHTML = h;
}
function buildNCurveTbl() {
  let h = '<tr><th>Punkt</th><th>Außen °C</th><th>Vorlauf nachts K</th></tr>';
  for (let i = 0; i < 7; i++)
    h += '<tr><td>' + (i + 1) + '</td><td><input id="nTa' + i + '" type="number" min="-30" max="30" oninput="drawNCurve()"></td>' +
         '<td><input id="nDv' + i + '" type="number" min="-15" max="5" oninput="drawNCurve()"></td></tr>';
  $('nCurveTbl').innerHTML = h;
}
function drawNCurve() {
  const cv = $('nCurveCv'); if (!cv || !$('hNcOn').checked) return;
  const g = cv.getContext('2d'), W = cv.width, H = cv.height;
  g.clearRect(0, 0, W, H); g.fillStyle = '#fff'; g.fillRect(0, 0, W, H);
  const pts = []; for (let i = 0; i < 7; i++) pts.push([Number($('nTa' + i).value), Number($('nDv' + i).value)]);
  const L = 40, R = 14, T = 14, B = 30, tMin = -20, tMax = 20, vMin = -10, vMax = 4;
  const x = t => L + (tMax - t) / (tMax - tMin) * (W - L - R), y = v => T + (vMax - v) / (vMax - vMin) * (H - T - B);
  g.font = '12px sans-serif'; g.strokeStyle = '#e6e9ee'; g.fillStyle = '#667';
  for (let t = tMin; t <= tMax; t += 5) { g.beginPath(); g.moveTo(x(t), T); g.lineTo(x(t), H - B); g.stroke(); g.fillText(t + '°', x(t) - 8, H - B + 16); }
  for (let v = vMin; v <= vMax; v += 2) { g.beginPath(); g.moveTo(L, y(v)); g.lineTo(W - R, y(v)); g.stroke(); g.fillText((v > 0 ? '+' : '') + v + ' K', 2, y(v) + 4); }
  g.strokeStyle = '#9aa3b2'; g.beginPath(); g.moveTo(L, y(0)); g.lineTo(W - R, y(0)); g.stroke();
  const off = $('hNoffOn').checked ? Number($('hNoffTa').value) : null;
  if (off !== null && off < tMax) {                      // Bereich Nacht-Aus
    g.fillStyle = 'rgba(217,71,58,.08)'; g.fillRect(L, T, x(off) - L, H - T - B);
    g.fillStyle = '#d9473a'; g.fillText('WP aus', L + 6, T + 14);
  }
  g.strokeStyle = '#6d4c9f'; g.lineWidth = 2.5; g.beginPath();
  g.moveTo(x(tMax), y(pts[0][1])); for (const p of pts) g.lineTo(x(p[0]), y(p[1])); g.lineTo(x(tMin), y(pts[6][1])); g.stroke();
  g.fillStyle = '#6d4c9f'; for (const p of pts) { g.beginPath(); g.arc(x(p[0]), y(p[1]), 4, 0, 7); g.fill(); }
  if (hcLast && hcLast.taF !== null && hcLast.nNow !== null && hcLast.ncOn) {
    const tx = x(Math.max(tMin, Math.min(tMax, hcLast.taF)));
    g.fillStyle = '#2f6fdb'; g.beginPath(); g.arc(tx, y(hcLast.nNow), 6, 0, 7); g.fill();
    g.fillText('jetzt', tx + 8, y(hcLast.nNow) - 8);
  }
  g.lineWidth = 1;
}
function drawCurve() {
  const cv = $('curveCv'), g = cv.getContext('2d'), W = cv.width, H = cv.height;
  g.clearRect(0, 0, W, H); g.fillStyle = '#fff'; g.fillRect(0, 0, W, H);
  const pts = []; for (let i = 0; i < 7; i++) pts.push([Number($('cTa' + i).value), Number($('cVl' + i).value)]);
  const L = 40, R = 14, T = 14, B = 30, tMin = -20, tMax = 20, vMin = 20, vMax = 60;
  const x = t => L + (tMax - t) / (tMax - tMin) * (W - L - R), y = v => T + (vMax - v) / (vMax - vMin) * (H - T - B);
  g.font = '12px sans-serif'; g.strokeStyle = '#e6e9ee'; g.fillStyle = '#667';
  for (let t = tMin; t <= tMax; t += 5) { g.beginPath(); g.moveTo(x(t), T); g.lineTo(x(t), H - B); g.stroke(); g.fillText(t + '°', x(t) - 8, H - B + 16); }
  for (let v = vMin; v <= vMax; v += 10) { g.beginPath(); g.moveTo(L, y(v)); g.lineTo(W - R, y(v)); g.stroke(); g.fillText(v + '°', 6, y(v) + 4); }
  g.strokeStyle = '#d9473a'; g.lineWidth = 2.5; g.beginPath();
  g.moveTo(x(tMax), y(pts[0][1])); for (const p of pts) g.lineTo(x(p[0]), y(p[1])); g.lineTo(x(tMin), y(pts[6][1])); g.stroke();
  g.fillStyle = '#d9473a'; for (const p of pts) { g.beginPath(); g.arc(x(p[0]), y(p[1]), 4, 0, 7); g.fill(); }
  if (hcLast && hcLast.taF !== null && hcLast.curve !== null) {
    g.fillStyle = '#2f6fdb'; g.beginPath(); g.arc(x(Math.max(tMin, Math.min(tMax, hcLast.taF))), y(hcLast.curve), 6, 0, 7); g.fill();
    g.fillText('jetzt', x(Math.max(tMin, Math.min(tMax, hcLast.taF))) + 8, y(hcLast.curve) - 8);
  }
  g.lineWidth = 1;
}
async function loadHc(full) {
  let c;
  try { c = await getJson('/hc'); } catch(e) { return; }
  hcLast = c;
  const rt = c.rt === null ? '–' : c.rt.toFixed(1);
  setV('hRt', rt, '°C'); setV('tRt', rt, '°C');
  $('hRh').textContent = c.rt === null ? 'Fühler fehlt (IO' + c.pin + ')' : 'Feuchte ' + c.rh.toFixed(0) + ' %';
  $('tRtS').textContent = 'Soll ' + c.rtSoll.toFixed(1) + (c.kamin ? ' · Kamin' : c.night ? ' · Nacht' : '');
  setV('hSoll', c.rtSoll.toFixed(1), '°C'); $('hSollL').textContent = c.night ? 'Raum soll (Nacht)' : 'Raum soll (Tag)';
  $('qSoll').textContent = c.rtSoll.toFixed(1) + ' °C'; $('qSollL').textContent = c.night ? 'Raum-Soll Nacht' : 'Raum-Soll Tag';
  q('qHc', c.en, c.en ? 'EIN · VL ' + (c.targetF === null ? '–' : c.targetF.toFixed(1) + ' °C') : 'AUS · VL von Hand');
  q('qRoom', c.roomOn, c.roomOn ? 'EIN · ' + (c.corr >= 0 ? '+' : '') + c.corr.toFixed(1) + ' K' : 'AUS');
  q('qAuto', c.auto, c.auto ? c.autoInfo : 'AUS · WP von Hand');
  q('qKamin', c.kamin, c.kamin ? 'AN seit ' + c.kMin + ' min' : 'aus');
  const corrTxt = !c.roomOn ? 'aus' : c.rt === null ? 'kein Raumfühler' : (c.corr >= 0 ? '+' : '') + c.corr.toFixed(1) + ' K' + (c.i ? ' (davon I ' + c.i.toFixed(1) + ')' : '');
  kv('hcKv', [['Außen (WP)', c.ta === null ? '–' : c.ta.toFixed(1) + ' °C'], ['Außen geglättet', c.taF === null ? '–' : c.taF.toFixed(1) + ' °C'],
    ['① VL laut Kurve', c.curve === null ? '–' : c.curve.toFixed(1) + ' °C'],
    ['② Raumeinfluss', corrTxt],
    ['③ Nacht', c.night ? 'ja, ' + (c.nNow > 0 ? '+' : '') + c.nNow.toFixed(1) + ' K' + (c.ncOn ? ' (Kurve)' : '') + (c.nOff ? ' · WP aus (Nacht-Aus)' : '') : 'nein'],
    ['= VL-Soll berechnet', c.targetF === null ? '–' : c.targetF.toFixed(1) + ' °C'],
    ['VL-Soll an der WP', c.wpSoll === null ? '–' : c.wpSoll + ' °C'],
    ['Schreiben', c.en ? c.vlInfo + ' · ' + c.vlW + '× heute' : 'Heizkurve aus – nur Anzeige'],
    ['④ WP auto ein/aus', c.auto ? c.autoInfo : 'aus'],
    ['⑤ Sperrzeit', c.lkEn ? c.lkInfo : 'aus'],
    ['Raumfühler', 'OK ' + c.dhtOk + ' · Fehler ' + c.dhtErr]]);
  if (full) {
    $('hEn').checked = c.en; $('hRoom').checked = c.roomOn; $('hDay').value = c.day; $('hNightSoll').value = c.nightSoll;
    $('hNight').checked = c.nightOn; $('hNs').value = hm(c.ns); $('hNe').value = hm(c.ne); $('hNshift').value = c.nshift;
    $('hNwOn').checked = !!c.nwOn; $('hNws').value = hm(c.nws ?? 1380); $('hNwe').value = hm(c.nwe ?? 480);
    $('hNcOn').checked = !!c.ncOn; $('hNoffOn').checked = !!c.noffOn; $('hNoffTa').value = c.noffTa ?? 10;
    if (c.nTa) for (let i = 0; i < 7; i++) { $('nTa' + i).value = c.nTa[i]; $('nDv' + i).value = c.nDv[i]; }
    $('hK').value = c.k; $('hKi').value = c.ki; $('hMax').value = c.max; $('hVlMin').value = c.vlMin; $('hVlMax').value = c.vlMax;
    $('hWs').value = c.ws; $('hDb').value = c.db; $('hTau').value = c.tau; $('hAuto').checked = c.auto; $('hMode').value = c.mode;
    $('hTaOn').value = c.taOn; $('hTaOff').value = c.taOff; $('hRtOn').value = c.rtOn; $('hRtOff').value = c.rtOff;
    $('hMinOn').value = c.minOn; $('hMinOff').value = c.minOff; $('hKTa').value = c.kTa; $('hKRt').value = c.kRt;
    $('hKMaxOn').checked = c.kMax > 0; $('hKMax').value = c.kMax > 0 ? c.kMax : 8;
    $('hPin').value = c.pin; $('hType').value = c.type; $('hOff').value = c.off;
    for (let i = 0; i < 7; i++) { $('cTa' + i).value = c.cTa[i]; $('cVl' + i).value = c.cVl[i]; }
    $('lkEn').checked = c.lkEn; $('lkCold').checked = c.lkCold; $('lkTa').value = c.lkTa; $('lkWw').checked = c.lkWw;
    if (c.lkF) for (let i = 0; i < 7; i++) {
      $('lkF' + i).value = hm(c.lkF[i]); $('lkT' + i).value = hm(c.lkT[i]);
      $('lkF2' + i).value = hm(c.lkF2 ? c.lkF2[i] : 0); $('lkT2' + i).value = hm(c.lkT2 ? c.lkT2[i] : 0);
    }
    markDirty('hc', false); $('hcMsg').textContent = ''; applyDeps();
  }
  $('lkState').textContent = !c.lkEn ? 'ausgeschaltet' : c.lkInfo + (c.lkOn ? ' – WP gesperrt' : '');
  $('lkState').style.color = c.lkOn ? 'var(--red)' : '';
  drawCurve(); drawNCurve();
}
async function saveHc() {
  const tm = id => { const [a, b] = ($(id).value || '00:00').split(':').map(Number); return a * 60 + b; };
  const p = new URLSearchParams({ save: 1, en: $('hEn').checked ? 1 : 0, roomOn: $('hRoom').checked ? 1 : 0,
    day: $('hDay').value, nightSoll: $('hNightSoll').value,
    nightOn: $('hNight').checked ? 1 : 0, ns: tm('hNs'), ne: tm('hNe'), nshift: $('hNshift').value,
    nwOn: $('hNwOn').checked ? 1 : 0, nws: tm('hNws'), nwe: tm('hNwe'), k: $('hK').value, ki: $('hKi').value,
    max: $('hMax').value, vlMin: $('hVlMin').value, vlMax: $('hVlMax').value, ws: $('hWs').value, db: $('hDb').value, tau: $('hTau').value,
    auto: $('hAuto').checked ? 1 : 0, mode: $('hMode').value, taOn: $('hTaOn').value, taOff: $('hTaOff').value,
    rtOn: $('hRtOn').value, rtOff: $('hRtOff').value, minOn: $('hMinOn').value, minOff: $('hMinOff').value,
    kTa: $('hKTa').value, kRt: $('hKRt').value, kMax: $('hKMaxOn').checked ? $('hKMax').value : 0,
    pin: $('hPin').value, type: $('hType').value, off: $('hOff').value,
    lkEn: $('lkEn').checked ? 1 : 0, lkCold: $('lkCold').checked ? 1 : 0, lkTa: $('lkTa').value, lkWw: $('lkWw').checked ? 1 : 0,
    lkF: [...Array(7).keys()].map(i => tm('lkF' + i)).join(','), lkT: [...Array(7).keys()].map(i => tm('lkT' + i)).join(','),
    lkF2: [...Array(7).keys()].map(i => tm('lkF2' + i)).join(','), lkT2: [...Array(7).keys()].map(i => tm('lkT2' + i)).join(','),
    cTa: [...Array(7).keys()].map(i => $('cTa' + i).value).join(','), cVl: [...Array(7).keys()].map(i => $('cVl' + i).value).join(','),
    ncOn: $('hNcOn').checked ? 1 : 0, noffOn: $('hNoffOn').checked ? 1 : 0, noffTa: $('hNoffTa').value,
    nTa: [...Array(7).keys()].map(i => $('nTa' + i).value).join(','), nDv: [...Array(7).keys()].map(i => $('nDv' + i).value).join(',') });
  try {
    const r = await getJson('/hc?' + p.toString());
    if (!r.ok) { $('hcMsg').textContent = 'Fehler: ' + r.err; return; }
    if (r.restart) { $('hcMsg').textContent = 'Gespeichert, Neustart...'; return; }
  } catch(e) { $('hcMsg').textContent = 'Keine Antwort'; return; }
  toast('Heizung gespeichert'); loadHc(true);
}
async function hcCmd(a) {
  try { await fetch('/hccmd?a=' + a); } catch(e) {}
  await loadHc(!dirty.hc);
  const m = {hc:['hEn','en'], room:['hRoom','roomOn'], auto:['hAuto','auto']}[a];
  if (m && hcLast) { $(m[0]).checked = hcLast[m[1]]; applyDeps(); }    // Schalter im Formular nachziehen
}

// ---------- Verdichter ----------
let fcLast = null;
function buildFcTbl() {
  let h = '<tr><th>Punkt</th><th>Außen °C</th><th>Heizen Hz</th><th>Warmwasser Hz</th></tr>';
  for (let i = 0; i < 7; i++)
    h += '<tr><td>' + (i + 1) + '</td><td><input id="fTa' + i + '" type="number" min="-30" max="30" oninput="drawFc()"></td>' +
         '<td><input id="fH' + i + '" type="number" min="30" max="85" oninput="drawFc()"></td>' +
         '<td><input id="fW' + i + '" type="number" min="30" max="85" oninput="drawFc()"></td></tr>';
  $('fcTbl').innerHTML = h;
}
function drawFc() {
  const cv = $('fcCv'), g = cv.getContext('2d'), W = cv.width, H = cv.height;
  g.clearRect(0, 0, W, H); g.fillStyle = '#fff'; g.fillRect(0, 0, W, H);
  const L = 40, R = 14, T = 14, B = 30, tMin = -20, tMax = 20, vMin = 30, vMax = 90;
  const x = t => L + (tMax - t) / (tMax - tMin) * (W - L - R), y = v => T + (vMax - v) / (vMax - vMin) * (H - T - B);
  g.font = '12px sans-serif'; g.strokeStyle = '#e6e9ee'; g.fillStyle = '#667';
  for (let t = tMin; t <= tMax; t += 5) { g.beginPath(); g.moveTo(x(t), T); g.lineTo(x(t), H - B); g.stroke(); g.fillText(t + '°', x(t) - 8, H - B + 16); }
  for (let v = vMin; v <= vMax; v += 10) { g.beginPath(); g.moveTo(L, y(v)); g.lineTo(W - R, y(v)); g.stroke(); g.fillText(v, 10, y(v) + 4); }
  const ser = (col, pre) => {
    const pts = []; for (let i = 0; i < 7; i++) pts.push([Number($('fTa' + i).value), Number($(pre + i).value)]);
    g.strokeStyle = col; g.lineWidth = 2.5; g.beginPath();
    g.moveTo(x(tMax), y(pts[0][1])); for (const p of pts) g.lineTo(x(p[0]), y(p[1])); g.lineTo(x(tMin), y(pts[6][1])); g.stroke();
    g.fillStyle = col; for (const p of pts) { g.beginPath(); g.arc(x(p[0]), y(p[1]), 4, 0, 7); g.fill(); }
  };
  ser('#2f9e5a', 'fH'); ser('#e88a1a', 'fW');
  g.lineWidth = 1; g.fillStyle = '#2f9e5a'; g.fillRect(L + 6, 8, 12, 4); g.fillStyle = '#333'; g.fillText('Heizen', L + 22, 14);
  g.fillStyle = '#e88a1a'; g.fillRect(L + 80, 8, 12, 4); g.fillStyle = '#333'; g.fillText('Warmwasser', L + 96, 14);
  if (fcLast && fcLast.taF !== null && fcLast.target > 0) {
    const tx = x(Math.max(tMin, Math.min(tMax, fcLast.taF)));
    g.fillStyle = '#2f6fdb'; g.beginPath(); g.arc(tx, y(fcLast.target), 6, 0, 7); g.fill(); g.fillText('jetzt', tx + 8, y(fcLast.target) - 8);
  }
}
async function loadFc(full) {
  let c;
  try { c = await getJson('/fc'); } catch(e) { return; }
  fcLast = c;
  setV('fHz', c.hz === null ? '–' : c.hz, 'Hz'); $('fMax').textContent = 'Grenze an der WP: ' + (c.maxHz === null ? '–' : c.maxHz + ' Hz');
  setV('fTgt', c.target > 0 ? c.target : '–', 'Hz'); $('fTgtL').textContent = 'Kurve ' + (c.ww ? 'Warmwasser' : 'Heizen');
  $('fInfo').textContent = c.en ? c.info : 'Automatik aus';
  q('qFc', c.en, c.en ? 'EIN · ' + (c.target > 0 ? c.target + ' Hz' : '–') : 'AUS · von Hand');
  kv('fcKv', [['Außen geglättet', c.taF === null ? '–' : c.taF.toFixed(1) + ' °C'], ['Betrieb', c.ww ? 'Warmwasser' : 'Heizen'],
    ['Nacht', c.night ? 'ja' : 'nein'], ['Grenze an der WP', c.maxHz === null ? '–' : c.maxHz + ' Hz'],
    ['Grenze laut Automatik', c.target > 0 ? c.target + ' Hz' + (c.fbCap ? ' (Frost-Bremse)' : '') : '–'], ['Automatik', c.en ? c.info : 'aus'],
    ['Abtauen', (c.dfNow ? 'läuft' + (c.dfInfo ? ' – ' + c.dfInfo : '') : c.dfInfo || 'nein') + ' · ' + c.dfCnt + '× heute'],
    ['Frost-Bremse', c.fbInfo + (c.fbCap && c.curve ? ' · Kurve ' + c.curve + ' Hz' : '')],
    ['Schreibvorgänge heute', 'Max Hz ' + c.hzW + ' · VL ' + c.vlW]]);
  if (full) {
    $('fEn').checked = c.en; $('fDb').value = c.db; $('fInt').value = c.int;
    $('fNightOn').checked = c.nightCap > 0; $('fNight').value = c.nightCap > 0 ? c.nightCap : 45;
    $('fDfOn').checked = c.dfOn; $('fDfHz').value = c.dfHz; $('fDfBack').value = c.dfBack;
    $('fOffOn').checked = c.offLower; $('fOffHz').value = c.offHz;
    $('fFbOn').checked = c.fbOn; $('fFbLo').value = c.fbLo; $('fFbHi').value = c.fbHi; $('fFbEv').value = c.fbEv;
    $('fFbStep').value = c.fbStep; $('fFbMin').value = c.fbMin; $('fFbSrc').value = c.fbSrc;
    for (let i = 0; i < 7; i++) { $('fTa' + i).value = c.ta[i]; $('fH' + i).value = c.hzHeat[i]; $('fW' + i).value = c.hzWw[i]; }
    markDirty('fc', false); $('fcMsg').textContent = ''; applyDeps();
  }
  drawFc();
}
async function saveFc() {
  const col = pre => [...Array(7).keys()].map(i => $(pre + i).value).join(',');
  const p = new URLSearchParams({ save: 1, en: $('fEn').checked ? 1 : 0, db: $('fDb').value, int: $('fInt').value,
    nightCap: $('fNightOn').checked ? $('fNight').value : 0, ta: col('fTa'), hh: col('fH'), hw: col('fW'),
    dfOn: $('fDfOn').checked ? 1 : 0, dfHz: $('fDfHz').value, dfBack: $('fDfBack').value,
    offLower: $('fOffOn').checked ? 1 : 0, offHz: $('fOffHz').value,
    fbOn: $('fFbOn').checked ? 1 : 0, fbLo: $('fFbLo').value, fbHi: $('fFbHi').value, fbEv: $('fFbEv').value,
    fbStep: $('fFbStep').value, fbMin: $('fFbMin').value, fbSrc: $('fFbSrc').value });
  try { const r = await getJson('/fc?' + p.toString()); if (!r.ok) { $('fcMsg').textContent = 'Fehler: ' + r.err; return; } }
  catch(e) { $('fcMsg').textContent = 'Keine Antwort'; return; }
  toast('Verdichter gespeichert'); loadFc(true);
}
// ---------- Abtau-Auswertung ----------
let dfRows = [];
async function loadDfl() {
  let t;
  try { const r = await fetch('/dfl'); t = await r.text(); } catch(e) { $('dfMsg').textContent = 'Keine Antwort'; return; }
  const num = v => (v === undefined || v === '') ? null : parseFloat(v.replace(',', '.'));
  dfRows = t.split('\n').filter(l => /^\d{4}-/.test(l)).map(l => {
    const f = l.split(';');
    return { t: f[0], dur: num(f[1]), ta: num(f[2]), hz: num(f[3]), ev: num(f[4]), coil: num(f[5]), run: num(f[6]),
             max: num(f[7]), cap: num(f[8]), ww: f[9] === 'WW' };
  });
  drawDfl();
}
function drawDfl() {
  const R = dfRows, avg = a => a.length ? a.reduce((x, y) => x + y, 0) / a.length : null;
  const fx = (v, d) => v === null || isNaN(v) ? '–' : v.toFixed(d);
  if (!R.length) {
    $('dfBin').innerHTML = ''; $('dfTbl').innerHTML = '';
    $('dfMsg').textContent = 'Noch keine Abtauung aufgezeichnet (ab v1.31).';
  }
  // nach Aussentemperatur gruppieren
  const bins = [[-99, -6], [-6, -3], [-3, 0], [0, 2], [2, 4], [4, 6], [6, 8], [8, 99]];
  let h = '<tr><th>Außen</th><th>Anzahl</th><th>Ø Laufzeit davor</th><th>Ø Dauer</th><th>Ø Hz</th><th>Ø Verdampfer</th></tr>';
  for (const [a, b] of bins) {
    const r = R.filter(x => x.ta !== null && x.ta >= a && x.ta < b);
    if (!r.length) continue;
    const run = r.filter(x => x.run !== null && !x.ww).map(x => x.run);
    const lab = a < -50 ? 'unter ' + b + ' °C' : b > 50 ? 'ab ' + a + ' °C' : a + ' bis ' + b + ' °C';
    h += '<tr><td>' + lab + '</td><td>' + r.length + '</td><td>' + (run.length ? fx(avg(run), 0) + ' min' : '–') + '</td><td>'
       + fx(avg(r.map(x => x.dur)) / 60, 1) + ' min</td><td>' + fx(avg(r.map(x => x.hz)), 0) + '</td><td>'
       + fx(avg(r.filter(x => x.ev !== null).map(x => x.ev)), 1) + ' °C</td></tr>';
  }
  if (R.length) $('dfBin').innerHTML = h;
  // letzte Abtauungen
  let g = '<tr><th>Beginn</th><th>Dauer</th><th>Außen</th><th>Hz</th><th>Verdampfer</th><th>Coil</th><th>Laufzeit davor</th><th>Bremse</th></tr>';
  for (const x of R.slice(-15).reverse())
    g += '<tr><td>' + x.t.slice(5) + (x.ww ? ' (WW)' : '') + '</td><td>' + fx(x.dur / 60, 1) + ' min</td><td>' + fx(x.ta, 1) + ' °C</td><td>'
       + fx(x.hz, 0) + '</td><td>' + fx(x.ev, 1) + ' °C</td><td>' + fx(x.coil, 1) + ' °C</td><td>'
       + (x.run === null ? '–' : x.run + ' min') + '</td><td>' + (x.cap ? x.cap + ' Hz' : '–') + '</td></tr>';
  if (R.length) {
    $('dfTbl').innerHTML = g;
    const days = {}; for (const x of R) days[x.t.slice(0, 10)] = (days[x.t.slice(0, 10)] || 0) + 1;
    const dk = Object.keys(days).sort().slice(-7);
    $('dfMsg').textContent = R.length + ' Abtauungen gespeichert · letzte Tage: ' + dk.map(k => k.slice(8, 10) + '.' + k.slice(5, 7) + '. ' + days[k] + '×').join(' · ');
  }
  // Streubild: Laufzeit zwischen den Abtauungen ueber Aussentemperatur, Farbe = Hz
  const cv = $('dfCv'); if (!cv) return;
  const g2 = cv.getContext('2d'), W = cv.width, H = cv.height, L = 40, B = 26, T = 22, Rr = 10;
  g2.clearRect(0, 0, W, H); g2.font = '11px system-ui'; g2.fillStyle = '#333';
  const P = R.filter(x => x.ta !== null && x.run !== null && !x.ww);
  if (!P.length) { g2.fillStyle = '#888'; g2.fillText('Streubild erscheint ab der zweiten Abtauung nach einem Neustart', L, H / 2); return; }
  let t0 = Math.floor(Math.min(...P.map(x => x.ta)) - 1), t1 = Math.ceil(Math.max(...P.map(x => x.ta)) + 1);
  const ym = Math.max(60, Math.ceil(Math.max(...P.map(x => x.run)) / 60) * 60), ys = ym > 240 ? 120 : ym > 120 ? 60 : 30;
  const x = v => L + (v - t0) / (t1 - t0) * (W - L - Rr), y = v => H - B - v / ym * (H - B - T);
  g2.strokeStyle = '#e3e7ee'; g2.lineWidth = 1; g2.textAlign = 'right';
  for (let v = 0; v <= ym; v += ys) { g2.beginPath(); g2.moveTo(L, y(v)); g2.lineTo(W - Rr, y(v)); g2.stroke(); g2.fillText(v.toFixed(0), L - 4, y(v) + 4); }
  g2.textAlign = 'center';
  const st = Math.max(1, Math.round((t1 - t0) / 8));
  for (let v = t0 + st; v < t1; v += st) g2.fillText(v + '°', x(v), H - B + 14);
  g2.textAlign = 'left'; g2.fillText('min Laufzeit zwischen Abtauungen · Farbe = Hz (grün wenig, rot viel)', L, 12);
  const hz = P.map(p => p.hz || 0), h0 = Math.min(...hz), h1 = Math.max(...hz);
  for (const p of P) {
    const k = h1 > h0 ? ((p.hz || 0) - h0) / (h1 - h0) : 0.5;
    g2.fillStyle = 'hsl(' + Math.round(120 - 120 * k) + ',70%,45%)';
    g2.beginPath(); g2.arc(x(p.ta), y(Math.min(p.run, ym)), p.cap ? 6 : 4, 0, 7); g2.fill();
    if (p.cap) { g2.strokeStyle = '#2f6fdb'; g2.lineWidth = 2; g2.stroke(); }
  }
  g2.fillStyle = '#555'; g2.textAlign = 'right'; g2.fillText(h0 + '–' + h1 + ' Hz · blauer Rand = Frost-Bremse aktiv', W - Rr, H - 4);
}
// ---------- WP-Parameter ----------
let parInit = false;
async function loadPar(read) {
  let c;
  try { c = await getJson('/wppar' + (read ? '?read=1' : '')); } catch(e) { $('parMsg').textContent = 'Keine Antwort'; return; }
  const k = v => v === null ? '–' : (v / 10).toFixed(1) + ' K (roh ' + v + ')';
  $('parP06').textContent = 'aktuell ' + k(c.p06); $('parP07').textContent = 'aktuell ' + k(c.p07);
  if (!parInit && c.valid) { $('p06Input').value = c.p06 / 10; $('p07Input').value = c.p07 / 10; parInit = true; }
  kv('parKv', [['P105 Plattformfrequenz Verdichter', c.p105 === null ? '–' : c.p105 + ' Hz'],
    ['P106 Plattform-Verzögerung', c.p106 === null ? '–' : c.p106 + ' (Einheit unklar)'],
    ['Gelesen vor', c.valid ? c.age + ' s' : 'noch nicht']]);
  if (read) $('parMsg').textContent = c.ok ? 'gelesen' : 'Lesen fehlgeschlagen';
}
async function setPar(p) {
  const v = Number($(p + 'Input').value);
  $('parMsg').textContent = 'Schreibe...';
  try {
    const c = await getJson('/wppar?set=' + p + '&v=' + v);
    $('parMsg').textContent = c.ok ? p.toUpperCase() + ' = ' + v + ' K übernommen' : 'Fehler: ' + (c.err || 'nicht übernommen');
  } catch(e) { $('parMsg').textContent = 'Keine Antwort'; }
  loadPar(false);
}

async function fcCmd(a) {
  try { await fetch('/fc?cmd=' + a); } catch(e) {}
  await loadFc(!dirty.fc);
  if (fcLast) { $('fEn').checked = fcLast.en; applyDeps(); }
}

// ---------- Warmwasser / Ventil ----------
const DAYS = ['Mo','Di','Mi','Do','Fr','Sa','So'];
const ACTS = [[0,'–'],[1,'WW laden'],[2,'Puffer'],[3,'Heizung']];
function buildSched() {
  const opts = ACTS.map(a => '<option value="' + a[0] + '">' + a[1] + '</option>').join('');
  let h = '';
  for (let d = 0; d < 7; d++) {
    h += '<div class="day"><b>' + DAYS[d] + '</b>';
    for (let s = 0; s < 4; s++)
      h += '<span><input type="time" id="t' + d + '_' + s + '"> <select id="a' + d + '_' + s + '">' + opts + '</select></span>';
    h += '</div>';
  }
  $('schedBox').innerHTML = h;
}
function copyDay(src, dst) {
  for (const d of dst) for (let s = 0; s < 4; s++) {
    $('t' + d + '_' + s).value = $('t' + src + '_' + s).value;
    $('a' + d + '_' + s).value = $('a' + src + '_' + s).value;
  }
  markDirty('ctrl', true);
}
function valveName(c) {
  if (c.drive == 1 && !c.slaveOnline) return 'Ventilmodul nicht erreichbar';
  if (c.vMoving) return 'fährt auf ' + c.vTarget + ' % (jetzt ' + Math.round(c.vPos) + ' %)';
  if (!c.vKnown) return 'Stellung unbekannt';
  const p = Math.round(c.vPos);
  return (p <= 1 ? 'Heizung' : p >= 99 ? 'Warmwasser' : 'Puffer ' + p + ' %');
}
let ctrlLast = null;
// Balkenanzeige Ventilstellung (0 = Heizung, 100 = Warmwasser)
function drawValveGauge(p, c) {
  const pos = Math.max(0, Math.min(100, Number(c.vPos) || 0));
  const known = c.enabled && c.vKnown && !(c.drive == 1 && !c.slaveOnline);
  $(p + 'VgFill').style.width = pos + '%';
  $(p + 'VgKnob').style.left = pos + '%';
  $(p + 'VgBuf').style.left = (c.buf || 50) + '%';
  const t = $(p + 'VgTgt');
  if (c.vMoving) { t.style.display = 'block'; t.style.left = c.vTarget + '%'; } else t.style.display = 'none';
  $(p + 'Vg').classList.toggle('unk', !known);
  let txt;
  if (!c.enabled) txt = 'Steuerung aus';
  else if (c.drive == 1 && !c.slaveOnline) txt = 'Ventilmodul nicht erreichbar';
  else if (c.vMoving) txt = 'fährt ' + Math.round(pos) + ' % → ' + c.vTarget + ' %';
  else if (!c.vKnown) txt = 'Stellung unbekannt';
  else txt = (pos <= 1 ? 'Heizung' : pos >= 99 ? 'Warmwasser' : 'Puffer') + ' · ' + Math.round(pos) + ' %' + (c.manual >= 0 ? ' (Hand)' : '');
  $(p + 'VgTxt').textContent = txt;
}
async function loadCtrl(full) {
  let c;
  try { c = await getJson('/ctrl'); } catch(e) { return; }
  ctrlLast = c;
  for (const p of ['o', 'w']) drawValveGauge(p, c);
  {                                                         // kleine Kachel in der Uebersicht
    const pos = Math.max(0, Math.min(100, Number(c.vPos) || 0));
    const known = c.enabled && c.vKnown && !(c.drive == 1 && !c.slaveOnline);
    $('tVgFill').style.width = pos + '%'; $('tVgKnob').style.left = pos + '%';
    $('tVgTgt').style.display = c.vMoving ? 'block' : 'none'; $('tVgTgt').style.left = c.vTarget + '%';
    $('tVgW').classList.toggle('unk', !known);
    $('tVg').textContent = !c.enabled ? 'aus' : !known ? 'unbekannt' : c.vMoving ? Math.round(pos) + ' % → ' + c.vTarget + ' %' :
      (pos <= 1 ? 'Heizung' : pos >= 99 ? 'Warmwasser' : 'Puffer ' + Math.round(pos) + ' %');
  }
  const v = c.enabled ? valveName(c) + (c.manual >= 0 ? ' (Hand)' : '') + (c.vFail ? ' – NOTFALL' : '') : 'Steuerung aus';
  const ww = c.wwTxt || 'keine Ladung';
  $('oValve').textContent = v; $('oWw').textContent = ww; $('oNext').textContent = c.enabled ? c.next : '–';
  const wpWw = Number(c.wwMode) === 1;                      // WW-Modus der WP: kein Ventil am ESP
  chip('chWw', c.wwTxt ? 'WW lädt' : wpWw ? 'WW ' + (c.enabled ? 'über WP' : 'aus') : 'Ventil ' + (c.enabled ? (c.vMoving ? 'fährt' : valveName(c).split(' ')[0]) : 'aus'), c.wwTxt ? 'on' : '');
  kv('ctrlKv', [['Steuerung', c.enabled ? 'aktiv' + (wpWw ? ' · WW-Modus der WP' : '') : 'aus'], wpWw ? null : ['Ventil', v], wpWw ? null : ['Betrieb', c.manual >= 0 ? 'Hand' : 'Automatik'],
    ['WW-Ladung', ww], ['Nächstes Ereignis', c.next], !wpWw && (c.abOn || c.abLow || c.abFix) && c.abInfo ? ['Puffer-Automatik', c.abInfo] : null,
    !wpWw && c.drive == 1 ? ['Ventilmodul', (c.slaveOnline ? 'online' : 'offline') + ' · OK ' + c.slaveOk + ' · Fehler ' + c.slaveErr + ' · Wiederholt ' + (c.slaveRetry ?? 0)] : null]);
  if (!full) return;
  $('cEnabled').checked = c.enabled; $('cWpOn').checked = c.wpOn; $('cCont').checked = c.cont !== false;
  $('cWwSoll').value = c.wwSoll; $('cBoost').value = c.boost; $('cMaxMin').value = c.maxMin;
  $('cVlMax').value = c.vlMax ?? 60; $('cPull').value = c.pull ?? 30;
  $('cAbOn').checked = !!c.abOn; $('cAbLow').checked = !!c.abLow; $('cAbK').value = c.abK ?? 3;
  $('cAbFix').checked = !!c.abFix; $('cAbFixT').value = c.abFixT ?? 35;
  $('cWwMode').value = c.wwMode ?? 0; FEAT.ww = Number(c.wwMode ?? 0); applyFeat();
  $('cCool').value = c.cool; $('cTravel').value = c.travel; $('cBuf').value = c.buf;
  $('cDefAct').value = c.defAct; $('cPwr').value = c.pinPwr; $('cDir').value = c.pinDir;
  $('cActLow').checked = c.actLow; $('cDirInv').checked = c.dirInv;
  $('cDrive').value = c.drive; $('cSlave').value = c.slaveId; $('cFsMin').value = c.fsMin;
  for (let d = 0; d < 7; d++) for (let s = 0; s < 4; s++) {
    const e = c.sched[d * 4 + s];
    $('t' + d + '_' + s).value = e[1] ? pad2(Math.floor(e[0] / 60)) + ':' + pad2(e[0] % 60) : '';
    $('a' + d + '_' + s).value = e[1];
  }
  markDirty('ctrl', false); $('ctrlMsg').textContent = ''; applyDeps();
}
async function saveCtrl() {
  const sch = [];
  for (let d = 0; d < 7; d++) for (let s = 0; s < 4; s++) {
    const t = $('t' + d + '_' + s).value, a = Number($('a' + d + '_' + s).value);
    if (!t || !a) { sch.push('0:0'); continue; }
    const [hh, mm] = t.split(':').map(Number);
    sch.push((hh * 60 + mm) + ':' + a);
  }
  const p = new URLSearchParams({
    enabled: $('cEnabled').checked ? 1 : 0, wpOn: $('cWpOn').checked ? 1 : 0, cont: $('cCont').checked ? 1 : 0,
    wwSoll: $('cWwSoll').value, boost: $('cBoost').value, maxMin: $('cMaxMin').value,
    vlMax: $('cVlMax').value, pull: $('cPull').value,
    abOn: $('cAbOn').checked ? 1 : 0, abLow: $('cAbLow').checked ? 1 : 0, abK: $('cAbK').value,
    abFix: $('cAbFix').checked ? 1 : 0, abFixT: $('cAbFixT').value, wwMode: $('cWwMode').value,
    cool: $('cCool').value, travel: $('cTravel').value, buf: $('cBuf').value, defAct: $('cDefAct').value,
    pinPwr: $('cPwr').value, pinDir: $('cDir').value,
    actLow: $('cActLow').checked ? 1 : 0, dirInv: $('cDirInv').checked ? 1 : 0, sched: sch.join(','),
    drive: $('cDrive').value, slaveId: $('cSlave').value, fsMin: $('cFsMin').value });
  try {
    const r = await getJson('/ctrlset?' + p.toString());
    if (!r.ok) { $('ctrlMsg').textContent = 'Fehler: ' + r.err; return; }
    if (r.restart) { $('ctrlMsg').textContent = 'Gespeichert, Neustart...'; return; }
  } catch(e) { $('ctrlMsg').textContent = 'Keine Antwort'; return; }
  toast('Warmwasser gespeichert'); loadCtrl(true);
}
async function ctrlCmd(a) {
  let t = '';
  if (ctrlLast && !ctrlLast.enabled) t = 'Die Ventilsteuerung ist aus – im Tab Warmwasser einschalten.';
  else { try { await fetch('/ctrlcmd?a=' + a); t = 'Gesendet'; } catch(e) { t = 'Keine Antwort'; } }
  $('ctrlCmdMsg').textContent = t; $('ctrlCmdMsg2').textContent = t;
  setTimeout(() => { $('ctrlCmdMsg').textContent = ''; $('ctrlCmdMsg2').textContent = ''; }, 5000);
  setTimeout(() => loadCtrl(false), 500);
}

// ---------- PV / MQTT / Elektrik ----------
async function loadPv(full) {
  let c;
  try { c = await getJson('/pvcfg'); } catch(e) { return; }
  kv('pvKv', [['MQTT', !c.mEnabled ? 'aus' : (c.connected ? 'verbunden mit ' + c.host : 'nicht verbunden (' + c.host + ')')],
    ['Nachrichten', 'gesendet ' + c.tx + ' · empfangen ' + c.rx], c.lastCmd ? ['Letzter Befehl', c.lastCmd] : null,
    ['SG-Ready', ['', 'SG1 Sperre', 'SG2 normal', 'SG3 PV-Überschuss', 'SG4 PV max'][c.sg]],
    ['Heizstab', c.heater], ['PV-WW-Ladung', c.pvLoad ? 'läuft' : 'nein']]);
  [1,2,3,4].forEach(n => $('sg' + n).classList.toggle('sel', c.sg === n));
  $('pvTopics').innerHTML = 'Der ESP sendet <b>' + c.base + '/state</b> (JSON). Befehle unter <b>' + c.base +
    '/cmd/</b>: sg 1–4 · vl °C · wwload 0/1/°C · heater min · power · mode · valve · rtsoll · hc · room · autowp · kamin · fc · console. ' +
    'PV-Daten fürs Display unter <b>' + c.base + '/pv/</b>: power · soc · bat · grid · house · today · yesterday.';
  kv('elKv', [['Verwendet', c.pElSrc ? '<b>Shelly (gemessen)</b>' : c.elSrc ? '<b style="color:#c0392b">Schätzung – Shelly fehlt</b>' : 'Schätzung'],
    ['Shelly', c.shOk ? f(c.shP,0) + ' W · vor ' + c.shAge + ' s' : c.shAge >= 0 ? 'keine Daten seit ' + c.shAge + ' s' : 'noch nichts empfangen'],
    ['MQTT vom Shelly', !c.connected ? '<b style="color:#c0392b">ESP nicht mit dem Broker verbunden</b>' :
      c.shAny ? c.shAny + ' Meldungen · zuletzt ' + c.shLast + ' (' + c.shLastLen + ' Byte)' + (c.shBad ? ' · <b style="color:#c0392b">' + c.shBad + ' nicht lesbar</b>' : '') :
      'keine Meldung unter ' + c.elTopic + '/… – Präfix, Broker und „Generic status update“ prüfen'],
    ['Zählerstand Shelly', c.shETot !== null && c.shETot !== undefined ? f(c.shETot,1) + ' kWh' : '–'],
    ['Schätzung WP', f(c.pElWp,0) + ' W'],
    ['Vorschlag Korrektur', c.pfacSug ? Math.round(c.pfacSug) + ' % <small>(Mittel bei laufendem Verdichter)</small>' : '–']]);
  if (!full) return;
  $('pElSrc').value = c.elSrc; $('pElTopic').value = c.elTopic;
  $('mEnabled').checked = c.mEnabled; $('mHost').value = c.host; $('mPort').value = c.port;
  $('mUser').value = c.user; $('mPass').value = c.pass; $('mBase').value = c.base; $('mInt').value = c.interval;
  $('pWw').checked = c.pvWw; $('pWwSoll').value = c.pvWwSoll; $('pWwHyst').value = c.pvWwHyst;
  $('pHeat').checked = c.heat; $('pHeatSoll').value = c.heatSoll; $('pShelly').value = c.shelly;
  $('pGen').value = c.gen; $('pHeatMax').value = c.heatMax; $('pSgTo').value = c.sgTo;
  $('pSg1').checked = c.sg1; $('pVl').checked = c.vl; $('pVlMin').value = c.vlMin;
  $('pVlMax').value = c.vlMax; $('pVlSec').value = c.vlSec;
  $('pPhases').value = c.phases; $('pPfac').value = c.pfac;
  markDirty('pv', false); markDirty('sys', false); $('pvMsg').textContent = ''; $('sysMsg').textContent = ''; applyDeps();
}
async function savePv() {
  const p = new URLSearchParams({ save: 1,
    mEnabled: $('mEnabled').checked ? 1 : 0, host: $('mHost').value, port: $('mPort').value,
    user: $('mUser').value, pass: $('mPass').value, base: $('mBase').value, interval: $('mInt').value,
    pvWw: $('pWw').checked ? 1 : 0, pvWwSoll: $('pWwSoll').value, pvWwHyst: $('pWwHyst').value,
    heat: $('pHeat').checked ? 1 : 0, heatSoll: $('pHeatSoll').value, shelly: $('pShelly').value,
    gen: $('pGen').value, heatMax: $('pHeatMax').value, sgTo: $('pSgTo').value,
    sg1: $('pSg1').checked ? 1 : 0, vl: $('pVl').checked ? 1 : 0, vlMin: $('pVlMin').value,
    vlMax: $('pVlMax').value, vlSec: $('pVlSec').value, phases: $('pPhases').value, pfac: $('pPfac').value,
    elSrc: $('pElSrc').value, elTopic: $('pElTopic').value.trim() });
  try {
    const r = await getJson('/pvcfg?' + p.toString());
    if (r.restart) { $('pvMsg').textContent = $('sysMsg').textContent = 'Gespeichert, Neustart...'; return; }
  } catch(e) { $('pvMsg').textContent = $('sysMsg').textContent = 'Keine Antwort'; return; }
  toast('Gespeichert'); loadPv(true);
}
async function pvCmd(a) { try { await fetch('/pvcmd?a=' + a); } catch(e) {} setTimeout(() => loadPv(false), 600); }

// ---------- NAS (FTP) ----------
async function loadFtp(full) {
  let c;
  try { c = await getJson('/ftpcfg'); } catch(e) { return; }
  const kb = v => v >= 1073741824 ? (v / 1073741824).toFixed(1) + ' GB' : v >= 1048576 ? (v / 1048576).toFixed(1) + ' MB' : Math.round(v / 1024) + ' KB';
  const t = s => s ? new Date(s * 1000).toLocaleString('de-DE') : '–';
  kv('ftpKv', [['Status', (c.busy ? '⏳ ' : '') + c.msg], ['Zuletzt erfolgreich', t(c.lastOk)],
    ['Seit Start übertragen', c.upFiles + ' Dateien · ' + kb(c.upBytes)],
    ['Datenspeicher', c.fsOk ? c.fsKind + ' · ' + kb(c.fsTotal - c.fsUsed) + ' frei von ' + kb(c.fsTotal) : '<b style="color:#c0392b">' + c.fsKind + '</b>']]);
  nasOn = !!(c.fEn && c.host);
  if (nasOn && !c.busy && (!c.cfgBak || Date.now() / 1000 - c.cfgBak > 86400) && !nasTimer) nasAuto();   // mind. 1x am Tag
  if (!full) return;
  $('nasEn').checked = c.fEn; $('nasHost').value = c.host; $('nasPort').value = c.port; $('nasUser').value = c.user;
  $('nasPass').value = c.pass; $('nasDir').value = c.dir; $('nasInt').value = c.int; $('nasKeep').value = c.keep;
  markDirty('ftp', false); $('ftpMsg').textContent = ''; applyDeps();
}
async function saveFtp() {
  const p = new URLSearchParams({ save: 1, fEn: $('nasEn').checked ? 1 : 0, host: $('nasHost').value.trim(), port: $('nasPort').value,
    user: $('nasUser').value.trim(), pass: $('nasPass').value, dir: $('nasDir').value.trim(), int: $('nasInt').value, keep: $('nasKeep').value });
  try { await getJson('/ftpcfg?' + p.toString()); } catch(e) { $('ftpMsg').textContent = 'Keine Antwort'; return; }
  toast('Gespeichert'); loadFtp(true);
}
async function ftpCmd(a) {
  try { await fetch('/ftpcmd?a=' + a); } catch(e) {}
  toast(a === 'restore' ? 'Hole Daten vom NAS …' : a === 'test' ? 'Teste Verbindung …' : 'Übertrage …');
  [1500, 5000, 12000].forEach(ms => setTimeout(() => loadFtp(false), ms));
}

// ---------- System ----------
let logPos = 0;
async function pollLog() {
  try {
    const res = await fetch('/log?since=' + logPos, {cache:'no-store'});
    const txt = await res.text();
    const p = Number(res.headers.get('X-Log-Pos'));
    if (!isNaN(p)) logPos = p;
    if (txt) {
      const atBottom = consoleEl.scrollTop + consoleEl.clientHeight >= consoleEl.scrollHeight - 20;
      consoleEl.textContent += txt;
      if (consoleEl.textContent.length > 60000) consoleEl.textContent = consoleEl.textContent.slice(-40000);
      if (atBottom) consoleEl.scrollTop = consoleEl.scrollHeight;
    }
  } catch(e) {}
}
async function sendCmd(c) {
  const el = $('cmdInput'), cmd = c || el.value.trim();
  if (!cmd) return;
  el.value = '';
  try { await fetch('/cmd?c=' + encodeURIComponent(cmd)); } catch(e) {}
  setTimeout(pollLog, 300);
}
async function loadMb() {
  try {
    const c = await getJson('/mbcfg');
    $('mbRx').value = c.rx; $('mbTx').value = c.tx; $('mbDe').value = c.de;
    $('mbMode').value = c.mode; $('mbBaud').value = c.baud; $('mbId').value = c.id; $('mbGap').value = c.gap;
  } catch(e) {}
}
async function saveMb() {
  const q = ['rx','tx','de','baud','id','gap'].map(k => k + '=' + encodeURIComponent($('mb' + k[0].toUpperCase() + k.slice(1)).value)).join('&')
          + '&mode=' + $('mbMode').value;
  try { await fetch('/mbcfg?save=1&' + q); } catch(e) {}
}
async function loadFiles() {
  const fd = $('files');
  try {
    const d = await getJson('/files');
    if (!d.sdOk) { fd.textContent = SDN + ' nicht gefunden'; return; }
    if (!d.files.length) { fd.textContent = 'Noch keine Dateien'; return; }
    d.files.sort((a, b) => b.name.localeCompare(a.name));
    fd.innerHTML = d.files.map(x => `<a href="/download?f=${encodeURIComponent(x.name)}">${x.name}</a> ${(x.size / 1024).toFixed(0)} kB`).join('<br>');
  } catch(e) { fd.textContent = 'Fehler beim Laden'; }
}
const RAWMAP = {4354:['Betriebs-Bits',0],4355:['Ausgangs-Bits',0],4356:['Eingangs-Bits',0],4358:['Coil',1],4359:['Heißgas',1],
  4360:['Außen',1],4361:['Sauggas',1],4362:['EVI ein',1],4363:['EVI aus',1],4364:['Flüssigleitung',1],4365:['Rücklauf',1],
  4366:['Vorlauf',1],4367:['Warmwasser',1],4368:['Durchfluss l/min',1],4369:['Delta T',1],4371:['Hochdruck MPa',2],
  4372:['Niederdruck MPa',2],4373:['Verdichter Hz',0],4374:['Lüfter 1 rpm',0],4375:['Lüfter 2 rpm',0],4378:['Soll-Frequenz Hz',0],
  4379:['Verdichterstrom A',1],4380:['IPM-Temperatur',0],4381:['Netzspannung V',0],4382:['Zwischenkreis V',0],4383:['T6',1],4384:['Raum T2',1],
  4385:['Verdampfer',1],4386:['Kondensator',1],4608:['Verdichterstarts',0],4610:['Laufzeit h',0],4611:['Pumpe %',0],4612:['Betriebsart',0]};
async function loadRaw() {
  let r;
  try { r = await getJson('/raw'); } catch(e) { return; }
  let h = '<tr><th>Register</th><th>roh</th><th>Bedeutung (vermutet)</th><th>skaliert</th></tr>';
  const add = (base, arr) => { if (!arr) return; arr.forEach((v, i) => {
    const a = base + i, m = RAWMAP[a], sv = v > 32767 ? v - 65536 : v;
    h += '<tr><td>' + a + '</td><td>' + v + '</td><td>' + (m ? m[0] : '') + '</td><td>' + (m ? (sv / Math.pow(10, m[1])).toFixed(m[1]) : '') + '</td></tr>';
  }); };
  add(4354, r.block); add(4608, r.cnt);
  $('rawTbl').innerHTML = h;
}

async function init() {
  loadFeat(); buildSched(); buildCurveTbl(); buildNCurveTbl(); buildLkTbl(); buildFcTbl(); buildSerBar(); markRange(); showTab(); applyDeps();
  $('stPrice').value = lsGet('avPrice', 0.30);
  canvas.addEventListener('mousemove', chartTip); canvas.addEventListener('mouseleave', () => $('tip').style.display = 'none');
  $('stChart').addEventListener('mousemove', stTip); $('stChart').addEventListener('mouseleave', () => $('stTip').style.display = 'none');
  loadHc(true); setInterval(() => loadHc(false), 5000);
  loadFc(true); setInterval(() => loadFc(false), 5000); loadDfl(); setInterval(loadDfl, 300000);
  loadPar(false); setInterval(() => { if (location.hash === '#verdichter') loadPar(false); }, 30000);
  loadPv(true); loadCtrl(true); loadMb(); loadFtp(true); pollLog();
  await loadCurrent();
  loadHistory();
  setInterval(loadCurrent, 2000);
  setInterval(() => loadCtrl(false), 2000);
  setInterval(() => loadPv(false), 5000);
  setInterval(() => loadFtp(false), 5000);
  setInterval(pollLog, 2000);
  setInterval(() => { if (view.type === 'h' && view.h <= 336 && location.hash === '#verlauf') loadHistory(); }, 30000);
  setInterval(() => { if (location.hash === '#statistik') loadStats(); }, 120000);
}
init();
</script>
</body>
</html>
)rawliteral";

// =====================================================
// HTTP-Handler (laufen in loop())
// =====================================================
void sendJson(const char* body) {
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json; charset=utf-8", body);
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleCurrent() {
  LiveData d = getLive();
  char msg[64];
  uint32_t age = getCmdMsg(msg, sizeof(msg));
  if (age > 60000) msg[0] = 0;

  static char b[3072];
  int n = 0;
  #define J(...) n += snprintf(b + n, sizeof(b) - n, __VA_ARGS__)
  J("{\"timeValid\":%s,\"nowUtc\":%lu", timeValid() ? "true" : "false", (unsigned long)nowUtc());
  J(",\"fw\":\"%s\",\"fwBuild\":\"%s\",\"plat\":\"%s\"", FW_VERSION, FW_BUILD, AV_PLATFORM);
  J(",\"vl\":%.1f,\"rl\":%.1f,\"ww\":%.1f", d.vorlauf, d.ruecklauf, d.warmwasser);
  if (d.flowValid)    J(",\"flow\":%.1f", d.flowLmin);           else J(",\"flow\":null");
  J(",\"hz\":%u", d.verdichterHz);
  if (d.fanRpmValid)  J(",\"fan1\":%u,\"fan2\":%u", d.fan1Rpm, d.fan2Rpm); else J(",\"fan1\":null,\"fan2\":null");
  if (d.maxHzValid)   J(",\"maxHz\":%u", d.maxVerdichterHz);     else J(",\"maxHz\":null");
  if (d.maxFanValid)  J(",\"maxFan\":%u", d.maxFanRpm);          else J(",\"maxFan\":null");
  if (d.sollValid)    J(",\"sv\":%.1f", d.sollwert);             else J(",\"sv\":null");
  if (d.wpStateValid) J(",\"wpOn\":%s", d.wpOnOffRaw ? "true" : "false"); else J(",\"wpOn\":null");
  if (d.wpModeValid)  J(",\"mode\":%u", d.wpModeRaw);            else J(",\"mode\":null");
  J(",\"ok\":%lu,\"err\":%lu", (unsigned long)d.okCount, (unsigned long)d.errCount);
  J(",\"sdOk\":%s,\"sdFreeMB\":%llu", sdOk ? "true" : "false", (sdTotalBytes - sdUsedBytes) >> 20);
  J(",\"logPts\":%lu,\"logCapH\":%.0f", (unsigned long)logCount, logCapacityHours());
  J(",\"rssi\":%d,\"uptime\":%lu", WiFi.RSSI(), (unsigned long)(millis() / 1000));
  J(",\"mbErr\":%u,\"mbErrTxt\":\"%s\",\"mbErrAge\":%lu", d.lastErr, mbErrText(d.lastErr),
    (unsigned long)(d.lastErr ? (millis() - d.lastErrMs) / 1000 : 0));
  J(",\"cmdMsg\":\"%s\"", msg);
  J(",\"mqtt\":%s,\"sg\":%d,\"heater\":\"%s\"", mqttConnected ? "true" : (mqttCfg.enabled ? "false" : "null"),
    (int)sgState, heaterInfo);
  // Detailwerte (null = nicht verfuegbar)
  auto JF = [&](const char* k, float v, int dec, bool ok) {
    if (ok) J(",\"%s\":%.*f", k, dec, v); else J(",\"%s\":null", k);
  };
  J(",\"x\":{\"ok\":%s", d.extValid ? "true" : "false");
  bool e = d.extValid, c = d.cntValid;
  JF("ta", d.ta, 1, e); JF("evap", d.evap, 1, e); JF("cond", d.cond, 1, e); JF("suction", d.suction, 1, e);
  JF("discharge", d.discharge, 1, e); JF("coil", d.coil, 1, e); JF("liquid", d.liquid, 1, e); JF("ipm", d.ipm, 1, e);
  JF("eviIn", d.eviIn, 1, e); JF("eviOut", d.eviOut, 1, e);
  JF("hp", d.hp, 2, e); JF("lp", d.lp, 2, e); JF("hzT", d.targetHz, 0, e); JF("amp", d.currentA, 1, e);
  JF("acV", d.acV, 0, e); JF("dcV", d.dcV, 0, e); JF("pEl", d.pEl, 0, e); JF("pTh", d.pTh, 0, e);
  JF("cop", d.pEl > 100 ? d.pTh / d.pEl : 0, 2, e && d.pEl > 100);
  JF("pElWp", d.pElWp, 0, e); JF("pElSh", d.pElSh, 0, e && !isnan(d.pElSh)); J(",\"pElSrc\":%u", (unsigned)d.pElSrc);
  JF("pump", d.pumpPct, 0, c); JF("starts", d.starts, 0, c); JF("runH", d.runtimeH, 0, c);
  JF("eElDay", d.eElDay / 1000.0, 2, true); JF("eThDay", d.eThDay / 1000.0, 2, true);
  JF("eElY", d.eElYest / 1000.0, 2, true); JF("eThY", d.eThYest / 1000.0, 2, true);
  JF("eElTot", d.eElTot / 1000.0, 1, true); JF("eThTot", d.eThTot / 1000.0, 1, true);
  J(",\"defrost\":%s,\"phases\":%u}", e ? ((d.state & 16) ? "true" : "false") : "null", gridPhases);
  // PV-Daten (MQTT von Venus OS)
  {
    const uint32_t AGE = 300000UL;
    float pw = pvGet(pvPower, AGE), soc = pvGet(pvSoc, 900000UL), bat = pvGet(pvBat, AGE),
          grid = pvGet(pvGrid, AGE), house = pvGet(pvHouse, AGE), td = pvTodayKwh(), yd = pvYestKwh();
    J(",\"pv\":{\"any\":%s", pvAny() ? "true" : "false");
    JF("power", pw, 0, !isnan(pw)); JF("soc", soc, 0, !isnan(soc)); JF("bat", bat, 0, !isnan(bat));
    JF("grid", grid, 0, !isnan(grid)); JF("house", house, 0, !isnan(house));
    JF("today", td, 2, !isnan(td)); JF("yest", yd, 2, !isnan(yd));
    J(",\"age\":%lu}", (unsigned long)(pvPower.ms ? (millis() - pvPower.ms) / 1000 : 0));
  }
  J(",\"aggBacklog\":%u", (unsigned)aggBacklog);
  J("}");
  #undef J
  sendJson(b);
}

void handleHistory() {
  uint32_t to = nowUtc(), from = 0;
  if (server.hasArg("from") && server.hasArg("to")) {          // fester Zeitraum (z. B. Monat)
    from = strtoul(server.arg("from").c_str(), nullptr, 10);
    to = min(to, (uint32_t)strtoul(server.arg("to").c_str(), nullptr, 10));
  } else if (server.hasArg("all")) {
    from = aggFirst ? dayKeyStart(aggFirst) : firstLoggedT();
    if (logCount && (!from || firstLoggedT() < from)) from = firstLoggedT();
  } else {
    float hours = server.hasArg("hours") ? server.arg("hours").toFloat() : 1;
    hours = constrain(hours, 0.1f, 24.0f * 3660);
    from = to ? to - (uint32_t)(hours * 3600.0f) : 0;
  }
  uint32_t bucket = LOG_INTERVAL_SEC;
  int n = (to && from < to) ? buildHistory(from, to, HIST_WEB_MAX, histWeb, &bucket, true) : 0;
  uint32_t first = aggFirst ? dayKeyStart(aggFirst) : firstLoggedT();
  if (logCount && firstLoggedT() < first) first = firstLoggedT();

  server.sendHeader("Cache-Control", "no-cache");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json; charset=utf-8", "");

  char buf[2048];
  int len = snprintf(buf, sizeof(buf), "{\"from\":%lu,\"to\":%lu,\"step\":%lu,\"count\":%d,\"first\":%lu,\"p\":[",
                     (unsigned long)from, (unsigned long)to, (unsigned long)bucket, n, (unsigned long)first);
  auto FV = [](char* o, size_t sz, float v, int dec) { if (isnan(v)) strcpy(o, "null"); else snprintf(o, sz, "%.*f", dec, v); };
  for (int i = 0; i < n; i++) {
    if (len > (int)sizeof(buf) - 280) { server.sendContent(buf, len); len = 0; }
    const HistOut& p = histWeb[i];
    if (i) buf[len++] = ',';
    if (p.gap) len += snprintf(buf + len, sizeof(buf) - len, "null,");
    char fl[12], rt[12], ta[12], ev[12], sv[12], cp[12], df[12], pe[12], pt[12], wf[12], fn[12];
    FV(fl, sizeof(fl), p.flow, 1); FV(rt, sizeof(rt), p.rt, 1); FV(ta, sizeof(ta), p.ta, 1); FV(ev, sizeof(ev), p.evap, 1);
    FV(sv, sizeof(sv), p.sv, 0); FV(cp, sizeof(cp), p.cop, 2); FV(df, sizeof(df), p.dfr, 2);
    FV(pe, sizeof(pe), p.pel, 0); FV(pt, sizeof(pt), p.pth, 0); FV(wf, sizeof(wf), p.wwf, 2); FV(fn, sizeof(fn), p.fan, 0);
    // [t, VL, RL, WW, Hz, Durchfluss, Raum, Aussen, Verdampfer, Soll, COP, Abtauanteil, P el W, P th W, WW-Anteil, Luefter rpm]
    len += snprintf(buf + len, sizeof(buf) - len, "[%lu,%.1f,%.1f,%.1f,%.0f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s]",
                    (unsigned long)p.t, p.vl, p.rl, p.ww, p.hz, fl, rt, ta, ev, sv, cp, df, pe, pt, wf, fn);
  }
  len += snprintf(buf + len, sizeof(buf) - len, "]}");
  server.sendContent(buf, len);
  server.sendContent("");
}

void handleSet() {
  float requested = server.arg("soll").toFloat();
  bool ok = false;
  if (requested >= SOLL_MIN && requested <= SOLL_MAX) ok = runCmdBlocking(CMD_SET_SOLL, requested, 60000);
  LiveData d = getLive();
  char b[128];
  char rb[12];
  if (d.sollValid) snprintf(rb, sizeof(rb), "%.1f", d.sollwert); else strcpy(rb, "null");
  snprintf(b, sizeof(b), "{\"ok\":%s,\"requested\":%.1f,\"readback\":%s}", ok ? "true" : "false", requested, rb);
  sendJson(b);
}

void sendBoolResult(bool ok) {
  LiveData d = getLive();
  char b[128];
  char wp[8], md[8];
  if (d.wpStateValid) strcpy(wp, d.wpOnOffRaw ? "true" : "false"); else strcpy(wp, "null");
  if (d.wpModeValid) snprintf(md, sizeof(md), "%u", d.wpModeRaw); else strcpy(md, "null");
  snprintf(b, sizeof(b), "{\"ok\":%s,\"wpOn\":%s,\"mode\":%s}", ok ? "true" : "false", wp, md);
  sendJson(b);
}

void handlePower() {
  String state = server.arg("state");
  bool ok = false;
  if (state == "on" || state == "1")       { powerCmd(true, "Web"); ok = true; }
  else if (state == "off" || state == "0") { powerCmd(false, "Web"); ok = true; }
  sendBoolResult(ok);
}

void handleMode() {
  int mode = server.arg("value").toInt();
  bool ok = false;
  if (mode == 1 || mode == 2 || mode == 4) ok = runCmdBlocking(CMD_MODE, mode, 60000);
  sendBoolResult(ok);
}

void handleSetMaxHz() {
  uint16_t hz = server.arg("hz").toInt();
  bool ok = false;
  if (hz >= MAX_HZ_MIN && hz <= MAX_HZ_MAX) ok = runCmdBlocking(CMD_MAXHZ, hz, 60000);
  LiveData d = getLive();
  char b[128], rb[8];
  if (d.maxHzValid) snprintf(rb, sizeof(rb), "%u", d.maxVerdichterHz); else strcpy(rb, "null");
  snprintf(b, sizeof(b), "{\"ok\":%s,\"requested\":%u,\"readback\":%s}", ok ? "true" : "false", hz, rb);
  sendJson(b);
}

void handleWpPar() {
  String set = server.arg("set");
  const char* err = nullptr;
  bool ok = true;
  if (server.hasArg("read")) ok = runCmdBlocking(CMD_READ_PARAMS, 0, 30000);
  if (set == "p06" || set == "p07") {
    float k = server.arg("v").toFloat();
    int idx = set == "p06" ? 2 : 3;
    if (!wpParValid) err = "Parameter noch nicht gelesen";
    else if (wpPar[idx] < 10 || wpPar[idx] > 150) err = "Skalierung unklar (Rohwert nicht 10..150) - bitte nicht schreiben";
    else if (k < 1.0f || k > 15.0f) err = "erlaubt 1..15 K";
    else ok = runCmdBlocking(set == "p06" ? CMD_SET_P06 : CMD_SET_P07, (float)lroundf(k * 10), 30000);
  }
  char b[320];
  auto V = [](int i, char* o, size_t n) { if (wpParValid) snprintf(o, n, "%u", wpPar[i]); else strcpy(o, "null"); };
  char a[8], c1[8], d[8], e[8];
  V(0, a, sizeof(a)); V(1, c1, sizeof(c1)); V(2, d, sizeof(d)); V(3, e, sizeof(e));
  snprintf(b, sizeof(b), "{\"ok\":%s,\"err\":%s%s%s,\"valid\":%s,\"age\":%lu,\"p105\":%s,\"p106\":%s,\"p06\":%s,\"p07\":%s}",
           (ok && !err) ? "true" : "false", err ? "\"" : "", err ? err : "null", err ? "\"" : "",
           wpParValid ? "true" : "false", (unsigned long)(wpParValid ? (millis() - wpParMs) / 1000 : 0), a, c1, d, e);
  sendJson(b);
}

void handleSetMaxFan() {
  uint16_t rpm = server.arg("rpm").toInt();
  bool ok = false;
  if (rpm >= FAN_RPM_MIN && rpm <= FAN_RPM_MAX) ok = runCmdBlocking(CMD_MAXFAN, rpm, 60000);
  LiveData d = getLive();
  char b[128], rb[8];
  if (d.maxFanValid) snprintf(rb, sizeof(rb), "%u", d.maxFanRpm); else strcpy(rb, "null");
  snprintf(b, sizeof(b), "{\"ok\":%s,\"requested\":%u,\"readback\":%s}", ok ? "true" : "false", rpm, rb);
  sendJson(b);
}

// =====================================================
// NAS-Archiv per FTP (z. B. Synology)
// =====================================================
// Der interne Speicher haelt die letzten Tage fein aufgeloest, die Stundenwerte und die Statistik.
// Das NAS bekommt alles als Archiv:
//   <dir>/JJJJ-MM/JJJJ-MM-TT.csv   Tagesdateien (alle 30 s ein Wert)
//   <dir>/JJJJ-MM/h_JJJJ-MM.csv    Stundenmittel des Monats
//   <dir>/stats.csv                Tageswerte
// Hochgeladen wird nur, was neu ist (SIZE + APPE). Fehlen intern Dateien (neu aufgesetzt),
// holt die Steuerung sie vom NAS zurueck.
// FTP-BEGIN
FtpCfg ftpCfg;

volatile bool ftpReqSync = false, ftpReqTest = false, ftpReqRestore = false, ftpBusy = false;
volatile bool storageReady = false;            // Verlauf erst laden, wenn evtl. Daten vom NAS geholt sind
char ftpMsg[96] = "aus";
volatile uint32_t ftpLastOk = 0, ftpLastTry = 0, ftpUpFiles = 0, ftpUpBytes = 0;
volatile uint32_t cfgNasLast = 0;             // letzte Einstellungs-Sicherung aufs NAS

static int32_t ftpDone[96];                    // Tage (JJJJMMTT) / Monate (JJJJMM), die auf dem NAS komplett sind
static int ftpDoneN = 0;
bool ftpIsDone(int32_t k) { for (int i = 0; i < ftpDoneN; i++) if (ftpDone[i] == k) return true; return false; }
void ftpMarkDone(int32_t k) {
  if (ftpIsDone(k)) return;
  if (ftpDoneN >= (int)(sizeof(ftpDone) / sizeof(ftpDone[0]))) { memmove(ftpDone, ftpDone + 1, sizeof(ftpDone) - sizeof(ftpDone[0])); ftpDoneN--; }
  ftpDone[ftpDoneN++] = k;
}

void ftpSetMsg(const char* fmt, ...) {
  char b[96];
  va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
  strlcpy(ftpMsg, b, sizeof(ftpMsg));
}

// <dir>/<sub>
void ftpPath(char* out, size_t n, const FtpCfg& f, const char* sub) {
  int bl = strlen(f.dir);
  while (bl > 0 && f.dir[bl - 1] == '/') bl--;
  snprintf(out, n, "%.*s/%s", bl, f.dir, sub);
}

class FtpClient {
  WiFiClient c;
  char host[40] = "";
  bool readLine(String& l, uint32_t dl) {
    l = "";
    while ((int32_t)(millis() - dl) < 0) {
      while (c.available()) {
        int ch = c.read();
        if (ch < 0) break;
        if (ch == '\n') { l.trim(); return true; }
        if (l.length() < 200) l += (char)ch;
      }
      if (!c.connected() && !c.available()) return false;
      delay(2);
    }
    return false;
  }
public:
  char last[128] = "";
  int code = 0;
  bool readReply(uint32_t toMs = 10000) {
    uint32_t dl = millis() + toMs;
    int first = 0;
    String line;
    for (;;) {
      if (!readLine(line, dl)) { code = 0; strlcpy(last, "keine Antwort vom Server", sizeof(last)); return false; }
      if (line.length() >= 3 && isdigit((unsigned char)line[0]) && isdigit((unsigned char)line[1]) && isdigit((unsigned char)line[2])) {
        int cd = line.substring(0, 3).toInt();
        if (!first) first = cd;
        if (cd == first && (line.length() == 3 || line[3] == ' ')) {   // letzte Zeile einer (mehrzeiligen) Antwort
          code = cd; strlcpy(last, line.c_str(), sizeof(last)); return true;
        }
      }
    }
  }
  int cmd(const char* fmt, ...) {
    char b[200];
    va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
    c.print(b); c.print("\r\n");
    return readReply() ? code : 0;
  }
  bool open(const FtpCfg& f) {
    strlcpy(host, f.host, sizeof(host));
    c.stop();
    if (!c.connect(f.host, f.port)) { snprintf(last, sizeof(last), "keine Verbindung zu %s:%u", f.host, f.port); return false; }
    if (!readReply() || code != 220) return false;
    int r = cmd("USER %s", f.user);
    if (r == 331) r = cmd("PASS %s", f.pass);
    if (r != 230) { snprintf(last, sizeof(last), "Anmeldung abgelehnt (%d) - Benutzer/Passwort/FTP-Recht pruefen", r); return false; }
    if (cmd("TYPE I") != 200) return false;
    return true;
  }
  bool alive() { return c.connected(); }
  void close() { if (c.connected()) cmd("QUIT"); c.stop(); }
  // passive Datenverbindung (IP aus der Antwort wird ignoriert, es zaehlt die Server-Adresse)
  bool pasv(WiFiClient& d) {
    if (cmd("PASV") != 227) return false;
    const char* p = strchr(last, '(');
    int h[6];
    if (!p || sscanf(p + 1, "%d,%d,%d,%d,%d,%d", &h[0], &h[1], &h[2], &h[3], &h[4], &h[5]) != 6) { strlcpy(last, "PASV-Antwort unlesbar", sizeof(last)); return false; }
    uint16_t port = (uint16_t)(h[4] * 256 + h[5]);
    if (!d.connect(host, port)) { snprintf(last, sizeof(last), "Datenverbindung Port %u geht nicht", port); return false; }
    return true;
  }
  // Groesse auf dem Server: >= 0, -1 = gibt es nicht, -2 = Fehler, -3 = SIZE wird nicht unterstuetzt
  long size(const char* path) {
    int r = cmd("SIZE %s", path);
    if (r == 213) return atol(last + 4);
    if (r == 550) return -1;
    if (r == 500 || r == 502 || r == 504) return -3;
    return -2;
  }
  void mkd(const char* path) { cmd("MKD %s", path); }      // gibt es schon -> Fehlermeldung, egal
  // lokale Datei ab "offset" hochladen (offset > 0: anhaengen)
  bool upload(const char* local, const char* remote, long offset, uint32_t& bytes) {
    File f = SD_MMC.open(local, FILE_READ);
    if (!f) { snprintf(last, sizeof(last), "%s nicht lesbar", local); return false; }
    if (offset > 0 && !f.seek(offset)) { f.close(); return false; }
    WiFiClient d;
    if (!pasv(d)) { f.close(); return false; }
    int r = cmd(offset > 0 ? "APPE %s" : "STOR %s", remote);
    if (r != 150 && r != 125) { d.stop(); f.close(); return false; }
    uint8_t buf[1024];
    int n;
    uint32_t sent = 0;
    bool ok = true;
    while (ok && (n = f.read(buf, sizeof(buf))) > 0) {
      int w = 0;
      uint32_t dl = millis() + 15000;
      while (w < n) {
        int k = d.write(buf + w, n - w);
        if (k > 0) { w += k; continue; }
        if (!d.connected() || (int32_t)(millis() - dl) >= 0) { ok = false; break; }
        delay(5);
      }
      sent += w;
    }
    f.close();
    d.stop();
    if (!readReply(30000) || (code != 226 && code != 250)) return false;
    if (!ok) { strlcpy(last, "Upload abgebrochen", sizeof(last)); return false; }
    bytes += sent;
    return true;
  }
  // Text aus dem Speicher hochladen (ueberschreibt)
  bool uploadBuf(const char* remote, const char* data, size_t len) {
    WiFiClient d;
    if (!pasv(d)) return false;
    int r = cmd("STOR %s", remote);
    if (r != 150 && r != 125) { d.stop(); return false; }
    size_t w = 0;
    uint32_t dl = millis() + 15000;
    while (w < len) {
      int k = d.write((const uint8_t*)data + w, len - w > 1024 ? 1024 : len - w);
      if (k > 0) { w += k; continue; }
      if (!d.connected() || (int32_t)(millis() - dl) >= 0) break;
      delay(5);
    }
    d.stop();
    return readReply(30000) && (code == 226 || code == 250) && w == len;
  }
  // Datei als Text holen: true = ok (out gefuellt), false = Fehler / gibt es nicht
  bool downloadStr(const char* remote, String& out, size_t maxLen) {
    WiFiClient d;
    out = "";
    if (!pasv(d)) return false;
    int r = cmd("RETR %s", remote);
    if (r != 150 && r != 125) { d.stop(); return false; }
    uint32_t idle = millis();
    bool ok = true;
    for (;;) {
      int a = d.available();
      if (a > 0) {
        int ch = d.read();
        if (ch >= 0) { if (out.length() < maxLen) out += (char)ch; else ok = false; idle = millis(); }
        continue;
      }
      if (!d.connected()) break;
      if (millis() - idle > 15000) { ok = false; break; }
      delay(2);
    }
    d.stop();
    if (!readReply(20000) || (code != 226 && code != 250)) ok = false;
    return ok;
  }
  // Dateinamen eines Ordners (eine Zeile je Datei); Ordner fehlt -> leer
  bool nlst(const char* path, String& out) {
    WiFiClient d;
    out = "";
    if (!pasv(d)) return false;
    int r = cmd("NLST %s", path);
    if (r == 550 || r == 450) { d.stop(); return true; }
    if (r != 150 && r != 125) { d.stop(); return false; }
    uint32_t idle = millis();
    for (;;) {
      int a = d.available();
      if (a > 0) { int ch = d.read(); if (ch >= 0 && out.length() < 16000) out += (char)ch; idle = millis(); continue; }
      if (!d.connected()) break;
      if (millis() - idle > 15000) break;
      delay(2);
    }
    d.stop();
    readReply(20000);
    return true;
  }
  // Datei holen: Bytes, -1 = gibt es nicht, -2 = Fehler
  long download(const char* remote, const char* local) {
    WiFiClient d;
    if (!pasv(d)) return -2;
    int r = cmd("RETR %s", remote);
    if (r == 550) { d.stop(); return -1; }
    if (r != 150 && r != 125) { d.stop(); return -2; }
    char tmp[40]; snprintf(tmp, sizeof(tmp), "%s.tmp", local);
    File f = SD_MMC.open(tmp, FILE_WRITE);
    if (!f) { d.stop(); return -2; }
    uint8_t buf[1024];
    long got = 0;
    bool ok = true;
    uint32_t idle = millis();
    for (;;) {
      int a = d.available();
      if (a > 0) {
        int n = d.read(buf, a > (int)sizeof(buf) ? (int)sizeof(buf) : a);
        if (n > 0) { if (f.write(buf, n) != (size_t)n) { ok = false; break; } got += n; idle = millis(); }
        continue;
      }
      if (!d.connected()) break;
      if (millis() - idle > 20000) { ok = false; break; }
      delay(2);
    }
    f.close();
    d.stop();
    if (!readReply(20000) || (code != 226 && code != 250)) ok = false;
    if (!ok) { SD_MMC.remove(tmp); return -2; }
    SD_MMC.remove(local);
    if (!SD_MMC.rename(tmp, local)) { SD_MMC.remove(tmp); return -2; }
    return got;
  }
};

// Name einer Log-Datei -> Schluessel und Monatsordner. 1 = Tag, 2 = Stunden, 3 = stats.csv, 0 = andere
int ftpFileKind(const char* nm, int32_t& key, char* month) {
  int y, m, d;
  if (strlen(nm) == 14 && sscanf(nm, "%4d-%2d-%2d.csv", &y, &m, &d) == 3) { key = y * 10000 + m * 100 + d; snprintf(month, 8, "%04d-%02d", y, m); return 1; }
  if (strlen(nm) == 13 && sscanf(nm, "h_%4d-%2d.csv", &y, &m) == 2) { key = y * 100 + m; snprintf(month, 8, "%04d-%02d", y, m); return 2; }
  if (!strcmp(nm, "stats.csv") || !strcmp(nm, "abtau.csv")) { key = 0; month[0] = 0; return 3; }
  return 0;
}

// eine Datei abgleichen; complete = NAS hat jetzt alles
bool ftpSyncOne(FtpClient& ftp, const FtpCfg& f, const char* nm, const char* month, bool& complete) {
  char local[40], sub[48], remote[112];
  snprintf(local, sizeof(local), "/log/%s", nm);
  if (month[0]) snprintf(sub, sizeof(sub), "%s/%s", month, nm); else strlcpy(sub, nm, sizeof(sub));
  ftpPath(remote, sizeof(remote), f, sub);
  File lf = SD_MMC.open(local, FILE_READ);
  if (!lf) return false;
  long ls = (long)lf.size();
  lf.close();
  complete = false;
  long rs = ftp.size(remote);
  if (rs == -2) return false;
  if (rs >= 0 && rs >= ls) {
    if (rs > ls) Log.printf("NAS: %s ist dort groesser als intern - nicht ueberschrieben\n", sub);
    complete = true;
    return true;
  }
  if (rs < 0 && month[0]) { char md[80]; ftpPath(md, sizeof(md), f, month); ftp.mkd(md); }   // Monatsordner anlegen
  uint32_t b = 0;
  if (!ftp.upload(local, remote, rs > 0 ? rs : 0, b)) {
    Log.printf("NAS: %s nicht uebertragen: %s\n", sub, ftp.last);
    return false;
  }
  ftpUpFiles++; ftpUpBytes += b;
  complete = true;
  return true;
}

// alles Neue aufs NAS
bool ftpSync(const FtpCfg& f) {
  FtpClient ftp;
  if (!ftp.open(f)) { ftpSetMsg("Fehler: %s", ftp.last); ftp.close(); return false; }
  if (strlen(f.dir) > 1) ftp.mkd(f.dir);
  int32_t today = dayKeyOf(nowUtc());
  int32_t thisMonth = today / 100;
  static char names[100][16];
  int nn = 0;
  File dir = SD_MMC.open("/log");
  if (dir) {
    File e;
    while ((e = dir.openNextFile()) && nn < 100) {
      const char* nm = e.name();
      int32_t k; char mo[8];
      if (!e.isDirectory() && strlen(nm) < sizeof(names[0]) && ftpFileKind(nm, k, mo)) strlcpy(names[nn++], nm, sizeof(names[0]));
      e.close();
    }
    dir.close();
  }
  uint32_t f0 = ftpUpFiles, b0 = ftpUpBytes;
  int err = 0, checked = 0;
  for (int i = 0; i < nn; i++) {
    int32_t k; char mo[8];
    int kind = ftpFileKind(names[i], k, mo);
    bool old = (kind == 1 && k < today) || (kind == 2 && k < thisMonth);
    if (old && ftpIsDone(k)) continue;              // schon komplett auf dem NAS
    bool complete = false;
    checked++;
    if (!ftpSyncOne(ftp, f, names[i], mo, complete)) { err++; if (!ftp.alive()) break; continue; }
    if (complete && old) ftpMarkDone(k);
  }
  ftp.close();
  uint32_t nf = ftpUpFiles - f0, nb = ftpUpBytes - b0;
  if (err) ftpSetMsg("%d Fehler, %lu Dateien uebertragen (%lu KB)", err, (unsigned long)nf, (unsigned long)(nb / 1024));
  else {
    ftpLastOk = nowUtc();
    ftpSetMsg("OK - %lu von %d Dateien aktualisiert (%lu KB)", (unsigned long)nf, checked, (unsigned long)(nb / 1024));
  }
  return err == 0;
}

// fehlende Dateien vom NAS holen (Neuaufsetzen): Statistik, Stundenwerte (36 Monate), letzte Tage
int ftpRestore(const FtpCfg& f) {
  FtpClient ftp;
  if (!ftp.open(f)) { ftpSetMsg("Fehler: %s", ftp.last); ftp.close(); return -1; }
  int n = 0;
  auto get = [&](const char* sub, const char* local) {
    if (SD_MMC.exists(local)) return;
    uint64_t tot = SD_MMC.totalBytes(), used = SD_MMC.usedBytes();
    if (tot && tot - used < 600UL * 1024) return;               // Platz lassen
    char remote[112]; ftpPath(remote, sizeof(remote), f, sub);
    long r = ftp.download(remote, local);
    if (r > 0) { n++; Log.printf("NAS: %s geholt (%ld KB)\n", sub, r / 1024); }
    else if (r == -2) Log.printf("NAS: %s nicht geholt: %s\n", sub, ftp.last);
  };
  get("stats.csv", "/log/stats.csv");
  get("abtau.csv", "/log/abtau.csv");
  time_t now = nowUtc();
  struct tm tmv; localtime_r(&now, &tmv);
  int y = tmv.tm_year + 1900, m = tmv.tm_mon + 1;
  for (int i = 0; i < 36 && ftp.alive(); i++) {
    char sub[32], local[32];
    snprintf(sub, sizeof(sub), "%04d-%02d/h_%04d-%02d.csv", y, m, y, m);
    snprintf(local, sizeof(local), "/log/h_%04d-%02d.csv", y, m);
    get(sub, local);
    if (--m == 0) { m = 12; y--; }
  }
  for (int d = 1; d < f.keepDays && d <= 30 && ftp.alive(); d++) {    // heute nicht (wird gerade geschrieben)
    time_t t = now - (time_t)d * 86400;
    struct tm dt; localtime_r(&t, &dt);
    char sub[32], local[32];
    snprintf(sub, sizeof(sub), "%04d-%02d/%04d-%02d-%02d.csv", dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_mday);
    snprintf(local, sizeof(local), "/log/%04d-%02d-%02d.csv", dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_mday);
    get(sub, local);
  }
  ftp.close();
  ftpSetMsg("vom NAS geholt: %d Dateien", n);
  Log.printf("NAS: %d Dateien in den internen Speicher geholt\n", n);
  return n;
}

// interner Speicher leer (neu aufgesetzt)? -> keine Statistik und keine alten Tagesdateien
bool ftpLocalEmpty() {
  if (SD_MMC.exists("/log/stats.csv")) return false;
  int32_t today = dayKeyOf(nowUtc());
  File dir = SD_MMC.open("/log");
  if (!dir) return true;
  bool empty = true;
  File e;
  while ((e = dir.openNextFile())) {
    int32_t k; char mo[8];
    int kind = ftpFileKind(e.name(), k, mo);
    e.close();
    if ((kind == 1 && k != today) || kind == 2) { empty = false; break; }
  }
  dir.close();
  return empty;
}

// alte Tagesdateien intern loeschen: nach keepDays, wenn sie auf dem NAS sind; bei knappem Platz die aeltesten
void storageCleanup(const FtpCfg& f) {
  if (!sdOk || !timeValid()) return;
  bool ftpOn = f.enabled && f.host[0];
  int32_t keepFrom = dayKeyOf(nowUtc() - (uint32_t)(max(2, (int)f.keepDays) - 1) * 86400UL);   // heute + (keepDays-1) Vortage bleiben
  int32_t days[100]; int nd = 0;
  File dir = SD_MMC.open("/log");
  if (!dir) return;
  File e;
  while ((e = dir.openNextFile()) && nd < 100) {
    int32_t k; char mo[8];
    if (ftpFileKind(e.name(), k, mo) == 1) days[nd++] = k;
    e.close();
  }
  dir.close();
  for (int i = 1; i < nd; i++) { int32_t v = days[i]; int j = i; while (j > 0 && days[j - 1] > v) { days[j] = days[j - 1]; j--; } days[j] = v; }
  auto del = [&](int32_t k, const char* why) {
    char p[32]; snprintf(p, sizeof(p), "/log/%04ld-%02ld-%02ld.csv", (long)(k / 10000), (long)(k / 100 % 100), (long)(k % 100));
    if (SD_MMC.remove(p)) Log.printf("Speicher: %s geloescht (%s)\n", p + 5, why);
  };
  int first = 0;
  if (ftpOn) {
    for (; first < nd && days[first] < keepFrom && ftpIsDone(days[first]); first++) del(days[first], "liegt auf dem NAS");
  }
  // weniger als 10 % frei: aelteste Tage loeschen (mindestens 2 Tage bleiben)
  for (int guard = 0; guard < 40 && nd - first > 2; guard++) {
    uint64_t tot = SD_MMC.totalBytes(), used = SD_MMC.usedBytes();
    if (!tot || tot - used >= tot / 10) break;
    del(days[first], ftpOn && ftpIsDone(days[first]) ? "Speicher fast voll" : "Speicher fast voll, NICHT auf dem NAS");
    first++;
  }
  updateSdSpace();
}

void ftpTask(void*) {
  bool initDone = false;
  uint32_t nextSync = millis() + 60000UL, nextClean = millis() + 300000UL;
  for (;;) {
    delay(500);
    FtpCfg f = ftpCfg;
    bool on = f.enabled && f.host[0];
    bool net = WiFi.status() == WL_CONNECTED && timeValid() && sdOk;
    if (!initDone) {
      if (!on) { storageReady = true; initDone = true; }
      else if (net) {
        if (ftpLocalEmpty()) {
          ftpBusy = true; ftpSetMsg("hole Daten vom NAS ...");
          Log.println("NAS: interner Speicher ist leer -> hole Daten vom NAS");
          ftpRestore(f);
          ftpBusy = false;
        }
        storageReady = true; initDone = true; nextSync = millis() + 30000UL;
      } else if (millis() > 120000UL) { storageReady = true; initDone = true; }   // nicht ewig warten
      continue;
    }
    if (!on && !ftpBusy) strlcpy(ftpMsg, f.enabled ? "kein Server eingetragen" : "aus", sizeof(ftpMsg));
    if (ftpReqTest) {
      ftpReqTest = false; ftpBusy = true; ftpSetMsg("teste ...");
      FtpClient ftp;
      if (ftp.open(f)) {
        int r = ftp.cmd("PWD");
        char md[80]; ftpPath(md, sizeof(md), f, "");
        ftpSetMsg("Verbindung OK (%s), Ordner %s", r == 257 ? "angemeldet" : "?", md);
      } else ftpSetMsg("Fehler: %s", ftp.last);
      ftp.close();
      ftpBusy = false;
    }
    if (ftpReqRestore) {
      ftpReqRestore = false;
      if (net) {
        ftpBusy = true; ftpSetMsg("hole Daten vom NAS ...");
        int n = ftpRestore(f);
        ftpBusy = false;
        if (n > 0) { Log.println("NAS: Daten geholt -> Neustart, damit Verlauf und Statistik sie laden"); delay(1500); restartRequested = true; }
      } else ftpSetMsg("kein WLAN / keine Uhrzeit");
    }
    if (on && net && (ftpReqSync || (int32_t)(millis() - nextSync) >= 0)) {
      ftpReqSync = false;
      nextSync = millis() + (uint32_t)max(1, (int)f.intervalMin) * 60000UL;
      ftpBusy = true; ftpSetMsg("uebertrage ..."); ftpLastTry = nowUtc();
      ftpSync(f);
      ftpBusy = false;
      storageCleanup(f);
      nextClean = millis() + 3600000UL;
    }
    if (sdOk && timeValid() && (int32_t)(millis() - nextClean) >= 0) { nextClean = millis() + 3600000UL; storageCleanup(f); }
  }
}
// FTP-END

void loadFtpCfg() { loadBlobRaw("ftp", &ftpCfg, sizeof(ftpCfg), FtpCfg().version); }
void saveFtpCfg() { saveBlobRaw("ftp", &ftpCfg, sizeof(ftpCfg)); }

// /ftpcfg: lesen (Status) bzw. speichern (save=1)
void handleFtpCfg() {
  if (server.hasArg("save")) {
    FtpCfg f = ftpCfg;
    auto I = [](const char* k, long def, long lo, long hi) -> long {
      return server.hasArg(k) ? constrain(server.arg(k).toInt(), lo, hi) : def;
    };
    auto T = [](const char* k, char* dst, size_t n) {
      if (!server.hasArg(k)) return;
      String v = server.arg(k); v.trim();
      strlcpy(dst, v.c_str(), n);
    };
    f.enabled = I("fEn", f.enabled, 0, 1);
    T("host", f.host, sizeof(f.host));
    f.port = I("port", f.port, 1, 65535);
    T("user", f.user, sizeof(f.user));
    if (server.hasArg("pass") && server.arg("pass") != "***") strlcpy(f.pass, server.arg("pass").c_str(), sizeof(f.pass));
    T("dir", f.dir, sizeof(f.dir));
    if (!f.dir[0]) strlcpy(f.dir, "/", sizeof(f.dir));
    f.intervalMin = I("int", f.intervalMin, 1, 1440);
    f.keepDays = I("keep", f.keepDays, 2, 60);
    ftpCfg = f;
    saveFtpCfg();
    Log.printf("NAS-Ablage %s (%s:%u, Ordner %s)\n", f.enabled ? "EIN" : "AUS", f.host, f.port, f.dir);
    if (f.enabled) ftpReqSync = true;
    sendJson("{\"ok\":true}");
    return;
  }
  FtpCfg f = ftpCfg;
  String o; o.reserve(700);
  auto B = [&](const char* k, bool v) { o += ",\""; o += k; o += "\":"; o += v ? "true" : "false"; };
  auto N = [&](const char* k, double v) { o += ",\""; o += k; o += "\":"; o += String(v, 0); };
  auto S = [&](const char* k, const char* v) {
    o += ",\""; o += k; o += "\":\"";
    for (const char* c = v; *c; c++) { if (*c == '"' || *c == '\\') o += '\\'; o += *c; }
    o += "\"";
  };
  o = "{\"ok\":true";
  B("fEn", f.enabled); S("host", f.host); N("port", f.port); S("user", f.user);
  S("pass", server.hasArg("export") ? f.pass : (f.pass[0] ? "***" : ""));
  S("dir", f.dir); N("int", f.intervalMin); N("keep", f.keepDays);
  B("busy", ftpBusy); S("msg", ftpMsg); N("lastOk", ftpLastOk); N("lastTry", ftpLastTry);
  N("upFiles", ftpUpFiles); N("upBytes", ftpUpBytes); N("cfgBak", cfgNasLast);
  B("fsOk", sdOk); S("fsKind", SD_MMC.kindName()); N("fsTotal", (double)sdTotalBytes); N("fsUsed", (double)sdUsedBytes);
  o += "}";
  sendJson(o.c_str());
}

// Einstellungen auf dem NAS: <dir>/einstellungen/einstellungen_JJJJ-MM-TT_<board>.json (eine je Tag) + aktuell.json
// Die Datei baut die Weboberflaeche (gleiches Format wie "Einstellungen herunterladen"), ESP und GIGA lesen beide.
void handleCfgNas() {
  String a = server.arg("a");
  FtpCfg f = ftpCfg;
  if (!f.enabled || !f.host[0]) { sendJson("{\"ok\":false,\"err\":\"NAS-Ablage ist ausgeschaltet\"}"); return; }
  FtpClient ftp;
  auto fail = [&](const char* what) {
    String o = "{\"ok\":false,\"err\":\"";
    o += what; o += ": ";
    for (const char* c = ftp.last; *c; c++) if (*c != '"' && *c != '\\') o += *c;
    o += "\"}";
    ftp.close();
    sendJson(o.c_str());
  };
  if (!ftp.open(f)) { fail("keine Verbindung"); return; }
  char dir[96]; ftpPath(dir, sizeof(dir), f, "einstellungen");
  if (a == "put") {
    String body = server.arg("plain");
    if (body.length() < 20 || body.indexOf("\"avarma\"") < 0) { ftp.close(); sendJson("{\"ok\":false,\"err\":\"keine Einstellungsdatei\"}"); return; }
    ftp.mkd(dir);
    time_t t = nowUtc(); struct tm tmv; localtime_r(&t, &tmv);
    char name[64], remote[160];
    snprintf(name, sizeof(name), "einstellungen_%04d-%02d-%02d_%s.json", tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
             strstr(AV_PLATFORM, "GIGA") ? "giga" : "esp32");
    snprintf(remote, sizeof(remote), "%s/%s", dir, name);
    bool ok = ftp.uploadBuf(remote, body.c_str(), body.length());
    snprintf(remote, sizeof(remote), "%s/aktuell.json", dir);
    ok = ok && ftp.uploadBuf(remote, body.c_str(), body.length());
    if (!ok) { fail("Hochladen"); return; }
    ftp.close();
    cfgNasLast = nowUtc();
    Log.printf("Einstellungen auf dem NAS gesichert (%s, %u Byte)\n", name, (unsigned)body.length());
    String o = "{\"ok\":true,\"name\":\""; o += name; o += "\"}";
    sendJson(o.c_str());
    return;
  }
  if (a == "list") {
    String txt;
    if (!ftp.nlst(dir, txt)) { fail("Liste"); return; }
    ftp.close();
    // Namen sammeln (Server liefern teils mit Pfad), neueste zuerst
    String names[60]; int n = 0;
    int pos = 0;
    while (pos < (int)txt.length() && n < 60) {
      int e = txt.indexOf('\n', pos); if (e < 0) e = txt.length();
      String l = txt.substring(pos, e); l.trim(); pos = e + 1;
      int sl = l.lastIndexOf('/'); if (sl >= 0) l = l.substring(sl + 1);
      if (l.endsWith(".json") && l.length() < 64 && l.indexOf('"') < 0) names[n++] = l;
    }
    for (int i = 1; i < n; i++) { String v = names[i]; int j = i; while (j > 0 && strcmp(names[j - 1].c_str(), v.c_str()) < 0) { names[j] = names[j - 1]; j--; } names[j] = v; }
    String o = "{\"ok\":true,\"files\":[";
    for (int i = 0; i < n; i++) { if (i) o += ','; o += '"'; o += names[i]; o += '"'; }
    o += "]}";
    sendJson(o.c_str());
    return;
  }
  if (a == "get") {
    String fn = server.arg("f");
    if (!fn.endsWith(".json") || fn.indexOf('/') >= 0 || fn.indexOf("..") >= 0 || fn.length() > 63) { ftp.close(); sendJson("{\"ok\":false,\"err\":\"Dateiname\"}"); return; }
    char remote[160]; snprintf(remote, sizeof(remote), "%s/%s", dir, fn.c_str());
    String out;
    if (!ftp.downloadStr(remote, out, 32768)) { fail("Laden"); return; }
    ftp.close();
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json; charset=utf-8", out);
    return;
  }
  ftp.close();
  sendJson("{\"ok\":false,\"err\":\"unbekannt\"}");
}

void handleFtpCmd() {
  String a = server.arg("a");
  if (a == "sync") ftpReqSync = true;
  else if (a == "test") ftpReqTest = true;
  else if (a == "restore") ftpReqRestore = true;
  sendJson("{\"ok\":true}");
}

void handleFiles() {
  sdFlush();
  consoleFlush();   // aktuelle Daten mit in die Datei
  String out = "{\"sdOk\":";
  out += sdOk ? "true" : "false";
  out += ",\"files\":[";
  if (sdOk) {
    File dir = SD_MMC.open("/log");
    if (dir && dir.isDirectory()) {
      bool first = true;
      File f = dir.openNextFile();
      while (f) {
        if (!f.isDirectory()) {
          String name = f.name();
          int slash = name.lastIndexOf('/');
          if (slash >= 0) name = name.substring(slash + 1);
          if (!first) out += ',';
          first = false;
          out += "{\"name\":\"" + name + "\",\"size\":" + String((unsigned long)f.size()) + "}";
        }
        f.close();
        f = dir.openNextFile();
      }
    }
  }
  out += "]}";
  sendJson(out.c_str());
}

void handleDownload() {
  String name = server.arg("f");
  bool okName = name.length() > 4 && name.length() < 24 && (name.endsWith(".csv") || name.endsWith(".txt"));
  for (size_t i = 0; okName && i < name.length(); i++) {
    char c = name[i];
    if (!(isdigit(c) || c == '-' || c == '.' || isalpha(c))) okName = false;
  }
  if (!sdOk || !okName) { server.send(404, "text/plain", "Nicht gefunden"); return; }
  sdFlush();
  consoleFlush();
  String path = "/log/" + name;
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) { server.send(404, "text/plain", "Nicht gefunden"); return; }
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + name + "\"");
  server.streamFile(f, name.endsWith(".txt") ? "text/plain; charset=utf-8" : "text/csv");
  f.close();
}

void handleLog() {
  uint32_t since = server.arg("since").toInt();
  static char out[sizeof(logRing) + 1];
  uint32_t n = 0, total;
  portENTER_CRITICAL(&logMux);
  total = logRingTotal;
  uint32_t start = since;
  if (start > total || total - start > sizeof(logRing))
    start = total > sizeof(logRing) ? total - sizeof(logRing) : 0;
  for (uint32_t i = start; i < total; i++) out[n++] = logRing[i % sizeof(logRing)];
  portEXIT_CRITICAL(&logMux);
  out[n] = 0;
  server.sendHeader("X-Log-Pos", String((unsigned long)total));
  server.sendHeader("Access-Control-Expose-Headers", "X-Log-Pos");
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/plain; charset=utf-8", out);
}

void handleCmd() {
  String c = server.arg("c");
  Log.printf("> %s\n", c.c_str());
  processSerialCmd(c);
  server.send(200, "text/plain", "ok");
}

void handleMbCfg() {
  if (server.hasArg("save")) {
    if (server.hasArg("rx"))   mbCfg.rx   = server.arg("rx").toInt();
    if (server.hasArg("tx"))   mbCfg.tx   = server.arg("tx").toInt();
    if (server.hasArg("de"))   mbCfg.de   = server.arg("de").toInt();
    if (server.hasArg("mode")) mbCfg.mode = server.arg("mode").toInt() ? 1 : 0;
    if (server.hasArg("baud")) mbCfg.baud = constrain(server.arg("baud").toInt(), 1200L, 115200L);
    if (server.hasArg("id"))   mbCfg.id   = constrain(server.arg("id").toInt(), 1L, 247L);
    if (server.hasArg("gap"))  mbCfg.gap  = constrain(server.arg("gap").toInt(), 0L, 20L);
    saveMbConfig();
    Log.println("Modbus-Einstellungen gespeichert -> Neustart");
    restartRequested = true;
  }
  char b[180];
  snprintf(b, sizeof(b), "{\"rx\":%d,\"tx\":%d,\"de\":%d,\"mode\":%u,\"baud\":%lu,\"id\":%u,\"gap\":%u}",
           mbCfg.rx, mbCfg.tx, mbCfg.de, mbCfg.mode, (unsigned long)mbCfg.baud, mbCfg.id, mbCfg.gap);
  sendJson(b);
}


void setupUpdateRoutes() {          // GIGA: Firmware nur ueber USB (Arduino-IDE)
  server.on("/update", []() {
    server.send(200, "text/html; charset=utf-8", "<!DOCTYPE html><meta charset='utf-8'><p style='font-family:Arial'>"
      "GIGA R1: Firmware bitte per USB-Kabel mit der Arduino-IDE hochladen. Die Einstellungen bleiben dabei erhalten.</p>"
      "<p><a href='/'>Zur&uuml;ck</a></p>");
  });
}

// Einstellungs-Import: der Browser schickt die Abschnitte nacheinander an die normalen
// Speicher-Adressen (gleiche Pruefungen wie beim Speichern im Web). Ein dabei
// angeforderter Neustart wird bis zum Ende zurueckgehalten.
void handleCfgImport() {
  if (server.hasArg("begin")) { restartHold = true; restartHoldMs = millis(); Log.println("Einstellungs-Import beginnt"); }
  if (server.hasArg("end")) {
    restartHold = false;
    Log.printf("Einstellungs-Import fertig (%s)\n", server.arg("from").c_str());
    restartRequested = true;                        // alle Einstellungen sauber neu laden
  }
  sendJson("{\"ok\":true}");
}

void handleCtrl() {
  CtrlCfg c = getCtrlCfg();
  char nx[40]; nextEventText(c, nx, sizeof(nx));
  String o; o.reserve(1200);
  o += "{\"enabled\":"; o += c.enabled ? "true" : "false";
  o += ",\"wpOn\":"; o += c.wpOnForWw ? "true" : "false";
  o += ",\"actLow\":"; o += c.actLow ? "true" : "false";
  o += ",\"dirInv\":"; o += c.dirInv ? "true" : "false";
  o += ",\"pinPwr\":" + String((int)c.pinPwr) + ",\"pinDir\":" + String((int)c.pinDir);
  o += ",\"travel\":" + String(c.travelSec) + ",\"buf\":" + String(c.bufPos);
  o += ",\"defAct\":" + String(c.defAct) + ",\"wwSoll\":" + String(c.wwSoll);
  o += ",\"boost\":" + String(c.boost) + ",\"maxMin\":" + String(c.maxMin) + ",\"cool\":" + String(c.coolSec);
  o += ",\"sched\":[";
  for (int d = 0; d < 7; d++) for (int s = 0; s < SCHED_SLOTS; s++) {
    if (d || s) o += ',';
    o += "[" + String((int)c.sched[d][s].min) + "," + String((int)c.sched[d][s].act) + "]";
  }
  o += "],\"vPos\":" + String(valvePosNow(), 1);
  o += ",\"vKnown\":"; o += valveKnown ? "true" : "false";
  o += ",\"vMoving\":"; o += valveMoving ? "true" : "false";
  o += ",\"vTarget\":" + String(valveTarget) + ",\"manual\":" + String(manualPos);
  o += ",\"ww\":" + String((int)wwState) + ",\"wwTxt\":\"" + String(wwInfo) + "\"";
  o += ",\"drive\":" + String((int)c.drive) + ",\"slaveId\":" + String((int)c.slaveId);
  o += ",\"fsMin\":" + String((int)c.fsMin);
  o += ",\"cont\":"; o += c.wwNoCont ? "false" : "true";
  o += ",\"pull\":" + String((int)c.wwPullMin) + ",\"vlMax\":" + String((int)c.wwVlMax);
  o += ",\"abOn\":"; o += c.abOn ? "true" : "false";
  o += ",\"abLow\":"; o += c.abLow ? "true" : "false";
  o += ",\"abFix\":"; o += c.abFixOn ? "true" : "false";
  o += ",\"abFixT\":" + String((int)c.abFixT);
  o += ",\"wwMode\":" + String((int)c.wwMode);
  o += ",\"abK\":" + String((int)c.abK) + ",\"abInfo\":\"" + String(abInfo) + "\"";
  o += ",\"slaveOnline\":"; o += slaveOnline ? "true" : "false";
  o += ",\"vFail\":"; o += valveFailsafe ? "true" : "false";
  o += ",\"slaveOk\":" + String((unsigned long)slaveOkCount) + ",\"slaveErr\":" + String((unsigned long)slaveErrCount) + ",\"slaveRetry\":" + String((unsigned long)slaveRetry);
  o += ",\"next\":\"" + String(nx) + "\"}";
  sendJson(o.c_str());
}

void handleCtrlSet() {
  CtrlCfg c = getCtrlCfg();
  auto argI = [](const char* k, long def, long lo, long hi) -> long {
    if (!server.hasArg(k)) return def;
    return constrain(server.arg(k).toInt(), lo, hi);
  };
  c.enabled   = argI("enabled", c.enabled, 0, 1);
  c.wpOnForWw = argI("wpOn", c.wpOnForWw, 0, 1);
  c.actLow    = argI("actLow", c.actLow, 0, 1);
  c.dirInv    = argI("dirInv", c.dirInv, 0, 1);
  c.wwSoll    = argI("wwSoll", c.wwSoll, 30, 65);
  c.boost     = argI("boost", c.boost, 0, 15);
  c.maxMin    = argI("maxMin", c.maxMin, 10, 480);
  c.coolSec   = argI("cool", c.coolSec, 0, 900);
  c.travelSec = argI("travel", c.travelSec, 10, 600);
  c.bufPos    = argI("buf", c.bufPos, 5, 95);
  c.defAct    = argI("defAct", c.defAct, ACT_PUFFER, ACT_HEIZ);
  int pp = argI("pinPwr", c.pinPwr, -1, 48), pd = argI("pinDir", c.pinDir, -1, 48);
  int drv = argI("drive", c.drive, 0, 1);
  int sid = argI("slaveId", c.slaveId, 2, 247);
  c.fsMin = argI("fsMin", c.fsMin, 0, 240);
  c.abOn  = argI("abOn", c.abOn, 0, 1);
  c.abLow = argI("abLow", c.abLow, 0, 1);
  c.abFixOn = argI("abFix", c.abFixOn, 0, 1);
  c.abFixT  = argI("abFixT", c.abFixT, 20, 60);
  c.wwMode  = argI("wwMode", c.wwMode, 0, 1);
  c.abK   = argI("abK", c.abK, -5, 15);
  if (server.hasArg("cont")) c.wwNoCont = argI("cont", 1, 0, 1) ? 0 : 1;
  c.wwPullMin = argI("pull", c.wwPullMin, 0, 120);
  c.wwVlMax   = argI("vlMax", c.wwVlMax, 40, 75);

  const char* err = nullptr;
  if (drv == 1 && sid == mbCfg.id) err = "Ventilmodul braucht eine andere ID als die Waermepumpe";
  for (int p : {pp, pd}) {
    if (drv == 1) break;                                  // Pins nur fuer Relais am ESP relevant
    if (p < 0) continue;
    if (!relayPinAllowed(p)) err = "Relais-Pin nicht erlaubt (" RELAY_PINS_TXT ")";
    else if (p == mbCfg.rx || p == mbCfg.tx || p == mbCfg.de) err = "Relais-Pin wird schon fuer Modbus benutzt";
  }
  if (drv == 0 && pp >= 0 && pp == pd) err = "Beide Relais auf demselben Pin";

  if (server.hasArg("sched")) {
    String sch = server.arg("sched");
    int idx = 0, pos = 0;
    while (idx < 7 * SCHED_SLOTS && pos <= (int)sch.length()) {
      int comma = sch.indexOf(',', pos); if (comma < 0) comma = sch.length();
      String item = sch.substring(pos, comma);
      int colon = item.indexOf(':');
      int m = colon > 0 ? item.substring(0, colon).toInt() : 0;
      int a = colon > 0 ? item.substring(colon + 1).toInt() : 0;
      SchedEv& e = c.sched[idx / SCHED_SLOTS][idx % SCHED_SLOTS];
      e.min = constrain(m, 0, 1439);
      e.act = (a >= ACT_WW && a <= ACT_HEIZ) ? a : ACT_NONE;
      idx++; pos = comma + 1;
    }
  }
  if (err) {
    char b[160]; snprintf(b, sizeof(b), "{\"ok\":false,\"err\":\"%s\"}", err);
    sendJson(b);
    return;
  }
  bool pinsChanged = (pp != c.pinPwr || pd != c.pinDir || c.actLow != ctrlCfg.actLow ||
                      drv != c.drive || sid != c.slaveId);
  c.pinPwr = pp; c.pinDir = pd; c.drive = drv; c.slaveId = sid;
  xSemaphoreTake(ctrlMutex, portMAX_DELAY); ctrlCfg = c; xSemaphoreGive(ctrlMutex);
  saveCtrlCfg();
  Log.printf("ESBE/WW-Einstellungen gespeichert (%s)\n", c.enabled ? "aktiv" : "inaktiv");
  sendJson(pinsChanged ? "{\"ok\":true,\"restart\":true}" : "{\"ok\":true}");
  if (pinsChanged) restartRequested = true;
}

void handleCtrlCmd() {
  String a = server.arg("a");
  if (!ctrlCfg.enabled)    { sendJson("{\"ok\":false}"); return; }
  if (a == "wwstart")      { reqWwManual = true; reqWwStart = true; }
  else if (a == "wwstop")  reqWwStop = true;
  else if (a == "vheiz")   reqValve = 0;
  else if (a == "vpuf")    reqValve = ctrlCfg.bufPos;
  else if (a == "vww")     reqValve = 100;
  else if (a == "vauto")   reqValve = -1;
  else if (a == "vref")    { if (ctrlCfg.drive == 1) slaveRefReq = true; else reqValveRef = true; }
  sendJson("{\"ok\":true}");
}

void handlePvCfg() {
  if (server.hasArg("save")) {
    MqttCfg m = mqttCfg;
    PvCfg p = pvCfg;
    auto I = [](const char* k, long def, long lo, long hi) -> long {
      return server.hasArg(k) ? constrain(server.arg(k).toInt(), lo, hi) : def;
    };
    m.enabled = I("mEnabled", m.enabled, 0, 1);
    if (server.hasArg("host")) strlcpy(m.host, server.arg("host").c_str(), sizeof(m.host));
    m.port = I("port", m.port, 1, 65535);
    if (server.hasArg("user")) strlcpy(m.user, server.arg("user").c_str(), sizeof(m.user));
    if (server.hasArg("pass") && server.arg("pass") != "***") strlcpy(m.pass, server.arg("pass").c_str(), sizeof(m.pass));
    if (server.hasArg("base") && server.arg("base").length()) strlcpy(m.base, server.arg("base").c_str(), sizeof(m.base));
    m.interval = I("interval", m.interval, 2, 300);

    p.pvWwEnable   = I("pvWw", p.pvWwEnable, 0, 1);
    p.pvWwSoll     = I("pvWwSoll", p.pvWwSoll, 40, 65);
    p.pvWwHyst     = I("pvWwHyst", p.pvWwHyst, 1, 20);
    p.heaterEnable = I("heat", p.heaterEnable, 0, 1);
    p.heaterSoll   = I("heatSoll", p.heaterSoll, 40, 85);
    if (server.hasArg("shelly")) strlcpy(p.shellyIp, server.arg("shelly").c_str(), sizeof(p.shellyIp));
    p.heaterGen    = I("gen", p.heaterGen, 1, 2);
    p.heaterMaxMin = I("heatMax", p.heaterMaxMin, 10, 720);
    p.sgTimeoutMin = I("sgTo", p.sgTimeoutMin, 0, 240);
    p.sg1Block     = I("sg1", p.sg1Block, 0, 1);
    p.vlMqtt       = I("vl", p.vlMqtt, 0, 1);
    p.vlMin        = I("vlMin", p.vlMin, 15, 60);
    p.vlMax        = I("vlMax", p.vlMax, p.vlMin, 65);
    p.vlMinSec     = I("vlSec", p.vlMinSec, 30, 3600);
    gridPhases     = I("phases", gridPhases, 1, 3) >= 2 ? 3 : 1;
    pElFactor      = I("pfac", pElFactor, 20, 300);
    uint8_t es     = I("elSrc", elSrc, 0, 1);
    char et[sizeof(elTopic)]; strlcpy(et, elTopic, sizeof(et));
    if (server.hasArg("elTopic")) {
      String t = server.arg("elTopic"); t.trim();
      while (t.endsWith("/")) t = t.substring(0, t.length() - 1);
      if (t.indexOf(' ') < 0 && t.indexOf('#') < 0 && t.indexOf('+') < 0) strlcpy(et, t.c_str(), sizeof(et));
    }
    if (es != elSrc) Log.printf("El. Leistung: Quelle %s\n", es ? "Shelly (MQTT)" : "Schaetzung aus der WP");
    elSrc = es;

    bool restart = memcmp(&m, &mqttCfg, sizeof(m)) != 0 || (strcmp(et, elTopic) != 0 && m.enabled);   // neues Abo
    strlcpy(elTopic, et, sizeof(elTopic));
    mqttCfg = m; pvCfg = p;
    saveMqttCfg(); savePvCfg();
    Log.printf("MQTT/PV-Einstellungen gespeichert%s\n", restart ? " -> Neustart" : "");
    sendJson(restart ? "{\"ok\":true,\"restart\":true}" : "{\"ok\":true}");
    if (restart) restartRequested = true;
    return;
  }
  MqttCfg m = mqttCfg; PvCfg p = pvCfg;
  String o; o.reserve(900);
  auto B = [&](const char* k, bool v) { o += ",\""; o += k; o += "\":"; o += v ? "true" : "false"; };
  auto N = [&](const char* k, long v) { o += ",\""; o += k; o += "\":"; o += String(v); };
  auto S = [&](const char* k, const char* v) {
    o += ",\""; o += k; o += "\":\"";
    for (const char* c = v; *c; c++) { if (*c == '"' || *c == '\\') o += '\\'; o += *c; }
    o += "\"";
  };
  o = "{\"ok\":true";
  B("mEnabled", m.enabled); S("host", m.host); N("port", m.port); S("user", m.user);
  S("pass", server.hasArg("export") ? m.pass : (m.pass[0] ? "***" : ""));   // export: Klartext fuer die Sicherungsdatei
  S("base", m.base); N("interval", m.interval);
  B("pvWw", p.pvWwEnable); N("pvWwSoll", p.pvWwSoll); N("pvWwHyst", p.pvWwHyst);
  B("heat", p.heaterEnable); N("heatSoll", p.heaterSoll); S("shelly", p.shellyIp); N("gen", p.heaterGen);
  N("heatMax", p.heaterMaxMin); N("sgTo", p.sgTimeoutMin); B("sg1", p.sg1Block); B("vl", p.vlMqtt);
  N("vlMin", p.vlMin); N("vlMax", p.vlMax); N("vlSec", p.vlMinSec);
  N("phases", gridPhases); N("pfac", pElFactor);
  N("elSrc", elSrc); S("elTopic", elTopic);
  {                                                    // Live-Zustand fuer die System-Karte
    LiveData d = getLive();
    auto FN = [&](const char* k, float v, int dec, bool ok) { o += ",\""; o += k; o += "\":"; o += (ok && !isnan(v)) ? String(v, dec) : String("null"); };
    bool sf = shFresh();
    B("shOk", sf); N("shAge", shMs ? (long)((millis() - shMs) / 1000) : -1); N("shRx", shRx);
    N("shAny", shAny); N("shBad", shBad); S("shLast", shLast); N("shLastLen", shLastLen);
    FN("shP", shP, 0, sf); FN("shETot", shETot, 2, shEMs != 0); FN("pElWp", d.pElWp, 0, d.extValid);
    N("pElSrc", d.pElSrc);
    FN("pfacSug", shCmpWp > 1000 ? (float)(shCmpSh / shCmpWp * 100.0) : NAN, 0, shCmpWp > 1000);
  }
  B("connected", mqttConnected); N("tx", mqttTxCount); N("rx", mqttRxCount); S("lastCmd", mqttLastCmd);
  N("sg", sgState); S("heater", heaterInfo); B("pvLoad", pvLoad);
  o += "}";
  sendJson(o.c_str());
}

void handleHc() {
  if (server.hasArg("save")) {
    HcCfg c = hcCfg;
    auto I = [](const char* k, long def, long lo, long hi) -> long {
      return server.hasArg(k) ? constrain(server.arg(k).toInt(), lo, hi) : def;
    };
    auto T = [](const char* k, long def, long lo, long hi) -> long {     // Zehntel aus Kommazahl
      return server.hasArg(k) ? constrain(lroundf(server.arg(k).toFloat() * 10), lo, hi) : def;
    };
    c.enabled = I("en", c.enabled, 0, 1);
    c.roomOn = I("roomOn", c.roomOn, 0, 1);
    c.rtDay10 = T("day", c.rtDay10, 100, 280);
    c.rtNight10 = T("nightSoll", c.rtNight10, 100, 280);
    c.nightOn = I("nightOn", c.nightOn, 0, 1);
    c.nightStart = I("ns", c.nightStart, 0, 1439);
    c.nightEnd = I("ne", c.nightEnd, 0, 1439);
    c.nightWeOn = I("nwOn", c.nightWeOn, 0, 1);
    c.nightWeStart = I("nws", c.nightWeStart, 0, 1439);
    c.nightWeEnd = I("nwe", c.nightWeEnd, 0, 1439);
    c.nightVlShift = I("nshift", c.nightVlShift, -15, 5);
    c.nCurveOn = I("ncOn", c.nCurveOn, 0, 1);
    c.nOffOn = I("noffOn", c.nOffOn, 0, 1);
    c.nOffTa = I("noffTa", c.nOffTa, -10, 25);
    c.roomK10 = T("k", c.roomK10, 0, 100);
    c.roomKi10 = T("ki", c.roomKi10, 0, 100);
    c.roomMax = I("max", c.roomMax, 0, 15);
    c.vlMin = I("vlMin", c.vlMin, 15, 60);
    c.vlMax = I("vlMax", c.vlMax, c.vlMin, 65);
    c.writeSec = I("ws", c.writeSec, 60, 3600);
    c.deadband10 = T("db", c.deadband10, 5, 50);
    c.taTau = I("tau", c.taTau, 0, 240);
    c.autoEnable = I("auto", c.autoEnable, 0, 1);
    c.autoMode = I("mode", c.autoMode, 1, 3);
    c.taOn = I("taOn", c.taOn, -20, 25);
    c.taOff = I("taOff", c.taOff, c.taOn, 30);
    c.rtOnBelow10 = T("rtOn", c.rtOnBelow10, 0, 50);
    c.rtOffAbove10 = T("rtOff", c.rtOffAbove10, 0, 50);
    c.minOnMin = I("minOn", c.minOnMin, 0, 240);
    c.minOffMin = I("minOff", c.minOffMin, 0, 240);
    c.kaminTa = I("kTa", c.kaminTa, 0, 15);
    c.kaminRt10 = T("kRt", c.kaminRt10, 0, 50);
    c.kaminMaxH = I("kMax", c.kaminMaxH, 0, 24);
    int pin = I("pin", c.sensPin, -1, 48);
    { int ty = I("type", c.sensType, 11, 40); c.sensType = ty == 11 ? 11 : ty == 40 ? 40 : 22; }
    c.sensOffset10 = T("off", c.sensOffset10, -50, 50);
    const char* err = nullptr;
    if (pin >= 0 && !relayPinAllowed(pin)) err = "Fuehler-Pin nicht erlaubt (" RELAY_PINS_TXT ")";
    else if (pin >= 0 && (pin == mbCfg.rx || pin == mbCfg.tx || pin == mbCfg.de)) err = "Pin wird fuer Modbus benutzt";
    else if (pin >= 0 && ctrlCfg.drive == 0 && (pin == ctrlCfg.pinPwr || pin == ctrlCfg.pinDir)) err = "Pin wird fuer ein Ventil-Relais benutzt";
    // Heizkurve: 7 Punkte "15,10,5,..." / "28,32,..."
    if (!err && server.hasArg("cTa") && server.hasArg("cVl")) {
      String a = server.arg("cTa"), v = server.arg("cVl");
      int ia = 0, iv = 0;
      for (int i = 0; i < HC_POINTS; i++) {
        int ca = a.indexOf(',', ia); if (ca < 0) ca = a.length();
        int cv = v.indexOf(',', iv); if (cv < 0) cv = v.length();
        c.curveTa[i] = constrain(a.substring(ia, ca).toInt(), -30L, 30L);
        c.curveVl[i] = constrain(v.substring(iv, cv).toInt(), 15L, 65L);
        ia = ca + 1; iv = cv + 1;
      }
      for (int i = 1; i < HC_POINTS; i++)
        if (c.curveTa[i] >= c.curveTa[i - 1]) { err = "Heizkurve: Aussentemperaturen muessen abnehmen"; break; }
    }
    // Nachtkurve: 7 Punkte Aussen / Verschiebung in K
    if (!err && server.hasArg("nTa") && server.hasArg("nDv")) {
      String a = server.arg("nTa"), v = server.arg("nDv");
      int ia = 0, iv = 0;
      for (int i = 0; i < HC_POINTS; i++) {
        int ca = a.indexOf(',', ia); if (ca < 0) ca = a.length();
        int cv = v.indexOf(',', iv); if (cv < 0) cv = v.length();
        c.nTa[i] = constrain(a.substring(ia, ca).toInt(), -30L, 30L);
        c.nDv[i] = constrain(v.substring(iv, cv).toInt(), -15L, 5L);
        ia = ca + 1; iv = cv + 1;
      }
      for (int i = 1; i < HC_POINTS; i++)
        if (c.nTa[i] >= c.nTa[i - 1]) { err = "Nachtkurve: Aussentemperaturen muessen abnehmen"; break; }
    }
    if (err) {
      char b[140]; snprintf(b, sizeof(b), "{\"ok\":false,\"err\":\"%s\"}", err);
      sendJson(b); return;
    }
    // Sperrzeiten
    LockCfg L = lockCfg;
    L.enabled = I("lkEn", L.enabled, 0, 1);
    L.coldOn = I("lkCold", L.coldOn, 0, 1);
    L.coldTa = I("lkTa", L.coldTa, -25, 20);
    L.wwAllow = I("lkWw", L.wwAllow, 0, 1);
    auto lkArr = [](const char* kf, const char* kt, uint16_t* fr, uint16_t* to) {
      if (!server.hasArg(kf) || !server.hasArg(kt)) return;
      String a = server.arg(kf), b = server.arg(kt);
      int ia = 0, ib = 0;
      for (int i = 0; i < 7; i++) {
        int ca = a.indexOf(',', ia); if (ca < 0) ca = a.length();
        int cb = b.indexOf(',', ib); if (cb < 0) cb = b.length();
        fr[i] = constrain(a.substring(ia, ca).toInt(), 0L, 1439L);
        to[i] = constrain(b.substring(ib, cb).toInt(), 0L, 1439L);
        ia = ca + 1; ib = cb + 1;
      }
    };
    lkArr("lkF", "lkT", L.from, L.to);
    lkArr("lkF2", "lkT2", L.from2, L.to2);
    bool restart = (pin != c.sensPin);
    c.sensPin = pin;
    hcCfg = c;
    saveHcCfg();
    lockCfg = L;
    saveLockCfg();
    Log.println("Heizregelung: Einstellungen gespeichert");
    sendJson(restart ? "{\"ok\":true,\"restart\":true}" : "{\"ok\":true}");
    if (restart) restartRequested = true;
    return;
  }
  HcCfg c = hcCfg;
  LiveData d = getLive();
  String o; o.reserve(1500);
  auto N = [&](const char* k, float v, int dec, bool ok) {
    o += ",\""; o += k; o += "\":"; if (ok && !isnan(v)) o += String(v, dec); else o += "null";
  };
  auto B = [&](const char* k, bool v) { o += ",\""; o += k; o += "\":"; o += v ? "true" : "false"; };
  o = "{\"ok\":true";
  bool rok = rtValid();
  N("rt", rtIst, 1, rok); N("rh", rhIst, 0, rok); N("rtSoll", hcRtSoll, 1, true);
  N("ta", d.ta, 1, d.extValid); N("taF", hcTaF, 1, true); N("curve", hcCurve, 1, true);
  N("corr", hcRoomCorr, 1, true); N("i", hcI, 1, true); N("target", hcTarget, 0, true);
  N("wpSoll", d.sollwert, 0, d.sollValid); B("night", hcNight);
  N("targetF", hcTargetF, 1, true); N("db", c.deadband10 / 10.0f, 1, true); N("vlW", vlWritesToday, 0, true);
  o += ",\"vlInfo\":\""; o += vlInfo; o += "\"";
  o += ",\"autoInfo\":\""; o += hcAutoInfo; o += "\"";
  N("dhtOk", dhtOk, 0, true); N("dhtErr", dhtErr, 0, true);
  B("en", c.enabled); N("day", c.rtDay10 / 10.0f, 1, true); N("nightSoll", c.rtNight10 / 10.0f, 1, true);
  B("nightOn", c.nightOn); N("ns", c.nightStart, 0, true); N("ne", c.nightEnd, 0, true); N("nshift", c.nightVlShift, 0, true);
  B("nwOn", c.nightWeOn); N("nws", c.nightWeStart, 0, true); N("nwe", c.nightWeEnd, 0, true);
  B("ncOn", c.nCurveOn); N("nNow", hcNShift, 1, true); B("noffOn", c.nOffOn); N("noffTa", c.nOffTa, 0, true); B("nOff", nightOffOn);
  o += ",\"nTa\":["; for (int i = 0; i < HC_POINTS; i++) { if (i) o += ','; o += String((int)c.nTa[i]); } o += "]";
  o += ",\"nDv\":["; for (int i = 0; i < HC_POINTS; i++) { if (i) o += ','; o += String((int)c.nDv[i]); } o += "]";
  N("k", c.roomK10 / 10.0f, 1, true); N("ki", c.roomKi10 / 10.0f, 1, true); N("max", c.roomMax, 0, true);
  N("vlMin", c.vlMin, 0, true); N("vlMax", c.vlMax, 0, true); N("ws", c.writeSec, 0, true); N("tau", c.taTau, 0, true);
  B("auto", c.autoEnable); N("mode", c.autoMode, 0, true); N("taOn", c.taOn, 0, true); N("taOff", c.taOff, 0, true);
  N("rtOn", c.rtOnBelow10 / 10.0f, 1, true); N("rtOff", c.rtOffAbove10 / 10.0f, 1, true);
  N("minOn", c.minOnMin, 0, true); N("minOff", c.minOffMin, 0, true);
  B("kamin", kaminOn); B("roomOn", c.roomOn); N("kTa", c.kaminTa, 0, true); N("kRt", c.kaminRt10 / 10.0f, 1, true); N("kMax", c.kaminMaxH, 0, true);
  N("kMin", kaminOn ? (millis() - kaminSinceMs) / 60000UL : 0, 0, true);
  N("pin", c.sensPin, 0, true); N("type", c.sensType, 0, true); N("off", c.sensOffset10 / 10.0f, 1, true);
  o += ",\"cTa\":["; for (int i = 0; i < HC_POINTS; i++) { if (i) o += ','; o += String((int)c.curveTa[i]); } o += "]";
  o += ",\"cVl\":["; for (int i = 0; i < HC_POINTS; i++) { if (i) o += ','; o += String((int)c.curveVl[i]); } o += "]";
  LockCfg L = lockCfg;
  B("lkEn", L.enabled); B("lkCold", L.coldOn); N("lkTa", L.coldTa, 0, true); B("lkWw", L.wwAllow);
  B("lkOn", lockOn); B("lkWin", lockWin);
  o += ",\"lkInfo\":\""; o += lockInfo; o += "\"";
  o += ",\"lkF\":["; for (int i = 0; i < 7; i++) { if (i) o += ','; o += String((int)L.from[i]); } o += "]";
  o += ",\"lkT\":["; for (int i = 0; i < 7; i++) { if (i) o += ','; o += String((int)L.to[i]); } o += "]";
  o += ",\"lkF2\":["; for (int i = 0; i < 7; i++) { if (i) o += ','; o += String((int)L.from2[i]); } o += "]";
  o += ",\"lkT2\":["; for (int i = 0; i < 7; i++) { if (i) o += ','; o += String((int)L.to2[i]); } o += "]";
  o += "}";
  sendJson(o.c_str());
}

void handleFc() {
  if (server.hasArg("cmd")) {
    fcCfg.enabled = !fcCfg.enabled; saveFcCfg();
    Log.printf("Frequenzkurve %s (Web)\n", fcCfg.enabled ? "EIN" : "AUS");
    sendJson("{\"ok\":true}"); return;
  }
  if (server.hasArg("save")) {
    FcCfg c = fcCfg;
    auto I = [](const char* k, long def, long lo, long hi) -> long {
      return server.hasArg(k) ? constrain(server.arg(k).toInt(), lo, hi) : def;
    };
    c.enabled = I("en", c.enabled, 0, 1);
    c.deadband = I("db", c.deadband, 1, 15);
    c.minIntMin = I("int", c.minIntMin, 5, 240);
    c.nightCap = I("nightCap", c.nightCap, 0, MAX_HZ_MAX);
    c.dfOn = I("dfOn", c.dfOn, 0, 1);
    c.dfHz = I("dfHz", c.dfHz, MAX_HZ_MIN, MAX_HZ_MAX);
    c.dfBackSec = I("dfBack", c.dfBackSec, 0, 1800);
    c.offLower = I("offLower", c.offLower, 0, 1);
    c.offHz = I("offHz", c.offHz, MAX_HZ_MIN, 60);
    c.fbOn = I("fbOn", c.fbOn, 0, 1);
    c.fbTaLo = I("fbLo", c.fbTaLo, -10, 10);
    c.fbTaHi = I("fbHi", c.fbTaHi, -5, 15);
    c.fbEvap = I("fbEv", c.fbEvap, -10, 5);
    c.fbStep = I("fbStep", c.fbStep, 2, 15);
    c.fbMinHz = I("fbMin", c.fbMinHz, MAX_HZ_MIN, 70);
    c.fbSrc = I("fbSrc", c.fbSrc, 0, 1);
    if (c.fbTaHi <= c.fbTaLo) c.fbTaHi = c.fbTaLo + 1;
    if (c.nightCap && c.nightCap < MAX_HZ_MIN) c.nightCap = MAX_HZ_MIN;
    const char* err = nullptr;
    if (server.hasArg("ta") && server.hasArg("hh") && server.hasArg("hw")) {
      String a = server.arg("ta"), h = server.arg("hh"), w = server.arg("hw");
      int ia = 0, ih = 0, iw = 0;
      for (int i = 0; i < HC_POINTS; i++) {
        int ca = a.indexOf(',', ia); if (ca < 0) ca = a.length();
        int ch = h.indexOf(',', ih); if (ch < 0) ch = h.length();
        int cw = w.indexOf(',', iw); if (cw < 0) cw = w.length();
        c.ta[i] = constrain(a.substring(ia, ca).toInt(), -30L, 30L);
        c.hzHeat[i] = constrain(h.substring(ih, ch).toInt(), (long)MAX_HZ_MIN, (long)MAX_HZ_MAX);
        c.hzWw[i] = constrain(w.substring(iw, cw).toInt(), (long)MAX_HZ_MIN, (long)MAX_HZ_MAX);
        ia = ca + 1; ih = ch + 1; iw = cw + 1;
      }
      for (int i = 1; i < HC_POINTS; i++)
        if (c.ta[i] >= c.ta[i - 1]) { err = "Aussentemperaturen muessen abnehmen"; break; }
    }
    if (err) { char b[120]; snprintf(b, sizeof(b), "{\"ok\":false,\"err\":\"%s\"}", err); sendJson(b); return; }
    fcCfg = c; saveFcCfg();
    Log.println("Frequenzkurve: Einstellungen gespeichert");
    sendJson("{\"ok\":true}");
    return;
  }
  FcCfg c = fcCfg;
  LiveData d = getLive();
  String o; o.reserve(700);
  auto N = [&](const char* k, float v, int dec, bool ok) {
    o += ",\""; o += k; o += "\":"; if (ok && !isnan(v)) o += String(v, dec); else o += "null";
  };
  auto B = [&](const char* k, bool v) { o += ",\""; o += k; o += "\":"; o += v ? "true" : "false"; };
  auto A = [&](const char* k, const int8_t* a8, const uint8_t* u8) {
    o += ",\""; o += k; o += "\":[";
    for (int i = 0; i < HC_POINTS; i++) { if (i) o += ','; o += String(a8 ? (int)a8[i] : (int)u8[i]); }
    o += "]";
  };
  o = "{\"ok\":true";
  B("en", c.enabled); N("db", c.deadband, 0, true); N("int", c.minIntMin, 0, true); N("nightCap", c.nightCap, 0, true);
  B("dfOn", c.dfOn); N("dfHz", c.dfHz, 0, true); N("dfBack", c.dfBackSec, 0, true); B("dfNow", dfActive);
  N("dfCnt", dfCountToday, 0, true);
  B("offLower", c.offLower); N("offHz", c.offHz, 0, true);
  B("fbOn", c.fbOn); N("fbLo", c.fbTaLo, 0, true); N("fbHi", c.fbTaHi, 0, true); N("fbEv", c.fbEvap, 0, true);
  N("fbStep", c.fbStep, 0, true); N("fbMin", c.fbMinHz, 0, true); N("fbSrc", c.fbSrc, 0, true);
  N("fbCap", fbCap, 0, fbCap > 0); N("curve", fcCurve, 0, fcCurve > 0);
  o += ",\"fbInfo\":\""; o += fbInfo; o += "\"";
  o += ",\"dfInfo\":\""; o += dfInfo; o += "\"";
  N("taF", hcTaF, 1, true); B("ww", fcWw); B("night", hcNight); N("target", fcTarget, 0, true);
  N("hz", d.verdichterHz, 0, d.hzValid); N("maxHz", d.maxVerdichterHz, 0, d.maxHzValid);
  N("hzW", hzWritesToday, 0, true); N("vlW", vlWritesToday, 0, true);
  o += ",\"info\":\""; o += fcInfo; o += "\"";
  A("ta", c.ta, nullptr); A("hzHeat", nullptr, c.hzHeat); A("hzWw", nullptr, c.hzWw);
  o += "}";
  sendJson(o.c_str());
}

// /dfl -> die letzten Abtauungen (Ende von abtau.csv, hoechstens 32 KB)
void handleDfLog() {
  String o;
  if (sdOk) {
    File f = SD_MMC.open("/log/abtau.csv", FILE_READ);
    if (f) {
      const uint32_t MAXB = 32768;
      uint32_t sz = f.size(), from = sz > MAXB ? sz - MAXB : 0;
      if (from) f.seek(from);
      o.reserve(sz - from + 16);
      static char buf[513];
      bool skip = from > 0;                 // angeschnittene erste Zeile weglassen
      int n;
      while ((n = f.read((uint8_t*)buf, 512)) > 0) {
        buf[n] = 0;
        const char* p = buf;
        if (skip) { const char* nl = strchr(p, '\n'); if (!nl) continue; p = nl + 1; skip = false; }
        o += p;
      }
      f.close();
    }
  }
  server.send(200, "text/plain; charset=utf-8", o);
}

// /feat: Funktionen ein-/ausblenden (PV/Energiemanagement)
void handleFeat() {
  if (server.hasArg("pv")) {
    PvCfg p = pvCfg;
    p.show = server.arg("pv").toInt() ? 1 : 0;
    if (p.show != pvCfg.show) {
      pvCfg = p; savePvCfg();
      if (!p.show) sgState = 2;
      Log.printf("Funktionen: PV/Energiemanagement %s\n", p.show ? "ein" : "aus");
    }
  }
  char b[64]; snprintf(b, sizeof(b), "{\"ok\":true,\"pv\":%s}", pvCfg.show ? "true" : "false");
  sendJson(b);
}

void handleHcCmd() {
  String a = server.arg("a");
  if (a == "plus") hcAdjustSoll(0.5f);
  else if (a == "minus") hcAdjustSoll(-0.5f);
  else if (a == "hc") { hcCfg.enabled = !hcCfg.enabled; saveHcCfg(); Log.printf("Heizkurve %s (Web)\n", hcCfg.enabled ? "EIN" : "AUS"); }
  else if (a == "kamin") kaminSet(!kaminOn, "Web");
  else if (a == "room") { hcCfg.roomOn = !hcCfg.roomOn; saveHcCfg(); Log.printf("Raumeinfluss %s (Web)\n", hcCfg.roomOn ? "EIN" : "AUS"); }
  else if (a == "auto") { hcCfg.autoEnable = !hcCfg.autoEnable; saveHcCfg(); Log.printf("Auto-WP %s (Web)\n", hcCfg.autoEnable ? "EIN" : "AUS"); }
  sendJson("{\"ok\":true}");
}

void handleRaw() {
  String o; o.reserve(700);
  o = "{\"block\":";
  if (rawBlockValid) { o += "["; for (int i = 0; i < 33; i++) { if (i) o += ','; o += String((unsigned)rawBlock[i]); } o += "]"; }
  else o += "null";
  o += ",\"cnt\":";
  if (rawCntValid) { o += "["; for (int i = 0; i < 5; i++) { if (i) o += ','; o += String((unsigned)rawCnt[i]); } o += "]"; }
  else o += "null";
  o += "}";
  sendJson(o.c_str());
}

void handlePvCmd() {
  String a = server.arg("a");
  if (a.startsWith("sg")) sgSet(a.substring(2).toInt(), "Web");
  else if (a.startsWith("heater")) {
    int m = a.substring(6).toInt();
    heaterManualUntil = m > 0 ? millis() + (uint32_t)m * 60000UL : 0;
    Log.printf("Heizstab Hand (Web): %d min\n", m);
  }
  sendJson("{\"ok\":true}");
}

void setupWebServer() {
  server.on("/pvcfg",     handlePvCfg);
  server.on("/feat",      handleFeat);
  server.on("/pvcmd",     handlePvCmd);
  server.on("/raw",       handleRaw);
  server.on("/hc",        handleHc);
  server.on("/hccmd",     handleHcCmd);
  server.on("/fc",        handleFc);
  server.on("/dfl",       handleDfLog);
  server.on("/stats",     handleStats);
  server.on("/ctrl",      handleCtrl);
  server.on("/ctrlset",   handleCtrlSet);
  server.on("/ctrlcmd",   handleCtrlCmd);
  server.on("/log",       handleLog);
  server.on("/cmd",       handleCmd);
  server.on("/mbcfg",     handleMbCfg);
  server.on("/cfgimport", handleCfgImport);
  setupUpdateRoutes();
  server.on("/",          handleRoot);
  server.on("/current",   handleCurrent);
  server.on("/history",   handleHistory);
  server.on("/set",       handleSet);
  server.on("/power",     handlePower);
  server.on("/mode",      handleMode);
  server.on("/setmaxhz",  handleSetMaxHz);
  server.on("/wppar",     handleWpPar);
  server.on("/setmaxfan", handleSetMaxFan);
  server.on("/files",     handleFiles);
  server.on("/ftpcfg",    handleFtpCfg);
  server.on("/ftpcmd",    handleFtpCmd);
  server.on("/cfgnas",    handleCfgNas);
  server.on("/download",  handleDownload);
  server.onNotFound(handleRoot);
  server.begin();
}

// =====================================================
// Setup
// =====================================================

void setup() {
  ctrlMutex = xSemaphoreCreateMutex();
  loadCtrlCfg();
  relaysInit();                              // Relais sofort definiert AUS
  loadMbConfig();
  if (mbCfg.de >= 0) { pinMode(mbCfg.de, OUTPUT); digitalWrite(mbCfg.de, LOW); }

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) delay(10);   // NICHT endlos warten (ohne USB)

  Log.printf("Start Avarma " AV_PLATFORM "  Firmware %s (kompiliert %s)\n", FW_VERSION, FW_BUILD);
  if (avResetByWatchdog()) Log.println("Letzter Neustart durch den Watchdog (Programm hing, z. B. USB-Stick antwortete nicht)");
  avBeforeRestart = []() { avKvFlush(true); };   // Einstellungen (falls auf dem Stick) vor dem Neustart sichern
  printHelp();

  liveMutex = xSemaphoreCreateMutex();
  loadMqttCfg();
  loadPvCfg();
  loadFtpCfg();
  loadHcCfg();
  loadLockCfg();
  loadFcCfg();
  dfLoad();
  kaminLoad();
  energyLoad();
  logMutex  = xSemaphoreCreateMutex();
  mbDoneSem = xSemaphoreCreateBinary();
  mbQueue   = xQueueCreate(8, sizeof(MbCmd));

  uiBegin();                                 // Display + Touch (GIGA: danach ist auch das SDRAM bereit)
  bootScreen("Init Speicher / " SD_NAME "...", nullptr);

  allocLogBuffer();
  if (!mountSd()) {
    Log.println(avDataMissing ? "Kein Datenspeicher: QSPI-Partition 4 fehlt (QSPIFormat ausfuehren) und kein USB-Stick"
                              : "Kein Datenspeicher gefunden");
    lastSdTryMs = millis();
  }

  bootScreen("Verbinde WLAN...", WIFI_SSID);
  connectWiFiBlocking();
  mqttStart();                               // verbindet sich selbststaendig neu
  configTzTime(TZ_INFO, NTP1, NTP2);         // NTP + Sommerzeit automatisch
  setupWebServer();

  if (WiFi.status() == WL_CONNECTED) bootScreen("WLAN OK", WiFi.localIP().toString().c_str());
  else bootScreen("WLAN fehlt", "versuche weiter...");
  delay(800);

  mbSetup();
  valveSlaveSetup();

  xTaskCreatePinnedToCore(modbusTask,  "modbus",  6144, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(displayTask, "display", 8192, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(ctrlTask,    "ctrl",    6144, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(heaterTask,  "heater",  6144, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(dhtTask,     "dht",     3072, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(ftpTask,     "ftp",     8192, nullptr, 1, nullptr, 0);   // NAS-Archiv
  // Watchdog: haengt loop() laenger als 30 s, startet der GIGA neu (Einstellungen bleiben erhalten)
  Log.println(avWdStart(30000) ? "Watchdog aktiv (30 s)" : "Watchdog nicht verfuegbar");
}

// =====================================================
// Loop: Web, Serial, Logging, SD, WLAN
// =====================================================
bool otaStarted = false;

void otaTick() {
  return;                                      // GIGA: Upload ueber USB (kein Netzwerk-Upload)
  if (!otaStarted && WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.setHostname(HOSTNAME);          // startet auch mDNS (avarma.local)
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() { Log.println("OTA-Update startet..."); sdFlush(); consoleFlush(); });
    ArduinoOTA.onEnd([]()   { Log.println("OTA-Update fertig, Neustart"); });
    ArduinoOTA.onError([](ota_error_t e) {
      const char* t = e == OTA_AUTH_ERROR ? "Passwort falsch" : e == OTA_BEGIN_ERROR ? "Start" :
                      e == OTA_CONNECT_ERROR ? "Verbindung zum PC (Firewall?)" :
                      e == OTA_RECEIVE_ERROR ? "Empfang" : e == OTA_END_ERROR ? "Abschluss" : "?";
      Log.printf("OTA-Fehler %u: %s\n", e, t);
    });
    ArduinoOTA.begin();
    MDNS.addService("http", "tcp", 80);
    otaStarted = true;
    Log.printf("OTA bereit: in der Arduino-IDE Port \"%s at %s\" waehlen\n",
               HOSTNAME, WiFi.localIP().toString().c_str());
  }
  if (otaStarted) ArduinoOTA.handle();
}

void loop() {
  avWdKick();
  handleSerial();
  ensureWiFi();
  avWdKick();
  otaTick();
  mqttTick();
  energySaveTick();
  server.handleClient();
  if (restartRequested && (!restartHold || millis() - restartHoldMs > 60000UL)) {
    restartRequested = false;
    sdFlush();
    consoleFlush();
    energySaveNow();
    delay(800);                                  // Webantwort noch rausschicken
    ESP.restart();
  }
  loggingTick();
  sdTick();
  avKvFlush();                                   // Einstellungen auf den Stick (nur falls QSPI fehlt)
  aggTick();
  delay(2);
}
