/*
  giga_ui.h  –  Oberflaeche fuer das GIGA Display Shield (800 x 480, Touch)
  =====================================================================
  Wird vom Hauptsketch an der Stelle des ESP-Display-Teils eingebunden und benutzt
  dessen globale Werte direkt (wie die ESP-Anzeige).

  Aufbau: Kopfzeile (Status, Uhr), Navigation links, Inhalt rechts.
  Seiten: Uebersicht, Verlauf, Heizung, Warmwasser, Verdichter, PV & SG, System
  Schrift: Inter (SIL Open Font License), geglaettet, siehe giga_fonts.h
*/
#pragma once
#include <Arduino_GigaDisplay_GFX.h>
#include <Arduino_GigaDisplayTouch.h>
#include "giga_fonts.h"

// ---- Einbaulage -------------------------------------------------------------
// 1 = quer, 3 = quer und um 180 Grad gedreht (je nachdem, wie herum das Display haengt)
#ifndef UI_ROTATION
#define UI_ROTATION 1
#endif
// Falls der Touch nicht zur Anzeige passt (mit Befehl TOUCH im Web/Serial pruefen):
// 0 = normal, 1 = X spiegeln, 2 = Y spiegeln, 3 = beide
#ifndef UI_TOUCH_FLIP
#define UI_TOUCH_FLIP 0
#endif
#define UI_RGB_LED 1            // RGB-LED auf dem Display-Shield zeigt den WP-Zustand

static const int UI_W = 800, UI_H = 480;
static const int HEAD_H = 46, NAV_W = 118;
static const int CX0 = 126, CY0 = 54, CX1 = 792, CY1 = 472;     // Inhaltsbereich
static const int CW = CX1 - CX0;

static GigaDisplay_GFX gfx;
static Arduino_GigaDisplayTouch touchPanel;
static uint16_t* fb = nullptr;
static bool needRedraw = true;

// ---- Seiten / Tasten ----------------------------------------------------------
enum : int { PG_HOME = 0, PG_CHART, PG_HEAT, PG_WW, PG_COMP, PG_PV, PG_SYS, PG_COUNT };
enum : int {
  U_NONE = -1,
  U_NAV = 100,                      // U_NAV + Seite
  U_ON = 200, U_OFF, U_WWNOW, U_KAMIN, U_AUTO, U_HC, U_ROOM, U_FC, U_RT_M, U_RT_P,
  U_RANGE = 230,                    // U_RANGE + i
  U_SER = 250,                      // U_SER + Reihe
  U_CHART = 270,
  U_V_HEIZ = 280, U_V_PUF, U_V_WW, U_V_AUTO, U_WWSTOP,
  U_SUB0 = 300, U_SUB1,
  U_SOLL_M = 310, U_SOLL_P, U_SOLL_SET, U_HZ_M, U_HZ_P, U_HZ_SET, U_FAN_M, U_FAN_P, U_FAN_SET,
  U_MODE_H = 330, U_MODE_W, U_MODE_K,
  U_BR_M = 340, U_BR_P, U_DIM_M, U_DIM_P, U_TH_DARK, U_TH_LIGHT, U_TOUCHDBG, U_RESTART
};

static int uiPage = PG_HOME;
static int compSub = 0;                       // Verdichter: 0 = Messwerte, 1 = Einstellen
static int chartRange = 1;                    // Index in CH_HOURS
static const int CH_HOURS[5] = {2, 8, 24, 72, 168};
static const char* CH_LBL[5] = {"2 h", "8 h", "24 h", "3 Tage", "7 Tage"};
static uint8_t chartSer = 0x0F;               // Bits: VL RL WW Hz Aussen Raum Soll
static int chartCursorX = -1;
static int uEditSoll = 35, uEditHz = 60, uEditFan = 450;
static bool uEditInit = false;
static uint32_t restartArmMs = 0;
static uint8_t uiBright = 100, uiDimMin = 5, uiTheme = 0;   // Helligkeit %, Abdunkeln nach x min (0 = nie), 0 = dunkel

struct UiBtn { int16_t x, y, w, h; int id; };
static UiBtn ubtn[64];
static int nUbtn = 0;

#define HIST_G_MAX 400
static HistOut histG[HIST_G_MAX + 1];

// ---- Farben -------------------------------------------------------------------
static uint16_t C_BG, C_PANEL, C_PANEL2, C_LINE, C_TEXT, C_DIM, C_ACC, C_ON_ACC, C_RED, C_BLUE, C_ORANGE,
                C_GREEN, C_YELLOW, C_PURPLE, C_BTN, C_HEAD, C_HEAD_T, C_GRID, C_SHADE;

static inline uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3); }

void uiSetTheme(uint8_t t) {
  uiTheme = t;
  if (t == 1) {                       // hell, wie die Weboberflaeche
    C_BG = rgb(238, 241, 245); C_PANEL = rgb(255, 255, 255); C_PANEL2 = rgb(244, 246, 249); C_LINE = rgb(214, 220, 228);
    C_TEXT = rgb(28, 33, 40); C_DIM = rgb(104, 113, 126); C_ACC = rgb(53, 112, 212); C_ON_ACC = rgb(255, 255, 255);
    C_RED = rgb(214, 64, 52); C_BLUE = rgb(53, 112, 212); C_ORANGE = rgb(230, 135, 20); C_GREEN = rgb(40, 160, 85);
    C_YELLOW = rgb(201, 164, 0); C_PURPLE = rgb(138, 79, 208); C_BTN = rgb(236, 239, 243);
    C_HEAD = rgb(30, 41, 59); C_HEAD_T = rgb(240, 244, 248); C_GRID = rgb(228, 232, 238); C_SHADE = rgb(214, 232, 255);
  } else {                            // dunkel
    C_BG = rgb(12, 14, 18); C_PANEL = rgb(28, 32, 40); C_PANEL2 = rgb(36, 41, 51); C_LINE = rgb(52, 58, 70);
    C_TEXT = rgb(235, 238, 242); C_DIM = rgb(140, 148, 162); C_ACC = rgb(70, 140, 250); C_ON_ACC = rgb(255, 255, 255);
    C_RED = rgb(240, 82, 70); C_BLUE = rgb(80, 140, 255); C_ORANGE = rgb(255, 165, 40); C_GREEN = rgb(60, 200, 100);
    C_YELLOW = rgb(240, 210, 60); C_PURPLE = rgb(170, 120, 240); C_BTN = rgb(46, 52, 64);
    C_HEAD = rgb(20, 24, 31); C_HEAD_T = rgb(235, 238, 242); C_GRID = rgb(44, 50, 60); C_SHADE = rgb(24, 46, 78);
  }
}

// ---- Pixel / Formen -------------------------------------------------------------
static int clipX0 = 0, clipY0 = 0, clipX1 = UI_W, clipY1 = UI_H;

static inline int fbIdx(int x, int y) {
#if UI_ROTATION == 3
  return y + (799 - x) * 480;
#else
  return (479 - y) + x * 480;
#endif
}
static inline uint16_t blend565(uint16_t bg, uint16_t fg, uint8_t a) {      // a = 0..15
  uint32_t r = ((fg >> 11) * a + (bg >> 11) * (15 - a)) / 15;
  uint32_t g = (((fg >> 5) & 63) * a + ((bg >> 5) & 63) * (15 - a)) / 15;
  uint32_t b = ((fg & 31) * a + (bg & 31) * (15 - a)) / 15;
  return (uint16_t)((r << 11) | (g << 5) | b);
}
static inline void plotA(int x, int y, uint16_t c, uint8_t a) {
  if (!fb || a == 0 || x < clipX0 || y < clipY0 || x >= clipX1 || y >= clipY1) return;
  uint16_t& d = fb[fbIdx(x, y)];
  d = a >= 15 ? c : blend565(d, c, a);
}
static void fillR(int x, int y, int w, int h, uint16_t c) {
  if (x < clipX0) { w -= clipX0 - x; x = clipX0; }
  if (y < clipY0) { h -= clipY0 - y; y = clipY0; }
  if (x + w > clipX1) w = clipX1 - x;
  if (y + h > clipY1) h = clipY1 - y;
  if (w > 0 && h > 0) gfx.fillRect(x, y, w, h, c);
}
// gefuelltes Rechteck mit geglaetteten runden Ecken
static void rrect(int x, int y, int w, int h, int r, uint16_t c) {
  if (r * 2 > h) r = h / 2;
  if (r * 2 > w) r = w / 2;
  fillR(x, y + r, w, h - 2 * r, c);
  fillR(x + r, y, w - 2 * r, r, c);
  fillR(x + r, y + h - r, w - 2 * r, r, c);
  for (int j = 0; j < r; j++) for (int i = 0; i < r; i++) {
    float dx = r - i - 0.5f, dy = r - j - 0.5f;
    float cov = r - sqrtf(dx * dx + dy * dy) + 0.5f;
    if (cov <= 0) continue;
    uint8_t a = cov >= 1 ? 15 : (uint8_t)(cov * 15 + 0.5f);
    plotA(x + i, y + j, c, a); plotA(x + w - 1 - i, y + j, c, a);
    plotA(x + i, y + h - 1 - j, c, a); plotA(x + w - 1 - i, y + h - 1 - j, c, a);
  }
}
static void disc(int cx, int cy, int r, uint16_t c) {
  for (int j = -r - 1; j <= r + 1; j++) for (int i = -r - 1; i <= r + 1; i++) {
    float cov = r - sqrtf((float)(i * i + j * j)) + 0.5f;
    if (cov > 0) plotA(cx + i, cy + j, c, cov >= 1 ? 15 : (uint8_t)(cov * 15 + 0.5f));
  }
}
static void ring(int cx, int cy, int r, int t, uint16_t c) {
  for (int j = -r - 1; j <= r + 1; j++) for (int i = -r - 1; i <= r + 1; i++) {
    float d = sqrtf((float)(i * i + j * j));
    float cov = min(r - d + 0.5f, d - (r - t) + 0.5f);
    if (cov > 0) plotA(cx + i, cy + j, c, cov >= 1 ? 15 : (uint8_t)(cov * 15 + 0.5f));
  }
}
// geglaettete Linie (Xiaolin Wu), w = 1 oder 2
static void aaLine(float x0, float y0, float x1, float y1, uint16_t c, int w) {
  bool steep = fabsf(y1 - y0) > fabsf(x1 - x0);
  if (steep) { float t = x0; x0 = y0; y0 = t; t = x1; x1 = y1; y1 = t; }
  if (x0 > x1) { float t = x0; x0 = x1; x1 = t; t = y0; y0 = y1; y1 = t; }
  float dx = x1 - x0, grad = dx < 0.001f ? 1 : (y1 - y0) / dx;
  float y = y0;
  for (int x = (int)roundf(x0); x <= (int)roundf(x1); x++) {
    int yi = (int)floorf(y);
    float f = y - yi;
    uint8_t a1 = (uint8_t)((1 - f) * 15 + 0.5f), a2 = (uint8_t)(f * 15 + 0.5f);
    for (int k = 0; k < w; k++) {
      if (steep) { plotA(yi + k, x, c, k == 0 ? a1 : 15); plotA(yi + k + 1, x, c, k == w - 1 ? a2 : 15); }
      else       { plotA(x, yi + k, c, k == 0 ? a1 : 15); plotA(x, yi + k + 1, c, k == w - 1 ? a2 : 15); }
    }
    y += grad;
  }
}

// ---- Text ---------------------------------------------------------------------
enum : int { AL_L = 0, AL_C = 1, AL_R = 2 };
enum : int { VA_TOP = 0, VA_MID = 1, VA_BASE = 2 };

static const char* utf8Next(const char* s, uint16_t& cp) {
  uint8_t c = (uint8_t)*s++;
  if (c < 0x80) cp = c;
  else if ((c & 0xE0) == 0xC0 && ((uint8_t)s[0] & 0xC0) == 0x80) { cp = (uint16_t)(((c & 0x1F) << 6) | ((uint8_t)s[0] & 0x3F)); s++; }
  else if ((c & 0xF0) == 0xE0 && ((uint8_t)s[0] & 0xC0) == 0x80 && ((uint8_t)s[1] & 0xC0) == 0x80) {
    cp = (uint16_t)(((c & 0x0F) << 12) | (((uint8_t)s[0] & 0x3F) << 6) | ((uint8_t)s[1] & 0x3F)); s += 2;
  } else cp = c;
  if (cp == 0x2013 || cp == 0x2014 || cp == 0x2212) cp = '-';      // Gedankenstrich / Minus
  else if (cp == 0x2192) cp = '>';
  else if (cp > 0xFF) cp = '?';
  return s;
}
static int txtW(const char* s, const AvFont& f) {
  int w = 0;
  while (*s) {
    uint16_t cp; s = utf8Next(s, cp);
    if (cp >= f.first && cp <= f.last) w += f.g[cp - f.first].adv;
  }
  return w;
}
// zeichnet Text; y je nach va: Oberkante der Grossbuchstaben, Mitte oder Grundlinie. Liefert die Breite.
static int txt(const char* s, int x, int y, const AvFont& f, uint16_t col, int al, int va) {
  int w = txtW(s, f);
  if (al == AL_C) x -= w / 2; else if (al == AL_R) x -= w;
  int base = va == VA_BASE ? y : va == VA_MID ? y + f.cap / 2 : y + f.cap;
  while (*s) {
    uint16_t cp; s = utf8Next(s, cp);
    if (cp < f.first || cp > f.last) continue;
    const AvGlyph& g = f.g[cp - f.first];
    const uint8_t* p = f.bmp + g.off;
    int gx = x + g.xo, gy = base + g.yo, n = 0;
    for (int yy = 0; yy < g.h; yy++)
      for (int xx = 0; xx < g.w; xx++, n++) {
        uint8_t a = (n & 1) ? (p[n >> 1] & 0x0F) : (p[n >> 1] >> 4);
        if (a) plotA(gx + xx, gy + yy, col, a);
      }
    x += g.adv;
  }
  return w;
}
// Text auf Breite kuerzen (mit ..)
static void txtFit(const char* s, int x, int y, int maxW, const AvFont& f, uint16_t col, int al, int va) {
  if (txtW(s, f) <= maxW) { txt(s, x, y, f, col, al, va); return; }
  char b[96]; strncpy(b, s, sizeof(b) - 3); b[sizeof(b) - 3] = 0;
  size_t n = strlen(b);
  while (n > 1) {
    b[--n] = 0;
    while (n && ((uint8_t)b[n - 1] & 0xC0) == 0x80) b[--n] = 0;   // keine halben UTF-8-Zeichen
    char t[100]; snprintf(t, sizeof(t), "%s..", b);
    if (txtW(t, f) <= maxW) { txt(t, x, y, f, col, al, va); return; }
  }
}

static const char* fmtNum(char* b, size_t n, float v, int dec, bool ok) {
  if (!ok || isnan(v)) { strncpy(b, "--", n); b[n - 1] = 0; return b; }
  snprintf(b, n, "%.*f", dec, v);
  return b;
}

// ---- Bausteine ------------------------------------------------------------------
static void addUBtn(int x, int y, int w, int h, int id) {
  if (nUbtn < (int)(sizeof(ubtn) / sizeof(ubtn[0]))) { ubtn[nUbtn].x = x; ubtn[nUbtn].y = y; ubtn[nUbtn].w = w; ubtn[nUbtn].h = h; ubtn[nUbtn].id = id; nUbtn++; }
}
static int uiHit(int x, int y) {
  for (int i = nUbtn - 1; i >= 0; i--) {
    const UiBtn& b = ubtn[i];
    if (x >= b.x - 4 && x < b.x + b.w + 4 && y >= b.y - 4 && y < b.y + b.h + 4) return b.id;
  }
  return U_NONE;
}
static void panel(int x, int y, int w, int h, const char* title) {
  rrect(x, y, w, h, 12, C_PANEL);
  if (title) txt(title, x + 16, y + 14, F17B, C_TEXT, AL_L, VA_TOP);
}
// Taste; stil 0 = normal, 1 = aktiv (Akzent), 2 = gruen, 3 = orange, 4 = rot
static void button(int x, int y, int w, int h, const char* l1, const char* l2, int id, int style) {
  uint16_t bg = C_BTN, fg = C_TEXT, f2 = C_DIM;
  if (style == 1) { bg = C_ACC; fg = C_ON_ACC; f2 = C_ON_ACC; }
  else if (style == 2) { bg = C_GREEN; fg = rgb(255, 255, 255); f2 = fg; }
  else if (style == 3) { bg = C_ORANGE; fg = rgb(20, 20, 20); f2 = fg; }
  else if (style == 4) { bg = C_RED; fg = rgb(255, 255, 255); f2 = fg; }
  rrect(x, y, w, h, 10, bg);
  const AvFont* f1 = strlen(l1) <= 2 ? &F22B : &F17B;               // zu breit -> kleinere Schrift
  if (txtW(l1, *f1) > w - 10) f1 = txtW(l1, F17) <= w - 10 ? &F17 : &F14;
  if (l2 && *l2) {
    txt(l1, x + w / 2, y + h / 2 - 10, *f1, fg, AL_C, VA_MID);
    txtFit(l2, x + w / 2, y + h / 2 + 11, w - 10, F14, f2, AL_C, VA_MID);
  } else txt(l1, x + w / 2, y + h / 2, *f1, fg, AL_C, VA_MID);
  addUBtn(x, y, w, h, id);
}
static void toggle(int x, int y, int w, int h, const char* name, bool on, const char* onTxt, const char* offTxt, int id) {
  button(x, y, w, h, name, on ? onTxt : offTxt, id, on ? 1 : 0);
}
// Kachel wie im Web: Titel, grosser Wert mit Einheit, Zusatzzeile; links ein Farbstreifen
static void tile(int x, int y, int w, int h, const char* label, const char* val, const char* unit,
                 uint16_t accent, const char* sub, uint16_t subCol) {
  rrect(x, y, w, h, 12, C_PANEL);
  rrect(x + 7, y + 14, 4, h - 28, 2, accent);
  txt(label, x + 20, y + 14, F14, C_DIM, AL_L, VA_TOP);
  int vw = txt(val, x + 20, y + 38, F32B, C_TEXT, AL_L, VA_TOP);
  if (unit && *unit) txt(unit, x + 24 + vw, y + 38 + F32B.cap, F17, C_DIM, AL_L, VA_BASE);
  if (sub && *sub) txtFit(sub, x + 20, y + h - 24, w - 28, F14, subCol, AL_L, VA_TOP);
}
static void chip(int x, int y, const char* t, uint16_t bg, uint16_t fg, int* wOut) {
  int w = txtW(t, F14) + 20;
  rrect(x, y, w, 26, 13, bg);
  txt(t, x + w / 2, y + 13, F14, fg, AL_C, VA_MID);
  if (wOut) *wOut = w;
}
static void kvRow(int x, int y, int w, const char* k, const char* v, bool stripe) {
  if (stripe) fillR(x, y - 6, w, 30, C_PANEL2);
  int vw = txt(v, x + w - 10, y + 9, F17B, C_TEXT, AL_R, VA_MID);
  txtFit(k, x + 10, y + 9, w - vw - 30, F14, C_DIM, AL_L, VA_MID);
}

// Ventilanzeige Heizung <-> Warmwasser (wie im Web)
static void valveBar(int x, int y, int w, bool big) {
  CtrlCfg cc = getCtrlCfg();
  bool known = cc.enabled && valveKnown && !(cc.drive == 1 && !slaveOnline);
  float vp = constrain(valvePosNow(), 0.0f, 100.0f);
  int bh = big ? 16 : 10, kr = big ? 13 : 9;
  rrect(x, y, w, bh, bh / 2, C_LINE);
  if (known) {
    uint16_t c = vp >= 99 ? C_ORANGE : vp <= 1 ? C_RED : C_YELLOW;
    rrect(x, y, max(bh, (int)(w * vp / 100.0f)), bh, bh / 2, c);
  }
  int bpx = x + w * cc.bufPos / 100;
  fillR(bpx, y + bh + 3, 2, 6, C_DIM);
  if (big) txt("Puffer", bpx, y + bh + 20, F14, C_DIM, AL_C, VA_MID);
  if (valveMoving) { int tx = x + w * constrain((int)valveTarget, 0, 100) / 100; fillR(tx - 1, y - 6, 3, bh + 12, C_BLUE); }
  int kx = x + (int)(w * vp / 100.0f);
  disc(kx, y + bh / 2, kr, known ? C_TEXT : C_DIM);
  disc(kx, y + bh / 2, kr - 3, known ? C_PANEL : C_LINE);
}

static uint16_t socCol(float soc) { return soc < 20 ? C_RED : soc < 50 ? C_YELLOW : C_GREEN; }

// Was macht die WP gerade? (Text + Farbe)
static uint16_t wpStatus(const LiveData& d, char* s, size_t n) {
  bool runs = d.hzValid && d.verdichterHz > 0;
  bool wwAct = (wwState == WW_HEAT || wwState == WW_MOVE);
  if (!d.wpStateValid) { strncpy(s, "WP: keine Verbindung", n); return C_RED; }
  if (!d.wpOnOffRaw) { strncpy(s, "WP aus", n); return C_DIM; }
  if (d.extValid && (d.state & 16)) { snprintf(s, n, "Abtauen  %u Hz", d.verdichterHz); return C_BLUE; }
  if (wwAct) { snprintf(s, n, "Warmwasser%s  %u Hz", pvLoad ? " (PV)" : "", d.verdichterHz); return C_ORANGE; }
  if (runs) { snprintf(s, n, "%s  %u Hz", d.wpModeValid ? modeName(d.wpModeRaw) : "läuft", d.verdichterHz); return C_GREEN; }
  strncpy(s, "bereit (Verdichter steht)", n);
  return C_TEXT;
}

// ---- Kopfzeile + Navigation -------------------------------------------------------
static void drawHead(const LiveData& d) {
  fillR(0, 0, UI_W, HEAD_H, C_HEAD);
  txt("Avarma R290", 16, HEAD_H / 2, F22B, C_HEAD_T, AL_L, VA_MID);
  // rechts: Uhr, WLAN, MQTT, Speicher (zuerst, damit links der Platz bekannt ist)
  char t[12] = "--:--:--";
  if (timeValid()) { time_t tt = time(nullptr); struct tm tmv; localtime_r(&tt, &tmv); snprintf(t, sizeof(t), "%02d:%02d:%02d", tmv.tm_hour, tmv.tm_min, tmv.tm_sec); }
  int tw = txt(t, UI_W - 16, HEAD_H / 2, F22B, C_HEAD_T, AL_R, VA_MID);
  int rx = UI_W - 16 - tw - 18;
  bool wifi = WiFi.status() == WL_CONNECTED;
  int rssi = wifi ? WiFi.RSSI() : -100;
  int bars = !wifi ? 0 : rssi > -55 ? 4 : rssi > -65 ? 3 : rssi > -75 ? 2 : 1;
  for (int i = 0; i < 4; i++) fillR(rx - 26 + i * 7, HEAD_H / 2 + 8 - (i + 1) * 4, 5, (i + 1) * 4, i < bars ? C_HEAD_T : rgb(70, 78, 92));
  if (!wifi) txt("WLAN", rx - 30, HEAD_H / 2, F14, C_RED, AL_R, VA_MID);
  rx -= wifi ? 40 : 76;
  int w;
  w = txtW("MQTT", F14) + 20; rx -= w;
  chip(rx, 10, "MQTT", mqttConnected ? C_GREEN : rgb(70, 78, 92), rgb(255, 255, 255), nullptr);
  w = txtW("USB", F14) + 20; rx -= w + 6;
  chip(rx, 10, "LOG", sdOk ? C_GREEN : rgb(70, 78, 92), rgb(255, 255, 255), nullptr);
  int xMax = rx - 14;                                   // bis hier darf die linke Statusanzeige gehen
  // links: Meldung oder WP-Zustand
  int x = 168;
  char msg[64];
  uint32_t age = getCmdMsg(msg, sizeof(msg));
  if (restoringHistory) txtFit("Lade Verlauf vom " SD_NAME " ...", x, HEAD_H / 2, xMax - x, F17B, C_YELLOW, AL_L, VA_MID);
  else if (age < 8000) txtFit(msg, x, HEAD_H / 2, xMax - x, F17B, C_YELLOW, AL_L, VA_MID);
  else {
    char s[48];
    uint16_t c = wpStatus(d, s, sizeof(s));
    chip(x, 10, d.wpStateValid ? (d.wpOnOffRaw ? "WP EIN" : "WP AUS") : "WP ?", d.wpStateValid && d.wpOnOffRaw ? C_GREEN : C_LINE,
         d.wpStateValid && d.wpOnOffRaw ? rgb(255, 255, 255) : C_HEAD_T, &w);
    x += w + 8;
    const char* sg = sgState == 4 ? "SG4" : sgState == 3 ? "SG3 PV" : sgState == 1 ? "SG1 Sperre" : nullptr;
    bool kc = kaminOn;
    int sw = txtW(s, F17) + 30;                          // Zustandstext hat Vorrang vor den Zusatz-Chips
    if (kc && x + 18 + sw + (sg ? txtW(sg, F14) + 26 : 0) + txtW("Kamin", F14) + 26 > xMax) kc = false;
    if (sg && x + 18 + sw + txtW(sg, F14) + 26 > xMax) sg = nullptr;
    int chipsW = (sg ? txtW(sg, F14) + 26 : 0) + (kc ? txtW("Kamin", F14) + 26 : 0);
    disc(x + 6, HEAD_H / 2, 5, c);
    x += 18;
    int room = xMax - x - chipsW;
    if (room > 60) { txtFit(s, x, HEAD_H / 2, room - 8, F17, C_HEAD_T, AL_L, VA_MID); x += min(room - 8, txtW(s, F17)) + 12; }
    if (sg && x + txtW(sg, F14) + 20 <= xMax) {
      chip(x, 10, sg, sgState == 1 ? C_RED : C_YELLOW, sgState == 1 ? rgb(255, 255, 255) : rgb(20, 20, 20), &w); x += w + 6;
    }
    if (kc && x + txtW("Kamin", F14) + 20 <= xMax) chip(x, 10, "Kamin", C_ORANGE, rgb(20, 20, 20), &w);
  }
}

static void drawNav() {
  static const char* names[PG_COUNT] = {"Übersicht", "Verlauf", "Heizung", "Warmwasser", "Verdichter", "PV & SG", "System"};
  fillR(0, HEAD_H, NAV_W, UI_H - HEAD_H, C_PANEL);
  int h = (UI_H - HEAD_H - 8) / PG_COUNT;
  for (int i = 0; i < PG_COUNT; i++) {
    int y = HEAD_H + 4 + i * h;
    bool act = uiPage == i;
    if (act) { rrect(6, y + 3, NAV_W - 12, h - 6, 10, C_PANEL2); rrect(6, y + 14, 4, h - 28, 2, C_ACC); }
    const AvFont& nf = act && txtW(names[i], F17B) <= NAV_W - 14 ? F17B : F17;
    txt(names[i], NAV_W / 2 + 3, y + h / 2, nf, act ? C_TEXT : C_DIM, AL_C, VA_MID);
    addUBtn(0, y, NAV_W, h, U_NAV + i);
  }
}

// ---- Seite: Uebersicht -------------------------------------------------------------
static void pageHome(const LiveData& d) {
  char v[16], s[64];
  const int W = (CW - 32) / 5, H = 100, G = 8;
  bool tv = d.tempsValid, e = d.extValid, runs = d.hzValid && d.verdichterHz > 0;
  bool wwAct = (wwState == WW_HEAT || wwState == WW_MOVE);
  int y = CY0;
  int x = CX0;
  if (tv) snprintf(s, sizeof(s), "Diff. %.1f K", d.vorlauf - d.ruecklauf); else s[0] = 0;
  tile(x, y, W, H, "Vorlauf", fmtNum(v, sizeof(v), d.vorlauf, 1, tv), "°C", C_RED, s, C_DIM); x += W + G;
  if (d.flowValid) snprintf(s, sizeof(s), "%.1f l/min", d.flowLmin); else s[0] = 0;
  tile(x, y, W, H, "Rücklauf", fmtNum(v, sizeof(v), d.ruecklauf, 1, tv), "°C", C_BLUE, s, C_DIM); x += W + G;
  if (wwAct) strcpy(s, pvLoad ? "lädt mit PV" : "lädt"); else snprintf(s, sizeof(s), "Soll %u °C", getCtrlCfg().wwSoll);
  tile(x, y, W, H, "Warmwasser", fmtNum(v, sizeof(v), d.warmwasser, 1, tv), "°C", C_ORANGE, s, wwAct ? C_ORANGE : C_DIM); x += W + G;
  snprintf(s, sizeof(s), "Soll %.1f", (float)hcRtSoll);
  tile(x, y, W, H, "Raum", fmtNum(v, sizeof(v), rtIst, 1, rtValid()), "°C", C_PURPLE, s, C_DIM); x += W + G;
  if (!isnan(hcTaF)) snprintf(s, sizeof(s), "gedämpft %.1f", (float)hcTaF); else s[0] = 0;
  tile(x, y, W, H, "Außen", fmtNum(v, sizeof(v), d.ta, 1, e), "°C", rgb(122, 138, 160), s, C_DIM);

  y += H + G; x = CX0;
  if (d.maxHzValid) snprintf(s, sizeof(s), "Max %u Hz%s", d.maxVerdichterHz, fcCfg.enabled ? " (A)" : ""); else s[0] = 0;
  tile(x, y, W, H, "Verdichter", fmtNum(v, sizeof(v), d.verdichterHz, 0, d.hzValid), "Hz", C_GREEN, s, C_DIM); x += W + G;
  if (e) snprintf(s, sizeof(s), "Strom %.2f kW", d.pEl / 1000.0f); else s[0] = 0;
  tile(x, y, W, H, "Wärme", fmtNum(v, sizeof(v), d.pTh / 1000.0f, 1, e && runs), "kW", rgb(192, 80, 77), s, C_DIM); x += W + G;
  bool copOk = e && d.pEl > 100 && runs;
  if (d.eElDay > 50) snprintf(s, sizeof(s), "heute %.1f", d.eThDay / d.eElDay); else strcpy(s, "heute --");
  tile(x, y, W, H, "COP jetzt", fmtNum(v, sizeof(v), copOk ? d.pTh / d.pEl : NAN, 1, copOk), "", C_GREEN, s, C_DIM); x += W + G;
  snprintf(s, sizeof(s), "Strom %.1f kWh", d.eElDay / 1000.0);
  tile(x, y, W, H, "Wärme heute", fmtNum(v, sizeof(v), d.eThDay / 1000.0, 1, true), "kWh", C_YELLOW, s, C_DIM); x += W + G;
  if (pvAny()) {
    float pw = pvGet(pvPower, 300000UL), soc = pvGet(pvSoc, 900000UL);
    if (!isnan(soc)) snprintf(s, sizeof(s), "Akku %.0f %%", soc); else s[0] = 0;
    tile(x, y, W, H, "PV", fmtNum(v, sizeof(v), pw / 1000.0f, 2, !isnan(pw)), "kW", C_YELLOW, s, C_DIM);
    if (!isnan(soc)) {
      int bw = W - 28; rrect(x + 20, y + H - 8, bw, 4, 2, C_LINE);
      rrect(x + 20, y + H - 8, max(4, (int)(bw * constrain(soc, 0.0f, 100.0f) / 100.0f)), 4, 2, socCol(soc));
    }
  } else tile(x, y, W, H, "PV", "--", "", C_LINE, "keine Daten", C_DIM);

  // Status + Ventil
  y += H + G;
  int ph = CY1 - y, lw = 310;
  panel(CX0, y, lw, ph, nullptr);
  uint16_t sc = wpStatus(d, s, sizeof(s));
  disc(CX0 + 24, y + 26, 7, sc);
  if (txtW(s, F22B) <= lw - 56) txt(s, CX0 + 40, y + 26, F22B, C_TEXT, AL_L, VA_MID);
  else txtFit(s, CX0 + 40, y + 26, lw - 56, F17B, C_TEXT, AL_L, VA_MID);
  txt("Heizung", CX0 + 18, y + 64, F14, C_RED, AL_L, VA_MID);
  txt("Warmwasser", CX0 + lw - 18, y + 64, F14, C_ORANGE, AL_R, VA_MID);
  valveBar(CX0 + 26, y + 80, lw - 52, false);
  CtrlCfg cc = getCtrlCfg();
  char vt[24]; valveShortText(vt, sizeof(vt));
  if (!cc.enabled) strcpy(s, "Ventil: Steuerung aus (Web)"); else snprintf(s, sizeof(s), "Ventil: %s", vt);
  txtFit(s, CX0 + 18, y + 116, lw - 36, F17, C_TEXT, AL_L, VA_MID);
  if (wwState != WW_IDLE && wwInfo[0]) txtFit(wwInfo, CX0 + 18, y + 142, lw - 36, F17, C_ORANGE, AL_L, VA_MID);
  else if (cc.enabled) { char nx[40]; nextEventText(cc, nx, sizeof(nx)); snprintf(s, sizeof(s), "Nächstes: %s", nx); txtFit(s, CX0 + 18, y + 142, lw - 36, F17, C_DIM, AL_L, VA_MID); }
  if (hcCfg.autoEnable) { snprintf(s, sizeof(s), "Auto: %s", hcAutoInfo); txtFit(s, CX0 + 18, y + 168, lw - 36, F14, lockOn ? C_RED : C_DIM, AL_L, VA_MID); }
  else if (lockOn) { snprintf(s, sizeof(s), "Sperrzeit: WP %s", lockInfo); txtFit(s, CX0 + 18, y + 168, lw - 36, F14, C_RED, AL_L, VA_MID); }
  else if (nightOffOn) { snprintf(s, sizeof(s), "Nacht-Aus: WP aus (aussen > %d C)", hcCfg.nOffTa); txtFit(s, CX0 + 18, y + 168, lw - 36, F14, C_RED, AL_L, VA_MID); }

  // Bedienung
  int bx = CX0 + lw + G, bw = CX1 - bx;
  panel(bx, y, bw, ph, nullptr);
  bool on = d.wpStateValid && d.wpOnOffRaw, off = d.wpStateValid && !d.wpOnOffRaw;
  const int b3 = (bw - 2 * 12 - 2 * 8) / 3, bh = 52;
  int x0 = bx + 12, y0 = y + 12;
  button(x0, y0, b3, bh, "WP EIN", nullptr, U_ON, on ? 2 : 0);
  button(x0 + b3 + 8, y0, b3, bh, "WP AUS", nullptr, U_OFF, off ? 1 : 0);
  if (wwAct) button(x0 + 2 * (b3 + 8), y0, b3, bh, "WW Stopp", nullptr, U_WWNOW, 3);
  else       button(x0 + 2 * (b3 + 8), y0, b3, bh, "Warmwasser", "jetzt laden", U_WWNOW, 0);
  y0 += bh + 8;
  button(x0, y0, b3, bh, "Kamin", kaminOn ? "AN" : "aus", U_KAMIN, kaminOn ? 3 : 0);
  toggle(x0 + b3 + 8, y0, b3, bh, "Auto-WP", hcCfg.autoEnable, "EIN", "aus", U_AUTO);
  toggle(x0 + 2 * (b3 + 8), y0, b3, bh, "Heizkurve", hcCfg.enabled, "EIN", "aus", U_HC);
  y0 += bh + 8;
  int sh = ph - (y0 - y) - 12;
  button(x0, y0, 64, sh, "-", nullptr, U_RT_M, 0);
  button(bx + bw - 12 - 64, y0, 64, sh, "+", nullptr, U_RT_P, 0);
  snprintf(s, sizeof(s), "%.1f °C", (float)hcRtSoll);
  txt(s, bx + bw / 2, y0 + sh / 2 - 8, F22B, C_TEXT, AL_C, VA_MID);
  txt(kaminOn ? "Raum-Soll (Kamin)" : hcNight ? "Raum-Soll Nacht" : "Raum-Soll Tag", bx + bw / 2, y0 + sh / 2 + 14, F14, C_DIM, AL_C, VA_MID);
}

// ---- Seite: Verlauf ---------------------------------------------------------------
static const char* SER_LBL[7] = {"Vorlauf", "Rücklauf", "WW", "Verdichter", "Außen", "Raum", "Soll VL"};
static uint16_t serCol(int i) {
  switch (i) { case 0: return C_RED; case 1: return C_BLUE; case 2: return C_ORANGE; case 3: return C_GREEN;
               case 4: return rgb(122, 138, 160); case 5: return C_PURPLE; default: return C_DIM; }
}
static float serVal(const HistOut& p, int i) {
  switch (i) { case 0: return p.vl; case 1: return p.rl; case 2: return p.ww; case 3: return p.hz;
               case 4: return p.ta; case 5: return p.rt; default: return p.sv; }
}

static void pageChart() {
  // Zeitraum
  int x = CX0;
  for (int i = 0; i < 5; i++) { button(x, CY0, 84, 40, CH_LBL[i], nullptr, U_RANGE + i, chartRange == i ? 1 : 0); x += 90; }
  // Reihen
  x = CX0;
  for (int i = 0; i < 7; i++) {
    bool on = chartSer & (1 << i);
    int w = txtW(SER_LBL[i], F14) + 42;
    rrect(x, CY0 + 48, w, 34, 10, on ? C_PANEL2 : C_BG);
    if (!on) { rrect(x, CY0 + 48, w, 34, 10, C_LINE); rrect(x + 1, CY0 + 49, w - 2, 32, 9, C_BG); }
    rrect(x + 10, CY0 + 63, 14, 4, 2, on ? serCol(i) : C_LINE);
    txt(SER_LBL[i], x + 30, CY0 + 65, F14, on ? C_TEXT : C_DIM, AL_L, VA_MID);
    addUBtn(x, CY0 + 48, w, 34, U_SER + i);
    x += w + 6;
  }
  int top = CY0 + 92;
  panel(CX0, top, CW, CY1 - top, nullptr);
  const int L = CX0 + 48, R = CX1 - 44, T = top + 18, B = CY1 - 34;
  const int PW = R - L, PH = B - T;
  if (!timeValid()) { txt("Uhrzeit noch nicht synchronisiert", CX0 + CW / 2, (T + B) / 2, F17, C_DIM, AL_C, VA_MID); return; }
  uint32_t to = nowUtc(), from = to - CH_HOURS[chartRange] * 3600UL, bucket = 0;
  int n = buildHistory(from, to, HIST_G_MAX, histG, &bucket, CH_HOURS[chartRange] > 24 * 13);
  if (n < 2) { txt("Noch nicht genug Daten", CX0 + CW / 2, (T + B) / 2, F17, C_DIM, AL_C, VA_MID); return; }
  float minT = 999, maxT = -999, maxHz = 10;
  for (int i = 0; i < n; i++) {
    const HistOut& p = histG[i];
    for (int s = 0; s < 7; s++) {
      if (s == 3 || !(chartSer & (1 << s))) continue;
      float v = serVal(p, s);
      if (isnan(v)) continue;
      minT = min(minT, v); maxT = max(maxT, v);
    }
    maxHz = max(maxHz, p.hz);
  }
  if (!(minT < maxT)) { minT = 20; maxT = 50; }
  float span = maxT - minT + 2, stepT = span <= 10 ? 2 : span <= 25 ? 5 : span <= 50 ? 10 : 20;
  minT = floorf((minT - 1) / stepT) * stepT; maxT = ceilf((maxT + 1) / stepT) * stepT;
  int nGrid = (int)lroundf((maxT - minT) / stepT);
  maxHz = ceilf(maxHz / (nGrid * 5.0f)) * nGrid * 5.0f;
  auto xF = [&](uint32_t t) { return L + (float)(t - from) / (float)(to - from) * PW; };
  auto yT = [&](float v) { return B - (v - minT) / (maxT - minT) * PH; };
  auto yH = [&](float v) { return B - v / maxHz * PH; };
  // Abtauen hinterlegen
  for (int i = 0; i < n; i++) if (histG[i].dfr > 0.3f) {
    int x0 = (int)xF(histG[i].t - bucket / 2), x1 = (int)xF(histG[i].t + bucket / 2) + 1;
    fillR(max(x0, L), T, min(x1, R) - max(x0, L), PH, C_SHADE);
  }
  char b[16];
  for (int i = 0; i <= nGrid; i++) {
    int y = B - i * PH / nGrid;
    fillR(L, y, PW, 1, C_GRID);
    snprintf(b, sizeof(b), "%.0f°", minT + stepT * i);
    txt(b, L - 8, y, F14, C_DIM, AL_R, VA_MID);
    if (chartSer & 8) { snprintf(b, sizeof(b), "%.0f", maxHz * i / nGrid); txt(b, R + 8, y, F14, C_GREEN, AL_L, VA_MID); }
  }
  int ticks = 6;
  for (int i = 0; i <= ticks; i++) {
    uint32_t t = from + (to - from) * i / ticks;
    time_t tt = t; struct tm tmv; localtime_r(&tt, &tmv);
    if (CH_HOURS[chartRange] > 24) snprintf(b, sizeof(b), "%02d.%02d. %02d h", tmv.tm_mday, tmv.tm_mon + 1, tmv.tm_hour);
    else snprintf(b, sizeof(b), "%02d:%02d", tmv.tm_hour, tmv.tm_min);
    int xx = L + PW * i / ticks;
    fillR(xx, B, 1, 5, C_DIM);
    txt(b, constrain(xx, L + 24, R - 24), B + 16, F14, C_DIM, AL_C, VA_MID);
  }
  const int order[7] = {3, 6, 4, 5, 2, 1, 0};                   // Vorlauf zuletzt (oben)
  for (int k = 0; k < 7; k++) {
    int s = order[k];
    if (!(chartSer & (1 << s))) continue;
    uint16_t c = serCol(s);
    float px = 0, py = 0; bool started = false;
    for (int i = 0; i < n; i++) {
      const HistOut& p = histG[i];
      float v = serVal(p, s);
      if (p.gap || isnan(v)) { started = false; if (isnan(v)) continue; }
      float xx = xF(p.t), yy = s == 3 ? yH(v) : yT(v);
      if (started) aaLine(px, py, xx, yy, c, s == 6 ? 1 : 2);
      px = xx; py = yy; started = true;
    }
  }
  addUBtn(L, T, PW, PH, U_CHART);
  // Cursor mit Werten
  if (chartCursorX >= L && chartCursorX <= R) {
    uint32_t tc = from + (uint32_t)((float)(chartCursorX - L) / PW * (to - from));
    int best = 0; uint32_t bd = 0xFFFFFFFF;
    for (int i = 0; i < n; i++) { uint32_t dd = histG[i].t > tc ? histG[i].t - tc : tc - histG[i].t; if (dd < bd) { bd = dd; best = i; } }
    const HistOut& p = histG[best];
    int cx = (int)xF(p.t);
    fillR(cx, T, 1, PH, C_TEXT);
    int rows = 1; for (int s = 0; s < 7; s++) if (chartSer & (1 << s)) rows++;
    int bw = 180, bh = 12 + rows * 22, bxp = cx + 12 + bw > R ? cx - 12 - bw : cx + 12;
    rrect(bxp, T + 6, bw, bh, 8, C_PANEL2);
    time_t tt = p.t; struct tm tmv; localtime_r(&tt, &tmv);
    snprintf(b, sizeof(b), "%02d.%02d. %02d:%02d", tmv.tm_mday, tmv.tm_mon + 1, tmv.tm_hour, tmv.tm_min);
    txt(b, bxp + 10, T + 22, F17B, C_TEXT, AL_L, VA_MID);
    int yy = T + 22;
    for (int s = 0; s < 7; s++) {
      if (!(chartSer & (1 << s))) continue;
      yy += 22;
      float v = serVal(p, s);
      rrect(bxp + 10, yy - 2, 12, 4, 2, serCol(s));
      txt(SER_LBL[s], bxp + 28, yy, F14, C_DIM, AL_L, VA_MID);
      char vb[16]; fmtNum(vb, sizeof(vb), v, s == 3 || s == 6 ? 0 : 1, !isnan(v));
      strncat(vb, s == 3 ? " Hz" : " °C", sizeof(vb) - strlen(vb) - 1);
      txt(vb, bxp + bw - 10, yy, F14, C_TEXT, AL_R, VA_MID);
    }
  }
}

// ---- Seite: Heizung --------------------------------------------------------------
static void pageHeat(const LiveData& d) {
  char s[72], v[16];
  HcCfg c = hcCfg;
  const int lw = 300;
  // links: Raum
  panel(CX0, CY0, lw, CY1 - CY0, kaminOn ? "Raum-Soll (Kamin)" : hcNight ? "Raum-Soll Nacht" : "Raum-Soll Tag");
  button(CX0 + 12, CY0 + 50, 56, 76, "-", nullptr, U_RT_M, 0);
  button(CX0 + lw - 68, CY0 + 50, 56, 76, "+", nullptr, U_RT_P, 0);
  snprintf(v, sizeof(v), "%.1f", (float)hcRtSoll);
  int w = txtW(v, F60);
  txt(v, CX0 + lw / 2 - 12, CY0 + 88, F60, C_TEXT, AL_C, VA_MID);
  txt("°C", CX0 + lw / 2 - 12 + w / 2 + 3, CY0 + 88 - F60.cap / 2 + 2, F14, C_DIM, AL_L, VA_TOP);
  int y = CY0 + 150;
  kvRow(CX0 + 12, y, lw - 24, "Raum ist", rtValid() ? (snprintf(s, sizeof(s), "%.1f °C", (float)rtIst), s) : "--", true); y += 30;
  kvRow(CX0 + 12, y, lw - 24, "Feuchte", rtValid() && !isnan(rhIst) ? (snprintf(s, sizeof(s), "%.0f %%", (float)rhIst), s) : "--", false); y += 30;
  kvRow(CX0 + 12, y, lw - 24, "Außen (gedämpft)", !isnan(hcTaF) ? (snprintf(s, sizeof(s), "%.1f °C", (float)hcTaF), s) : "--", true); y += 30;
  if (kaminOn) snprintf(s, sizeof(s), "an seit %lu min", (unsigned long)((millis() - kaminSinceMs) / 60000UL));
  else snprintf(s, sizeof(s), "aus");
  kvRow(CX0 + 12, y, lw - 24, "Kamin", s, false);
  int bw = (lw - 24 - 8) / 2, by = CY1 - 12 - 2 * 58 - 8;
  toggle(CX0 + 12, by, bw, 58, "Heizkurve", c.enabled, "EIN", "aus", U_HC);
  toggle(CX0 + 20 + bw, by, bw, 58, "Raumeinfluss", c.roomOn, "EIN", "aus", U_ROOM);
  toggle(CX0 + 12, by + 66, bw, 58, "Auto-WP", c.autoEnable, "EIN", "aus", U_AUTO);
  button(CX0 + 20 + bw, by + 66, bw, 58, "Kamin", kaminOn ? "AN" : "aus", U_KAMIN, kaminOn ? 3 : 0);

  // rechts: Heizkurve
  int rx = CX0 + lw + 8, rw = CX1 - rx;
  panel(rx, CY0, rw, CY1 - CY0, "Heizkurve");
  if (!c.enabled) txt("aus - VL-Soll wird nicht geschrieben", rx + rw - 16, CY0 + 22, F14, C_DIM, AL_R, VA_MID);
  const int L = rx + 52, R = rx + rw - 20, T = CY0 + 52, B = CY0 + 236;
  int tMin = c.curveTa[HC_POINTS - 1], tMax = c.curveTa[0];
  if (tMax <= tMin) { tMax = 15; tMin = -15; }
  float vMin = 20, vMax = 60;
  for (int i = 0; i < HC_POINTS; i++) { vMin = min(vMin, (float)c.curveVl[i] - 2); vMax = max(vMax, (float)c.curveVl[i] + 2); }
  vMin = floorf(vMin / 5) * 5; vMax = ceilf(vMax / 5) * 5;
  auto xA = [&](float ta) { return L + (tMax - ta) / (float)(tMax - tMin) * (R - L); };    // links warm, rechts kalt
  auto yV = [&](float vl) { return B - (vl - vMin) / (vMax - vMin) * (B - T); };
  for (float vv = vMin; vv <= vMax + 0.1f; vv += 5) {
    int yy = (int)yV(vv); fillR(L, yy, R - L, 1, C_GRID);
    snprintf(v, sizeof(v), "%.0f°", vv); txt(v, L - 8, yy, F14, C_DIM, AL_R, VA_MID);
  }
  for (int i = 0; i < HC_POINTS; i++) {
    int xx = (int)xA(c.curveTa[i]);
    snprintf(v, sizeof(v), "%d°", c.curveTa[i]); txt(v, xx, B + 14, F14, C_DIM, AL_C, VA_MID);
  }
  for (int i = 1; i < HC_POINTS; i++)
    aaLine(xA(c.curveTa[i - 1]), yV(c.curveVl[i - 1]), xA(c.curveTa[i]), yV(c.curveVl[i]), c.enabled ? C_RED : C_DIM, 2);
  for (int i = 0; i < HC_POINTS; i++) disc((int)xA(c.curveTa[i]), (int)yV(c.curveVl[i]), 4, c.enabled ? C_RED : C_DIM);
  if (!isnan(hcTaF) && !isnan(hcCurve)) {
    float ta = constrain((float)hcTaF, (float)tMin, (float)tMax);
    int px = (int)xA(ta);
    fillR(px, T, 1, B - T, C_DIM);
    ring(px, (int)yV(hcCurve), 8, 3, C_TEXT);
    if (!isnan(hcTargetF)) disc(px, (int)yV(constrain((float)hcTargetF, vMin, vMax)), 5, C_ACC);
  }
  y = B + 40;
  int cw2 = (rw - 32) / 2;
  kvRow(rx + 12, y, cw2, "Kurve", !isnan(hcCurve) ? (snprintf(s, sizeof(s), "%.1f °C", (float)hcCurve), s) : "--", true);
  kvRow(rx + 20 + cw2, y, cw2, "Raumeinfluss", (snprintf(s, sizeof(s), "%+.1f K", (float)hcRoomCorr), s), true); y += 30;
  kvRow(rx + 12, y, cw2, "Ziel VL", !isnan(hcTargetF) ? (snprintf(s, sizeof(s), "%.1f °C", (float)hcTargetF), s) : "--", false);
  kvRow(rx + 20 + cw2, y, cw2, "Soll an WP", d.sollValid ? (snprintf(s, sizeof(s), "%.0f °C", d.sollwert), s) : "--", false); y += 30;
  snprintf(s, sizeof(s), "VL-Soll: %s, heute %u x geschrieben", vlInfo, (unsigned)vlWritesToday);
  txtFit(s, rx + 22, y + 6, rw - 44, F14, C_DIM, AL_L, VA_MID); y += 24;
  snprintf(s, sizeof(s), "Auto-WP: %s", c.autoEnable ? hcAutoInfo : "aus (WP von Hand)");
  txtFit(s, rx + 22, y + 6, rw - 44, F14, C_DIM, AL_L, VA_MID); y += 22;
  if (lockCfg.enabled) { snprintf(s, sizeof(s), "Sperrzeit: %s", lockInfo); txtFit(s, rx + 22, y + 6, rw - 44, F14, lockOn ? C_RED : C_DIM, AL_L, VA_MID); }
}

// ---- Seite: Warmwasser -------------------------------------------------------------
static void pageWw(const LiveData& d) {
  char s[72], v[16];
  CtrlCfg cc = getCtrlCfg();
  bool wwAct = (wwState == WW_HEAT || wwState == WW_MOVE);
  const int lw = 300, th = 250;
  panel(CX0, CY0, lw, th, "Warmwasser");
  fmtNum(v, sizeof(v), d.warmwasser, 1, d.tempsValid);
  int w = txtW(v, F60);
  txt(v, CX0 + lw / 2 - 12, CY0 + 92, F60, wwAct ? C_ORANGE : C_TEXT, AL_C, VA_MID);
  txt("°C", CX0 + lw / 2 - 12 + w / 2 + 4, CY0 + 92 - F60.cap / 2 + 2, F17B, C_DIM, AL_L, VA_TOP);
  snprintf(s, sizeof(s), "Soll %u °C", cc.wwSoll);
  txt(s, CX0 + lw / 2, CY0 + 140, F17, C_DIM, AL_C, VA_MID);
  if (!cc.enabled) txt("Ventilsteuerung im Web aktivieren", CX0 + lw / 2, CY0 + 168, F14, C_DIM, AL_C, VA_MID);
  else if (wwAct && wwInfo[0]) txtFit(wwInfo, CX0 + lw / 2, CY0 + 168, lw - 30, F14, C_ORANGE, AL_C, VA_MID);
  else { char nx[40]; nextEventText(cc, nx, sizeof(nx)); snprintf(s, sizeof(s), "Nächstes: %s", nx); txtFit(s, CX0 + lw / 2, CY0 + 168, lw - 30, F14, C_DIM, AL_C, VA_MID); }
  if (wwAct) {
    button(CX0 + 12, CY0 + th - 66, lw - 24, 54, "Ladung abbrechen", nullptr, U_WWSTOP, 3);
  } else {
    button(CX0 + 12, CY0 + th - 66, lw - 24, 54, "Warmwasser jetzt laden", nullptr, U_WWNOW, 0);
  }

  // Ventil
  int rx = CX0 + lw + 8, rw = CX1 - rx;
  panel(rx, CY0, rw, th, "Umschaltventil");
  char vt[24]; valveShortText(vt, sizeof(vt));
  txt(vt, rx + rw - 16, CY0 + 22, F17B, valveMoving ? C_YELLOW : C_DIM, AL_R, VA_MID);
  txt("Heizung", rx + 22, CY0 + 62, F17B, C_RED, AL_L, VA_MID);
  txt("Warmwasser", rx + rw - 22, CY0 + 62, F17B, C_ORANGE, AL_R, VA_MID);
  valveBar(rx + 34, CY0 + 88, rw - 68, true);
  if (!cc.enabled) s[0] = 0;
  else if (cc.drive == 1 && !slaveOnline) strcpy(s, "Ventilmodul: keine Verbindung");
  else if (valveFailsafe) strcpy(s, "Ventilmodul: Notbetrieb (Heizung)");
  else snprintf(s, sizeof(s), "%s%s", manualPos >= 0 ? "Hand: " : "Automatik", manualPos >= 0 ? (manualPos == 0 ? "Heizung" : manualPos >= 100 ? "Warmwasser" : "Puffer") : "");
  txt(s, rx + rw / 2, CY0 + 142, F14, C_DIM, AL_C, VA_MID);
  int b4 = (rw - 24 - 3 * 8) / 4, by = CY0 + th - 66;
  button(rx + 12, by, b4, 54, "Heizung", nullptr, U_V_HEIZ, manualPos == 0 ? 1 : 0);
  button(rx + 12 + (b4 + 8), by, b4, 54, "Puffer", nullptr, U_V_PUF, manualPos > 0 && manualPos < 100 ? 1 : 0);
  button(rx + 12 + 2 * (b4 + 8), by, b4, 54, "WW", nullptr, U_V_WW, manualPos >= 100 ? 1 : 0);
  button(rx + 12 + 3 * (b4 + 8), by, b4, 54, "Auto", nullptr, U_V_AUTO, manualPos < 0 ? 1 : 0);

  // Zeitplan
  int ty = CY0 + th + 8;
  panel(CX0, ty, CW, CY1 - ty, "Zeitplan");
  {
    int lx = CX1 - 16;
    const char* lt[3] = {"Heizung", "Puffer", "Warmwasser"};
    uint16_t lc[3] = {C_RED, C_YELLOW, C_ORANGE};
    for (int i = 2; i >= 0; i--) { int w2 = txt(lt[i], lx, ty + 22, F14, C_DIM, AL_R, VA_MID); disc(lx - w2 - 10, ty + 22, 5, lc[i]); lx -= w2 + 30; }
  }
  static const char* WD[7] = {"Mo", "Di", "Mi", "Do", "Fr", "Sa", "So"};
  int today = -1;
  if (timeValid()) { time_t tt = time(nullptr); struct tm tmv; localtime_r(&tt, &tmv); today = (tmv.tm_wday + 6) % 7; }
  int colW = (CW - 24) / 7, gy = ty + 40;
  for (int dd = 0; dd < 7; dd++) {
    int x = CX0 + 12 + dd * colW;
    if (dd == today) rrect(x + 2, gy - 4, colW - 4, CY1 - gy - 8, 8, C_PANEL2);
    txt(WD[dd], x + colW / 2, gy + 10, F17B, dd == today ? C_ACC : C_DIM, AL_C, VA_MID);
    int yy = gy + 32;
    for (int k = 0; k < SCHED_SLOTS; k++) {
      const SchedEv& e = cc.sched[dd][k];
      if (e.act == ACT_NONE) continue;
      snprintf(s, sizeof(s), "%02u:%02u", e.min / 60, e.min % 60);
      uint16_t ac = e.act == ACT_WW ? C_ORANGE : e.act == ACT_PUFFER ? C_YELLOW : C_RED;
      rrect(x + 8, yy - 11, colW - 16, 22, 11, C_PANEL2);
      disc(x + 20, yy, 5, ac);
      txt(s, x + colW / 2 + 6, yy, F14, C_TEXT, AL_C, VA_MID);
      yy += 26;
    }
  }
}

// ---- Seite: Verdichter -------------------------------------------------------------
static void pageComp(const LiveData& d) {
  button(CX0, CY0, 150, 40, "Messwerte", nullptr, U_SUB0, compSub == 0 ? 1 : 0);
  button(CX0 + 158, CY0, 150, 40, "Einstellen", nullptr, U_SUB1, compSub == 1 ? 1 : 0);
  char b[40], v1[12], v2[12];
  int top = CY0 + 48;
  if (compSub == 0) {
    bool e = d.extValid, c = d.cntValid, tv = d.tempsValid;
    int cw2 = (CW - 8) / 2;
    panel(CX0, top, cw2, CY1 - top, nullptr);
    panel(CX0 + cw2 + 8, top, cw2, CY1 - top, nullptr);
    int x0 = CX0 + 8, x1 = CX0 + cw2 + 16, w = cw2 - 16, y = top + 16;
    char t[40];
    const int dy = 33;
    #define ROWL(i, k, val) kvRow(x0, y + (i) * dy, w, k, val, (i) % 2 == 0)
    #define ROWR(i, k, val) kvRow(x1, y + (i) * dy, w, k, val, (i) % 2 == 0)
    snprintf(t, sizeof(t), "%s °C", fmtNum(v1, sizeof(v1), d.ta, 1, e)); ROWL(0, "Außen", t);
    snprintf(t, sizeof(t), "%s / %s °C", fmtNum(v1, sizeof(v1), d.vorlauf, 1, tv), fmtNum(v2, sizeof(v2), d.ruecklauf, 1, tv)); ROWL(1, "Vorlauf / Rücklauf", t);
    snprintf(t, sizeof(t), "%s °C", fmtNum(v1, sizeof(v1), d.warmwasser, 1, tv)); ROWL(2, "Warmwasser", t);
    snprintf(t, sizeof(t), "%s l/min", fmtNum(v1, sizeof(v1), d.flowLmin, 1, d.flowValid)); ROWL(3, "Durchfluss", t);
    snprintf(t, sizeof(t), "%s %%", fmtNum(v1, sizeof(v1), d.pumpPct, 0, c)); ROWL(4, "Pumpe", t);
    snprintf(t, sizeof(t), "%s kW", fmtNum(v1, sizeof(v1), d.pTh / 1000.0f, 2, e)); ROWL(5, "Wärmeleistung", t);
    snprintf(t, sizeof(t), "%s kW", fmtNum(v1, sizeof(v1), d.pEl / 1000.0f, 2, e)); ROWL(6, d.pElSrc ? "Strom (Shelly)" : "Strom (geschätzt)", t);
    ROWL(7, "COP", fmtNum(v1, sizeof(v1), d.pEl > 100 ? d.pTh / d.pEl : NAN, 2, e && d.pEl > 100));
    if (hcTarget >= 0) snprintf(t, sizeof(t), "%s °C (Kurve %d)", fmtNum(v1, sizeof(v1), d.sollwert, 0, d.sollValid), (int)hcTarget);
    else snprintf(t, sizeof(t), "%s °C", fmtNum(v1, sizeof(v1), d.sollwert, 0, d.sollValid));
    ROWL(8, "Soll VL", t);
    snprintf(t, sizeof(t), "%s%s", d.wpModeValid ? modeName(d.wpModeRaw) : "--", e && (d.state & 16) ? ", Abtauen" : ""); ROWL(9, "Betriebsart", t);
    snprintf(t, sizeof(t), "%s / %s h", fmtNum(v1, sizeof(v1), d.starts, 0, c), fmtNum(v2, sizeof(v2), d.runtimeH, 0, c)); ROWL(10, "Starts / Laufzeit", t);
    snprintf(t, sizeof(t), "%s Hz", fmtNum(v1, sizeof(v1), d.verdichterHz, 0, d.hzValid)); ROWR(0, "Verdichter", t);
    snprintf(t, sizeof(t), "%s / %s Hz%s", fmtNum(v1, sizeof(v1), d.targetHz, 0, e), fmtNum(v2, sizeof(v2), d.maxVerdichterHz, 0, d.maxHzValid), fcCfg.enabled ? " A" : "");
    ROWR(1, "Ziel / Max", t);
    snprintf(t, sizeof(t), "%s / %s", fmtNum(v1, sizeof(v1), d.fan1Rpm, 0, d.fanRpmValid), fmtNum(v2, sizeof(v2), d.fan2Rpm, 0, d.fanRpmValid)); ROWR(2, "Lüfter 1 / 2 rpm", t);
    snprintf(t, sizeof(t), "%s / %s MPa", fmtNum(v1, sizeof(v1), d.hp, 2, e), fmtNum(v2, sizeof(v2), d.lp, 2, e)); ROWR(3, "Hochdruck / Niederdruck", t);
    snprintf(t, sizeof(t), "%s °C", fmtNum(v1, sizeof(v1), d.discharge, 1, e)); ROWR(4, "Heißgas", t);
    snprintf(t, sizeof(t), "%s °C", fmtNum(v1, sizeof(v1), d.suction, 1, e)); ROWR(5, "Sauggas", t);
    snprintf(t, sizeof(t), "%s / %s °C", fmtNum(v1, sizeof(v1), d.evap, 1, e), fmtNum(v2, sizeof(v2), d.cond, 1, e)); ROWR(6, "Verdampfer / Kondensator", t);
    snprintf(t, sizeof(t), "%s V  %s A", fmtNum(v1, sizeof(v1), d.acV, 0, e), fmtNum(v2, sizeof(v2), d.currentA, 1, e)); ROWR(7, "Netz", t);
    snprintf(t, sizeof(t), "%s V", fmtNum(v1, sizeof(v1), d.dcV, 0, e)); ROWR(8, "Zwischenkreis", t);
    snprintf(t, sizeof(t), "%s °C", fmtNum(v1, sizeof(v1), d.ipm, 0, e)); ROWR(9, "IPM", t);
    if (d.lastErr && millis() - d.lastErrMs < 60000) snprintf(t, sizeof(t), "Fehler %u (%s)", d.lastErr, mbErrText(d.lastErr));
    else snprintf(t, sizeof(t), "%lu ok / %lu Fehler", (unsigned long)d.okCount, (unsigned long)d.errCount);
    ROWR(10, "Modbus", t);
    #undef ROWL
    #undef ROWR
    return;
  }
  // Einstellen
  if (!uEditInit) {
    if (d.sollValid)   uEditSoll = constrain((int)lroundf(d.sollwert), SOLL_MIN, SOLL_MAX);
    if (d.maxHzValid)  uEditHz   = constrain((int)d.maxVerdichterHz, (int)MAX_HZ_MIN, (int)MAX_HZ_MAX);
    if (d.maxFanValid) uEditFan  = constrain((int)d.maxFanRpm, (int)FAN_RPM_MIN, (int)FAN_RPM_MAX);
    uEditInit = true;
  }
  panel(CX0, top, CW, CY1 - top, nullptr);
  struct Row { const char* name; int val; const char* unit; int idM, idP, idS; char cur[40]; };
  Row rows[3];
  const char* rn[3] = {"Soll Vorlauf", "Max. Verdichter", "Max. Lüfter"};
  const char* ru[3] = {"°C", "Hz", "rpm"};
  const int rv[3] = {uEditSoll, uEditHz, uEditFan};
  const int rid[3][3] = {{U_SOLL_M, U_SOLL_P, U_SOLL_SET}, {U_HZ_M, U_HZ_P, U_HZ_SET}, {U_FAN_M, U_FAN_P, U_FAN_SET}};
  for (int i = 0; i < 3; i++) {
    rows[i].name = rn[i]; rows[i].unit = ru[i]; rows[i].val = rv[i];
    rows[i].idM = rid[i][0]; rows[i].idP = rid[i][1]; rows[i].idS = rid[i][2]; rows[i].cur[0] = 0;
  }
  if (d.sollValid) snprintf(rows[0].cur, 40, hcCfg.enabled ? "an WP: %.0f °C (Heizkurve aktiv)" : "an WP: %.0f °C", d.sollwert); else strcpy(rows[0].cur, "an WP: --");
  if (d.maxHzValid) snprintf(rows[1].cur, 40, fcCfg.enabled ? "an WP: %u Hz (Automatik)" : "an WP: %u Hz", d.maxVerdichterHz); else strcpy(rows[1].cur, "an WP: --");
  if (d.maxFanValid) snprintf(rows[2].cur, 40, "an WP: %u rpm", d.maxFanRpm); else strcpy(rows[2].cur, "an WP: --");
  int y = top + 14;
  for (int i = 0; i < 3; i++) {
    txt(rows[i].name, CX0 + 20, y + 16, F17B, C_TEXT, AL_L, VA_MID);
    txt(rows[i].cur, CX0 + 20, y + 40, F14, C_DIM, AL_L, VA_MID);
    int bx = CX0 + 300;
    button(bx, y, 60, 54, "-", nullptr, rows[i].idM, 0);
    rrect(bx + 66, y, 120, 54, 10, C_PANEL2);
    snprintf(b, sizeof(b), "%d", rows[i].val);
    txt(b, bx + 126, y + 22, F22B, C_TEXT, AL_C, VA_MID);
    txt(rows[i].unit, bx + 126, y + 42, F14, C_DIM, AL_C, VA_MID);
    button(bx + 192, y, 60, 54, "+", nullptr, rows[i].idP, 0);
    button(CX1 - 16 - 100, y, 100, 54, "Setzen", nullptr, rows[i].idS, 1);
    y += 66;
  }
  fillR(CX0 + 16, y - 4, CW - 32, 1, C_LINE);
  y += 8;
  txt("Betriebsart", CX0 + 20, y + 27, F17B, C_TEXT, AL_L, VA_MID);
  int bx = CX0 + 300, b3 = 112;
  button(bx, y, b3, 54, "Heizen", nullptr, U_MODE_H, d.wpModeValid && d.wpModeRaw == 2 ? 1 : 0);
  button(bx + b3 + 8, y, b3, 54, "Warmwasser", nullptr, U_MODE_W, d.wpModeValid && d.wpModeRaw == 1 ? 1 : 0);
  button(bx + 2 * (b3 + 8), y, b3, 54, "Kühlen", nullptr, U_MODE_K, d.wpModeValid && d.wpModeRaw == 4 ? 1 : 0);
  y += 66;
  toggle(CX0 + 16, y, 200, 54, "Frequenz-Automatik", fcCfg.enabled, "EIN (Kurve)", "aus (von Hand)", U_FC);
  snprintf(b, sizeof(b), "%s", fcInfo); txtFit(b, CX0 + 232, y + 14, CW - 250, F14, C_DIM, AL_L, VA_MID);
  snprintf(b, sizeof(b), "Heute geschrieben: VL %u x, Hz %u x", (unsigned)vlWritesToday, (unsigned)hzWritesToday);
  txt(b, CX0 + 232, y + 36, F14, C_DIM, AL_L, VA_MID);
  snprintf(b, sizeof(b), "Abtauungen heute: %u%s", (unsigned)dfCountToday, dfActive ? " (läuft)" : "");
  txt(b, CX0 + 232, y + 56, F14, dfActive ? C_BLUE : C_DIM, AL_L, VA_MID);
}

// ---- Seite: PV ----------------------------------------------------------------------
static void pagePv(const LiveData& d) {
  char v[16], s[64];
  if (!pvAny()) {
    panel(CX0, CY0, CW, CY1 - CY0, "PV / Victron (MQTT)");
    txt("Keine PV-Daten empfangen", CX0 + CW / 2, CY0 + 80, F22B, C_TEXT, AL_C, VA_MID);
    const char* l[] = {"Node-RED sendet per MQTT an:", "avarma/pv/power  -  PV-Leistung in W", "avarma/pv/soc  -  Batterie in %",
                       "avarma/pv/bat  -  Batterie in W (+ = laden)", "avarma/pv/grid  -  Netz in W (+ = Bezug)",
                       "avarma/pv/house  -  Verbrauch in W", "avarma/pv/today  -  Ertrag heute in kWh (optional)"};
    for (int i = 0; i < 7; i++) txt(l[i], CX0 + 60, CY0 + 140 + i * 30, i ? F17 : F17B, i ? C_DIM : C_TEXT, AL_L, VA_MID);
    return;
  }
  const uint32_t AGE = 300000UL;
  float pw = pvGet(pvPower, AGE), soc = pvGet(pvSoc, 900000UL), bat = pvGet(pvBat, AGE), grid = pvGet(pvGrid, AGE), house = pvGet(pvHouse, AGE);
  float td = pvTodayKwh(), yd = pvYestKwh();
  const int W = (CW - 16) / 3, H = 104, G = 8;
  int x = CX0, y = CY0;
  snprintf(s, sizeof(s), "heute %s kWh", fmtNum(v, sizeof(v), td, 1, !isnan(td)));
  tile(x, y, W, H, "PV-Leistung", fmtNum(v, sizeof(v), pw / 1000.0f, 2, !isnan(pw)), "kW", C_YELLOW, s, C_DIM); x += W + G;
  if (!isnan(bat)) snprintf(s, sizeof(s), "%s %.2f kW", bat >= 0 ? "lädt" : "entlädt", fabsf(bat) / 1000.0f); else s[0] = 0;
  tile(x, y, W, H, "Batterie", fmtNum(v, sizeof(v), soc, 0, !isnan(soc)), "%", isnan(soc) ? C_LINE : socCol(soc), s, C_DIM);
  if (!isnan(soc)) {
    int bw = W - 28; rrect(x + 20, y + H - 8, bw, 4, 2, C_LINE);
    rrect(x + 20, y + H - 8, max(4, (int)(bw * constrain(soc, 0.0f, 100.0f) / 100.0f)), 4, 2, socCol(soc));
  }
  x += W + G;
  tile(x, y, W, H, "Netz", fmtNum(v, sizeof(v), fabsf(grid) / 1000.0f, 2, !isnan(grid)), "kW", isnan(grid) ? C_LINE : grid > 50 ? C_RED : C_GREEN,
       isnan(grid) ? "" : grid >= 0 ? "Bezug" : "Einspeisung", C_DIM);
  x = CX0; y += H + G;
  float wp = d.extValid ? d.pEl : NAN;
  if (!isnan(house) && !isnan(wp) && house > 100) snprintf(s, sizeof(s), "davon WP %.0f %%", constrain(wp / house * 100, 0.0f, 100.0f)); else s[0] = 0;
  tile(x, y, W, H, "Hausverbrauch", fmtNum(v, sizeof(v), house / 1000.0f, 2, !isnan(house)), "kW", C_ACC, s, C_DIM); x += W + G;
  snprintf(s, sizeof(s), "Wärme %s kW", fmtNum(v, sizeof(v), d.pTh / 1000.0f, 1, d.extValid));
  tile(x, y, W, H, "Wärmepumpe", fmtNum(v, sizeof(v), wp / 1000.0f, 2, !isnan(wp)), "kW", C_GREEN, s, C_DIM); x += W + G;
  snprintf(s, sizeof(s), "gestern %s kWh", fmtNum(v, sizeof(v), yd, 1, !isnan(yd)));
  tile(x, y, W, H, "PV heute", fmtNum(v, sizeof(v), td, 1, !isnan(td)), "kWh", C_YELLOW, s, C_DIM);
  y += H + G;
  panel(CX0, y, CW, CY1 - y, "PV-Nutzung");
  static const char* sgTxt[5] = {"?", "SG1 – Sperre", "SG2 – normal", "SG3 – PV-Überschuss", "SG4 – großer Überschuss"};
  int yy = y + 56;
  kvRow(CX0 + 12, yy, CW / 2 - 20, "SG-Ready", sgTxt[(sgState >= 1 && sgState <= 4) ? sgState : 0], true);
  kvRow(CX0 + CW / 2 + 8, yy, CW / 2 - 20, "Heizstab", heaterOn ? "AN" : pvCfg.heaterEnable ? "aus" : "nicht aktiv", true); yy += 32;
  if (pvLoad) strcpy(s, "läuft");
  else if (wwState == WW_HEAT || wwState == WW_MOVE) strcpy(s, "WW-Ladung läuft");
  else strcpy(s, pvCfg.pvWwEnable ? "bereit" : "aus");
  kvRow(CX0 + 12, yy, CW / 2 - 20, "PV-Warmwasser", s, false);
  uint32_t age = pvPower.ms ? (millis() - pvPower.ms) / 1000 : 0;
  snprintf(s, sizeof(s), "vor %lu s", (unsigned long)age);
  kvRow(CX0 + CW / 2 + 8, yy, CW / 2 - 20, "Letzte Daten", s, false);
  yy += 32;
  kvRow(CX0 + 12, yy, CW - 24, "Heizstab-Status", heaterInfo, true);
}

// ---- Seite: System ----------------------------------------------------------------
static void pageSys(const LiveData& d) {
  char s[64];
  int lw = 370;
  panel(CX0, CY0, lw, CY1 - CY0, "Gerät");
  int y = CY0 + 50, w = lw - 24, x = CX0 + 12, i = 0;
  auto row = [&](const char* k, const char* v) { kvRow(x, y, w, k, v, i % 2 == 0); y += 31; i++; };
  row("Firmware", "v" FW_VERSION " (" AV_PLATFORM ")");
  if (WiFi.status() == WL_CONNECTED) { snprintf(s, sizeof(s), "%s  (%ld dBm)", WiFi.localIP().toString().c_str(), (long)WiFi.RSSI()); row("WLAN", s); }
  else row("WLAN", "nicht verbunden");
  row("MQTT", mqttConnected ? "verbunden" : mqttCfg.enabled ? "getrennt" : "aus");
  if (sdOk) {
    double fr = (sdTotalBytes - sdUsedBytes) / 1048576.0, tot = sdTotalBytes / 1048576.0;
    if (tot >= 1024) snprintf(s, sizeof(s), "%.1f von %.1f GB frei", fr / 1024, tot / 1024);
    else snprintf(s, sizeof(s), "%.1f von %.1f MB frei (%s)", fr, tot, SD_MMC.kind() == 1 ? "intern" : "USB");
    row(SD_NAME, s);
  } else row(SD_NAME, "nicht gefunden");
  if (!ftpCfg.enabled) row("NAS", "aus");
  else if (ftpBusy) row("NAS", "überträgt ...");
  else if (ftpLastOk && !strncmp(ftpMsg, "OK", 2)) {
    time_t t = ftpLastOk; struct tm tmv; localtime_r(&t, &tmv);
    snprintf(s, sizeof(s), "OK, %02d:%02d", tmv.tm_hour, tmv.tm_min); row("NAS", s);
  } else row("NAS", strncmp(ftpMsg, "Fehler", 6) ? "wartet" : "Fehler (siehe Web)");
  snprintf(s, sizeof(s), "%lu ok / %lu Fehler", (unsigned long)d.okCount, (unsigned long)d.errCount); row("Modbus WP", s);
  CtrlCfg cc = getCtrlCfg();
  if (cc.drive == 1) { snprintf(s, sizeof(s), "%s (%lu / %lu)", slaveOnline ? "online" : "offline", (unsigned long)slaveOkCount, (unsigned long)slaveErrCount); row("Ventilmodul", s); }
  if (hcCfg.sensType == 40) snprintf(s, sizeof(s), "SHT4x: %lu ok / %lu Fehler", (unsigned long)dhtOk, (unsigned long)dhtErr);
  else snprintf(s, sizeof(s), "DHT%u an D%d: %lu / %lu", hcCfg.sensType, hcCfg.sensPin, (unsigned long)dhtOk, (unsigned long)dhtErr);
  row("Raumfühler", s);
  uint32_t up = millis() / 1000;
  snprintf(s, sizeof(s), "%lu d %02lu:%02lu", (unsigned long)(up / 86400), (unsigned long)(up / 3600 % 24), (unsigned long)(up / 60 % 60)); row("Laufzeit", s);
  row("Uhrzeit", timeValid() ? "per NTP gestellt" : "noch nicht gestellt");
  snprintf(s, sizeof(s), "%lu kB frei", (unsigned long)(ESP.getFreeHeap() / 1024)); row("Arbeitsspeicher", s);

  int rx = CX0 + lw + 8, rw = CX1 - rx;
  panel(rx, CY0, rw, CY1 - CY0, "Anzeige");
  y = CY0 + 52;
  auto stepper = [&](const char* name, const char* val, int idM, int idP) {
    txt(name, rx + 18, y + 6, F14, C_DIM, AL_L, VA_MID);
    int cw3 = rw - 36, bw3 = 64;
    button(rx + 18, y + 20, bw3, 48, "-", nullptr, idM, 0);
    rrect(rx + 18 + bw3 + 6, y + 20, cw3 - 2 * bw3 - 12, 48, 10, C_PANEL2);
    txt(val, rx + 18 + cw3 / 2, y + 44, F17B, C_TEXT, AL_C, VA_MID);
    button(rx + 18 + cw3 - bw3, y + 20, bw3, 48, "+", nullptr, idP, 0);
    y += 80;
  };
  snprintf(s, sizeof(s), "%u %%", uiBright); stepper("Helligkeit", s, U_BR_M, U_BR_P);
  if (uiDimMin) snprintf(s, sizeof(s), "%u min", uiDimMin); else strcpy(s, "nie");
  stepper("Abdunkeln nach", s, U_DIM_M, U_DIM_P);
  txt("Farben", rx + 18, y + 6, F14, C_DIM, AL_L, VA_MID);
  int hw = (rw - 36 - 8) / 2;
  button(rx + 18, y + 20, hw, 48, "Dunkel", nullptr, U_TH_DARK, uiTheme == 0 ? 1 : 0);
  button(rx + 26 + hw, y + 20, hw, 48, "Hell", nullptr, U_TH_LIGHT, uiTheme == 1 ? 1 : 0);
  y += 80;
  toggle(rx + 18, y, hw, 54, "Touch-Test", touchDebug, "an", "aus", U_TOUCHDBG);
  bool armed = restartArmMs && millis() - restartArmMs < 4000;
  button(rx + 26 + hw, y, hw, 54, armed ? "Nochmal" : "Neustart", armed ? "tippen" : nullptr, U_RESTART, armed ? 4 : 0);
  y += 72;
  txtFit("Alle Einstellungen der Regelung im Web:", rx + 18, y, rw - 36, F14, C_DIM, AL_L, VA_MID);
  if (WiFi.status() == WL_CONNECTED) { snprintf(s, sizeof(s), "http://%s", WiFi.localIP().toString().c_str()); txt(s, rx + 18, y + 22, F17B, C_ACC, AL_L, VA_MID); }
}

// ---- Gesamtbild ------------------------------------------------------------------
static void uiRender() {
  if (!fb) return;
  LiveData d = getLive();
  nUbtn = 0;
  gfx.startBuffering();
  gfx.fillScreen(C_BG);
  drawHead(d);
  drawNav();
  switch (uiPage) {
    case PG_HOME:  pageHome(d); break;
    case PG_CHART: pageChart(); break;
    case PG_HEAT:  pageHeat(d); break;
    case PG_WW:    pageWw(d); break;
    case PG_COMP:  pageComp(d); break;
    case PG_PV:    pagePv(d); break;
    default:       pageSys(d); break;
  }
  if (touchDebug && millis() - lastTouchMs < 3000 && lastTouchX >= 0) {
    fillR(lastTouchX - 16, lastTouchY, 33, 1, C_YELLOW);
    fillR(lastTouchX, lastTouchY - 16, 1, 33, C_YELLOW);
    ring(lastTouchX, lastTouchY, 10, 2, C_YELLOW);
  }
  gfx.endBuffering();
}

// ---- Hintergrundbeleuchtung (Software-PWM wie GigaDisplayBacklight, 100 Hz) ----
static mbed::DigitalOut* blPin = nullptr;
static mbed::Ticker* blTicker = nullptr;
static volatile uint8_t blLevel = 10, blCnt = 0;
static void blTick() { blCnt = blCnt >= 9 ? 0 : blCnt + 1; blPin->write(blCnt < blLevel ? 1 : 0); }
static void uiBacklight(uint8_t pct) {
  uint8_t lv = (uint8_t)constrain((pct + 5) / 10, 1, 10);
  if (!blPin) {
    blPin = new mbed::DigitalOut(PB_12, 1);
    blTicker = new mbed::Ticker();
    blTicker->attach(mbed::callback(blTick), std::chrono::microseconds(1000));
  }
  blLevel = lv;
}

// ---- RGB-LED auf dem Shield (I2C 0x50 an Wire1) --------------------------------
static bool ledOk = false;
static void ledWrite(uint8_t reg, uint8_t v) { Wire1.beginTransmission(0x50); Wire1.write(reg); Wire1.write(v); Wire1.endTransmission(); }
static void ledSet(uint8_t r, uint8_t g, uint8_t b) {
  static uint32_t last = 0xFFFFFFFF;
  uint32_t k = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  if (!ledOk || k == last) return;
  last = k;
  ledWrite(0x10, r); ledWrite(0x11, g); ledWrite(0x12, b); ledWrite(0x2b, 0xc5);
}
static void ledTick(const LiveData& d) {
#if UI_RGB_LED
  if (!d.wpStateValid) ledSet(30, 0, 0);
  else if (!d.wpOnOffRaw) ledSet(0, 0, 0);
  else if (d.extValid && (d.state & 16)) ledSet(0, 8, 30);
  else if (wwState == WW_HEAT || wwState == WW_MOVE) ledSet(30, 12, 0);
  else if (d.hzValid && d.verdichterHz > 0) ledSet(0, 25, 4);
  else ledSet(3, 3, 3);
#else
  (void)d;
#endif
}

// ---- Einstellungen der Anzeige ------------------------------------------------------
static void uiLoadPrefs() {
  Preferences p; p.begin("ui", true);
  uiBright = p.getUChar("bright", 100); uiDimMin = p.getUChar("dim", 5); uint8_t th = p.getUChar("theme", 0);
  p.end();
  uiBright = constrain(uiBright, 10, 100); if (uiDimMin > 60) uiDimMin = 5;
  uiSetTheme(th > 1 ? 0 : th);
}
static void uiSavePrefs() {
  Preferences p; p.begin("ui", false);
  p.putUChar("bright", uiBright); p.putUChar("dim", uiDimMin); p.putUChar("theme", uiTheme);
  p.end();
}

// Befehl an den Modbus-Task
static void uiSubmit(CmdType t, float v) {
  if (!submitCmd(t, v, false)) { Log.println("Display: Befehls-Warteschlange voll -> Befehl verworfen"); setCmdMsg("Befehl verworfen, nochmal"); }
}

static bool uiRepeatable(int id) {
  return id == U_SOLL_M || id == U_SOLL_P || id == U_HZ_M || id == U_HZ_P || id == U_FAN_M || id == U_FAN_P;
}

static void uiTap(int id, int tx, int ty) {
  (void)ty;
  if (id == U_NONE) return;
  if (id >= U_NAV && id < U_NAV + PG_COUNT) { uiPage = id - U_NAV; uEditInit = false; chartCursorX = -1; return; }
  if (id >= U_RANGE && id < U_RANGE + 5) { chartRange = id - U_RANGE; chartCursorX = -1; return; }
  if (id >= U_SER && id < U_SER + 7) { chartSer ^= (uint8_t)(1 << (id - U_SER)); if (!chartSer) chartSer = 1; return; }
  switch (id) {
    case U_CHART: chartCursorX = (abs(tx - chartCursorX) < 6) ? -1 : tx; break;
    case U_ON:  powerCmd(true, "Display");  setCmdMsg("Sende WP EIN..."); break;
    case U_OFF: powerCmd(false, "Display"); setCmdMsg("Sende WP AUS..."); break;
    case U_WWNOW:
    case U_WWSTOP:
      if (!getCtrlCfg().enabled) { setCmdMsg("Ventilsteuerung im Web aktivieren"); break; }
      if (wwState == WW_MOVE || wwState == WW_HEAT) { reqWwStop = true; setCmdMsg("WW-Ladung wird beendet"); }
      else if (id == U_WWNOW) { reqWwManual = true; reqWwStart = true; setCmdMsg("Warmwasser jetzt: startet..."); }
      break;
    case U_KAMIN:
      kaminSet(!kaminOn, "Display");
      if (kaminOn && !hcCfg.autoEnable) setCmdMsg("Kamin AN (wirkt nur mit Auto-WP)");
      else setCmdMsg(kaminOn ? "Kamin AN: Auto-Grenzen -%u K" : "Kamin AUS", hcCfg.kaminTa);
      break;
    case U_AUTO:
      hcCfg.autoEnable = !hcCfg.autoEnable;
      if (hcCfg.autoMode < 1 || hcCfg.autoMode > 3) hcCfg.autoMode = 1;
      saveHcCfg(); setCmdMsg("Auto-WP %s", hcCfg.autoEnable ? "EIN" : "AUS");
      Log.printf("Display: Auto-WP %s\n", hcCfg.autoEnable ? "EIN" : "AUS");
      break;
    case U_HC:
      hcCfg.enabled = !hcCfg.enabled; saveHcCfg(); setCmdMsg("Heizkurve %s", hcCfg.enabled ? "EIN" : "AUS");
      Log.printf("Display: Heizkurve %s\n", hcCfg.enabled ? "EIN" : "AUS");
      break;
    case U_ROOM:
      hcCfg.roomOn = !hcCfg.roomOn; saveHcCfg(); setCmdMsg("Raumeinfluss %s", hcCfg.roomOn ? "EIN" : "AUS");
      Log.printf("Display: Raumeinfluss %s\n", hcCfg.roomOn ? "EIN" : "AUS");
      break;
    case U_FC:
      fcCfg.enabled = !fcCfg.enabled; saveFcCfg(); setCmdMsg("Frequenz %s", fcCfg.enabled ? "Automatik" : "manuell");
      Log.printf("Display: Frequenzkurve %s\n", fcCfg.enabled ? "EIN" : "AUS");
      break;
    case U_RT_M: hcAdjustSoll(-0.5f); break;
    case U_RT_P: hcAdjustSoll(+0.5f); break;
    case U_V_HEIZ: if (getCtrlCfg().enabled) { reqValve = 0; setCmdMsg("Ventil: Heizung"); } break;
    case U_V_PUF:  if (getCtrlCfg().enabled) { reqValve = getCtrlCfg().bufPos; setCmdMsg("Ventil: Puffer"); } break;
    case U_V_WW:   if (getCtrlCfg().enabled) { reqValve = 100; setCmdMsg("Ventil: Warmwasser"); } break;
    case U_V_AUTO: if (getCtrlCfg().enabled) { reqValve = -1; setCmdMsg("Ventil: Automatik"); } break;
    case U_SUB0: compSub = 0; break;
    case U_SUB1: compSub = 1; uEditInit = false; break;
    case U_SOLL_M: uEditSoll = max(SOLL_MIN, uEditSoll - 1); break;
    case U_SOLL_P: uEditSoll = min(SOLL_MAX, uEditSoll + 1); break;
    case U_HZ_M:   uEditHz = max((int)MAX_HZ_MIN, uEditHz - 1); break;
    case U_HZ_P:   uEditHz = min((int)MAX_HZ_MAX, uEditHz + 1); break;
    case U_FAN_M:  uEditFan = max((int)FAN_RPM_MIN, uEditFan - 10); break;
    case U_FAN_P:  uEditFan = min((int)FAN_RPM_MAX, uEditFan + 10); break;
    case U_SOLL_SET: uiSubmit(CMD_SET_SOLL, uEditSoll); setCmdMsg("Sende Sollwert %d °C...", uEditSoll); break;
    case U_HZ_SET:   uiSubmit(CMD_MAXHZ, uEditHz);     setCmdMsg("Sende Max Hz %d...", uEditHz); break;
    case U_FAN_SET:  uiSubmit(CMD_MAXFAN, uEditFan);   setCmdMsg("Sende Max Lüfter %d...", uEditFan); break;
    case U_MODE_H: uiSubmit(CMD_MODE, 2); setCmdMsg("Sende Betriebsart Heizen..."); break;
    case U_MODE_W: uiSubmit(CMD_MODE, 1); setCmdMsg("Sende Betriebsart Warmwasser..."); break;
    case U_MODE_K: uiSubmit(CMD_MODE, 4); setCmdMsg("Sende Betriebsart Kühlen..."); break;
    case U_BR_M: uiBright = max(10, uiBright - 10); uiBacklight(uiBright); uiSavePrefs(); break;
    case U_BR_P: uiBright = min(100, uiBright + 10); uiBacklight(uiBright); uiSavePrefs(); break;
    case U_DIM_M: uiDimMin = uiDimMin <= 1 ? 0 : uiDimMin <= 5 ? uiDimMin - 1 : uiDimMin - 5; uiSavePrefs(); break;
    case U_DIM_P: uiDimMin = uiDimMin < 5 ? uiDimMin + 1 : min(60, uiDimMin + 5); uiSavePrefs(); break;
    case U_TH_DARK:  uiSetTheme(0); uiSavePrefs(); break;
    case U_TH_LIGHT: uiSetTheme(1); uiSavePrefs(); break;
    case U_TOUCHDBG: touchDebug = !touchDebug; break;
    case U_RESTART:
      if (restartArmMs && millis() - restartArmMs < 4000) { Log.println("Display: Neustart"); restartRequested = true; }
      else restartArmMs = millis();
      break;
  }
}

// ---- Start + Task -----------------------------------------------------------------
static void bootScreen(const char* line1, const char* line2) {
  if (!fb) return;
  gfx.startBuffering();
  gfx.fillScreen(C_BG);
  txt("Avarma R290", UI_W / 2, 170, F32B, C_TEXT, AL_C, VA_MID);
  txt("v" FW_VERSION "  ·  " AV_PLATFORM, UI_W / 2, 214, F17, C_DIM, AL_C, VA_MID);
  if (line1) txt(line1, UI_W / 2, 280, F22B, C_TEXT, AL_C, VA_MID);
  if (line2) txt(line2, UI_W / 2, 314, F17, C_DIM, AL_C, VA_MID);
  gfx.endBuffering();
}

void uiBegin() {
  uiSetTheme(0);
  gfx.begin();                                  // startet auch das SDRAM (Heap fuer den Verlauf)
  gfx.setRotation(UI_ROTATION);
  fb = gfx.getBuffer();
  gigaSdramReady = fb != nullptr;
  touchChipFound = touchPanel.begin();
  Log.printf("Display 800x480: %s, Touch GT911: %s\n", fb ? "OK" : "FEHLER", touchChipFound ? "gefunden" : "NICHT gefunden");
  Log.printf("Einstellungen gespeichert in: %s\n", gigaPrefsBackend());
#if UI_RGB_LED
  if (touchChipFound) { ledOk = true; ledWrite(0x1, 0xF1); ledWrite(0x2, 0xFF); ledSet(0, 0, 0); }
#endif
  uiLoadPrefs();
  uiBacklight(uiBright);
}

static bool uiTouch(int& x, int& y) {
  if (!touchChipFound) return false;
  GDTpoint_t pts[GT911_MAX_CONTACTS];
  uint8_t n = touchPanel.getTouchPoints(pts);
  if (n == 0) return false;
  int rx = pts[0].x, ry = pts[0].y;               // Rohwerte: x 0..479, y 0..799 (Panel hochkant)
#if UI_ROTATION == 3
  x = 799 - ry; y = rx;
#else
  x = ry; y = 479 - rx;
#endif
#if UI_TOUCH_FLIP & 1
  x = UI_W - 1 - x;
#endif
#if UI_TOUCH_FLIP & 2
  y = UI_H - 1 - y;
#endif
  x = constrain(x, 0, UI_W - 1); y = constrain(y, 0, UI_H - 1);
  return true;
}

void displayTask(void*) {
  bool touchDown = false, wakeTouch = false, dimmed = false;
  uint32_t releaseStart = 0, pressMs = 0, lastRepeat = 0, lastInteraction = millis(), lastDraw = 0;
  int pressedId = U_NONE;
  for (;;) {
    uint32_t ms = millis();
    int tx, ty;
    bool touched = uiTouch(tx, ty);
    if (touched) {
      releaseStart = 0;
      lastInteraction = ms;
      if (touchDebug && (tx != lastTouchX || ty != lastTouchY)) {
        Log.printf("Touch x=%d y=%d -> Taste %d\n", tx, ty, uiHit(tx, ty));
        needRedraw = true;
      }
      lastTouchX = tx; lastTouchY = ty; lastTouchMs = ms;
      if (!touchDown) {
        touchDown = true;
        if (dimmed) { uiBacklight(uiBright); dimmed = false; wakeTouch = true; }      // erster Tipp weckt nur auf
        else { pressedId = uiHit(tx, ty); pressMs = lastRepeat = ms; uiTap(pressedId, tx, ty); }
        needRedraw = true;
      } else if (!wakeTouch && uiRepeatable(pressedId) && ms - pressMs > 450 && ms - lastRepeat > 90) {
        lastRepeat = ms; uiTap(pressedId, tx, ty); needRedraw = true;
      }
    } else if (touchDown) {
      if (!releaseStart) releaseStart = ms;
      if (ms - releaseStart >= 60) { touchDown = false; wakeTouch = false; pressedId = U_NONE; releaseStart = 0; }
    }
    if (!dimmed && uiDimMin && ms - lastInteraction > uiDimMin * 60000UL) { uiBacklight(min((int)uiBright, 20)); dimmed = true; }
    if (needRedraw || ms - lastDraw >= 1000) {
      needRedraw = false; lastDraw = ms;
      uiRender();
      ledTick(getLive());
    }
    delay(20);
  }
}
