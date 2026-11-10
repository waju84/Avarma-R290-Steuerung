/*
  giga_compat.h  –  Plattform-Schicht fuer das Arduino GIGA R1 WiFi (Mbed-OS-Kern)
  =====================================================================
  Der Hauptsketch ist fuer ESP32-S3 und GIGA derselbe. Diese Datei stellt auf dem GIGA
  die ESP32-Funktionen bereit, die der Sketch benutzt:

    - FreeRTOS: Tasks, Mutex, Semaphore, Queue          -> rtos::Thread / Mutex / Semaphore
    - WebServer (ESP-API)                                -> eigener kleiner HTTP-Server
    - Preferences (ESP-API)                              -> TDBStore auf QSPI-Partition 3
                                                            (Ersatz: Dateien auf dem USB-Stick)
    - SD_MMC + File                                      -> interner QSPI-Speicher (LittleFS), sonst USB-Stick
    - esp-mqtt (esp_mqtt_client_*)                      -> PubSubClient in eigenem Thread
    - HTTPClient (nur GET, fuer den Shelly)              -> WiFiClient
    - configTzTime / NTP / localtime_r / mktime          -> eigener NTP-Thread, MEZ/MESZ fest
    - ESP.restart(), ps_malloc() ...                     -> NVIC_SystemReset(), SDRAM

  Benoetigte Bibliotheken (Bibliotheksverwalter):
    - Arduino_GigaDisplay_GFX, Arduino_GigaDisplayTouch, Adafruit GFX Library
    - Arduino_USBHostMbed5
    - PubSubClient (Nick O'Leary)
    - ModbusMaster (Doc Walker)
*/
#pragma once

#include <Arduino.h>
#include <mbed.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <SDRAM.h>
#include <Arduino_USBHostMbed5.h>
#include <FATFileSystem.h>
#include <LittleFileSystem.h>
#include <BlockDevice.h>
#include <MBRBlockDevice.h>
#include <TDBStore.h>
#include <PubSubClient.h>
#include <functional>
#include <initializer_list>
#include <chrono>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <errno.h>

// =====================================================================
// Typen fuer die Display-Oberflaeche (giga_ui.h). Stehen hier oben, weil die
// Arduino-IDE automatische Funktions-Prototypen vor die erste Funktion setzt.
// =====================================================================
struct AvGlyph { uint32_t off; uint8_t w, h, adv; int8_t xo, yo; };
struct AvFont  { const uint8_t* bmp; const AvGlyph* g; uint8_t first, last; uint8_t ascent, lineH, cap; };

// =====================================================================
// FreeRTOS-Ersatz
// =====================================================================
typedef int      BaseType_t;
typedef uint32_t TickType_t;
#define pdTRUE   1
#define pdFALSE  0
#define pdPASS   1
#define portMAX_DELAY      0xFFFFFFFFUL
#define pdMS_TO_TICKS(x)   ((TickType_t)(x))

inline void vTaskDelay(TickType_t ms) { delay(ms); }

// kurze kritische Abschnitte (Log-Ringpuffer, DHT-Bitlesen): nur den Taskwechsel sperren,
// Interrupts laufen weiter (sonst gingen beim DHT-Lesen Modbus-Bytes verloren - der GIGA hat nur einen Kern)
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
static int32_t avLockPrev = 0;
inline void avCritEnter() { avLockPrev = osKernelLock(); }
inline void avCritExit()  { osKernelRestoreLock(avLockPrev); }
#define portENTER_CRITICAL(m) avCritEnter()
#define portEXIT_CRITICAL(m)  avCritExit()

struct AvSem {
  bool isMutex;
  rtos::Mutex* mtx;
  rtos::Semaphore* sem;
};
typedef AvSem* SemaphoreHandle_t;

inline SemaphoreHandle_t xSemaphoreCreateMutex() {
  AvSem* s = new AvSem; s->isMutex = true; s->mtx = new rtos::Mutex(); s->sem = nullptr; return s;
}
inline SemaphoreHandle_t xSemaphoreCreateBinary() {
  AvSem* s = new AvSem; s->isMutex = false; s->mtx = nullptr; s->sem = new rtos::Semaphore(0, 1); return s;
}
inline bool avSemAcquire(rtos::Semaphore* s, TickType_t t) {
  if (t == portMAX_DELAY) { s->acquire(); return true; }
  if (t == 0) return s->try_acquire();
  return s->try_acquire_for(std::chrono::milliseconds(t));
}
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t t) {
  if (!s) return pdFALSE;
  if (s->isMutex) {
    if (t == portMAX_DELAY) { s->mtx->lock(); return pdTRUE; }
    return s->mtx->trylock_for(std::chrono::milliseconds(t)) ? pdTRUE : pdFALSE;
  }
  return avSemAcquire(s->sem, t) ? pdTRUE : pdFALSE;
}
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t s) {
  if (!s) return pdFALSE;
  if (s->isMutex) s->mtx->unlock(); else s->sem->release();
  return pdTRUE;
}

struct AvQueue {
  uint8_t* buf; size_t item, cap, head, count;
  rtos::Mutex m;
  rtos::Semaphore* items;
  rtos::Semaphore* spaces;
};
typedef AvQueue* QueueHandle_t;

inline QueueHandle_t xQueueCreate(size_t n, size_t itemSize) {
  AvQueue* q = new AvQueue;
  q->buf = (uint8_t*)malloc(n * itemSize); q->item = itemSize; q->cap = n; q->head = 0; q->count = 0;
  q->items = new rtos::Semaphore(0, (uint16_t)n);
  q->spaces = new rtos::Semaphore((int32_t)n, (uint16_t)n);
  return q;
}
inline BaseType_t xQueueSend(QueueHandle_t q, const void* it, TickType_t t) {
  if (!q || !avSemAcquire(q->spaces, t)) return pdFALSE;
  q->m.lock();
  memcpy(q->buf + ((q->head + q->count) % q->cap) * q->item, it, q->item);
  q->count++;
  q->m.unlock();
  q->items->release();
  return pdTRUE;
}
inline BaseType_t xQueueReceive(QueueHandle_t q, void* it, TickType_t t) {
  if (!q || !avSemAcquire(q->items, t)) return pdFALSE;
  q->m.lock();
  memcpy(it, q->buf + q->head * q->item, q->item);
  q->head = (q->head + 1) % q->cap; q->count--;
  q->m.unlock();
  q->spaces->release();
  return pdTRUE;
}

typedef void (*TaskFunction_t)(void*);
typedef void* TaskHandle_t;
struct AvTaskStart {
  TaskFunction_t fn; void* arg;
  void run() { fn(arg); }
};
// Kern-Nummer wird ignoriert (der M7 macht alles), Prioritaet 2 = etwas hoeher (Modbus)
inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char* name, uint32_t stack, void* arg,
                                          int prio, TaskHandle_t* handle, int core) {
  (void)core;
  AvTaskStart* ts = new AvTaskStart; ts->fn = fn; ts->arg = arg;
  rtos::Thread* th = new rtos::Thread(prio >= 2 ? osPriorityAboveNormal : osPriorityNormal, stack + 2048, nullptr, name);
  th->start(mbed::callback(ts, &AvTaskStart::run));
  if (handle) *handle = th;
  return pdPASS;
}

// =====================================================================
// ESP-Systemfunktionen
// =====================================================================
static bool gigaSdramReady = false;          // wird nach dem Start des Displays gesetzt (SDRAM-Heap)

inline bool psramFound() { return gigaSdramReady; }
inline void* ps_malloc(size_t n) { return gigaSdramReady ? SDRAM.malloc(n) : nullptr; }

static void (*avBeforeRestart)() = nullptr;      // z. B. Einstellungs-Cache noch auf den Stick schreiben

// Watchdog: haengt loop() (z. B. weil der USB-Stick nicht mehr antwortet), startet der GIGA neu.
static bool avWdOn = false;
#if defined(DEVICE_WATCHDOG) && DEVICE_WATCHDOG
inline void avWdKick() { if (avWdOn) mbed::Watchdog::get_instance().kick(); }
inline bool avWdStart(uint32_t ms) {
  mbed::Watchdog& w = mbed::Watchdog::get_instance();
  uint32_t mx = w.get_max_timeout();
  if (mx && ms > mx) ms = mx;
  avWdOn = w.start(ms);
  return avWdOn;
}
#else
inline void avWdKick() {}
inline bool avWdStart(uint32_t) { return false; }
#endif
// letzter Neustart durch den Watchdog? (liest und loescht die Reset-Flags)
inline bool avResetByWatchdog() {
#if defined(RCC_RSR_IWDG1RSTF) && defined(RCC_RSR_RMVF)
  bool wd = (RCC->RSR & RCC_RSR_IWDG1RSTF) != 0;
  RCC->RSR |= RCC_RSR_RMVF;
  return wd;
#else
  return false;
#endif
}

struct AvEspClass {
  void restart() { if (avBeforeRestart) avBeforeRestart(); delay(100); NVIC_SystemReset(); }
  uint32_t getFreeHeap() {
    mbed_stats_heap_t st; mbed_stats_heap_get(&st);
    return st.reserved_size > st.current_size ? (uint32_t)(st.reserved_size - st.current_size) : 0;
  }
  uint32_t getFreePsram() { return 0; }
  uint64_t getEfuseMac() {
    uint8_t m[6] = {0, 0, 0, 0, 0, 0};
    WiFi.macAddress(m);
    uint64_t v = 0;
    for (int i = 0; i < 6; i++) v |= (uint64_t)m[5 - i] << (8 * i);   // untere 24 Bit = geraetespezifisch
    return v;
  }
};
static AvEspClass ESP;

// mDNS und Netzwerk-Upload gibt es auf dem GIGA nicht -> leere Platzhalter
struct AvMdnsClass { bool begin(const char*) { return false; } void addService(const char*, const char*, int) {} };
static AvMdnsClass MDNS;

typedef enum { OTA_AUTH_ERROR, OTA_BEGIN_ERROR, OTA_CONNECT_ERROR, OTA_RECEIVE_ERROR, OTA_END_ERROR } ota_error_t;
struct AvOtaClass {
  void setHostname(const char*) {}
  void setPassword(const char*) {}
  void onStart(std::function<void()>) {}
  void onEnd(std::function<void()>) {}
  void onError(std::function<void(ota_error_t)>) {}
  void begin() {}
  void handle() {}
};
static AvOtaClass ArduinoOTA;

// =====================================================================
// Zeit: NTP + MEZ/MESZ (Europa/Berlin, fest eingebaut)
// =====================================================================
inline int32_t avDaysFromCivil(int32_t y, uint32_t m, uint32_t d) {     // 1970-01-01 = 0
  y -= m <= 2;
  const int32_t era = (y >= 0 ? y : y - 399) / 400;
  const uint32_t yoe = (uint32_t)(y - era * 400);
  const uint32_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int32_t)doe - 719468;
}
inline time_t avTimegm(const struct tm* t) {                            // wie timegm, normalisiert Felder
  int32_t y = t->tm_year + 1900, mo = t->tm_mon;
  y += mo / 12; mo %= 12; if (mo < 0) { mo += 12; y--; }
  int64_t days = avDaysFromCivil(y, mo + 1, 1) + (t->tm_mday - 1);
  return (time_t)(days * 86400LL + t->tm_hour * 3600LL + t->tm_min * 60LL + t->tm_sec);
}
inline time_t avLastSundayUtc(int year, int month) {                    // letzter Sonntag, 01:00 UTC
  int32_t dLast = avDaysFromCivil(year, month + 1, 1) - 1;               // letzter Tag des Monats
  int wd = (int)((dLast + 4) % 7); if (wd < 0) wd += 7;                  // 0 = Sonntag (1.1.1970 = Do)
  return (time_t)((int64_t)(dLast - wd) * 86400LL + 3600);
}
inline bool avIsDst(time_t utc) {
  struct tm g; gmtime_r(&utc, &g);
  int y = g.tm_year + 1900;
  return utc >= avLastSundayUtc(y, 3) && utc < avLastSundayUtc(y, 10);
}
inline struct tm* av_localtime_r(const time_t* t, struct tm* out) {
  bool dst = avIsDst(*t);
  time_t lt = *t + (dst ? 7200 : 3600);
  gmtime_r(&lt, out);
  out->tm_isdst = dst ? 1 : 0;
  return out;
}
inline time_t av_mktime(struct tm* t) {                                 // Ortszeit -> UTC (normalisiert t)
  time_t base = avTimegm(t);
  time_t u = base - 3600;
  if (avIsDst(u - 3600)) u = base - 7200;
  av_localtime_r(&u, t);
  return u;
}
#define localtime_r av_localtime_r
#define mktime      av_mktime

static const char* avNtpServer[2] = {"pool.ntp.org", "ptbtime1.ptb.de"};
static volatile uint32_t avNtpOkMs = 0;
static volatile bool avNtpSynced = false;

inline bool avNtpQuery(const char* server) {
  WiFiUDP udp;
  if (!udp.begin(2390)) return false;
  uint8_t pkt[48];
  memset(pkt, 0, sizeof(pkt));
  pkt[0] = 0x1B;                                         // LI 0, Version 3, Client
  bool ok = false;
  if (udp.beginPacket(server, 123) && udp.write(pkt, sizeof(pkt)) == sizeof(pkt) && udp.endPacket()) {
    uint32_t t0 = millis();
    while (millis() - t0 < 1500) {
      if (udp.parsePacket() >= 48) {
        udp.read(pkt, sizeof(pkt));
        uint32_t secs = ((uint32_t)pkt[40] << 24) | ((uint32_t)pkt[41] << 16) | ((uint32_t)pkt[42] << 8) | pkt[43];
        if (secs > 2208988800UL + 1700000000UL) {
          set_time((time_t)(secs - 2208988800UL));
          ok = true;
        }
        break;
      }
      delay(20);
    }
  }
  udp.stop();
  return ok;
}
inline void avNtpTask(void*) {
  for (;;) {
    bool due = !avNtpSynced || millis() - avNtpOkMs > 6UL * 3600UL * 1000UL;
    if (due && WiFi.status() == WL_CONNECTED) {
      for (int i = 0; i < 2; i++) {
        if (avNtpServer[i] && avNtpQuery(avNtpServer[i])) { avNtpSynced = true; avNtpOkMs = millis(); break; }
      }
    }
    delay(avNtpSynced ? 60000 : 5000);
  }
}
// TZ-Text wird ignoriert: MEZ/MESZ ist fest eingebaut (av_localtime_r)
inline void configTzTime(const char* tz, const char* s1, const char* s2) {
  (void)tz;
  static bool started = false;
  avNtpServer[0] = s1; avNtpServer[1] = s2;
  if (!started) { started = true; xTaskCreatePinnedToCore(avNtpTask, "ntp", 3072, nullptr, 1, nullptr, 0); }
}

// =====================================================================
// Dateien im internen Speicher (QSPI) oder auf dem USB-Stick (ersetzt SD_MMC / File)
// =====================================================================
#define FILE_READ   "r"
#define FILE_WRITE  "w"
#define FILE_APPEND "a"
enum { CARD_NONE = 0, CARD_MMC, CARD_SD, CARD_SDHC, CARD_UNKNOWN };
#define SDMMC_FREQ_DEFAULT 20000

struct AvFileImpl {
  int refs;
  FILE* fp;
  DIR* dp;
  bool lazy;                 // Datei aus einer Verzeichnisliste: erst beim Lesen oeffnen
  char path[64];
};
// groesserer Puffer = weniger, groessere USB-Zugriffe (Standard sind sehr kleine Bloecke)
inline FILE* avFopen(const char* f, const char* mode) {
  FILE* fp = fopen(f, mode);
  if (fp) setvbuf(fp, nullptr, _IOFBF, 4096);
  return fp;
}

class File : public Stream {
  AvFileImpl* p = nullptr;
  void release() {
    if (p && --p->refs == 0) {
      if (p->fp) fclose(p->fp);
      if (p->dp) closedir(p->dp);
      delete p;
    }
    p = nullptr;
  }
public:
  File() {}
  File(FILE* fp, DIR* dp, const char* path, bool lazy = false) {
    p = new AvFileImpl; p->refs = 1; p->fp = fp; p->dp = dp; p->lazy = lazy;
    strncpy(p->path, path, sizeof(p->path) - 1); p->path[sizeof(p->path) - 1] = 0;
  }
  File(const File& o) : Stream() { p = o.p; if (p) p->refs++; }
  File& operator=(const File& o) { if (o.p) o.p->refs++; release(); p = o.p; return *this; }
  ~File() { release(); }
  explicit operator bool() const { return p && (p->fp || p->dp || p->lazy); }
  bool ensure() {                             // Datei aus Verzeichnisliste jetzt oeffnen
    if (!p) return false;
    if (!p->fp && p->lazy) { p->lazy = false; p->fp = avFopen(p->path, "r"); }
    return p->fp != nullptr;
  }

  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t* b, size_t n) override { avWdKick(); return (p && p->fp) ? fwrite(b, 1, n, p->fp) : 0; }
  using Print::write;
  int available() override {
    if (!ensure()) return 0;
    long pos = ftell(p->fp), end = (long)size();
    return end > pos ? (int)min(end - pos, 0x7FFFL) : 0;
  }
  int read() override { if (!ensure()) return -1; int c = fgetc(p->fp); return c == EOF ? -1 : c; }
  int peek() override { if (!ensure()) return -1; int c = fgetc(p->fp); if (c == EOF) return -1; ungetc(c, p->fp); return c; }
  void flush() override { if (p && p->fp) fflush(p->fp); }
  int read(uint8_t* buf, size_t n) { avWdKick(); return ensure() ? (int)fread(buf, 1, n, p->fp) : -1; }
  bool seek(uint32_t pos) { return ensure() && fseek(p->fp, pos, SEEK_SET) == 0; }
  size_t position() { return (p && p->fp) ? (size_t)ftell(p->fp) : 0; }
  size_t size() {                             // ohne fseek (das wuerde den Schreibpuffer leeren)
    if (!p) return 0;
    struct stat st;
    if (p->fp) {
      fflush(p->fp);
      if (fstat(fileno(p->fp), &st) == 0 && S_ISREG(st.st_mode)) return (size_t)st.st_size;
      long pos = ftell(p->fp); fseek(p->fp, 0, SEEK_END); long end = ftell(p->fp); fseek(p->fp, pos, SEEK_SET);
      return end > 0 ? (size_t)end : 0;
    }
    if (p->lazy && stat(p->path, &st) == 0) return (size_t)st.st_size;
    return 0;
  }
  void close() { release(); }
  bool isDirectory() const { return p && p->dp; }
  const char* name() const {
    if (!p) return "";
    const char* s = strrchr(p->path, '/');
    return s ? s + 1 : p->path;
  }
  const char* path() const { return p ? p->path : ""; }
  File openNextFile();
};

static USBHostMSD avMsd;
static mbed::FATFileSystem avUsbFs("usb");
// interner Speicher: QSPI-Flash, Partition 4 (von QSPIFormat angelegt), LittleFS (verteilt die Schreibzugriffe)
static mbed::LittleFileSystem avDataFs("data");
static mbed::MBRBlockDevice* avDataPart = nullptr;
static bool avDataMissing = false;          // Partition 4 fehlt -> USB-Stick als Ersatz
static bool avDataFormatted = false;        // Partition war leer/unbekannt und wurde neu formatiert
static const char* avFsRoot = "/data";      // "/data" (intern) oder "/usb" (Stick)
static int avFsKind = 0;                    // 0 = keiner, 1 = intern, 2 = USB-Stick

class AvUsbFsClass {
  bool mounted = false;
  mbed::DigitalOut* vbus = nullptr;
  bool mountInternal() {
    if (avDataMissing) return false;
    if (!avDataPart) {
      mbed::BlockDevice* root = mbed::BlockDevice::get_default_instance();
      avDataPart = root ? new mbed::MBRBlockDevice(root, 4) : nullptr;
      if (!avDataPart || avDataPart->init() != 0) { delete avDataPart; avDataPart = nullptr; avDataMissing = true; return false; }
    }
    if (avDataFs.mount(avDataPart) != 0) {            // noch nicht LittleFS (leer oder FAT) -> einmal einrichten
      if (avDataFs.reformat(avDataPart) != 0) return false;
      avDataFormatted = true;
    }
    return true;
  }
  bool mountUsb() {
    if (!vbus) vbus = new mbed::DigitalOut(PB_8, 1);       // 5 V am USB-A-Port einschalten
    if (!avMsd.connected()) {
      bool ok = false;
      for (int i = 0; i < 6 && !ok; i++) { ok = avMsd.connect(); if (!ok) delay(250); }
      if (!ok) return false;
    }
    return avUsbFs.mount(&avMsd) == 0;
  }
public:
  static void full(char* out, size_t n, const char* path) { snprintf(out, n, "%s%s%s", avFsRoot, path[0] == '/' ? "" : "/", path); }
  // Parameter wie SD_MMC.begin() beim ESP, werden ignoriert
  bool begin(const char* mp = "/sdcard", bool mode1bit = false, bool fmt = false, int freq = 0, int maxFiles = 5) {
    (void)mp; (void)mode1bit; (void)fmt; (void)freq; (void)maxFiles;
    if (mounted) return true;
    if (avFsKind != 2 && mountInternal()) { avFsKind = 1; avFsRoot = "/data"; mounted = true; return true; }
    if (avFsKind == 1) return false;                         // intern lief schon -> nicht auf den Stick wechseln
    if (mountUsb()) { avFsKind = 2; avFsRoot = "/usb"; mounted = true; return true; }
    return false;
  }
  void end() {
    if (mounted) { if (avFsKind == 1) avDataFs.unmount(); else avUsbFs.unmount(); }
    mounted = false;
  }
  const char* kindName() const { return avFsKind == 1 ? "interner Speicher (QSPI)" : avFsKind == 2 ? "USB-Stick" : "kein Speicher"; }
  int kind() const { return avFsKind; }
  bool isMounted() const { return mounted; }
  void setPins(int, int, int, int, int, int) {}
  uint8_t cardType() { return mounted ? CARD_SDHC : CARD_NONE; }
  bool exists(const char* path) {
    if (!mounted) return false;
    char f[80]; full(f, sizeof(f), path);
    struct stat st;
    return stat(f, &st) == 0;
  }
  bool exists(const String& path) { return exists(path.c_str()); }
  File open(const char* path, const char* mode = FILE_READ) {
    if (!mounted) return File();
    char f[80]; full(f, sizeof(f), path);
    struct stat st;
    if (strcmp(mode, "r") == 0 && stat(f, &st) == 0 && S_ISDIR(st.st_mode)) {
      DIR* d = opendir(f);
      return d ? File(nullptr, d, f) : File();
    }
    FILE* fp = avFopen(f, strcmp(mode, "a") == 0 ? "a+" : mode);
    return fp ? File(fp, nullptr, f) : File();
  }
  File open(const String& path, const char* mode = FILE_READ) { return open(path.c_str(), mode); }
  bool mkdir(const char* path) { char f[80]; full(f, sizeof(f), path); return ::mkdir(f, 0777) == 0; }
  bool remove(const char* path) { char f[80]; full(f, sizeof(f), path); return ::remove(f) == 0; }
  bool rename(const char* a, const char* b) {
    char fa[80], fb[80]; full(fa, sizeof(fa), a); full(fb, sizeof(fb), b);
    return ::rename(fa, fb) == 0;
  }
  bool vfs(struct statvfs& st) {
    if (!mounted) return false;
    return (avFsKind == 1 ? avDataFs.statvfs("/", &st) : avUsbFs.statvfs("/", &st)) == 0;
  }
  uint64_t totalBytes() {
    struct statvfs st;
    if (!vfs(st)) return 0;
    return (uint64_t)st.f_blocks * (st.f_frsize ? st.f_frsize : st.f_bsize);
  }
  uint64_t usedBytes() {
    struct statvfs st;
    if (!vfs(st)) return 0;
    uint64_t bs = st.f_frsize ? st.f_frsize : st.f_bsize;
    return (uint64_t)(st.f_blocks - st.f_bfree) * bs;
  }
};
static AvUsbFsClass SD_MMC;

inline File File::openNextFile() {
  if (!p || !p->dp) return File();
  for (;;) {
    struct dirent* e = readdir(p->dp);
    if (!e) return File();
    if (e->d_name[0] == '.') continue;
    char f[96];
    snprintf(f, sizeof(f), "%s/%s", p->path, e->d_name);
    if (e->d_type == DT_DIR) { DIR* d = opendir(f); if (d) return File(nullptr, d, f); continue; }
    return File(nullptr, nullptr, f, true);   // erst oeffnen, wenn gelesen wird (Liste ist so viel schneller)
  }
}

// =====================================================================
// Preferences (ESP-API) -> TDBStore auf QSPI-Partition 3, sonst Dateien auf dem USB-Stick
// =====================================================================
class AvKv {
public:
  mbed::TDBStore* store = nullptr;
  bool tried = false, usbFallback = false;
  rtos::Mutex m;
  void init() {
    if (tried) return;
    tried = true;
    mbed::BlockDevice* root = mbed::BlockDevice::get_default_instance();
    mbed::MBRBlockDevice* part = root ? new mbed::MBRBlockDevice(root, 3) : nullptr;
    if (part && part->init() == 0) {
      store = new mbed::TDBStore(part);
      if (store->init() != 0) { delete store; store = nullptr; }
    }
    if (!store) usbFallback = true;
  }
  static void keyName(char* out, size_t n, const char* ns, const char* key) { snprintf(out, n, "%s_%s", ns, key); }
  static void fileName(char* out, size_t n, const char* ns, const char* key) { snprintf(out, n, "%s/prefs/%s_%s.bin", avFsRoot, ns, key); }
  uint32_t lastUsbTry = 0;
  bool usbReady() {
    if (!SD_MMC.isMounted()) {
      if (lastUsbTry && millis() - lastUsbTry < 30000UL) return false;   // nicht bei jedem Zugriff 1,5 s warten
      lastUsbTry = millis();
      SD_MMC.begin();
    }
    if (!SD_MMC.isMounted()) return false;
    struct stat st; char d[24]; snprintf(d, sizeof(d), "%s/prefs", avFsRoot);
    if (stat(d, &st) != 0) ::mkdir(d, 0777);
    return true;
  }
  // Liegen die Einstellungen auf dem USB-Stick, werden sie im RAM gehalten und nur aus loop()
  // (flushUsb) auf den Stick geschrieben: so greifen Regel-, Modbus- und Display-Task nie auf den
  // Stick zu (deren kleine Stacks und gleichzeitige USB-Zugriffe fuehrten zu Haengern).
  struct Ent { char key[44]; uint16_t len; uint8_t* data; bool dirty; bool gone; };
  Ent cache[48]; int nCache = 0;
  uint32_t lastFlush = 0;
  Ent* find(const char* k) { for (int i = 0; i < nCache; i++) if (!strcmp(cache[i].key, k)) return &cache[i]; return nullptr; }
  Ent* put(const char* k, const void* buf, size_t n, bool dirty) {
    Ent* e = find(k);
    if (!e) {
      if (nCache >= (int)(sizeof(cache) / sizeof(cache[0]))) return nullptr;
      e = &cache[nCache++]; strncpy(e->key, k, sizeof(e->key) - 1); e->key[sizeof(e->key) - 1] = 0; e->data = nullptr; e->len = 0;
    }
    if (e->len != n || !e->data) { free(e->data); e->data = (uint8_t*)malloc(n ? n : 1); }
    if (!e->data) { e->len = 0; return nullptr; }
    memcpy(e->data, buf, n); e->len = (uint16_t)n; e->dirty = dirty; e->gone = false;
    return e;
  }
  // nur aus loop() aufrufen
  void flushUsb(bool force = false) {
    if (!usbFallback || (!force && millis() - lastFlush < 2000)) return;
    lastFlush = millis();
    for (int i = 0; i < nCache; i++) {
      m.lock();
      Ent& e = cache[i];
      if (!e.dirty) { m.unlock(); continue; }
      size_t n = e.len; bool gone = e.gone;
      uint8_t tmp[300];
      if (n > sizeof(tmp)) n = sizeof(tmp);
      if (!gone) memcpy(tmp, e.data, n);
      char k[44]; strncpy(k, e.key, sizeof(k));
      e.dirty = false;
      m.unlock();
      if (!usbReady()) { m.lock(); e.dirty = true; m.unlock(); return; }
      char f[72]; snprintf(f, sizeof(f), "%s/prefs/%s.bin", avFsRoot, k);
      if (gone) { ::remove(f); continue; }
      FILE* fp = fopen(f, "w");
      bool ok = fp && fwrite(tmp, 1, n, fp) == n;
      if (fp) fclose(fp);
      if (!ok) { m.lock(); e.dirty = true; m.unlock(); }
    }
  }
  size_t get(const char* ns, const char* key, void* buf, size_t n) {
    init();
    m.lock();
    size_t got = 0;
    if (usbFallback) {
      char k[48]; keyName(k, sizeof(k), ns, key);
      Ent* e = find(k);
      if (!e) {                                  // einmal vom Stick lesen (meist beim Start)
        uint8_t tmp[300]; size_t r = 0;
        if (usbReady()) {
          char f[64]; fileName(f, sizeof(f), ns, key); FILE* fp = fopen(f, "r");
          if (fp) { r = fread(tmp, 1, sizeof(tmp), fp); fclose(fp); }
          e = put(k, tmp, r, false);
          if (e && !r) e->gone = true;           // "gibt es nicht" auch merken (nur wenn der Stick da war)
        }
      }
      if (e && !e->gone) { got = min(n, (size_t)e->len); memcpy(buf, e->data, got); }
    } else if (store) {
      char k[48]; keyName(k, sizeof(k), ns, key);
      size_t act = 0;
      if (store->get(k, buf, n, &act) == 0) got = act;
    } else if (usbReady()) {
      char f[64]; fileName(f, sizeof(f), ns, key);
      FILE* fp = fopen(f, "r");
      if (fp) { got = fread(buf, 1, n, fp); fclose(fp); }
    }
    m.unlock();
    return got;
  }
  size_t length(const char* ns, const char* key) {
    init();
    if (usbFallback) {                           // ueber den Cache (liest die Datei bei Bedarf)
      uint8_t tmp[300];
      return get(ns, key, tmp, sizeof(tmp));
    }
    m.lock();
    size_t len = 0;
    if (store) {
      char k[48]; keyName(k, sizeof(k), ns, key);
      mbed::KVStore::info_t inf;
      if (store->get_info(k, &inf) == 0) len = inf.size;
    } else if (usbReady()) {
      char f[64]; fileName(f, sizeof(f), ns, key);
      struct stat st;
      if (stat(f, &st) == 0) len = st.st_size;
    }
    m.unlock();
    return len;
  }
  size_t set(const char* ns, const char* key, const void* buf, size_t n) {
    init();
    m.lock();
    size_t done = 0;
    if (usbFallback) {
      char k[48]; keyName(k, sizeof(k), ns, key);
      if (n <= 300 && put(k, buf, n, true)) done = n;   // wird von loop() auf den Stick geschrieben
    } else if (store) {
      char k[48]; keyName(k, sizeof(k), ns, key);
      if (store->set(k, buf, n, 0) == 0) done = n;
    } else if (usbReady()) {
      char f[64]; fileName(f, sizeof(f), ns, key);
      FILE* fp = fopen(f, "w");
      if (fp) { done = fwrite(buf, 1, n, fp); fclose(fp); }
    }
    m.unlock();
    return done;
  }
  bool remove(const char* ns, const char* key) {
    init();
    m.lock();
    bool ok = false;
    if (usbFallback) {
      char k[48]; keyName(k, sizeof(k), ns, key);
      Ent* e = find(k);
      if (!e) { uint8_t z = 0; e = put(k, &z, 1, true); }
      if (e) { e->gone = true; e->dirty = true; ok = true; }
    } else if (store) { char k[48]; keyName(k, sizeof(k), ns, key); ok = store->remove(k) == 0; }
    else if (usbReady()) { char f[64]; fileName(f, sizeof(f), ns, key); ok = ::remove(f) == 0; }
    m.unlock();
    return ok;
  }
};
static AvKv avKv;
inline void avKvFlush(bool force = false) { avKv.flushUsb(force); }
inline const char* gigaPrefsBackend() {
  avKv.init();
  return avKv.store ? "QSPI-Flash (Partition 3)" : "Datenspeicher, Ordner /prefs (QSPI-Partition 3 fehlt)";
}

class Preferences {
  char ns[16] = "";
  bool ro = false;
  template <typename T> T getT(const char* key, T def) {
    T v; return avKv.get(ns, key, &v, sizeof(v)) == sizeof(v) ? v : def;
  }
  template <typename T> size_t putT(const char* key, T v) {
    if (ro) return 0;
    T old;
    if (avKv.get(ns, key, &old, sizeof(old)) == sizeof(old) && memcmp(&old, &v, sizeof(v)) == 0) return sizeof(v);   // unveraendert
    return avKv.set(ns, key, &v, sizeof(v));
  }
public:
  bool begin(const char* name, bool readOnly = false) {
    strncpy(ns, name, sizeof(ns) - 1); ns[sizeof(ns) - 1] = 0; ro = readOnly; return true;
  }
  void end() {}
  bool remove(const char* key) { return !ro && avKv.remove(ns, key); }
  size_t getBytesLength(const char* key) { return avKv.length(ns, key); }
  size_t getBytes(const char* key, void* buf, size_t maxLen) { return avKv.get(ns, key, buf, maxLen); }
  size_t putBytes(const char* key, const void* v, size_t len) { return ro ? 0 : avKv.set(ns, key, v, len); }
  int8_t   getChar(const char* k, int8_t d = 0)       { return getT<int8_t>(k, d); }
  uint8_t  getUChar(const char* k, uint8_t d = 0)     { return getT<uint8_t>(k, d); }
  int16_t  getShort(const char* k, int16_t d = 0)     { return getT<int16_t>(k, d); }
  uint16_t getUShort(const char* k, uint16_t d = 0)   { return getT<uint16_t>(k, d); }
  int32_t  getInt(const char* k, int32_t d = 0)       { return getT<int32_t>(k, d); }
  uint32_t getUInt(const char* k, uint32_t d = 0)     { return getT<uint32_t>(k, d); }
  int32_t  getLong(const char* k, int32_t d = 0)      { return getT<int32_t>(k, d); }
  uint32_t getULong(const char* k, uint32_t d = 0)    { return getT<uint32_t>(k, d); }
  bool     getBool(const char* k, bool d = false)     { return getT<uint8_t>(k, d ? 1 : 0) != 0; }
  float    getFloat(const char* k, float d = NAN)     { return getT<float>(k, d); }
  double   getDouble(const char* k, double d = NAN)   { return getT<double>(k, d); }
  size_t putChar(const char* k, int8_t v)     { return putT<int8_t>(k, v); }
  size_t putUChar(const char* k, uint8_t v)   { return putT<uint8_t>(k, v); }
  size_t putShort(const char* k, int16_t v)   { return putT<int16_t>(k, v); }
  size_t putUShort(const char* k, uint16_t v) { return putT<uint16_t>(k, v); }
  size_t putInt(const char* k, int32_t v)     { return putT<int32_t>(k, v); }
  size_t putUInt(const char* k, uint32_t v)   { return putT<uint32_t>(k, v); }
  size_t putLong(const char* k, int32_t v)    { return putT<int32_t>(k, v); }
  size_t putULong(const char* k, uint32_t v)  { return putT<uint32_t>(k, v); }
  size_t putBool(const char* k, bool v)       { return putT<uint8_t>(k, v ? 1 : 0); }
  size_t putFloat(const char* k, float v)     { return putT<float>(k, v); }
  size_t putDouble(const char* k, double v)   { return putT<double>(k, v); }
};

// =====================================================================
// WebServer (Teilmenge der ESP32-API, reicht fuer diesen Sketch)
// =====================================================================
enum HTTPMethod { HTTP_ANY = 0, HTTP_GET = 1, HTTP_POST = 2 };
enum HTTPUploadStatus { UPLOAD_FILE_START, UPLOAD_FILE_WRITE, UPLOAD_FILE_END, UPLOAD_FILE_ABORTED };
struct HTTPUpload { HTTPUploadStatus status; String filename; size_t totalSize; size_t currentSize; uint8_t buf[16]; };
#define CONTENT_LENGTH_UNKNOWN ((size_t)-1)

class WebServer {
public:
  typedef std::function<void(void)> THandlerFunction;
private:
  struct Route { const char* uri; HTTPMethod m; THandlerFunction fn; };
  static const int MAX_ROUTES = 48, MAX_ARGS = 48;
  Route routes[MAX_ROUTES];
  int nRoutes = 0;
  THandlerFunction notFound;
  WiFiServer srv;
  WiFiClient cl;
  bool started = false;
  String curUri;
  HTTPMethod curMethod = HTTP_GET;
  String argN[MAX_ARGS], argV[MAX_ARGS];
  int nArgs = 0;
  String extraHeaders;
  size_t contentLen = CONTENT_LENGTH_UNKNOWN;
  bool lenSet = false, headSent = false;
  uint8_t out[1460];
  size_t outN = 0;
  HTTPUpload up;

  static int hexv(char c) { return c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1; }
  static String urlDecode(const char* s, size_t n) {
    String r; r.reserve(n);
    for (size_t i = 0; i < n; i++) {
      char c = s[i];
      if (c == '+') r += ' ';
      else if (c == '%' && i + 2 < n && hexv(s[i + 1]) >= 0 && hexv(s[i + 2]) >= 0) { r += (char)(hexv(s[i + 1]) * 16 + hexv(s[i + 2])); i += 2; }
      else r += c;
    }
    return r;
  }
  void parseArgs(const char* q, size_t n) {
    size_t i = 0;
    while (i < n && nArgs < MAX_ARGS) {
      size_t j = i; while (j < n && q[j] != '&') j++;
      size_t eq = i; while (eq < j && q[eq] != '=') eq++;
      if (j > i) {
        argN[nArgs] = urlDecode(q + i, eq - i);
        argV[nArgs] = eq < j ? urlDecode(q + eq + 1, j - eq - 1) : String("");
        nArgs++;
      }
      i = j + 1;
    }
  }
  bool readLine(String& line, uint32_t deadline) {
    line = "";
    while ((int32_t)(millis() - deadline) < 0) {
      if (cl.available()) {
        int c = cl.read();
        if (c < 0) continue;
        if (c == '\n') return true;
        if (c != '\r' && line.length() < 2048) line += (char)c;
      } else {
        if (!cl.connected()) return false;
        delay(1);
      }
    }
    return false;
  }
  void rawWrite(const uint8_t* b, size_t n) {
    uint32_t t0 = millis();
    while (n && cl.connected() && millis() - t0 < 8000) {
      size_t w = cl.write(b, n);
      if (w == 0) { delay(2); continue; }
      b += w; n -= w;
    }
  }
  void outFlush() { if (outN) { rawWrite(out, outN); outN = 0; } }
  void outWrite(const uint8_t* b, size_t n) {
    if (n >= sizeof(out)) { outFlush(); rawWrite(b, n); return; }
    if (outN + n > sizeof(out)) outFlush();
    memcpy(out + outN, b, n); outN += n;
  }
  void outStr(const char* s) { outWrite((const uint8_t*)s, strlen(s)); }
  static const char* reason(int code) {
    switch (code) { case 200: return "OK"; case 204: return "No Content"; case 302: return "Found"; case 400: return "Bad Request";
                    case 404: return "Not Found"; case 500: return "Internal Server Error"; default: return "OK"; }
  }
  void sendHead(int code, const char* type, size_t len) {
    if (headSent) return;
    char h[160];
    snprintf(h, sizeof(h), "HTTP/1.1 %d %s\r\nContent-Type: %s\r\n", code, reason(code), type && *type ? type : "text/html");
    outStr(h);
    if (len != CONTENT_LENGTH_UNKNOWN) { snprintf(h, sizeof(h), "Content-Length: %u\r\n", (unsigned)len); outStr(h); }
    outStr(extraHeaders.c_str());
    outStr("Connection: close\r\n\r\n");
    headSent = true;
  }
public:
  explicit WebServer(int port = 80) : srv(port) {}
  void begin() {}
  void on(const char* uri, THandlerFunction fn) { on(uri, HTTP_ANY, fn); }
  void on(const char* uri, HTTPMethod m, THandlerFunction fn) {
    if (nRoutes < MAX_ROUTES) { routes[nRoutes].uri = uri; routes[nRoutes].m = m; routes[nRoutes].fn = fn; nRoutes++; }
  }
  void on(const char* uri, HTTPMethod m, THandlerFunction fn, THandlerFunction ufn) { (void)ufn; on(uri, m, fn); }
  void onNotFound(THandlerFunction fn) { notFound = fn; }

  String arg(const char* name) const { for (int i = 0; i < nArgs; i++) if (argN[i] == name) return argV[i]; return String(""); }
  String arg(const String& name) const { return arg(name.c_str()); }
  String arg(int i) const { return i >= 0 && i < nArgs ? argV[i] : String(""); }
  String argName(int i) const { return i >= 0 && i < nArgs ? argN[i] : String(""); }
  int args() const { return nArgs; }
  bool hasArg(const char* name) const { for (int i = 0; i < nArgs; i++) if (argN[i] == name) return true; return false; }
  bool hasArg(const String& name) const { return hasArg(name.c_str()); }
  const String& uri() const { return curUri; }
  HTTPMethod method() const { return curMethod; }
  HTTPUpload& upload() { return up; }
  WiFiClient& client() { return cl; }

  void sendHeader(const String& name, const String& value, bool first = false) {
    String h = name + ": " + value + "\r\n";
    if (first) extraHeaders = h + extraHeaders; else extraHeaders += h;
  }
  void setContentLength(size_t len) { contentLen = len; lenSet = true; }
  void send(int code, const char* type, const char* content, size_t n) {
    sendHead(code, type, lenSet ? contentLen : n);
    if (n) outWrite((const uint8_t*)content, n);
  }
  void send(int code, const char* type, const char* content) { send(code, type, content, content ? strlen(content) : 0); }
  void send(int code, const char* type, const String& content) { send(code, type, content.c_str(), content.length()); }
  void send(int code, const String& type, const String& content) { send(code, type.c_str(), content.c_str(), content.length()); }
  void send(int code) { send(code, "text/plain", "", 0); }
  void send_P(int code, const char* type, const char* content) { send(code, type, content); }
  void sendContent(const char* s, size_t n) { if (!headSent) sendHead(200, "text/plain", CONTENT_LENGTH_UNKNOWN); if (n) outWrite((const uint8_t*)s, n); }
  void sendContent(const char* s) { sendContent(s, strlen(s)); }
  void sendContent(const String& s) { sendContent(s.c_str(), s.length()); }
  size_t streamFile(File& f, const String& type) {
    size_t total = f.size();
    sendHead(200, type.c_str(), total);
    uint8_t b[1024];
    size_t sent = 0;
    int n;
    while ((n = f.read(b, sizeof(b))) > 0) { outWrite(b, n); sent += n; avWdKick(); }
    return sent;
  }

  void handleClient() {
    if (!started) {
      if (WiFi.status() != WL_CONNECTED) return;
      srv.begin();
      started = true;
    }
    cl = srv.accept();
    if (!cl) return;
    nArgs = 0; extraHeaders = ""; lenSet = false; headSent = false; outN = 0; contentLen = CONTENT_LENGTH_UNKNOWN;
    uint32_t deadline = millis() + 2500;
    String line;
    if (!readLine(line, deadline)) { cl.stop(); return; }
    int sp1 = line.indexOf(' '), sp2 = line.indexOf(' ', sp1 + 1);
    if (sp1 < 0 || sp2 < 0) { cl.stop(); return; }
    String m = line.substring(0, sp1), target = line.substring(sp1 + 1, sp2);
    curMethod = m == "POST" ? HTTP_POST : HTTP_GET;
    int qm = target.indexOf('?');
    curUri = qm >= 0 ? target.substring(0, qm) : target;
    if (qm >= 0) parseArgs(target.c_str() + qm + 1, target.length() - qm - 1);
    size_t bodyLen = 0;
    bool form = false;
    while (readLine(line, deadline)) {
      if (line.length() == 0) break;
      int c = line.indexOf(':');
      if (c <= 0) continue;
      String k = line.substring(0, c); k.toLowerCase();
      String v = line.substring(c + 1); v.trim();
      if (k == "content-length") bodyLen = v.toInt();
      else if (k == "content-type") form = v.startsWith("application/x-www-form-urlencoded");
    }
    if (bodyLen > 0 && bodyLen <= 16384) {
      char* body = (char*)malloc(bodyLen + 1);
      size_t got = 0;
      if (body) {
        while (got < bodyLen && (int32_t)(millis() - deadline) < 0) {
          int a = cl.available();
          if (a > 0) { int r = cl.read((uint8_t*)body + got, min((size_t)a, bodyLen - got)); if (r > 0) got += r; }
          else if (!cl.connected()) break;
          else delay(1);
        }
        body[got] = 0;
        if (form) parseArgs(body, got);
        else if (nArgs < MAX_ARGS) { argN[nArgs] = "plain"; argV[nArgs] = String(body); nArgs++; }
        free(body);
      }
    }
    bool handled = false;
    for (int i = 0; i < nRoutes && !handled; i++) {
      if (curUri == routes[i].uri && (routes[i].m == HTTP_ANY || routes[i].m == curMethod)) { routes[i].fn(); handled = true; }
    }
    if (!handled) {
      if (notFound) notFound();
      else send(404, "text/plain", "Nicht gefunden");
    }
    if (!headSent) send(500, "text/plain", "");
    outFlush();
    cl.stop();
  }
};

// =====================================================================
// HTTPClient (nur einfache GET-Anfragen ohne TLS, z. B. Shelly)
// =====================================================================
class HTTPClient {
  String host, path;
  uint16_t port = 80;
  uint32_t connMs = 3000, toMs = 3000;
  WiFiClient c;
public:
  void setConnectTimeout(int32_t ms) { connMs = ms; }
  void setTimeout(uint16_t ms) { toMs = ms; }
  bool begin(const String& url) {
    String u = url;
    if (!u.startsWith("http://")) return false;
    u = u.substring(7);
    int sl = u.indexOf('/');
    String hp = sl >= 0 ? u.substring(0, sl) : u;
    path = sl >= 0 ? u.substring(sl) : String("/");
    int co = hp.indexOf(':');
    host = co >= 0 ? hp.substring(0, co) : hp;
    port = co >= 0 ? (uint16_t)hp.substring(co + 1).toInt() : 80;
    return host.length() > 0;
  }
  bool begin(const char* url) { return begin(String(url)); }
  int GET() {
    c.setSocketTimeout(connMs);
    if (!c.connect(host.c_str(), port)) return -1;
    String req = "GET " + path + " HTTP/1.0\r\nHost: " + host + "\r\nConnection: close\r\n\r\n";
    c.write((const uint8_t*)req.c_str(), req.length());
    uint32_t t0 = millis();
    String line;
    while (millis() - t0 < toMs) {
      while (c.available()) {
        int ch = c.read();
        if (ch == '\n') {
          int sp = line.indexOf(' ');
          return sp > 0 ? line.substring(sp + 1, sp + 4).toInt() : -2;
        }
        if (ch >= 0 && ch != '\r' && line.length() < 120) line += (char)ch;
      }
      if (!c.connected() && !c.available()) break;
      delay(5);
    }
    return -11;
  }
  void end() { c.stop(); }
};

// =====================================================================
// MQTT: esp-mqtt-API auf Basis von PubSubClient (eigener Thread)
// =====================================================================
typedef const char* esp_event_base_t;
typedef enum {
  MQTT_EVENT_ANY = -1, MQTT_EVENT_ERROR = 0, MQTT_EVENT_CONNECTED, MQTT_EVENT_DISCONNECTED,
  MQTT_EVENT_SUBSCRIBED, MQTT_EVENT_UNSUBSCRIBED, MQTT_EVENT_PUBLISHED, MQTT_EVENT_DATA
} esp_mqtt_event_id_t;
#define ESP_EVENT_ANY_ID (-1)

struct esp_mqtt_event_t {
  esp_mqtt_event_id_t event_id;
  const char* topic; int topic_len;
  const char* data;  int data_len;
};
typedef esp_mqtt_event_t* esp_mqtt_event_handle_t;
typedef void (*esp_event_handler_t)(void* args, esp_event_base_t base, int32_t id, void* data);

struct esp_mqtt_client_config_t {
  struct { struct { const char* uri; } address; } broker;
  struct { const char* username; const char* client_id; struct { const char* password; } authentication; } credentials;
  struct { int keepalive; struct { const char* topic; const char* msg; int qos; int retain; } last_will; } session;
  struct { int reconnect_timeout_ms; } network;
  struct { int stack_size; } task;
};

class AvMqtt {
public:
  char host[48] = "", user[40] = "", pass[56] = "", id[40] = "", willTopic[64] = "", willMsg[24] = "";
  uint16_t port = 1883;
  int keepalive = 30, willQos = 0, willRetain = 0;
  uint32_t reconnectMs = 10000;
  esp_event_handler_t handler = nullptr;
  void* handlerArgs = nullptr;
  WiFiClient net;
  PubSubClient ps;
  rtos::Mutex m;
  bool wasConnected = false;
  int msgId = 0;
  AvMqtt() : ps(net) {}

  void emit(esp_mqtt_event_id_t ev, const char* t = nullptr, int tl = 0, const char* d = nullptr, int dl = 0) {
    if (!handler) return;
    esp_mqtt_event_t e; e.event_id = ev; e.topic = t; e.topic_len = tl; e.data = d; e.data_len = dl;
    handler(handlerArgs, "MQTT", (int32_t)ev, &e);
  }
  static AvMqtt* self;
  static void onMsg(char* topic, uint8_t* payload, unsigned int len) {
    if (self) self->emit(MQTT_EVENT_DATA, topic, (int)strlen(topic), (const char*)payload, (int)len);
  }
  void run() {
    uint32_t lastTry = 0;
    bool first = true;
    for (;;) {
      bool wifi = WiFi.status() == WL_CONNECTED;
      m.lock();
      bool conn = wifi && ps.connected();
      if (conn) ps.loop();
      else if (wifi && (first || millis() - lastTry >= reconnectMs)) {
        first = false; lastTry = millis();
        bool ok = ps.connect(id, user[0] ? user : nullptr, pass[0] ? pass : nullptr,
                             willTopic[0] ? willTopic : nullptr, willQos, willRetain != 0, willMsg);
        if (ok) { wasConnected = true; emit(MQTT_EVENT_CONNECTED); }
      }
      if (!ps.connected() && wasConnected) { wasConnected = false; emit(MQTT_EVENT_DISCONNECTED); }
      m.unlock();
      delay(conn ? 10 : 200);
    }
  }
  static void task(void* p) { ((AvMqtt*)p)->run(); }
};
AvMqtt* AvMqtt::self = nullptr;
typedef AvMqtt* esp_mqtt_client_handle_t;

inline esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t* cfg) {
  AvMqtt* c = new AvMqtt();
  const char* u = cfg->broker.address.uri ? cfg->broker.address.uri : "";
  if (strncmp(u, "mqtt://", 7) == 0) u += 7;
  const char* colon = strrchr(u, ':');
  size_t hl = colon ? (size_t)(colon - u) : strlen(u);
  if (hl >= sizeof(c->host)) hl = sizeof(c->host) - 1;
  memcpy(c->host, u, hl); c->host[hl] = 0;
  c->port = colon ? (uint16_t)atoi(colon + 1) : 1883;
  if (cfg->credentials.client_id) strncpy(c->id, cfg->credentials.client_id, sizeof(c->id) - 1);
  if (cfg->credentials.username) strncpy(c->user, cfg->credentials.username, sizeof(c->user) - 1);
  if (cfg->credentials.authentication.password) strncpy(c->pass, cfg->credentials.authentication.password, sizeof(c->pass) - 1);
  if (cfg->session.last_will.topic) strncpy(c->willTopic, cfg->session.last_will.topic, sizeof(c->willTopic) - 1);
  if (cfg->session.last_will.msg) strncpy(c->willMsg, cfg->session.last_will.msg, sizeof(c->willMsg) - 1);
  c->willQos = cfg->session.last_will.qos; c->willRetain = cfg->session.last_will.retain;
  if (cfg->session.keepalive) c->keepalive = cfg->session.keepalive;
  if (cfg->network.reconnect_timeout_ms) c->reconnectMs = cfg->network.reconnect_timeout_ms;
  c->ps.setServer(c->host, c->port);
  c->ps.setKeepAlive(c->keepalive);
  c->ps.setSocketTimeout(5);
  c->ps.setBufferSize(3072);                  // Status-JSON ist groesser als die 256 Byte Standard
  c->ps.setCallback(AvMqtt::onMsg);
  AvMqtt::self = c;
  return c;
}
inline int esp_mqtt_client_register_event(esp_mqtt_client_handle_t c, esp_mqtt_event_id_t, esp_event_handler_t h, void* args) {
  if (!c) return -1;
  c->handler = h; c->handlerArgs = args; return 0;
}
inline int esp_mqtt_client_start(esp_mqtt_client_handle_t c) {
  if (!c) return -1;
  xTaskCreatePinnedToCore(AvMqtt::task, "mqtt", 6144, c, 1, nullptr, 0);
  return 0;
}
inline int esp_mqtt_client_publish(esp_mqtt_client_handle_t c, const char* topic, const char* data, int len, int qos, int retain) {
  (void)qos;
  if (!c) return -1;
  if (len <= 0) len = data ? strlen(data) : 0;
  if (!c->m.trylock()) return -1;          // MQTT-Thread verbindet gerade -> nicht blockieren
  bool ok = c->ps.connected() && c->ps.publish(topic, (const uint8_t*)data, (unsigned int)len, retain != 0);
  int id = ok ? ++c->msgId : -1;
  c->m.unlock();
  return id;
}
inline int esp_mqtt_client_subscribe_single(esp_mqtt_client_handle_t c, const char* topic, int qos) {
  if (!c) return -1;
  c->m.lock();
  bool ok = c->ps.subscribe(topic, qos > 1 ? 1 : qos);
  c->m.unlock();
  return ok ? 1 : -1;
}
