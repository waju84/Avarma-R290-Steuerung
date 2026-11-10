# Verkabelung

Alle Platinen arbeiten mit **3,3 V** – keine 5-V-Signale an die Pins. 230-V-Teile (Ventilmotor, Shelly) nur durch eine Elektrofachkraft.

## Übersicht

```
                      RS485 (A/B + GND), als Linie verlegt
  ┌──────────────┐        ┌────────────────────┐        ┌───────────────────┐
  │ GIGA / ESP32 │──A/B───│ Ventilmodul (ID 2) │──A/B───│ Avarma R290 (ID 1)│
  │ + RS485-Modul│        │ optional, UNO R4   │        │ Modbus-Klemme     │
  └──────────────┘        └────────────────────┘        └───────────────────┘
        │ D20/D21 bzw. IO3
        └── Raumfühler (SHT4x oder DHT22)
```

## Arduino GIGA R1 WiFi + GIGA Display Shield

Das Display-Shield steckt auf der Rückseite (Stecker J5/J6). Alle Pins oben bleiben frei.

| Funktion | GIGA-Pin | Bemerkung |
|---|---|---|
| RS485 RO (Empfang) | D0 (RX0) | Serial1 |
| RS485 DI (Senden) | D1 (TX0) | Serial1 |
| RS485 DE + /RE (gebrückt) | D2 | im Web änderbar |
| RS485 VCC / GND | 3V3 / GND | 3,3-V-Modul (MAX3485, SP3485) |
| Raumfühler SHT4x SDA / SCL | D20 / D21 | I2C „Wire“, empfohlen |
| Raumfühler DHT22 Daten | D3 | 10 kΩ nach 3V3, falls nicht auf dem Modul |
| Ventil-Relais Motorstrom | D4 | nur ohne Ventilmodul |
| Ventil-Relais Richtung | D5 | nur ohne Ventilmodul |
| Logdaten | intern (QSPI) | Archiv optional auf dem NAS |
| Versorgung | USB-C 5 V (mind. 2 A) oder VIN 6–24 V | |

Display steht auf dem Kopf? In `giga_ui.h` `UI_ROTATION` 1 ↔ 3 tauschen. Touch versetzt? Display → System → Touch-Test, dann `UI_TOUCH_FLIP` anpassen.

## ESP32-S3 (Hosyond / LCDWIKI ES3C28P, 2,8")

Display, Touch und microSD sind auf der Platine. Frei am Erweiterungsstecker: IO2, IO3, IO14, IO21 (dazu IO43/IO44).

| Funktion | ESP-Pin | Bemerkung |
|---|---|---|
| RS485 RO (Empfang) | IO14 | UART1 |
| RS485 DI (Senden) | IO21 | UART1 |
| RS485 DE + /RE (gebrückt) | IO2 | im Web änderbar |
| RS485 VCC / GND | 3V3 / GND | 3,3-V-Modul |
| Raumfühler DHT22 Daten | IO3 | |
| Ventil-Relais | im Web einstellen | IO43 / IO44 (IO3 ist mit dem DHT22 belegt) – oder Ventilmodul |

## RS485 / Modbus

- Modul: 3,3-V-Typ mit DE/RE-Anschluss (MAX3485 / SP3485). Ein 5-V-MAX485 geht nur mit Pegelwandler auf RO.
- **A an A, B an B.** Kommt keine Antwort: A und B tauschen (die Hersteller beschriften das unterschiedlich).
- Kabel: verdrilltes Paar für A/B (z. B. aus CAT5/CAT6 oder J-Y(St)Y 2×2×0,8), eine Ader des zweiten Paares als gemeinsame Masse.
- Als Linie verlegen (kein Stern), Stichleitungen kürzer als ca. 0,5 m. 120-Ω-Abschluss nur an den beiden Enden,
  bei 9600 Baud und wenigen Metern meist nicht nötig.
- WP: Modbus-ID 1, 9600 Baud. Ventilmodul: ID 2.

## Raumfühler

- **SHT45 / SHT41** (I2C) empfohlen – genauer und stabiler. Im Web: Heizung → Für Fortgeschrittene → Fühler-Typ „SHT4x“ (GIGA).
- **DHT22** geht auch; Kabel bei 3,3 V höchstens ca. 3 m.
- Nicht direkt an der Platine/dem Display montieren (Eigenwärme), mind. 15–20 cm Abstand, frei umströmt.

## Umschaltventil (nur Warmwasser-Variante „Umschaltventil vom ESP“)

ESBE-3-Wege-Ventil mit **3-Punkt-Stellmotor** (230 V). Zwei Relais:

| Relais | Aufgabe |
|---|---|
| 1 – Motorstrom | schaltet die 230 V für den Motor ein/aus |
| 2 – Richtung | Ruhekontakt (NC) = Richtung Heizung, Arbeitskontakt (NO) = Richtung WW |

Laufzeit Endlage → Endlage (Standard 130 s) und Puffer-Stellung im Web unter Warmwasser → Ventil-Hardware einstellen.

**Ventilmodul** (`ESBE_Slave_R4`): Arduino UNO R4 Minima als Modbus-Slave (ID 2) am selben RS485-Bus, fährt den Motor
selbstständig und fährt bei Ausfall des Masters auf Heizung. Pins: MAX485 an D0/D1, DE+RE an D6, Relais an D7 (Motorstrom),
D8 (Richtung), D9/D10 Reserve, OLED 0,91" an I2C. Bibliotheken: U8g2, ModbusRTUSlave (3.x).

## Shelly zur Strommessung

Siehe [README](../README.md#strommessung-mit-shelly-optional). Die Stromzangen bzw. der Shelly gehören in die Zuleitung der WP,
hinter den Leitungsschutzschalter der WP.
