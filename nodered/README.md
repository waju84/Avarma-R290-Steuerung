# Node-RED-Flows (optional)

Nur nötig, wenn PV-Daten / SG-Ready aus einem Victron-System kommen oder die WP in VRM erscheinen soll.
Import in Node-RED: Menü → Import → Datei wählen.

| Datei | Zweck | Voraussetzung |
|---|---|---|
| `victron_pv_sg_flow.json` | PV, Batterie, Netz, Haus vom Victron-System → `avarma/pv/...`; PV-Überschuss → SG-Ready (`avarma/cmd/sg`); Hand-Befehle zum Testen | Node-RED auf Venus OS (Cerbo GX) mit Palette „Victron Energy“; Broker `127.0.0.1` |
| `shelly_wp_flow.json` | Shelly-Werte der WP in Node-RED sichtbar machen (Test) | Broker-Adresse im Knoten „MQTT-Broker“ eintragen |
| `shelly_wp_victron_flow.json` | WP als eigener Verbraucher („Wärmepumpe“) in VRM, Werte vom Shelly | Venus OS ab 3.80, Palette „Victron Energy“ mit virtuellem Gerät |

Anpassen:
- Die **MPPT-Knoten** (Tagesertrag) zeigen auf die Laderegler der eigenen Anlage – jeden Knoten öffnen und das eigene MPPT auswählen.
- Ohne Batterie in Abschnitt 2 `SOC_MIN` auf 0 setzen.
- Das MQTT-Präfix `avarma` muss zum Präfix in der Avarma-Oberfläche passen (System → MQTT).
