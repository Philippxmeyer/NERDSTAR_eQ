# NERDSTAR eQ – Bedienungsanleitung

Diese Anleitung beschreibt die aktuelle **equatoriale (RA/Dec)** Firmware.

## 1) Überblick

NERDSTAR eQ steuert zwei Schrittmotorachsen:
- **RA (Rektaszension)**
- **Dec (Deklination)**

Die Bewegungslogik ist auf RA/Dec ausgelegt. Alte Alt/Az-Bezeichner werden nur noch als Kompatibilitätsalias akzeptiert.

---

## 2) Achsenlogik & Grenzen

- **RA** läuft zyklisch (0–360°).
- **Dec** hat in der aktuellen Firmware **keine softwareseitige Soft-Begrenzung**.
- Tracking-Raten werden in **Grad/Sekunde auf RA und Dec** angegeben.

Wichtige Umrechnungen:
- `stepsToRaDegrees()` / `raDegreesToSteps()`
- `stepsToDecDegrees()` / `decDegreesToSteps()`

Kompatibilität:
- `stepsToAzDegrees()` entspricht intern `stepsToRaDegrees()`
- `stepsToAltDegrees()` entspricht intern `stepsToDecDegrees()`

---

## 3) Inbetriebnahme

1. Firmware bauen und auf den Controller flashen.
2. Hardware gemäß `config.h` verdrahten.
3. Starten und Serielle Ausgabe prüfen (`[MAIN] Boot`, danach `[MAIN] Ready`).
4. Kalibrierung, Motorinvertierung und Backlash konfigurieren.

### 3.1 Schritt-für-Schritt: Raspberry Pi installieren (SmartScope)

> Fokus: kompletter Installationsprozess auf dem Raspberry Pi (Repo klonen, Installer, ASTAP).

1. **Raspberry Pi vorbereiten**  
   Raspberry Pi OS (Bookworm, 64-bit) installieren, Pi starten und Terminal öffnen.

2. **Repository klonen**
   ```bash
   cd ~
   git clone https://github.com/Philippxmeyer/NERDSTAR_eQ.git
   cd NERDSTAR_eQ/smartscope
   ```

3. **SmartScope-Installer ausführen**
   ```bash
   sudo bash install.sh
   ```
   Danach einmal **neu einloggen oder rebooten**, damit Gruppenzugehörigkeiten (z. B. `dialout`) sicher aktiv sind.

4. **ASTAP installieren (für Plate Solving)**
   ```bash
   # aktuelle ARM64-Datei von https://www.hnsky.org/astap.htm verwenden
   wget https://www.hnsky.org/astap_armv8.deb
   sudo apt install ./astap_armv8.deb
   ```

5. **ASTAP-Sternkatalog einspielen (z. B. G18)**
   ```bash
   sudo mkdir -p /usr/share/astap
   sudo cp g18_* /usr/share/astap/
   ```
   (Die `g18_*`-Dateien vorher von der ASTAP-Seite herunterladen und entpacken.)

6. **ESP32/NERDSTAR eQ per USB verbinden und Port prüfen**
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*
   ```
   Falls vorhanden, alternativ den udev-Link aus dem Installer nutzen: `/dev/nerdstar`.

7. **Dienst starten und prüfen**
   ```bash
   sudo systemctl start smartscope
   sudo systemctl status smartscope
   ```
   Optional für Autostart beim Boot:
   ```bash
   sudo systemctl enable smartscope
   ```

8. **Verbindung in Stellarmate/INDI testen**  
   Treiber **„LX200 Generic“** wählen, Serial-Port auf `/dev/ttyUSB*`, `/dev/ttyACM*` oder `/dev/nerdstar` setzen und **9600 baud** konfigurieren.

9. **Handshake und Grundfunktion prüfen**  
   LX200-Testkommando (`:GR#` oder `:GVP#`) senden, danach kurze Bewegung testen und `STOP_ALL` als Not-Stopp verifizieren.

Empfehlung nach Umbau von Alt/Az auf eQ:
- Achsen neu nullen
- Schritte/Grad neu einmessen
- Backlash- und Invert-Werte neu prüfen

---

## 4) Serielle Kommandos

### 4.1 Achsenparameter

Folgende Achsnamen sind gültig:
- Neu: `RA`, `DEC`
- Alt (Alias): `AZ`, `ALT`

### 4.2 Nützliche Kommandos

- `SET_MANUAL_SPS <AXIS> <steps_per_second>`
- `SET_GOTO_SPS <AXIS> <steps_per_second>`
- `SET_TRACKING_ENABLED <0|1>`
- `SET_TRACKING_RATES <ra_deg_per_sec> <dec_deg_per_sec>`
- `STOP_ALL`

Koordinaten/Schritte:
- `STEPS_TO_RA <steps>` (Alias: `STEPS_TO_AZ`)
- `STEPS_TO_DEC <steps>` (Alias: `STEPS_TO_ALT`)
- `RA_TO_STEPS <deg>` (Alias: `AZ_TO_STEPS`)
- `DEC_TO_STEPS <deg>` (Alias: `ALT_TO_STEPS`)

---

## 5) Stellarmate / INDI (USB)

Die Firmware stellt auf dem USB-Serial-Port einen LX200-kompatiblen
Kommandokanal bereit. Die Anbindung ist rein kabelgebunden – es gibt weder
WLAN noch TCP.

1. ESP32 per USB mit dem Stellarmate / Raspberry Pi verbinden.
2. In Stellarmate den Treiber „LX200 Generic" (oder kompatibel) wählen.
3. Serial-Port auf den neu entstandenen `/dev/ttyUSB*` bzw. `/dev/ttyACM*`
   setzen und **9600 baud** konfigurieren.
4. Ein Handshake-Kommando (`:GR#`, `:GVP#`) liefert direkt nach dem Boot gültige
   Antworten – es werden keine Debug-Nachrichten über USB gesendet.

### 5.1 Standort- und Zeitsynchronisation

Der Stellarmate schickt beim Verbinden üblicherweise den Standort (`:St`,
`:Sg`), den UTC-Offset (`:SG`) sowie Datum und Uhrzeit (`:SC`, `:SL`). Die
Firmware:

- speichert Breitengrad (nord-positiv) und Längengrad
  (intern ost-positiv, bei `:Gg#` wird wieder west-positiv ausgegeben) in
  EEPROM,
- merkt sich den UTC-Offset in Minuten und
- kombiniert `:SC`+`:SL` mit dem gespeicherten Offset zu einer UTC-Epoch und
  stellt damit die DS3231-RTC.

Vorher gespeicherte Werte bleiben über einen Reboot erhalten und können über
`:Gt#` / `:Gg#` zurückgelesen werden.

---

## 6) Fehlersuche

- Keine Bewegung: `SET_TRACKING_ENABLED`, Goto-/Manuellraten prüfen.
- Falsche Richtung: `SET_MOTOR_INVERT` anpassen.
- Dec stoppt nicht mehr durch Software-Limit; dann eher Motorinvertierung, Kalibrierung oder Mechanik prüfen.
- Ungenaue Ziele: Kalibrierung und Backlash neu einstellen.

---

## 7) Sicherheit

- Mechanische Endanschläge zusätzlich zur Softwarebegrenzung vorsehen.
- Vor schnellem Goto Kabelwege und Kollisionen prüfen.
- `STOP_ALL` als Not-Stopp jederzeit verfügbar halten.
