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

1. Firmware für die passende Rolle bauen (typisch Main-Controller mit `DEVICE_ROLE_MAIN`).
2. Hardware gemäß `config.h` verdrahten.
3. Starten und Serielle Ausgabe prüfen (`[MAIN] Boot`, danach `[MAIN] Ready`).
4. Kalibrierung, Motorinvertierung und Backlash konfigurieren.

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

## 5) Stellarium

Die Firmware enthält einen TCP-Server (LX200-kompatibel) über den konfigurierten Port.
Für Remote-Steuerung:
1. AP/WLAN aktivieren
2. Host-IP ermitteln
3. In Stellarium ein LX200-/Telescope-Gerät mit passendem Port eintragen

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
