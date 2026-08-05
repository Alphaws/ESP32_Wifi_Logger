# ESP32-S3 AutoTracker — Zero Data Loss CAN Telemetria Kliens

**Verzió:** `2.2.0_DynamicPids`  
**Platform:** ESP32-S3 DevKitC-1  
**Framework:** Arduino (PlatformIO)

---

## 📋 Leírás

Ez az ESP32-S3 firmware az `autotracker.hu` platform IoT oldala. CAN Bus adatot olvas az autó OBD2 csatlakozójáról és WebSocket-en keresztül valós időben streameli a szerverre.

---

## 🔌 Pin Kiosztás

| Pin | Funkció |
|---|---|
| GPIO 5 | TWAI CAN TX |
| GPIO 4 | TWAI CAN RX |
| GPIO 48 | RGB LED (beépített) |

---

## 🚀 Főbb Funkciók

### Nulladatvesztés Ring Buffer
- 300 rekord tárolható RAM-ban offline időszakban (~5 perc)
- Wi-Fi csatlakozás közben is folyamatosan rögzíti a CAN Bus adatot
- Csatlakozás után automatikusan flush-ol max 5 rekord/ciklus sebességgel

### Dinamikus PID Szűrő
- A szerver `pid_config` üzenetben küldi a kért mezők listáját
- Az ESP32 csak a kért mezőket szerepelteti a telemetria JSON-ban
- Kisebb adatcsomag = kisebb sávszélesség felhasználás

### Szimulált Teszt Adat (`sim_mode`)
- Ha nincs valódi CAN Bus jel (pl. az autóhoz nincs csatlakoztatva), az ESP32 véletlenszerű tesztadatokat generál
- A telemetria csomagban `"sim_mode": true` jelzi ezt a frontendnek
- Valódi CAN Bus esetén: `"sim_mode": false`

### RGB LED Állapotjelzés
| Szín | Állapot |
|---|---|
| 🔴 Piros | Offline / nincs Wi-Fi / nincs WebSocket |
| 🔵 Kék | WebSocket kapcsolódva |
| 🟢 Zöld villanás | Telemetria sikeresen elküldve |

---

## 🛠️ Beállítás & Flash

### Konfiguráció (`src/main.cpp`)
```cpp
const char* VEHICLE_VIN = "WDB2110001A123456";  // Módosítsd!
const char* WIFI_SSID = "awshotspot";            // Módosítsd!
const char* WIFI_PASS = "12345678";              // Módosítsd!
const IPAddress FALLBACK_API_IP(159, 195, 55, 240);
const uint16_t WS_PORT = 4000;
```

### Flash Parancs
```bash
# Port felszabadítása (ha foglalt)
fuser -k /dev/ttyACM0 2>/dev/null

# Feltöltés
pio run --target upload --upload-port /dev/ttyACM0
```

### Serial Monitor
```bash
pio device monitor --port /dev/ttyACM0 --baud 115200
```

---

## 📡 WebSocket Protokoll

### Kézfogás (csatlakozáskor egyszer)
```json
{
  "type": "handshake",
  "vin": "WDB2110001A123456",
  "mac": "e0:72:a1:d6:9c:b0",
  "rssi": -65,
  "firmware_version": "2.2.0_DynamicPids"
}
```

### Telemetria (500ms-ként)
```json
{
  "type": "telemetry",
  "vin": "WDB2110001A123456",
  "mode": "NORMAL",
  "sim_mode": false,
  "telemetry": {
    "speed_kmh": 87,
    "rpm": 2340,
    "coolant_temp_c": 91,
    "battery_v": 13.8
  }
}
```

### Fogadható üzenetek (Szerver → ESP32)
```json
// PID szűrő konfiguráció
{ "type": "pid_config", "requestedPids": ["speed_kmh", "rpm", "coolant_temp_c", "battery_v"] }

// Üzemmód váltás
{ "type": "mode_change", "mode": "SERVICE", "update_interval_ms": 500 }

// Konfiguráció válasz (kézfogás után)
{ "type": "config_response", "mode": "NORMAL", "update_interval_ms": 1000 }
```

---

## 📦 Függőségek (`platformio.ini`)

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
upload_speed = 921600

lib_deps =
    marian-craciunescu/ESP32Ping @ ^1.7
    bblanchon/ArduinoJson @ ^6.21.3
    links2004/WebSockets @ ^2.4.1
```

---

## 🔗 Kapcsolódó Projekt

- **Backend + Frontend:** `/home/alphaws/Dev/Projects/autotracker_hu`
- **GitHub:** [Alphaws/autotracker_hu](https://github.com/Alphaws/autotracker_hu)
- **BrainVault:** `/home/alphaws/BrainVault/10_Projects/autotracker_hu`

---

*Utolsó frissítés: 2026-08-05*
