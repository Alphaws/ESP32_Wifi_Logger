# ESP32-S3 Wi-Fi Scanner & Logger

Ez a projekt egy ESP32-S3 mikrovezérlőre írt Wi-Fi hálózat kereső és konzolos beolvasó alkalmazás PlatformIO alapokon.

## Funkciók
- **Automatikus és periodikus Wi-Fi pásztázás** (10 másodpercenként).
- **Részletes hálózati információk kilistázása**:
  - SSID (Rejtett hálózatok detektálása: `<Hidden Network>`)
  - RSSI (Jelerősség dBm-ben és vizuális `[==== ]` oszlop diagramon)
  - Csatorna száma (Channel)
  - BSSID (Access Point MAC címe)
  - Biztonsági protokoll (Open, WEP, WPA, WPA2, WPA3, WPA2/WPA3, Enterprise)
- **ESP32-S3 USB CDC soros konzol támogatás** (`-DARDUINO_USB_CDC_ON_BOOT=1`).
- **Összegző statisztika** minden mérés végén (Összesen, Nyílt, Titkosított).

## Követelmények & Fejlesztőkészlet
- **Hardware**: ESP32-S3 DevKit (pl. ESP32-S3-DevKitC-1)
- **Software**: PlatformIO Core (`pio`)

## Használat / Parancsok

### 1. Projekt fordítása
```bash
pio run
```

### 2. Szoftver feltöltése az ESP32-S3 mikrokontrollerre
```bash
pio run -t upload
```

### 3. Soros monitor elindítása (115200 baud)
```bash
pio device monitor
```

### 4. Feltöltés és Monitor egyszerre
```bash
pio run -t upload -t monitor
```
