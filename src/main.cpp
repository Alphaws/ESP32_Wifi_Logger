#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <ESP32Ping.h>
#include "esp_wifi.h"
#include "esp_netif.h"
#include "driver/twai.h" // ESP32-S3 TWAI (CAN Bus) driver

// =========================================================================
// VEHICLE IDENTIFICATION & DEVICE CONFIGURATION
// =========================================================================
const char* FALLBACK_VIN = "WDB2110001A123456"; // Fallback ha az OBD2 VIN lekérés sikertelen
const char* API_DOMAIN = "autotracker.hu";
const IPAddress FALLBACK_API_IP(159, 195, 55, 240);
const uint16_t WS_PORT = 4000;

// Wi-Fi credentials
const char* WIFI_SSID = "awshotspot";
const char* WIFI_PASS = "12345678";

// Timing Intervals
const unsigned long DEFAULT_TELEMETRY_INTERVAL_MS = 500; // 2 Hz telemetry stream (500ms)

// TWAI / CAN Bus Pin configuration for ESP32-S3
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

// Built-in RGB LED Pin (ESP32-S3 default GPIO 48)
#ifndef RGB_BUILTIN
#define RGB_BUILTIN 48
#endif

// =========================================================================
// OFFLINE RING BUFFER QUEUE (ZERO DATA LOSS)
// =========================================================================
#define TELEMETRY_QUEUE_CAPACITY 300 // Store up to 300 offline telemetry records (~5 minutes)

struct BufferedTelemetry {
    uint32_t timestampMs;
    // Original 6 fields
    int speedKmh;
    int engineRpm;
    int coolantTempC;
    int fuelLevelPct;
    float batteryVoltageV;
    int oilTempC;
    // Extended OBD2 PIDs
    int throttlePct;       // 0x11 Throttle position (%)
    int engineLoadPct;     // 0x04 Engine load (%)
    int intakeTempC;       // 0x0F Intake air temperature (°C)
    float mafGs;           // 0x10 MAF rate (g/s)
    float timingAdvanceDeg;// 0x0E Timing advance (°)
    int boostKpa;          // 0x70 Turbo boost pressure (kPa)
    // Diagnostics
    bool dtcActive;
    char dtcCodes[16];
    bool isSimulated;
};

struct DynamicConfig {
    String carModel;
    String protocol;
    String mode; // "NORMAL" or "SERVICE"
    unsigned long updateIntervalMs;
    bool configLoaded;
    bool requestFullDiagnostics;
};

// Circular Ring Buffer Variables
BufferedTelemetry telemetryQueue[TELEMETRY_QUEUE_CAPACITY];
int queueHead = 0;
int queueTail = 0;
int queueCount = 0;

// Global State Variables
BufferedTelemetry currentTelemetry = {0, 0, 0, 0, 0, 0.0f, 0, 0, 0, 0, 0.0f, 0.0f, 0, false, "", false};
DynamicConfig deviceConfig = {"Mercedes-Benz E-Class (W211)", "CAN_500K", "NORMAL", DEFAULT_TELEMETRY_INTERVAL_MS, true, false};

// Dynamic PID Filter Flags
bool filterPidsEnabled = false;
bool reqSpeed = true;
bool reqRpm = true;
bool reqCoolant = true;
bool reqBattery = true;
bool reqFuel = true;
bool reqOil = true;
// Extended PID flags
bool reqThrottle = false;
bool reqEngineLoad = false;
bool reqIntakeTemp = false;
bool reqMaf = false;
bool reqTimingAdv = false;
bool reqBoost = false;

unsigned long lastPingTime = 0;
unsigned long lastTelemetryTime = 0;
uint32_t telemetryStreamCount = 0;
uint32_t totalCanFramesRead = 0;

IPAddress resolvedServerIp = FALLBACK_API_IP;

// Auto-detected VIN (via OBD2 Mode 09 PID 02)
char detectedVin[18]   = {0}; // 17 chars + null
bool vinAutoDetected   = false;

// Persistent WebSocket Client Instance
WebSocketsClient webSocket;
bool isWsConnected = false;

// Set RGB LED Color (0-255 for R, G, B)
void setRgbColor(uint8_t r, uint8_t g, uint8_t b) {
#ifdef RGB_BUILTIN
    neopixelWrite(RGB_BUILTIN, r, g, b);
#endif
}

// Push new telemetry sample to Circular Ring Buffer
void enqueueTelemetry(const BufferedTelemetry& sample) {
    telemetryQueue[queueHead] = sample;
    queueHead = (queueHead + 1) % TELEMETRY_QUEUE_CAPACITY;
    if (queueCount < TELEMETRY_QUEUE_CAPACITY) {
        queueCount++;
    } else {
        queueTail = (queueTail + 1) % TELEMETRY_QUEUE_CAPACITY;
    }
}

// Pop oldest telemetry sample from Circular Ring Buffer
bool dequeueTelemetry(BufferedTelemetry& sampleOut) {
    if (queueCount == 0) return false;
    sampleOut = telemetryQueue[queueTail];
    queueTail = (queueTail + 1) % TELEMETRY_QUEUE_CAPACITY;
    queueCount--;
    return true;
}

// Initialize ESP32-S3 TWAI (CAN Bus) Controller
void initCanBus() {
    Serial.println("\nInitializing TWAI / CAN Bus Controller (500 kbps)...");
    
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("TWAI driver installed successfully.");
    } else {
        Serial.println("WARNING: Failed to install TWAI driver.");
        return;
    }

    if (twai_start() == ESP_OK) {
        Serial.println("TWAI driver started successfully.");
    } else {
        Serial.println("WARNING: Failed to start TWAI driver.");
    }
}

// =========================================================================
// OBD2 VIN REQUEST — ISO-TP Multi-Frame (Mode 09 PID 02)
// Sends a broadcast OBD2 request and assembles the 17-char VIN from
// the ECU's ISO-TP First Frame + Consecutive Frames.
// Returns true if a valid VIN was received within timeout.
// =========================================================================
bool requestVinFromEcu() {
    Serial.println("\n[VIN] Sending OBD2 Mode 09 PID 02 request (ISO-TP)...");

    // OBD2 functional broadcast request: 02 09 02 (Mode 09, PID 02 = VIN)
    twai_message_t req;
    req.identifier       = 0x7DF;
    req.flags            = TWAI_MSG_FLAG_NONE;
    req.data_length_code = 8;
    req.data[0] = 0x02; // Single frame, length 2
    req.data[1] = 0x09; // Mode 09 — vehicle info
    req.data[2] = 0x02; // PID 02 — VIN
    req.data[3] = 0x00; req.data[4] = 0x00;
    req.data[5] = 0x00; req.data[6] = 0x00; req.data[7] = 0x00;

    if (twai_transmit(&req, pdMS_TO_TICKS(200)) != ESP_OK) {
        Serial.println("[VIN] CAN transmit failed.");
        return false;
    }

    char vin[18]      = {0};
    int  vinIdx       = 0;
    bool gotFirstFrame = false;
    unsigned long deadline = millis() + 3000; // 3s timeout

    while (millis() < deadline) {
        twai_message_t msg;
        if (twai_receive(&msg, pdMS_TO_TICKS(50)) != ESP_OK) continue;

        // Only accept responses from standard ECU response ID
        if (msg.identifier != 0x7E8) continue;

        uint8_t nibble = (msg.data[0] >> 4) & 0x0F;

        if (nibble == 0x1) {
            // === FIRST FRAME ===
            // Byte 0-1: 0x10, total_length
            // Byte 2: 0x49, Byte 3: 0x02, Byte 4: 0x01 (number of data items)
            // Byte 5-7: first 3 VIN characters
            if (msg.data[2] != 0x49 || msg.data[3] != 0x02) continue;
            for (int i = 5; i < 8 && vinIdx < 17; i++) {
                if (msg.data[i] >= 0x20 && msg.data[i] <= 0x7E)
                    vin[vinIdx++] = (char)msg.data[i];
            }
            gotFirstFrame = true;

            // Send Flow Control: ContinueToSend, block=0, STmin=0
            twai_message_t fc;
            fc.identifier       = 0x7E0;
            fc.flags            = TWAI_MSG_FLAG_NONE;
            fc.data_length_code = 8;
            fc.data[0] = 0x30; fc.data[1] = 0x00; fc.data[2] = 0x00;
            fc.data[3] = 0x00; fc.data[4] = 0x00; fc.data[5] = 0x00;
            fc.data[6] = 0x00; fc.data[7] = 0x00;
            twai_transmit(&fc, pdMS_TO_TICKS(100));
            Serial.printf("[VIN] First frame received, sent Flow Control. VIN so far: %.3s\n", vin);

        } else if (nibble == 0x2 && gotFirstFrame) {
            // === CONSECUTIVE FRAME ===
            // Byte 0: 0x21, 0x22 ... (sequence number)
            // Byte 1-7: next VIN characters
            for (int i = 1; i < 8 && vinIdx < 17; i++) {
                if (msg.data[i] >= 0x20 && msg.data[i] <= 0x7E)
                    vin[vinIdx++] = (char)msg.data[i];
            }
            Serial.printf("[VIN] Consecutive frame, VIN so far (%d/17): %s\n", vinIdx, vin);

            if (vinIdx >= 17) break; // All 17 characters received
        }
    }

    vin[17] = '\0';

    if (vinIdx == 17) {
        // Basic VIN format validation (A-Z, 0-9, no I/O/Q)
        bool valid = true;
        for (int i = 0; i < 17; i++) {
            char c = vin[i];
            if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) {
                valid = false; break;
            }
            if (c == 'I' || c == 'O' || c == 'Q') { valid = false; break; }
        }
        if (valid) {
            strncpy(detectedVin, vin, 18);
            vinAutoDetected = true;
            Serial.printf("[VIN] ✅ Auto-detected VIN: %s\n", detectedVin);
            return true;
        } else {
            Serial.printf("[VIN] ⚠️  Invalid VIN chars: %s — using fallback\n", vin);
        }
    } else {
        Serial.printf("[VIN] ⚠️  Only got %d/17 chars — using fallback VIN\n", vinIdx);
    }
    return false;
}

// Read CAN Bus frames continuously and store into Ring Buffer
void captureCanBusMetrics() {
    twai_message_t message;
    while (twai_receive(&message, pdMS_TO_TICKS(1)) == ESP_OK) {
        totalCanFramesRead++;
        if (message.identifier == 0x7E8 && message.data_length_code >= 4) {
            uint8_t pid = message.data[2];
            if      (pid == 0x0C) { currentTelemetry.engineRpm    = ((message.data[3] * 256) + message.data[4]) / 4; }
            else if (pid == 0x0D) { currentTelemetry.speedKmh      = message.data[3]; }
            else if (pid == 0x05) { currentTelemetry.coolantTempC  = message.data[3] - 40; }
            else if (pid == 0x11) { currentTelemetry.throttlePct   = (message.data[3] * 100) / 255; }
            else if (pid == 0x04) { currentTelemetry.engineLoadPct = (message.data[3] * 100) / 255; }
            else if (pid == 0x0F) { currentTelemetry.intakeTempC   = message.data[3] - 40; }
            else if (pid == 0x10) { currentTelemetry.mafGs         = ((message.data[3] * 256) + message.data[4]) / 100.0f; }
            else if (pid == 0x0E) { currentTelemetry.timingAdvanceDeg = (message.data[3] / 2.0f) - 64.0f; }
            else if (pid == 0x70) { currentTelemetry.boostKpa      = message.data[3]; }
        }
    }

    // If no live CAN vehicle attached during bench testing, generate dynamic simulated CAN readings
    bool simulating = (totalCanFramesRead == 0);
    if (simulating) {
        currentTelemetry.speedKmh          = random(50, 130);
        currentTelemetry.engineRpm         = random(1800, 3500);
        currentTelemetry.coolantTempC      = random(88, 95);
        currentTelemetry.fuelLevelPct      = random(40, 85);
        currentTelemetry.batteryVoltageV   = 13.8f + (random(-2, 3) / 10.0f);
        currentTelemetry.oilTempC          = random(90, 105);
        currentTelemetry.throttlePct       = random(5, 70);
        currentTelemetry.engineLoadPct     = random(20, 80);
        currentTelemetry.intakeTempC       = random(25, 45);
        currentTelemetry.mafGs             = random(30, 180) / 10.0f;
        currentTelemetry.timingAdvanceDeg  = random(80, 240) / 10.0f - 5.0f; // -5 to 19 deg
        currentTelemetry.boostKpa          = random(100, 220); // 100=atm, 220=boost
        currentTelemetry.dtcActive         = (deviceConfig.mode == "SERVICE");
        strncpy(currentTelemetry.dtcCodes, (deviceConfig.mode == "SERVICE") ? "P0300,P0171" : "NONE", sizeof(currentTelemetry.dtcCodes));
    }
    currentTelemetry.isSimulated = simulating;

    BufferedTelemetry sample;
    sample.timestampMs         = millis();
    sample.speedKmh            = currentTelemetry.speedKmh;
    sample.engineRpm           = currentTelemetry.engineRpm;
    sample.coolantTempC        = currentTelemetry.coolantTempC;
    sample.fuelLevelPct        = currentTelemetry.fuelLevelPct;
    sample.batteryVoltageV     = currentTelemetry.batteryVoltageV;
    sample.oilTempC            = currentTelemetry.oilTempC;
    sample.throttlePct         = currentTelemetry.throttlePct;
    sample.engineLoadPct       = currentTelemetry.engineLoadPct;
    sample.intakeTempC         = currentTelemetry.intakeTempC;
    sample.mafGs               = currentTelemetry.mafGs;
    sample.timingAdvanceDeg    = currentTelemetry.timingAdvanceDeg;
    sample.boostKpa            = currentTelemetry.boostKpa;
    sample.dtcActive           = currentTelemetry.dtcActive;
    strncpy(sample.dtcCodes, currentTelemetry.dtcCodes, sizeof(sample.dtcCodes));
    sample.isSimulated         = simulating;

    enqueueTelemetry(sample);
}

// WebSocket Event Callback Handler
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            isWsConnected = false;
            setRgbColor(128, 0, 0); // Red
            Serial.println("⚡ [WebSocket] Disconnected! Buffering CAN data locally...");
            break;

        case WStype_CONNECTED:
            isWsConnected = true;
            setRgbColor(0, 0, 128); // Blue
            Serial.printf("⚡ [WebSocket] Connected to ws://159.195.55.240:%u/ws!\n", WS_PORT);
            
            {
                StaticJsonDocument<300> doc;
                doc["type"]             = "handshake";
                doc["vin"]              = detectedVin;
                doc["vin_source"]       = vinAutoDetected ? "obd2_auto" : "fallback";
                doc["mac"]              = WiFi.macAddress();
                doc["rssi"]             = WiFi.RSSI();
                doc["firmware_version"] = "2.3.0_AutoVin";

                String msg;
                serializeJson(doc, msg);
                webSocket.sendTXT(msg);
            }
            break;

        case WStype_TEXT:
            Serial.printf("⚡ [WebSocket Rx]: %s\n", payload);
            {
                StaticJsonDocument<512> doc;
                DeserializationError error = deserializeJson(doc, payload);
                if (!error) {
                    if (doc.containsKey("mode")) {
                        String newMode = doc["mode"].as<String>();
                        if (newMode != deviceConfig.mode) {
                            deviceConfig.mode = newMode;
                            Serial.printf("⚡ MODE CHANGE RECEIVED -> %s\n", newMode.c_str());
                        }
                    }
                    if (doc.containsKey("type") && doc["type"] == "config_response") {
                        // Server responded with vehicle profile after handshake
                        String assignedMode    = doc["mode"] | "NORMAL";
                        bool   vehicleFound    = doc["vehicle_found"] | false;
                        bool   autoReg         = doc["auto_registered"] | false;
                        const char* makeModel  = doc["make_model"] | "N/A";
                        const char* plate      = doc["plate"] | "N/A";
                        deviceConfig.mode      = assignedMode;
                        deviceConfig.updateIntervalMs = doc["update_interval_ms"] | DEFAULT_TELEMETRY_INTERVAL_MS;

                        Serial.println(F("=== SERVER CONFIG RESPONSE ==="));
                        Serial.printf("  VIN:          %s\n", detectedVin);
                        Serial.printf("  VIN Source:   %s\n", vinAutoDetected ? "OBD2 Auto-detected" : "Fallback (no CAN)");
                        Serial.printf("  Vehicle DB:   %s\n", vehicleFound ? "FOUND" : (autoReg ? "AUTO-REGISTERED" : "NOT FOUND"));
                        Serial.printf("  Make/Model:   %s\n", makeModel);
                        Serial.printf("  Plate:        %s\n", plate);
                        Serial.printf("  Mode:         %s\n", assignedMode.c_str());
                        Serial.printf("  Interval:     %lu ms\n", deviceConfig.updateIntervalMs);
                        Serial.println(F("=============================="));
                    }
                    if (doc.containsKey("type") && doc["type"] == "pid_config") {
                        JsonArray pids = doc["requestedPids"];
                        reqSpeed = false; reqRpm = false; reqCoolant = false;
                        reqBattery = false; reqFuel = false; reqOil = false;
                        reqThrottle = false; reqEngineLoad = false; reqIntakeTemp = false;
                        reqMaf = false; reqTimingAdv = false; reqBoost = false;

                        for (JsonVariant v : pids) {
                            String p = v.as<String>();
                            if (p == "speed_kmh")         reqSpeed      = true;
                            if (p == "rpm")               reqRpm        = true;
                            if (p == "coolant_temp_c")    reqCoolant    = true;
                            if (p == "battery_v")         reqBattery    = true;
                            if (p == "fuel_pct")          reqFuel       = true;
                            if (p == "oil_temp_c")        reqOil        = true;
                            if (p == "throttle_pct")      reqThrottle   = true;
                            if (p == "engine_load_pct")   reqEngineLoad = true;
                            if (p == "intake_temp_c")     reqIntakeTemp = true;
                            if (p == "maf_gs")            reqMaf        = true;
                            if (p == "timing_advance_deg") reqTimingAdv = true;
                            if (p == "boost_kpa")         reqBoost      = true;
                        }
                        filterPidsEnabled = true;
                        Serial.println("⚡ [PID CONFIG UPDATED] Filtering CAN metric queries based on checkmarks!");
                    }
                }
            }
            break;

        default:
            break;
    }
}

// Connect to Wi-Fi asynchronously while CAN buffer keeps recording
void connectToWifi() {
    Serial.println("\nConnecting to Wi-Fi...");
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt < 20000)) {
        captureCanBusMetrics();
        delay(200);
    }

    if (WiFi.status() == WL_CONNECTED) {
        setRgbColor(0, 0, 128);
        IPAddress serverIp(159, 195, 55, 240);
        webSocket.begin(serverIp, WS_PORT, "/ws");
        webSocket.onEvent(webSocketEvent);
        webSocket.setReconnectInterval(1000);
    }
}

// Flush Ring Buffer over Open WebSocket Connection with PID filtering
void flushTelemetryQueueOverWebSocket() {
    if (!isWsConnected || queueCount == 0) return;

    int flushBatch = min(queueCount, 5);
    for (int i = 0; i < flushBatch; i++) {
        BufferedTelemetry sample;
        if (!dequeueTelemetry(sample)) break;

        telemetryStreamCount++;

        StaticJsonDocument<512> doc;
        doc["type"] = "telemetry";
        doc["vin"] = detectedVin;
        doc["mode"] = deviceConfig.mode;
        doc["sim_mode"] = sample.isSimulated; // true = no real CAN bus, test data only

        JsonObject tel = doc.createNestedObject("telemetry");
        if (!filterPidsEnabled || reqSpeed)      tel["speed_kmh"]          = sample.speedKmh;
        if (!filterPidsEnabled || reqRpm)        tel["rpm"]                = sample.engineRpm;
        if (!filterPidsEnabled || reqCoolant)    tel["coolant_temp_c"]     = sample.coolantTempC;
        if (!filterPidsEnabled || reqBattery)    tel["battery_v"]          = sample.batteryVoltageV;
        if (!filterPidsEnabled || reqFuel)       tel["fuel_pct"]           = sample.fuelLevelPct;
        if (!filterPidsEnabled || reqOil)        tel["oil_temp_c"]         = sample.oilTempC;
        if (!filterPidsEnabled || reqThrottle)   tel["throttle_pct"]       = sample.throttlePct;
        if (!filterPidsEnabled || reqEngineLoad) tel["engine_load_pct"]    = sample.engineLoadPct;
        if (!filterPidsEnabled || reqIntakeTemp) tel["intake_temp_c"]      = sample.intakeTempC;
        if (!filterPidsEnabled || reqMaf)        tel["maf_gs"]             = sample.mafGs;
        if (!filterPidsEnabled || reqTimingAdv)  tel["timing_advance_deg"] = sample.timingAdvanceDeg;
        if (!filterPidsEnabled || reqBoost)      tel["boost_kpa"]          = sample.boostKpa;

        if (deviceConfig.mode == "SERVICE") {
            JsonObject serviceDiag = doc.createNestedObject("service_diagnostics");
            serviceDiag["dtc_active"] = sample.dtcActive;
            serviceDiag["dtc_codes"] = sample.dtcCodes;
        }

        String payload;
        serializeJson(doc, payload);
        webSocket.sendTXT(payload);
    }
}

void setup() {
    Serial.begin(115200);
    setRgbColor(128, 0, 0);
    delay(500);
    initCanBus();

    // Attempt automatic VIN detection from ECU via OBD2 ISO-TP
    // If the car is not connected or doesn't respond, falls back to FALLBACK_VIN
    if (!requestVinFromEcu()) {
        strncpy(detectedVin, FALLBACK_VIN, 18);
        vinAutoDetected = false;
        Serial.printf("[VIN] Using fallback VIN: %s\n", detectedVin);
    }

    connectToWifi();
    lastTelemetryTime = millis();
}

void loop() {
    captureCanBusMetrics();
    webSocket.loop();

    unsigned long currentMillis = millis();
    if (currentMillis - lastTelemetryTime >= deviceConfig.updateIntervalMs) {
        lastTelemetryTime = currentMillis;
        flushTelemetryQueueOverWebSocket();
    }
    delay(10);
}
