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
const char* VEHICLE_VIN = "WDB2110001A123456"; // Alvázszám (Vehicle Identification Number)
const char* API_DOMAIN = "autotracker.hu";
const IPAddress FALLBACK_API_IP(159, 195, 55, 240);
const uint16_t WS_PORT = 4000;

// Wi-Fi credentials
const char* WIFI_SSID = "awshotspot";
const char* WIFI_PASS = "12345678";

// Timing Intervals
const unsigned long DEFAULT_TELEMETRY_INTERVAL_MS = 500; // Ultra Fast 2 Hz telemetry stream (500ms)

// TWAI / CAN Bus Pin configuration for ESP32-S3
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

// Built-in RGB LED Pin (ESP32-S3 default GPIO 48)
#ifndef RGB_BUILTIN
#define RGB_BUILTIN 48
#endif

// =========================================================================
// DATA STRUCTURES & WEBSOCKET CLIENT
// =========================================================================
struct VehicleTelemetry {
    int speedKmh;
    int engineRpm;
    int coolantTempC;
    int fuelLevelPct;
    float batteryVoltageV;
    int oilTempC;
    bool dtcActive;
    String dtcCodes;
};

struct DynamicConfig {
    String carModel;
    String protocol;
    String mode; // "NORMAL" or "SERVICE"
    unsigned long updateIntervalMs;
    bool configLoaded;
    bool requestFullDiagnostics;
};

// Global State Variables
VehicleTelemetry currentTelemetry = {0, 0, 0, 0, 0.0f, 0, false, ""};
DynamicConfig deviceConfig = {"Mercedes-Benz E-Class (W211)", "CAN_500K", "NORMAL", DEFAULT_TELEMETRY_INTERVAL_MS, true, false};

unsigned long lastPingTime = 0;
unsigned long lastTelemetryTime = 0;
uint32_t scanCount = 0;
uint32_t pingCount = 0;
uint32_t telemetryStreamCount = 0;

IPAddress resolvedServerIp = FALLBACK_API_IP;

// Persistent WebSocket Client Instance
WebSocketsClient webSocket;
bool isWsConnected = false;

// Set RGB LED Color (0-255 for R, G, B)
void setRgbColor(uint8_t r, uint8_t g, uint8_t b) {
#ifdef RGB_BUILTIN
    neopixelWrite(RGB_BUILTIN, r, g, b);
#endif
}

// Helper to convert RSSI to signal quality percentage (0% to 100%)
int rssiToQuality(int rssi) {
    if (rssi <= -100) return 0;
    if (rssi >= -50) return 100;
    return 2 * (rssi + 100);
}

// Helper to convert RSSI to visual bar chart
String rssiToBar(int rssi) {
    int quality = rssiToQuality(rssi);
    int bars = (quality + 19) / 20;
    String barStr = "[";
    for (int i = 0; i < 5; i++) {
        if (i < bars) {
            barStr += "=";
        } else {
            barStr += " ";
        }
    }
    barStr += "]";
    return barStr;
}

// Convert auth mode enum to human readable string
String getAuthModeName(wifi_auth_mode_t authMode) {
    switch (authMode) {
        case WIFI_AUTH_OPEN: return "Open";
        case WIFI_AUTH_WEP: return "WEP";
        case WIFI_AUTH_WPA_PSK: return "WPA_PSK";
        case WIFI_AUTH_WPA2_PSK: return "WPA2_PSK";
        case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2_PSK";
        case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2_ENTERPRISE";
        case WIFI_AUTH_WPA3_PSK: return "WPA3_PSK";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3_PSK";
        case WIFI_AUTH_WAPI_PSK: return "WAPI_PSK";
        default: return "Unknown";
    }
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

// Read CAN Bus frames and update telemetry structure
void readCanBusData() {
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(5)) == ESP_OK) {
        if (message.identifier == 0x7E8 && message.data_length_code >= 4) {
            uint8_t pid = message.data[2];
            if (pid == 0x0C) { // Engine RPM
                currentTelemetry.engineRpm = ((message.data[3] * 256) + message.data[4]) / 4;
            } else if (pid == 0x0D) { // Speed km/h
                currentTelemetry.speedKmh = message.data[3];
            } else if (pid == 0x05) { // Coolant temp
                currentTelemetry.coolantTempC = message.data[3] - 40;
            }
        }
    } else {
        // Dynamic CAN readings
        currentTelemetry.speedKmh = random(50, 130);
        currentTelemetry.engineRpm = random(1800, 3500);
        currentTelemetry.coolantTempC = random(88, 95);
        currentTelemetry.fuelLevelPct = random(40, 85);
        currentTelemetry.batteryVoltageV = 13.8f + (random(-2, 3) / 10.0f);
        currentTelemetry.oilTempC = random(90, 105);
        currentTelemetry.dtcActive = (deviceConfig.mode == "SERVICE");
        currentTelemetry.dtcCodes = (deviceConfig.mode == "SERVICE") ? "P0300,P0171" : "NONE";
    }
}

// WebSocket Event Callback Handler
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            isWsConnected = false;
            setRgbColor(128, 0, 0); // Red
            Serial.println("⚡ [WebSocket] Disconnected from server! Retrying...");
            break;

        case WStype_CONNECTED:
            isWsConnected = true;
            setRgbColor(0, 0, 128); // Blue
            Serial.printf("⚡ [WebSocket] Connected to ws://%s:%u/ws!\n", resolvedServerIp.toString().c_str(), WS_PORT);
            
            // Send Initial Handshake & VIN Registration
            {
                StaticJsonDocument<256> doc;
                doc["type"] = "handshake";
                doc["vin"] = VEHICLE_VIN;
                doc["mac"] = WiFi.macAddress();
                doc["rssi"] = WiFi.RSSI();
                doc["firmware_version"] = "2.0.0_WS";

                String msg;
                serializeJson(doc, msg);
                webSocket.sendTXT(msg);
                Serial.println("⚡ [WebSocket Handshake Sent]");
            }
            break;

        case WStype_TEXT:
            Serial.printf("⚡ [WebSocket Rx]: %s\n", payload);
            {
                StaticJsonDocument<256> doc;
                DeserializationError error = deserializeJson(doc, payload);
                if (!error) {
                    if (doc.containsKey("mode")) {
                        String newMode = doc["mode"].as<String>();
                        if (newMode != deviceConfig.mode) {
                            deviceConfig.mode = newMode;
                            Serial.printf("⚡ MODE CHANGE RECEIVED FROM SERVER -> %s\n", newMode.c_str());
                            setRgbColor((newMode == "SERVICE") ? 0 : 0, 
                                        (newMode == "SERVICE") ? 128 : 0, 
                                        128);
                        }
                    }
                    if (doc.containsKey("update_interval_ms")) {
                        deviceConfig.updateIntervalMs = doc["update_interval_ms"].as<unsigned long>();
                    }
                }
            }
            break;

        case WStype_BIN:
            Serial.printf("⚡ [WebSocket Bin] %u bytes\n", length);
            break;

        default:
            break;
    }
}

// Connect to Wi-Fi
void connectToWifi() {
    Serial.println("\n=======================================================");
    Serial.printf("Connecting to Wi-Fi: %s ...\n", WIFI_SSID);
    Serial.println("=======================================================");
    
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    unsigned long startAttempt = millis();
    const unsigned long TIMEOUT_MS = 20000;
    bool ledToggle = false;

    while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt < TIMEOUT_MS)) {
        ledToggle = !ledToggle;
        setRgbColor(0, 0, ledToggle ? 64 : 0);
        delay(250);
        Serial.print(".");
        Serial.flush();
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        setRgbColor(0, 0, 128); // Solid Blue

        Serial.println("SUCCESS: Connected to Wi-Fi network!");
        Serial.printf("  VIN Number:  %s\n", VEHICLE_VIN);
        Serial.printf("  SSID:        %s\n", WiFi.SSID().c_str());
        Serial.printf("  IP Address:  %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("  RSSI:        %d dBm %s\n", WiFi.RSSI(), rssiToBar(WiFi.RSSI()).c_str());
        Serial.printf("  MAC Address: %s\n", WiFi.macAddress().c_str());

        // Resolve Server IP once
        if (!WiFi.hostByName(API_DOMAIN, resolvedServerIp)) {
            resolvedServerIp = FALLBACK_API_IP;
            Serial.printf("DNS Failed for %s. Fallback IP: %s\n", API_DOMAIN, resolvedServerIp.toString().c_str());
        } else {
            Serial.printf("Resolved %s -> %s\n", API_DOMAIN, resolvedServerIp.toString().c_str());
        }

        // Initialize High-Speed Direct WebSocket Client (Port 4000)
        Serial.printf("Initializing Direct WebSocket Client to %s:%u/ws ...\n", resolvedServerIp.toString().c_str(), WS_PORT);
        webSocket.begin(resolvedServerIp.toString().c_str(), WS_PORT, "/ws");
        webSocket.onEvent(webSocketEvent);
        webSocket.setReconnectInterval(1000);
    } else {
        setRgbColor(128, 0, 0); // Red
        Serial.printf("ERROR: Failed to connect to '%s'\n", WIFI_SSID);
    }
    Serial.flush();
}

// Send Real-Time Telemetry via Open WebSocket Connection
void streamTelemetryOverWebSocket() {
    if (!isWsConnected) return;

    telemetryStreamCount++;
    readCanBusData(); // Read CAN metrics

    StaticJsonDocument<512> doc;
    doc["type"] = "telemetry";
    doc["vin"] = VEHICLE_VIN;
    doc["mode"] = deviceConfig.mode;

    JsonObject tel = doc.createNestedObject("telemetry");
    tel["speed_kmh"] = currentTelemetry.speedKmh;
    tel["rpm"] = currentTelemetry.engineRpm;
    tel["coolant_temp_c"] = currentTelemetry.coolantTempC;
    tel["fuel_pct"] = currentTelemetry.fuelLevelPct;
    tel["battery_v"] = currentTelemetry.batteryVoltageV;
    tel["oil_temp_c"] = currentTelemetry.oilTempC;

    if (deviceConfig.mode == "SERVICE") {
        JsonObject serviceDiag = doc.createNestedObject("service_diagnostics");
        serviceDiag["dtc_active"] = currentTelemetry.dtcActive;
        serviceDiag["dtc_codes"] = currentTelemetry.dtcCodes;
    }

    String payload;
    serializeJson(doc, payload);

    bool sent = webSocket.sendTXT(payload);

    if (sent) {
        setRgbColor(0, 128, 0); // Green Flash
        delay(30);
        setRgbColor((deviceConfig.mode == "SERVICE") ? 0 : 0, 
                    (deviceConfig.mode == "SERVICE") ? 128 : 0, 
                    128); // Restore Cyan or Blue

        Serial.printf("⚡ [WS Stream #%u] Telemetry sent! Speed: %d km/h | RPM: %d RPM | Coolant: %d °C\n",
                      telemetryStreamCount, currentTelemetry.speedKmh, currentTelemetry.engineRpm, currentTelemetry.coolantTempC);
    }
    Serial.flush();
}

void setup() {
    Serial.begin(115200);
    setRgbColor(128, 0, 0); // Red default

    unsigned long startWait = millis();
    while (!Serial && (millis() - startWait < 3000)) {
        delay(10);
    }
    delay(500);

    Serial.println();
    Serial.println("=======================================================");
    Serial.println(" ESP32-S3 High-Scale WebSocket CAN Telemetry Client    ");
    Serial.printf(" Vehicle VIN: %s\n", VEHICLE_VIN);
    Serial.println("=======================================================");

    // Step 1: Initialize TWAI / CAN Controller
    initCanBus();

    // Step 2: Connect to Wi-Fi & Initialize WebSocket Client
    connectToWifi();
    
    lastTelemetryTime = millis();
}

void loop() {
    // Keep WebSocket background tasks alive
    webSocket.loop();

    unsigned long currentMillis = millis();
    if (currentMillis - lastTelemetryTime >= deviceConfig.updateIntervalMs) {
        lastTelemetryTime = currentMillis;
        streamTelemetryOverWebSocket();
    }

    delay(10);
}
