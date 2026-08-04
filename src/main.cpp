#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
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

// Wi-Fi credentials
const char* WIFI_SSID = "awshotspot";
const char* WIFI_PASS = "12345678";

// Timing Intervals
const unsigned long PING_INTERVAL_MS = 10000;            // Ping test every 10s
const unsigned long DEFAULT_TELEMETRY_INTERVAL_MS = 5000; // Default telemetry push every 5s

// TWAI / CAN Bus Pin configuration for ESP32-S3
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

// Built-in RGB LED Pin (ESP32-S3 default GPIO 48)
#ifndef RGB_BUILTIN
#define RGB_BUILTIN 48
#endif

// =========================================================================
// DATA STRUCTURES
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
DynamicConfig deviceConfig = {"Unknown Vehicle", "CAN_500K", "NORMAL", DEFAULT_TELEMETRY_INTERVAL_MS, false, false};

unsigned long lastPingTime = 0;
unsigned long lastTelemetryTime = 0;
uint32_t scanCount = 0;
uint32_t pingCount = 0;
uint32_t telemetryPushCount = 0;

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
    if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
        Serial.printf("CAN Frame Rx -> ID: 0x%03X DLC: %d Data:", message.identifier, message.data_length_code);
        for (int i = 0; i < message.data_length_code; i++) {
            Serial.printf(" %02X", message.data[i]);
        }
        Serial.println();

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
        // Simulated CAN readings if live vehicle is not connected during test
        currentTelemetry.speedKmh = random(50, 110);
        currentTelemetry.engineRpm = random(1800, 3200);
        currentTelemetry.coolantTempC = random(88, 94);
        currentTelemetry.fuelLevelPct = random(40, 85);
        currentTelemetry.batteryVoltageV = 13.8f + (random(-2, 3) / 10.0f);
        currentTelemetry.oilTempC = random(90, 105);
        currentTelemetry.dtcActive = (deviceConfig.mode == "SERVICE");
        currentTelemetry.dtcCodes = (deviceConfig.mode == "SERVICE") ? "P0300,P0171" : "NONE";
    }
}

// Scan Wi-Fi networks
void performWifiScan() {
    scanCount++;
    Serial.println("\n=======================================================");
    Serial.printf("  ESP32-S3 Wi-Fi Scanner - Scan #%u\n", scanCount);
    Serial.println("=======================================================");
    Serial.println("Scanning for available Wi-Fi networks...");
    Serial.flush();

    int n = WiFi.scanNetworks(false, true);

    if (n == 0) {
        Serial.println("No Wi-Fi networks found.");
    } else if (n < 0) {
        Serial.printf("Error occurred during Wi-Fi scan: %d\n", n);
    } else {
        Serial.printf("Found %d network(s):\n\n", n);
        Serial.printf("%-4s | %-32s | %-4s | %-7s | %-6s | %-17s | %-16s\n",
                      "No.", "SSID", "Ch", "RSSI", "Signal", "BSSID (MAC)", "Security");
        Serial.println("---------------------------------------------------------------------------------------------------");

        int openCount = 0;
        int secureCount = 0;

        for (int i = 0; i < n; ++i) {
            String ssid = WiFi.SSID(i);
            if (ssid.length() == 0) ssid = "<Hidden Network>";
            if (ssid.length() > 32) ssid = ssid.substring(0, 29) + "...";

            int32_t rssi = WiFi.RSSI(i);
            int32_t channel = WiFi.channel(i);
            String bssid = WiFi.BSSIDstr(i);
            wifi_auth_mode_t auth = WiFi.encryptionType(i);
            String security = getAuthModeName(auth);
            String bar = rssiToBar(rssi);

            if (auth == WIFI_AUTH_OPEN) openCount++;
            else secureCount++;

            Serial.printf("%-4d | %-32s | %-4d | %-4d dBm | %-6s | %-17s | %-16s\n",
                          i + 1, ssid.c_str(), channel, rssi, bar.c_str(), bssid.c_str(), security.c_str());
        }

        Serial.println("---------------------------------------------------------------------------------------------------");
        Serial.printf("Summary: Total: %d | Open: %d | Encrypted: %d\n", n, openCount, secureCount);
    }

    WiFi.scanDelete();
    Serial.flush();
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
        Serial.printf("  Gateway:     %s\n", WiFi.gatewayIP().toString().c_str());
        Serial.printf("  DNS Server:  %s\n", WiFi.dnsIP().toString().c_str());
        Serial.printf("  RSSI:        %d dBm %s\n", WiFi.RSSI(), rssiToBar(WiFi.RSSI()).c_str());
        Serial.printf("  MAC Address: %s\n", WiFi.macAddress().c_str());

        Serial.println("Waiting 2s for network stack stabilization...");
        Serial.flush();
        delay(2000);
    } else {
        setRgbColor(128, 0, 0); // Red
        Serial.printf("ERROR: Failed to connect to '%s'. Status code: %d\n", WIFI_SSID, WiFi.status());
    }
    Serial.flush();
}

// Perform Ping Test
bool performPingTest() {
    pingCount++;
    Serial.println("\n-------------------------------------------------------");
    Serial.printf("Ping Test #%u -> %s\n", pingCount, API_DOMAIN);
    Serial.println("-------------------------------------------------------");

    if (WiFi.status() != WL_CONNECTED) {
        setRgbColor(128, 0, 0);
        Serial.println("WARNING: Wi-Fi disconnected! Reconnecting...");
        connectToWifi();
        if (WiFi.status() != WL_CONNECTED) {
            return false;
        }
    }

    IPAddress targetIp;
    bool resolved = WiFi.hostByName(API_DOMAIN, targetIp);
    if (resolved) {
        Serial.printf("Resolved %s -> %s\n", API_DOMAIN, targetIp.toString().c_str());
    } else {
        targetIp = FALLBACK_API_IP;
        Serial.printf("DNS Failed for %s. Using Fallback IP: %s\n", API_DOMAIN, targetIp.toString().c_str());
    }

    bool pingResult = Ping.ping(targetIp, 4);

    if (pingResult) {
        float avgTime = Ping.averageTime();
        Serial.printf("PING SUCCESS! Target: %s (%s) | Avg Time: %.2f ms\n",
                      API_DOMAIN, targetIp.toString().c_str(), avgTime);
        setRgbColor(0, 128, 0); // Green Flash
        delay(400);
        setRgbColor((deviceConfig.mode == "SERVICE") ? 0 : 0, 
                    (deviceConfig.mode == "SERVICE") ? 128 : 0, 
                    128); // Restore Cyan or Blue
        return true;
    } else {
        Serial.printf("PING FAILED! Host %s (%s) did not respond.\n", API_DOMAIN, targetIp.toString().c_str());
        setRgbColor(128, 0, 0);
        delay(400);
        if (WiFi.status() == WL_CONNECTED) setRgbColor(0, 0, 128);
        return false;
    }
}

// Robust HTTPS POST helper supporting automatic 308 redirect handling & IP fallback with SNI Host header
int executeApiPost(const String& path, const String& jsonBody, String& responseOut) {
    WiFiClientSecure client;
    client.setInsecure(); // Bypass SSL certificate verification for maximum compatibility
    client.setTimeout(10000);

    IPAddress targetIp;
    bool resolved = WiFi.hostByName(API_DOMAIN, targetIp);
    if (!resolved) {
        targetIp = FALLBACK_API_IP;
    }

    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS); // Follow 301/302/307/308 redirects automatically
    http.setTimeout(10000);

    String fullUrl = "https://" + targetIp.toString() + path;

    http.begin(client, fullUrl);
    http.addHeader("Host", API_DOMAIN);
    http.addHeader("Content-Type", "application/json");

    Serial.printf("POST -> https://%s%s (via %s)\nPayload: %s\n",
                  API_DOMAIN, path.c_str(), targetIp.toString().c_str(), jsonBody.c_str());

    int httpCode = http.POST(jsonBody);

    if (httpCode > 0) {
        responseOut = http.getString();
        Serial.printf("HTTP Status: %d | Server Response: %s\n", httpCode, responseOut.c_str());
    } else {
        Serial.printf("HTTP Client Error: %s (Code %d)\n", http.errorToString(httpCode).c_str(), httpCode);
    }

    http.end();
    return httpCode;
}

// Fetch Vehicle Configuration and Task from autotracker.hu API
void fetchVehicleConfigFromApi() {
    if (WiFi.status() != WL_CONNECTED) return;

    Serial.println("\n=======================================================");
    Serial.printf(" API Query: Registering VIN '%s' with autotracker.hu\n", VEHICLE_VIN);
    Serial.println("=======================================================");

    StaticJsonDocument<256> reqDoc;
    reqDoc["vin"] = VEHICLE_VIN;
    reqDoc["mac"] = WiFi.macAddress();
    reqDoc["rssi"] = WiFi.RSSI();
    reqDoc["firmware_version"] = "1.2.0";

    String requestBody;
    serializeJson(reqDoc, requestBody);

    String response;
    int httpResponseCode = executeApiPost("/api/v1/vehicle/config", requestBody, response);

    if (httpResponseCode == 200 || httpResponseCode == 201) {
        StaticJsonDocument<512> resDoc;
        DeserializationError error = deserializeJson(resDoc, response);

        if (!error) {
            deviceConfig.carModel = resDoc["car_model"] | "Mercedes-Benz W211";
            deviceConfig.protocol = resDoc["protocol"] | "CAN_500K_OBD2";
            deviceConfig.mode = resDoc["mode"] | "NORMAL";
            deviceConfig.updateIntervalMs = resDoc["update_interval_ms"] | 5000;
            deviceConfig.requestFullDiagnostics = resDoc["request_full_diagnostics"] | false;
            deviceConfig.configLoaded = true;

            Serial.println("\n--- API VEHICLE TASK CONFIGURATION LOADED ---");
            Serial.printf("  Car Model:       %s\n", deviceConfig.carModel.c_str());
            Serial.printf("  Protocol:        %s\n", deviceConfig.protocol.c_str());
            Serial.printf("  Operational Mode:%s\n", deviceConfig.mode.c_str());
            Serial.printf("  Update Interval: %lu ms\n", deviceConfig.updateIntervalMs);
            Serial.printf("  Diagnostics:     %s\n", deviceConfig.requestFullDiagnostics ? "ENABLED (SERVICE)" : "STANDARD");
            Serial.println("--------------------------------------------");

            if (deviceConfig.mode == "SERVICE") {
                setRgbColor(0, 128, 128); // Cyan
            }
        } else {
            Serial.printf("JSON Parse Error: %s\n", error.c_str());
        }
    }

    Serial.flush();
}

// Send CAN Telemetry Data to autotracker.hu API
void sendTelemetryToApi() {
    if (WiFi.status() != WL_CONNECTED) return;

    telemetryPushCount++;
    readCanBusData(); // Fetch latest CAN bus metrics

    Serial.println("\n-------------------------------------------------------");
    Serial.printf(" Telemetry Push #%u -> autotracker.hu API\n", telemetryPushCount);
    Serial.println("-------------------------------------------------------");

    StaticJsonDocument<512> doc;
    doc["vin"] = VEHICLE_VIN;
    doc["timestamp"] = millis();
    doc["mode"] = deviceConfig.mode;

    JsonObject telemetry = doc.createNestedObject("telemetry");
    telemetry["speed_kmh"] = currentTelemetry.speedKmh;
    telemetry["rpm"] = currentTelemetry.engineRpm;
    telemetry["coolant_temp_c"] = currentTelemetry.coolantTempC;
    telemetry["fuel_pct"] = currentTelemetry.fuelLevelPct;
    telemetry["battery_v"] = currentTelemetry.batteryVoltageV;
    telemetry["oil_temp_c"] = currentTelemetry.oilTempC;

    if (deviceConfig.mode == "SERVICE" || deviceConfig.requestFullDiagnostics) {
        JsonObject serviceDiag = doc.createNestedObject("service_diagnostics");
        serviceDiag["dtc_active"] = currentTelemetry.dtcActive;
        serviceDiag["dtc_codes"] = currentTelemetry.dtcCodes;
        serviceDiag["can_error_counter"] = 0;
    }

    String payload;
    serializeJson(doc, payload);

    String response;
    int httpCode = executeApiPost("/api/v1/vehicle/telemetry", payload, response);

    if (httpCode == 200 || httpCode == 201) {
        setRgbColor(0, 128, 0); // Green Flash
        delay(300);
        setRgbColor((deviceConfig.mode == "SERVICE") ? 0 : 0, 
                    (deviceConfig.mode == "SERVICE") ? 128 : 0, 
                    128); // Cyan if SERVICE, Blue if NORMAL

        StaticJsonDocument<256> respDoc;
        if (!deserializeJson(respDoc, response)) {
            if (respDoc.containsKey("mode") && respDoc["mode"] != deviceConfig.mode) {
                Serial.printf("CONFIG CHANGE DETECTED FROM SERVER! New Mode: %s\n", respDoc["mode"].as<const char*>());
                fetchVehicleConfigFromApi();
            }
        }
    } else {
        setRgbColor(128, 0, 0); // Red
        delay(300);
        setRgbColor(0, 0, 128);
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
    Serial.println(" ESP32-S3 CAN Telemetry & Autotracker API System       ");
    Serial.printf(" Vehicle VIN: %s\n", VEHICLE_VIN);
    Serial.println("=======================================================");

    // Step 1: Initialize TWAI / CAN Controller
    initCanBus();

    // Step 2: Initial Wi-Fi scan
    performWifiScan();
    delay(1000);

    // Step 3: Connect to Wi-Fi (awshotspot)
    connectToWifi();

    // Step 4: Perform initial Ping & API handshake
    if (WiFi.status() == WL_CONNECTED) {
        if (performPingTest()) {
            fetchVehicleConfigFromApi();
            sendTelemetryToApi();
        }
    }
    
    lastPingTime = millis();
    lastTelemetryTime = millis();
}

void loop() {
    unsigned long currentMillis = millis();

    unsigned long interval = deviceConfig.configLoaded ? deviceConfig.updateIntervalMs : DEFAULT_TELEMETRY_INTERVAL_MS;
    if (currentMillis - lastTelemetryTime >= interval) {
        lastTelemetryTime = currentMillis;
        sendTelemetryToApi();
    }

    delay(50);
}
