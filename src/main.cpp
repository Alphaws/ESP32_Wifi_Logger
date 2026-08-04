#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Ping.h>
#include "esp_wifi.h"

// Define built-in RGB LED pin if not defined by board variant (ESP32-S3 default is GPIO 48)
#ifndef RGB_BUILTIN
#define RGB_BUILTIN 48
#endif

// Wi-Fi credentials
const char* WIFI_SSID = "awshotspot";
const char* WIFI_PASS = "12345678";

// Target host to ping
const char* PING_HOST = "autotracker.hu";

// Intervals
const unsigned long PING_INTERVAL_MS = 10000; // Ping every 10 seconds
unsigned long lastPingTime = 0;
uint32_t scanCount = 0;
uint32_t pingCount = 0;

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
    int bars = (quality + 19) / 20; // 0 to 5 bars
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
        case WIFI_AUTH_OPEN:
            return "Open";
        case WIFI_AUTH_WEP:
            return "WEP";
        case WIFI_AUTH_WPA_PSK:
            return "WPA_PSK";
        case WIFI_AUTH_WPA2_PSK:
            return "WPA2_PSK";
        case WIFI_AUTH_WPA_WPA2_PSK:
            return "WPA/WPA2_PSK";
        case WIFI_AUTH_WPA2_ENTERPRISE:
            return "WPA2_ENTERPRISE";
        case WIFI_AUTH_WPA3_PSK:
            return "WPA3_PSK";
        case WIFI_AUTH_WPA2_WPA3_PSK:
            return "WPA2/WPA3_PSK";
        case WIFI_AUTH_WAPI_PSK:
            return "WAPI_PSK";
        default:
            return "Unknown";
    }
}

void performWifiScan() {
    scanCount++;
    Serial.println("\n=======================================================");
    Serial.printf("  ESP32-S3 Wi-Fi Scanner - Scan #%u\n", scanCount);
    Serial.println("=======================================================");
    Serial.println("Scanning for available Wi-Fi networks...");
    Serial.flush();

    int n = WiFi.scanNetworks(false /* async */, true /* show_hidden */);

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
            if (ssid.length() == 0) {
                ssid = "<Hidden Network>";
            }

            if (ssid.length() > 32) {
                ssid = ssid.substring(0, 29) + "...";
            }

            int32_t rssi = WiFi.RSSI(i);
            int32_t channel = WiFi.channel(i);
            String bssid = WiFi.BSSIDstr(i);
            wifi_auth_mode_t auth = WiFi.encryptionType(i);
            String security = getAuthModeName(auth);
            String bar = rssiToBar(rssi);

            if (auth == WIFI_AUTH_OPEN) {
                openCount++;
            } else {
                secureCount++;
            }

            Serial.printf("%-4d | %-32s | %-4d | %-4d dBm | %-6s | %-17s | %-16s\n",
                          i + 1,
                          ssid.c_str(),
                          channel,
                          rssi,
                          bar.c_str(),
                          bssid.c_str(),
                          security.c_str());
        }

        Serial.println("---------------------------------------------------------------------------------------------------");
        Serial.printf("Summary: Total: %d | Open: %d | Encrypted: %d\n", n, openCount, secureCount);
    }

    WiFi.scanDelete();
    Serial.flush();
}

void connectToWifi() {
    Serial.println("\n=======================================================");
    Serial.printf("Connecting to Wi-Fi: %s ...\n", WIFI_SSID);
    Serial.println("=======================================================");
    
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    unsigned long startAttempt = millis();
    const unsigned long TIMEOUT_MS = 20000; // 20s timeout
    bool ledToggle = false;

    // Blink Blue while connecting
    while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt < TIMEOUT_MS)) {
        ledToggle = !ledToggle;
        if (ledToggle) {
            setRgbColor(0, 0, 64); // Soft Blue ON
        } else {
            setRgbColor(0, 0, 0);  // OFF
        }
        delay(250);
        Serial.print(".");
        Serial.flush();
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        // Solid Blue when connected
        setRgbColor(0, 0, 128);

        Serial.println("SUCCESS: Connected to Wi-Fi network!");
        Serial.printf("  SSID:        %s\n", WiFi.SSID().c_str());
        Serial.printf("  IP Address:  %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("  Subnet Mask: %s\n", WiFi.subnetMask().toString().c_str());
        Serial.printf("  Gateway:     %s\n", WiFi.gatewayIP().toString().c_str());
        Serial.printf("  DNS Server:  %s\n", WiFi.dnsIP().toString().c_str());
        Serial.printf("  RSSI:        %d dBm %s\n", WiFi.RSSI(), rssiToBar(WiFi.RSSI()).c_str());
        Serial.printf("  MAC Address: %s\n", WiFi.macAddress().c_str());

        Serial.println("Waiting 2s for network stack, ARP & gateway routing stabilization...");
        Serial.flush();
        delay(2000);
    } else {
        // Red when connection fails
        setRgbColor(128, 0, 0);
        Serial.printf("ERROR: Failed to connect to '%s'. Status code: %d\n", WIFI_SSID, WiFi.status());
    }
    Serial.flush();
}

void performPingTest() {
    pingCount++;
    Serial.println("\n-------------------------------------------------------");
    Serial.printf("Ping Test #%u -> %s\n", pingCount, PING_HOST);
    Serial.println("-------------------------------------------------------");

    if (WiFi.status() != WL_CONNECTED) {
        setRgbColor(128, 0, 0); // Red if disconnected
        Serial.println("WARNING: Wi-Fi disconnected! Reconnecting...");
        connectToWifi();
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("ERROR: Cannot ping target host because Wi-Fi is disconnected.");
            return;
        }
    }

    // Resolve DNS
    IPAddress targetIp;
    bool resolved = WiFi.hostByName(PING_HOST, targetIp);

    if (resolved) {
        Serial.printf("Resolved %s -> %s\n", PING_HOST, targetIp.toString().c_str());
    } else {
        Serial.printf("WARNING: DNS resolution failed for %s. Attempting ping directly by hostname...\n", PING_HOST);
    }

    // Send 4 ICMP ping packets
    bool pingResult = Ping.ping(PING_HOST, 4);

    if (pingResult) {
        float avgTime = Ping.averageTime();
        Serial.printf("PING SUCCESS! Target: %s (%s)\n", PING_HOST, targetIp.toString().c_str());
        Serial.printf("  Average Response Time: %.2f ms\n", avgTime);
        Serial.printf("  Wi-Fi Signal (RSSI):   %d dBm\n", WiFi.RSSI());

        // Green flash for 500ms on successful ping
        setRgbColor(0, 128, 0); // Green Flash
        delay(500);
        setRgbColor(0, 0, 128); // Back to Solid Blue
    } else {
        Serial.printf("PING FAILED! Host %s (%s) did not respond to ICMP ping.\n",
                      PING_HOST, targetIp.toString().c_str());
        // Red flash on ping failure
        setRgbColor(128, 0, 0);
        delay(500);
        if (WiFi.status() == WL_CONNECTED) {
            setRgbColor(0, 0, 128);
        }
    }
    Serial.flush();
}

void setup() {
    Serial.begin(115200);
    
    // Default LED status: RED
    setRgbColor(128, 0, 0);

    unsigned long startWait = millis();
    while (!Serial && (millis() - startWait < 3000)) {
        delay(10);
    }
    delay(500);

    Serial.println();
    Serial.println("=======================================================");
    Serial.println(" ESP32-S3 Wi-Fi Scanner, Ping & RGB LED Indicator      ");
    Serial.println("=======================================================");

    // Step 1: Initial Wi-Fi scan (LED remains RED)
    performWifiScan();
    delay(1000);

    // Step 2: Connect to awshotspot (LED blinks BLUE while connecting, then SOLID BLUE)
    connectToWifi();

    // Step 3: Perform initial ping test (LED flashes GREEN on ping response)
    if (WiFi.status() == WL_CONNECTED) {
        performPingTest();
    }
    
    lastPingTime = millis();
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastPingTime >= PING_INTERVAL_MS) {
        lastPingTime = currentMillis;
        performPingTest();
    }

    delay(50);
}
