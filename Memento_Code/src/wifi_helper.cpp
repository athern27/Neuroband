#include "wifi_helper.h"

WiFiHelper::WiFiHelper(const char* ssid, const char* password) {
    _ssid = ssid;
    _password = password;
}

void WiFiHelper::begin(unsigned int timeoutSeconds) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(_ssid, _password);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < (timeoutSeconds * 1000UL)) {
        delay(500);
        Serial.print('.');
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("WiFi connected: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println();
        Serial.println("WiFi connection failed (timeout)");
    }
}

bool WiFiHelper::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

String WiFiHelper::getIP() {
    return WiFi.localIP().toString();
}

int32_t WiFiHelper::getRSSI() {
    return WiFi.RSSI();
}

void WiFiHelper::printStatus() {
    if (isConnected()) {
        Serial.print("IP: ");
        Serial.println(getIP());
        Serial.print("RSSI: ");
        Serial.print(getRSSI());
        Serial.println(" dBm");
    } else {
        Serial.println("WiFi: not connected");
    }
}
