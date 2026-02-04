#include "uart.h"

UART::UART() {
}

void UART::begin(unsigned long baud) {
    Serial.begin(baud);
}

void UART::printHeader(const char* title) {
    Serial.println(title);
    for (int i = 0; i < strlen(title); i++) {
        Serial.print("=");
    }
    Serial.println();
}

void UART::printSeparator() {
    Serial.println("----------------------");
}

void UART::printWiFiStatus() {
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("RSSI: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    } else {
        Serial.println("WiFi: not connected");
    }
}

void UART::printSensorData(const char* name, float value, const char* unit) {
    Serial.print(name);
    Serial.print(": ");
    Serial.print(value);
    if (strlen(unit) > 0) {
        Serial.print(" ");
        Serial.print(unit);
    }
    Serial.println();
}

void UART::printSensorData(const char* name, int value, const char* unit) {
    Serial.print(name);
    Serial.print(": ");
    Serial.print(value);
    if (strlen(unit) > 0) {
        Serial.print(" ");
        Serial.print(unit);
    }
    Serial.println();
}

void UART::printMemoryInfo() {
    Serial.print("Free heap: ");
    Serial.print(esp_get_free_heap_size());
    Serial.println(" bytes");
}

void UART::printTelemetry(float bpm, int avgBpm) {
    printSeparator();
    Serial.println("UART Telemetry");
    printWiFiStatus();
    printSensorData("BPM", (int)bpm);
    printSensorData("Avg BPM", avgBpm);
    printMemoryInfo();
    printSeparator();
}
