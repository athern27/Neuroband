#ifndef UART_H
#define UART_H

#include <Arduino.h>
#include <WiFi.h>

class UART {
public:
    UART();
    
    void begin(unsigned long baud = 115200);
    void printHeader(const char* title);
    void printSeparator();
    void printWiFiStatus();
    void printSensorData(const char* name, float value, const char* unit = "");
    void printSensorData(const char* name, int value, const char* unit = "");
    void printMemoryInfo();
    void printTelemetry(float bpm, int avgBpm);
};

#endif
