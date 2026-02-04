#ifndef WIFI_HELPER_H
#define WIFI_HELPER_H

#include <Arduino.h>
#include <WiFi.h>

class WiFiHelper {
public:
    WiFiHelper(const char* ssid, const char* password);

    void begin(unsigned int timeoutSeconds = 10);
    bool isConnected();
    String getIP();
    int32_t getRSSI();
    void printStatus();

private:
    const char* _ssid;
    const char* _password;
};

#endif
