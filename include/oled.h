#ifndef OLED_H
#define OLED_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <WiFi.h>

#define TFT_MOSI 35
#define TFT_SCLK 36
#define TFT_MISO 37
#define TFT_CS   39
#define TFT_DC   40
#define TFT_RST  38
#define TFT_BACKLIGHT 45

// Heart beat animation settings
#define HEART_BEAT_DURATION 150  // How long the heart stays big (ms)

class OLED {
private:
    Adafruit_ST7789* tft;
    bool heartBig;
    unsigned long lastHeartBeat;
    int lastDisplayedBPM;
    bool lastFingerState;
    
    void drawHeart(bool big);
    
public:
    OLED();
    
    void begin();
    void clear();
    void setBrightness(bool on);
    void showInitScreen(const char* title);
    void updateStatus(float bpm, int avgBpm, long irValue = 0, bool fingerDetected = true);
    void triggerHeartBeat();
    void updateHeartAnimation();
    void showText(int x, int y, const char* text, uint16_t color = ST77XX_WHITE, uint8_t size = 2);
    void clearArea(int x, int y, int width, int height);
};

#endif
