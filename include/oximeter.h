#ifndef OXIMETER_H
#define OXIMETER_H

#include <Arduino.h>
#include <MAX30105.h>
#include <heartRate.h>

#define OXIMETER_RATE_SIZE 8          // Increased for better averaging
#define OXIMETER_IR_THRESHOLD 50000
#define IR_BUF_SIZE 32                  // Increased buffer for better peak detection
#define MIN_BEAT_INTERVAL_MS 400        // 150 BPM max (60000/400)
#define MAX_BEAT_INTERVAL_MS 1500       // 40 BPM min (60000/1500)
#define MIN_PEAK_RANGE 3000             // Increased for less false positives
#define MAX_BPM_CHANGE 15               // Max BPM change per beat (prevents jumps)
#define BPM_SMOOTHING_FACTOR 0.3f       // Weight for new BPM readings (0.0-1.0)

class Oximeter {
private:
    MAX30105* sensor;
    byte rates[OXIMETER_RATE_SIZE];
    byte rateSpot;
    long lastBeat;
    float beatsPerMinute;
    int beatAvg;
    float lastBPM;
    float smoothedBPM;                  // Smoothed BPM for stable readings
    int lastAvg;
    unsigned long irBuf[IR_BUF_SIZE];
    uint8_t irBufIndex;
    unsigned long lastIR;
    unsigned long lastPeakTime;
    bool risingEdge;
    int validBeatCount;                 // Count of valid beats for initial stabilization
    volatile bool beatDetected;         // Flag set when beat detected
    
public:
    Oximeter(MAX30105* max30105);
    
    void begin();
    void update();
    float getBPM();
    int getAvgBPM();
    long getIR();
    bool isFingerDetected();
    bool wasBeatDetected();             // Returns true once if beat was detected
};

#endif
