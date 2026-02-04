#ifndef FALL_DETECTION_H
#define FALL_DETECTION_H

#include <Arduino.h>
#include <Adafruit_LIS3DH.h>

#define FALL_BUFFER_SIZE 180
#define FALL_SAMPLE_INTERVAL_MS 50

class FallDetection {
private:
    Adafruit_LIS3DH* accelerometer;
    float buffer[FALL_BUFFER_SIZE];
    int buffer_index;
    unsigned long last_sample_time;
    
    static int raw_feature_get_data(size_t offset, size_t length, float *out_ptr);
    static float* static_buffer;
    
public:
    FallDetection(Adafruit_LIS3DH* lis);
    
    void begin();
    bool update();
    void runInference();
    const char* getLastActivity();
    
private:
private:
    char last_activity[16];
};

#endif