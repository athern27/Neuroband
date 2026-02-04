#ifndef ECG_H
#define ECG_H

#include <Arduino.h>

#define ECG_ANALOG_PIN A0
#define ECG_ADC_RESOLUTION 12
#define ECG_REFERENCE_VOLTAGE 3.3
#define ECG_SAMPLE_RATE 125  // Hz

class ECG {
private:
    int pin;
    int rawValue;
    float voltage;
    float filteredValue;
    
    // Timing for sample rate control
    unsigned long lastSampleTime;
    unsigned long sampleInterval;  // microseconds
    
    // Filter state variables (4 biquad sections)
    float z1_1, z2_1;
    float z1_2, z2_2;
    float z1_3, z2_3;
    float z1_4, z2_4;
    
    // Band-pass Butterworth IIR filter
    float applyFilter(float input);
    
public:
    ECG(int analogPin = ECG_ANALOG_PIN);
    
    void begin();
    bool update();  // Returns true when new sample is ready
    int getRawValue();
    float getVoltage();
    float getFilteredValue();
};

#endif
