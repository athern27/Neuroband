// ECG Module with BioAmp EXG Pill Filter
// Based on: https://github.com/upsidedownlabs/BioAmp-EXG-Pill
// Band-Pass Butterworth IIR digital filter
// Sampling rate: 125.0 Hz, frequency: [0.5, 44.5] Hz

#include "ecg.h"

ECG::ECG(int analogPin) {
    pin = analogPin;
    rawValue = 0;
    voltage = 0.0;
    filteredValue = 0.0;
    lastSampleTime = 0;
    sampleInterval = 1000000 / ECG_SAMPLE_RATE;  // microseconds
    
    // Initialize filter state
    z1_1 = z2_1 = 0.0;
    z1_2 = z2_2 = 0.0;
    z1_3 = z2_3 = 0.0;
    z1_4 = z2_4 = 0.0;
}

void ECG::begin() {
    analogReadResolution(ECG_ADC_RESOLUTION);
    analogSetAttenuation(ADC_11db);
    lastSampleTime = micros();
    Serial.println("ECG initialized with BioAmp filter (125 Hz, 0.5-44.5 Hz bandpass)");
}

// Band-Pass Butterworth IIR digital filter, order 4
// Implemented as second-order sections (biquads)
float ECG::applyFilter(float input) {
    float output = input;
    
    // Section 1
    {
        float x = output - 0.70682283f * z1_1 - 0.15621030f * z2_1;
        output = 0.28064917f * x + 0.56129834f * z1_1 + 0.28064917f * z2_1;
        z2_1 = z1_1;
        z1_1 = x;
    }
    
    // Section 2
    {
        float x = output - 0.95028224f * z1_2 - 0.54073140f * z2_2;
        output = 1.00000000f * x + 2.00000000f * z1_2 + 1.00000000f * z2_2;
        z2_2 = z1_2;
        z1_2 = x;
    }
    
    // Section 3
    {
        float x = output - (-1.95360385f) * z1_3 - 0.95423412f * z2_3;
        output = 1.00000000f * x + (-2.00000000f) * z1_3 + 1.00000000f * z2_3;
        z2_3 = z1_3;
        z1_3 = x;
    }
    
    // Section 4
    {
        float x = output - (-1.98048558f) * z1_4 - 0.98111344f * z2_4;
        output = 1.00000000f * x + (-2.00000000f) * z1_4 + 1.00000000f * z2_4;
        z2_4 = z1_4;
        z1_4 = x;
    }
    
    return output;
}

bool ECG::update() {
    unsigned long currentTime = micros();
    unsigned long elapsed = currentTime - lastSampleTime;
    
    // Check if it's time for a new sample
    if (elapsed >= sampleInterval) {
        lastSampleTime = currentTime;
        
        // Read raw ADC value
        rawValue = analogRead(pin);
        voltage = (rawValue / 4095.0f) * ECG_REFERENCE_VOLTAGE;
        
        // Apply bandpass filter
        filteredValue = applyFilter((float)rawValue);
        
        return true;  // New sample ready
    }
    
    return false;  // No new sample yet
}

int ECG::getRawValue() {
    return rawValue;
}

float ECG::getVoltage() {
    return voltage;
}

float ECG::getFilteredValue() {
    return filteredValue;
}
