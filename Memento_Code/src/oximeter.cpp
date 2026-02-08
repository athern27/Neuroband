#include "oximeter.h"

Oximeter::Oximeter(MAX30105* max30105) {
    sensor = max30105;
    rateSpot = 0;
    lastBeat = 0;
    beatsPerMinute = 0;
    beatAvg = 0;
    lastBPM = 0.0f;
    smoothedBPM = 0.0f;
    lastAvg = 0;
    irBufIndex = 0;
    lastIR = 0;
    lastPeakTime = 0;
    risingEdge = false;
    validBeatCount = 0;
    beatDetected = false;
    
    for (byte i = 0; i < OXIMETER_RATE_SIZE; i++) {
        rates[i] = 0;
    }
    for (byte i = 0; i < IR_BUF_SIZE; i++) {
        irBuf[i] = 0;
    }
}

void Oximeter::begin() {
    byte ledBrightness = 60;
    byte sampleAverage = 4;
    byte ledMode = 2;
    int sampleRate = 100;
    int pulseWidth = 411;
    int adcRange = 4096;
    
    sensor->setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

void Oximeter::update() {
    long irValue = sensor->getIR();
    
    if (irValue > OXIMETER_IR_THRESHOLD) {
        bool beat = checkForBeat(irValue);

        irBuf[irBufIndex] = (unsigned long)irValue;
        irBufIndex = (irBufIndex + 1) % IR_BUF_SIZE;

        if (!beat) {
            unsigned long minIR = irBuf[0];
            unsigned long maxIR = irBuf[0];
            for (int i = 1; i < IR_BUF_SIZE; i++) {
                if (irBuf[i] < minIR) minIR = irBuf[i];
                if (irBuf[i] > maxIR) maxIR = irBuf[i];
            }
            
            unsigned long range = maxIR - minIR;

            if (range >= MIN_PEAK_RANGE) {
                unsigned long threshold = minIR + (range / 2);

                if ((unsigned long)irValue > lastIR) {
                    risingEdge = true;
                } else if (risingEdge && (unsigned long)irValue < lastIR) {
                    unsigned long now = millis();
                    if (lastIR > threshold && 
                        (lastPeakTime == 0 || (now - lastPeakTime) >= MIN_BEAT_INTERVAL_MS)) {
                        if (lastPeakTime > 0 && (now - lastPeakTime) < MAX_BEAT_INTERVAL_MS) {
                            beat = true;
                        }
                        lastPeakTime = now;
                    }
                    risingEdge = false;
                }
            }
        }
        lastIR = (unsigned long)irValue;
        
        if (beat) {
            unsigned long now = millis();
            if (lastBeat == 0) {
                lastBeat = now;
            } else {
                long delta = (long)(now - lastBeat);
                lastBeat = now;
                float rawBPM = 60000.0f / (float)delta;
                
                // Validate BPM is in reasonable range (40-180 BPM)
                if (rawBPM >= 40 && rawBPM <= 180) {
                    validBeatCount++;
                    
                    // For first few beats, initialize smoothed BPM
                    if (validBeatCount <= 3) {
                        smoothedBPM = rawBPM;
                        beatsPerMinute = rawBPM;
                    } else {
                        // Limit rate of change - prevent sudden jumps
                        float bpmDiff = rawBPM - smoothedBPM;
                        if (bpmDiff > MAX_BPM_CHANGE) {
                            rawBPM = smoothedBPM + MAX_BPM_CHANGE;
                        } else if (bpmDiff < -MAX_BPM_CHANGE) {
                            rawBPM = smoothedBPM - MAX_BPM_CHANGE;
                        }
                        
                        // Apply exponential smoothing
                        smoothedBPM = (BPM_SMOOTHING_FACTOR * rawBPM) + ((1.0f - BPM_SMOOTHING_FACTOR) * smoothedBPM);
                        beatsPerMinute = smoothedBPM;
                    }
                    
                    // Store in rates array for averaging
                    rates[rateSpot++] = (byte)beatsPerMinute;
                    rateSpot %= OXIMETER_RATE_SIZE;
                    
                    // Calculate average from all stored rates
                    int sum = 0;
                    int count = 0;
                    for (byte i = 0; i < OXIMETER_RATE_SIZE; i++) {
                        if (rates[i] > 0) {
                            sum += rates[i];
                            count++;
                        }
                    }
                    if (count > 0) {
                        beatAvg = sum / count;
                    }
                    
                    lastBPM = beatsPerMinute;
                    lastAvg = beatAvg;
                    beatDetected = true;  // Signal that a beat was detected
                }
            }
        }
        
        Serial.print("Oximeter - IR: ");
        Serial.print(irValue);
        Serial.print(", BPM: ");
        Serial.println(lastBPM, 1);
    } else {
        // No finger - reset state for fresh readings when finger returns
        validBeatCount = 0;
        lastBeat = 0;
        lastPeakTime = 0;
        smoothedBPM = 0.0f;
        Serial.println("Oximeter - No finger detected");
    }
}

bool Oximeter::wasBeatDetected() {
    if (beatDetected) {
        beatDetected = false;
        return true;
    }
    return false;
}

float Oximeter::getBPM() {
    return beatsPerMinute;
}

int Oximeter::getAvgBPM() {
    return beatAvg;
}

long Oximeter::getIR() {
    return sensor->getIR();
}

bool Oximeter::isFingerDetected() {
    return sensor->getIR() > OXIMETER_IR_THRESHOLD;
}
