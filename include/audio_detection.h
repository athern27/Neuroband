#ifndef AUDIO_DETECTION_H
#define AUDIO_DETECTION_H

#include <Arduino.h>

#define MIC_PIN 2
#define AUDIO_SAMPLE_RATE 16000

class AudioDetection {
private:
    typedef struct {
        int16_t *buffer;
        uint8_t buf_ready;
        uint32_t buf_count;
        uint32_t n_samples;
    } inference_t;
    
    inference_t inference;
    bool record_status;
    unsigned long last_sample_us;
    String lastDetection;
    float lastConfidence;
    
    static AudioDetection* instance;
    static void capture_samples_wrapper(void* arg);
    void capture_samples();
    int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);
    bool microphone_inference_start(uint32_t n_samples);
    bool microphone_inference_record();
    void microphone_inference_end();
    
public:
    AudioDetection();
    
    void begin();
    void update();
    String getLastDetection();
    float getLastConfidence();
};

#endif
