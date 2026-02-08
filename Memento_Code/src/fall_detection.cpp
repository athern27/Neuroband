#include "fall_detection.h"
#include <neuroband_inferencing.h>

float* FallDetection::static_buffer = nullptr;

FallDetection::FallDetection(Adafruit_LIS3DH* lis) {
    accelerometer = lis;
    buffer_index = 0;
    last_sample_time = 0;
    strcpy(last_activity, "unknown");
}

void FallDetection::begin() {
    Serial.println("Fall Detection initialized");
    Serial.print("Model frequency: ");
    Serial.print(EI_CLASSIFIER_FREQUENCY);
    Serial.println(" Hz");
    Serial.print("Inference window: ");
    Serial.print((float)EI_CLASSIFIER_RAW_SAMPLE_COUNT / EI_CLASSIFIER_FREQUENCY);
    Serial.println(" seconds");
    Serial.print("Classes: ");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        Serial.print(ei_classifier_inferencing_categories[ix]);
        if (ix < EI_CLASSIFIER_LABEL_COUNT - 1) Serial.print(", ");
    }
    Serial.println();
    
    static_buffer = buffer;
}

bool FallDetection::update() {
    unsigned long current_time = millis();
    
    if (current_time - last_sample_time >= FALL_SAMPLE_INTERVAL_MS) {
        last_sample_time = current_time;
        
        sensors_event_t event;
        accelerometer->getEvent(&event);
        
        if (buffer_index < FALL_BUFFER_SIZE - 2) {
            buffer[buffer_index++] = event.acceleration.x;
            buffer[buffer_index++] = event.acceleration.y;
            buffer[buffer_index++] = event.acceleration.z;
        }
        
        if (buffer_index >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            return true;
        }
    }
    
    return false;
}

void FallDetection::runInference() {
    if (buffer_index < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        Serial.println("Buffer not full, skipping inference");
        return;
    }
    
    signal_t signal;
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = &raw_feature_get_data;
    
    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
    
    if (res != EI_IMPULSE_OK) {
        Serial.print("ERR: Failed to run classifier (");
        Serial.print(res);
        Serial.println(")");
        buffer_index = 0;
        return;
    }
    
    Serial.println("Predictions:");
    float max_value = 0.0;
    int max_index = 0;
    
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        Serial.print("  ");
        Serial.print(result.classification[ix].label);
        Serial.print(": ");
        Serial.print(result.classification[ix].value, 4);
        Serial.println();
        
        if (result.classification[ix].value > max_value) {
            max_value = result.classification[ix].value;
            max_index = ix;
        }
    }
    
    strncpy(last_activity, result.classification[max_index].label, sizeof(last_activity) - 1);
    last_activity[sizeof(last_activity) - 1] = '\0';
    
    Serial.print("Detected: ");
    Serial.print(last_activity);
    Serial.print(" (");
    Serial.print(max_value * 100, 1);
    Serial.println("%)");
    Serial.println();
    
    buffer_index = 0;
}

const char* FallDetection::getLastActivity() {
    return last_activity;
}

int FallDetection::raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    if (static_buffer == nullptr) {
        return -1;
    }
    
    memcpy(out_ptr, static_buffer + offset, length * sizeof(float));
    return 0;
}