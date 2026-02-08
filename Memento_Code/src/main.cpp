#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <MAX30105.h>
#include "fall_detection.h"
#include "uart.h"
#include "oled.h"
#include "oximeter.h"
#include "ecg.h"
#include "wifi_helper.h"
#include "aw9523.h"

// ==================== WiFi & MQTT Configuration ====================
const char* ssid = "TP-Link_6391";
const char* password = "AKMR@159";
const char* mqtt_server = "192.168.1.130";  // Your MQTT broker IP

// MQTT Topics
const char* TOPIC_MOVEMENT_STATUS = "esp32/movement_status";
const char* TOPIC_BLOOD_PRESSURE = "esp32/blood_pressure_value";
const char* TOPIC_ASK_ECG = "esp32/ask_ecg_data";
const char* TOPIC_RECEIVE_ECG = "esp32/recieve_ecg_data";
const char* TOPIC_HELP_STATUS = "esp32/help_status";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ==================== AW9523 Button & Speaker Pin Definitions ====================
// CONFIRMED button mappings from hardware testing:
// Port0: SELECT = bit 1 (P0_1)
// Port1: UP=bit5, DOWN=bit7, LEFT=bit6, RIGHT=bit4, OK=bit3
#define AW9523_PIN_SPEAKER_EN   0   // P0_0 - Speaker enable
#define AW9523_PIN_BTN_UP       13  // P1_5 (bit 5 of Port1)
#define AW9523_PIN_BTN_DOWN     15  // P1_7 (bit 7 of Port1)
#define AW9523_PIN_BTN_LEFT     14  // P1_6 (bit 6 of Port1)
#define AW9523_PIN_BTN_RIGHT    12  // P1_4 (bit 4 of Port1)
#define AW9523_PIN_BTN_SELECT   1   // P0_1 (bit 1 of Port0)
#define AW9523_PIN_BTN_OK       11  // P1_3 (bit 3 of Port1)
#define OK_BUTTON_PIN           11  // ESP32 GPIO for OK button (stops SOS)
#define SPEAKER_PIN             46  // ESP32 GPIO for speaker PWM

// Speaker PWM settings
const int PWM_CHANNEL = 0;
const int PWM_RES = 8;
const int TONE_FREQ = 1000;

// SOS timing (in ms)
const int DOT_DURATION = 150;
const int DASH_DURATION = 450;
const int SYMBOL_GAP = 150;
const int LETTER_GAP = 450;

AW9523 gpioExpander(AW9523_DEFAULT_ADDR);
bool aw9523Available = false;

// ==================== ECG Circular Buffer ====================
// 20 seconds of data at 125 Hz = 2500 samples
#define ECG_BUFFER_SIZE 2500
float ecgBuffer[ECG_BUFFER_SIZE];
volatile int ecgBufferHead = 0;
volatile int ecgBufferCount = 0;
volatile bool ecgDataRequested = false;
SemaphoreHandle_t ecgBufferMutex;

// ==================== Help Detection State ====================
// Thresholds and timing
#define LOW_BPM_THRESHOLD 60
#define LOW_BPM_DURATION_MS 20000  // 20 seconds
#define CONSECUTIVE_FALL_THRESHOLD 5

// State variables (protected by helpStateMutex)
volatile bool helpActive = false;
volatile bool sosPlaying = false;
volatile unsigned long lowBpmStartTime = 0;
volatile bool lowBpmConditionMet = false;
volatile int consecutiveFallCount = 0;
volatile bool okButtonPressed = false;

// Synchronization primitives
SemaphoreHandle_t helpStateMutex;         // Mutex for help state variables
SemaphoreHandle_t sosStopSemaphore;       // Binary semaphore to stop SOS
SemaphoreHandle_t i2cMutex;               // Mutex for I2C bus access

// FreeRTOS Software Timers
TimerHandle_t lowBpmTimer;                // 20-second one-shot timer for low BPM condition
TimerHandle_t mqttReconnectTimer;         // 5-second periodic timer for MQTT reconnection
TimerHandle_t fallResetTimer;             // Timer to reset fall counter after inactivity

#define I2C_SDA 34
#define I2C_SCL 33

Adafruit_LIS3DH lis = Adafruit_LIS3DH();
MAX30105 particleSensor;
FallDetection fallDetector(&lis);
Oximeter oximeter(&particleSensor);
ECG ecg;
UART uart;
OLED oled;
WiFiHelper wifi(ssid, password);

volatile float g_beatsPerMinute = 0.0;
volatile int g_beatAvg = 0;
volatile float g_ecgFiltered = 0.0;

TaskHandle_t ECG_Task_Handle;
TaskHandle_t Oximeter_Task_Handle;
TaskHandle_t Accelerometer_Task_Handle;
TaskHandle_t Display_Task_Handle;
TaskHandle_t UART_Debug_Task_Handle;
TaskHandle_t MQTT_Send_Task_Handle;
TaskHandle_t MQTT_ECG_Task_Handle;
TaskHandle_t Button_Task_Handle;
TaskHandle_t SOS_Task_Handle;
TaskHandle_t Help_Monitor_Task_Handle;

void ECG_Analog_Task(void *pvParameters);
void Oximeter_I2C_Task(void *pvParameters);
void Accelerometer_Task(void *pvParameters);
void Display_Task(void *pvParameters);
void UART_Debug_Task(void *pvParameters);
void Send_To_NodeRed_Task(void *pvParameters);
void Receive_And_Send_ECG_Task(void *pvParameters);
void Button_Task(void *pvParameters);
void SOS_Task(void *pvParameters);
void Help_Monitor_Task(void *pvParameters);
void initSensors();
void initDisplay();
void initMQTT();
void initButtons();
void initSpeaker();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void pushECGSample(float value);
String getECGBufferAsString();
void triggerHelp(const char* reason);
void stopHelp();
bool checkHelpButton();
void playSOS();

// Timer callback declarations
void lowBpmTimerCallback(TimerHandle_t xTimer);
void mqttReconnectTimerCallback(TimerHandle_t xTimer);
void fallResetTimerCallback(TimerHandle_t xTimer);

void setup() {
  uart.begin(115200);
  delay(1000);

  uart.printHeader("Neuroband ESP32-S3 FreeRTOS Project");

  Wire.begin(I2C_SDA, I2C_SCL);

  // Create synchronization primitives
  ecgBufferMutex = xSemaphoreCreateMutex();
  helpStateMutex = xSemaphoreCreateMutex();
  sosStopSemaphore = xSemaphoreCreateBinary();
  i2cMutex = xSemaphoreCreateMutex();
  
  // Create software timers
  lowBpmTimer = xTimerCreate(
    "LowBpmTimer",
    pdMS_TO_TICKS(LOW_BPM_DURATION_MS),  // 20 seconds
    pdFALSE,                              // One-shot
    NULL,
    lowBpmTimerCallback
  );
  
  mqttReconnectTimer = xTimerCreate(
    "MqttReconnect",
    pdMS_TO_TICKS(5000),                  // 5 seconds
    pdTRUE,                               // Auto-reload (periodic)
    NULL,
    mqttReconnectTimerCallback
  );
  
  fallResetTimer = xTimerCreate(
    "FallReset",
    pdMS_TO_TICKS(3000),                  // 3 seconds to reset fall counter
    pdFALSE,                              // One-shot
    NULL,
    fallResetTimerCallback
  );
  
  // Initialize ECG buffer with zeros
  memset(ecgBuffer, 0, sizeof(ecgBuffer));

  initDisplay();
  initButtons();
  initSpeaker();
  wifi.begin();
  initMQTT();
  initSensors();

  xTaskCreatePinnedToCore(
    ECG_Analog_Task,
    "ECG_Analog",
    4096,
    NULL,
    2,
    &ECG_Task_Handle,
    0
  );

  xTaskCreatePinnedToCore(
    Oximeter_I2C_Task,
    "Oximeter_I2C",
    4096,
    NULL,
    2,
    &Oximeter_Task_Handle,
    0
  );

  xTaskCreatePinnedToCore(
    Accelerometer_Task,
    "Accelerometer",
    8192,  // Increased stack for Fall Detecton
    NULL,
    1,
    &Accelerometer_Task_Handle,
    1
  );

  xTaskCreatePinnedToCore(
    Display_Task,
    "Display",
    4096,
    NULL,
    1,
    &Display_Task_Handle,
    0
  );

  xTaskCreatePinnedToCore(
    UART_Debug_Task,
    "UART_Debug",
    4096,
    NULL,
    1,
    &UART_Debug_Task_Handle,
    0
  );

  xTaskCreatePinnedToCore(
    Send_To_NodeRed_Task,
    "MQTT_Send",
    4096,
    NULL,
    1,
    &MQTT_Send_Task_Handle,
    0
  );

  xTaskCreatePinnedToCore(
    Receive_And_Send_ECG_Task,
    "MQTT_ECG",
    8192,  // Larger stack for ECG string building
    NULL,
    1,
    &MQTT_ECG_Task_Handle,
    0
  );

  xTaskCreatePinnedToCore(
    Button_Task,
    "Button",
    4096,
    NULL,
    2,  // Higher priority for responsive button handling
    &Button_Task_Handle,
    1
  );

  xTaskCreatePinnedToCore(
    SOS_Task,
    "SOS",
    4096,
    NULL,
    1,
    &SOS_Task_Handle,
    1
  );

  xTaskCreatePinnedToCore(
    Help_Monitor_Task,
    "HelpMonitor",
    4096,
    NULL,
    1,
    &Help_Monitor_Task_Handle,
    0
  );

  Serial.println("All FreeRTOS tasks created successfully");
  Serial.println("Task scheduling started...");
}

void loop() {
  // Keep MQTT connection alive
  if (mqttClient.connected()) {
    mqttClient.loop();
  }
  
  // Yield to other tasks and prevent watchdog issues
  vTaskDelay(pdMS_TO_TICKS(50));
}

// ==================== MQTT Functions ====================

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT received [");
  Serial.print(topic);
  Serial.print("]: ");
  
  // Convert payload to string
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  Serial.println(message);
  
  // Check if it's the ECG data request
  if (strcmp(topic, TOPIC_ASK_ECG) == 0) {
    if (message[0] == '1') {
      Serial.println("ECG data requested!");
      ecgDataRequested = true;
    }
  }
}

void initMQTT() {
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(25000);  // Large buffer for ECG data (2500 samples * ~8 chars + commas)
  Serial.println("MQTT initialized");
}

void reconnectMQTT() {
  if (!mqttClient.connected() && WiFi.status() == WL_CONNECTED) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("NeurobandESP32")) {
      Serial.println("connected");
      // Subscribe to ECG request topic
      mqttClient.subscribe(TOPIC_ASK_ECG);
      Serial.println("Subscribed to: esp32/ask_ecg_data");
    } else {
      Serial.print("failed, rc=");
      Serial.println(mqttClient.state());
    }
  }
}

// ==================== ECG Buffer Functions ====================

void pushECGSample(float value) {
  if (xSemaphoreTake(ecgBufferMutex, portMAX_DELAY) == pdTRUE) {
    ecgBuffer[ecgBufferHead] = value;
    ecgBufferHead = (ecgBufferHead + 1) % ECG_BUFFER_SIZE;
    if (ecgBufferCount < ECG_BUFFER_SIZE) {
      ecgBufferCount++;
    }
    xSemaphoreGive(ecgBufferMutex);
  }
}

String getECGBufferAsString() {
  String result = "";
  
  if (xSemaphoreTake(ecgBufferMutex, portMAX_DELAY) == pdTRUE) {
    int count = ecgBufferCount;
    int startIdx = (ecgBufferHead - count + ECG_BUFFER_SIZE) % ECG_BUFFER_SIZE;
    
    for (int i = 0; i < count; i++) {
      int idx = (startIdx + i) % ECG_BUFFER_SIZE;
      if (i > 0) {
        result += ",";
      }
      result += String(ecgBuffer[idx], 4);  // 4 decimal places
    }
    xSemaphoreGive(ecgBufferMutex);
  }
  
  return result;
}

// ==================== Timer Callbacks ====================

void lowBpmTimerCallback(TimerHandle_t xTimer) {
  // Timer fired - 20 seconds of low BPM has passed
  if (xSemaphoreTake(helpStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (!lowBpmConditionMet) {
      lowBpmConditionMet = true;
      Serial.println("[Timer] Low BPM condition met - waiting for button press");
    }
    xSemaphoreGive(helpStateMutex);
  }
}

void mqttReconnectTimerCallback(TimerHandle_t xTimer) {
  // Attempt MQTT reconnection if disconnected
  if (!mqttClient.connected() && WiFi.status() == WL_CONNECTED) {
    Serial.println("[Timer] Attempting MQTT reconnection...");
    reconnectMQTT();
  }
}

void fallResetTimerCallback(TimerHandle_t xTimer) {
  // Reset fall counter after timeout
  if (xSemaphoreTake(helpStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (consecutiveFallCount > 0) {
      Serial.printf("[Timer] Fall counter reset (was %d)\n", consecutiveFallCount);
      consecutiveFallCount = 0;
    }
    xSemaphoreGive(helpStateMutex);
  }
}



void initSensors() {
  Serial.println("\nInitializing sensors...");

  if (!lis.begin(0x19)) {
    Serial.println("ERROR: LIS3DH accelerometer not found!");
    Serial.println("Check wiring and I2C address");
  } else {
    Serial.println("LIS3DH accelerometer initialized");
    lis.setRange(LIS3DH_RANGE_4_G);
    lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  }

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR: MAX30105 sensor not found!");
    Serial.println("Check wiring and I2C connection");
  } else {
    Serial.println("MAX30105 sensor initialized");
    oximeter.begin();
  }

  ecg.begin();

  Serial.println("All sensors initialized\n");
  
  // Initialize fall detection
  fallDetector.begin();
}

void initDisplay()
{
  oled.begin();
  oled.showInitScreen("Neuroband");
}

void ECG_Analog_Task(void *pvParameters) {
  Serial.println("ECG_Analog Task started on Core 0 (125 Hz sampling)");

  while(1) {
    // update() returns true when a new sample is ready (at 125 Hz)
    if (ecg.update()) {
      g_ecgFiltered = ecg.getFilteredValue();
      
      // Push to circular buffer for MQTT transmission
      pushECGSample(g_ecgFiltered);
    }
    
    // Small delay to prevent watchdog issues, actual timing controlled by ECG class
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void Oximeter_I2C_Task(void *pvParameters) {
  Serial.println("Oximeter_I2C Task started on Core 0");

  const TickType_t xDelay = pdMS_TO_TICKS(20);

  while(1) {
    // Protect I2C access
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      oximeter.update();
      xSemaphoreGive(i2cMutex);
    }
    
    g_beatsPerMinute = oximeter.getBPM();
    g_beatAvg = oximeter.getAvgBPM();
    
    // Trigger heart animation on beat detection
    if (oximeter.wasBeatDetected()) {
      oled.triggerHeartBeat();
    }

    vTaskDelay(xDelay);
  }
}

void Display_Task(void *pvParameters)
{
  const TickType_t delayTicks = pdMS_TO_TICKS(100);  // Check every 100ms
  
  // Debounce state for finger detection
  static bool displayFingerDetected = false;
  static unsigned long lastFingerTime = 0;
  static unsigned long noFingerStartTime = 0;
  const unsigned long NO_FINGER_DELAY_MS = 500;  // Wait 500ms before showing "no finger"
  
  while (1) {
    long irValue = oximeter.getIR();
    bool currentFingerDetected = (irValue > 50000);
    
    if (currentFingerDetected) {
      // Finger detected - update immediately
      displayFingerDetected = true;
      lastFingerTime = millis();
      noFingerStartTime = 0;
    } else {
      // No finger - only update display after delay
      if (displayFingerDetected) {
        // Was showing finger, start timeout
        if (noFingerStartTime == 0) {
          noFingerStartTime = millis();
        } else if ((millis() - noFingerStartTime) >= NO_FINGER_DELAY_MS) {
          // Timeout expired, now show "no finger"
          displayFingerDetected = false;
        }
        // Keep showing last reading while waiting
      }
    }
    
    oled.updateStatus(g_beatsPerMinute, g_beatAvg, irValue, displayFingerDetected);
    vTaskDelay(delayTicks);
  }
}

void UART_Debug_Task(void *pvParameters)
{
  const TickType_t delayTicks = pdMS_TO_TICKS(1000);

  while (1) {
    uart.printTelemetry(g_beatsPerMinute, g_beatAvg);
    vTaskDelay(delayTicks);
  }
}

void Accelerometer_Task(void *pvParameters) {
  Serial.println("Accelerometer Task started on Core 1");

  while(1) {
    // Protect I2C access
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (fallDetector.update()) {
        fallDetector.runInference();
      }
      xSemaphoreGive(i2cMutex);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==================== MQTT Tasks ====================

void Send_To_NodeRed_Task(void *pvParameters) {
  Serial.println("Send_To_NodeRed Task started");
  
  const TickType_t delayTicks = pdMS_TO_TICKS(1000);  // Send every 1 second
  
  while(1) {
    // Reconnect MQTT if needed (once per loop, not spamming)
    if (!mqttClient.connected() && WiFi.status() == WL_CONNECTED) {
      reconnectMQTT();
    }
    
    if (mqttClient.connected()) {
      // Publish movement status (fall detection)
      const char* movementStatus = fallDetector.getLastActivity();
      mqttClient.publish(TOPIC_MOVEMENT_STATUS, movementStatus);
      
      // Publish blood pressure / BPM value
      char bpmString[16];
      snprintf(bpmString, sizeof(bpmString), "%.1f", g_beatsPerMinute);
      mqttClient.publish(TOPIC_BLOOD_PRESSURE, bpmString);
      
      // Publish help status
      if (helpActive) {
        mqttClient.publish(TOPIC_HELP_STATUS, "HELP_ACTIVE");
      } else {
        mqttClient.publish(TOPIC_HELP_STATUS, "HELP_OK");
      }
      
      Serial.printf("MQTT Published - Movement: %s, BPM: %s\n", movementStatus, bpmString);
    }
    
    vTaskDelay(delayTicks);
  }
}

void Receive_And_Send_ECG_Task(void *pvParameters) {
  Serial.println("Receive_And_Send_ECG Task started");
  
  const TickType_t delayTicks = pdMS_TO_TICKS(50);  // Check every 50ms for faster response
  
  while(1) {
    // Check if ECG data was requested
    if (ecgDataRequested) {
      // Clear flag immediately
      ecgDataRequested = false;
      
      Serial.println("[ECG] Data request received, waiting for MQTT connection...");
      
      // Wait for MQTT connection (with timeout)
      int waitCount = 0;
      while (!mqttClient.connected() && waitCount < 50) {  // 2.5 second timeout
        vTaskDelay(pdMS_TO_TICKS(50));
        waitCount++;
      }
      
      if (mqttClient.connected()) {
        Serial.println("[ECG] Building ECG data string...");
        
        // Get ECG buffer as comma-separated string
        String ecgData = getECGBufferAsString();
        
        if (ecgData.length() > 0) {
          Serial.printf("[ECG] Sending %d bytes of ECG data (%d samples)\n", 
                        ecgData.length(), ecgBufferCount);
          
          // Ensure MQTT loop is run before publishing
          mqttClient.loop();
          vTaskDelay(pdMS_TO_TICKS(10));
          
          // Publish ECG data
          if (mqttClient.publish(TOPIC_RECEIVE_ECG, ecgData.c_str(), true)) {
            Serial.println("[ECG] Data sent successfully");
          } else {
            Serial.println("[ECG] Failed to send data - buffer may be too small");
            // Try sending a shorter message
            mqttClient.publish(TOPIC_RECEIVE_ECG, "ERROR:DATA_TOO_LARGE");
          }
        } else {
          mqttClient.publish(TOPIC_RECEIVE_ECG, "NO_DATA");
          Serial.println("[ECG] No ECG data available");
        }
      } else {
        Serial.println("[ECG] MQTT not connected, cannot send data");
      }
    }
    
    vTaskDelay(delayTicks);
  }
}

// ==================== Button & Speaker Initialization ====================

void initButtons() {
  // Small delay to let I2C bus settle after sensor initialization
  delay(100);
  
  // Initialize AW9523 for buttons with retry
  bool aw9523Found = false;
  for (int attempt = 0; attempt < 3 && !aw9523Found; attempt++) {
    if (attempt > 0) {
      Serial.printf("AW9523: Retry attempt %d...\n", attempt + 1);
      delay(100);
    }
    aw9523Found = gpioExpander.begin(&Wire);
  }
  
  if (aw9523Found) {
    aw9523Available = true;
    
    // Set GCR register for push-pull output on Port 0
    gpioExpander.writeRegister(0x11, 0x10);
    
    // Configure speaker enable as output
    gpioExpander.pinMode(AW9523_PIN_SPEAKER_EN, OUTPUT);
    gpioExpander.setGPIOMode(AW9523_PIN_SPEAKER_EN);
    
    // Configure buttons as inputs (they have external pull-ups)
    gpioExpander.pinMode(AW9523_PIN_BTN_UP, INPUT);
    gpioExpander.pinMode(AW9523_PIN_BTN_DOWN, INPUT);
    gpioExpander.pinMode(AW9523_PIN_BTN_LEFT, INPUT);
    gpioExpander.pinMode(AW9523_PIN_BTN_RIGHT, INPUT);
    gpioExpander.pinMode(AW9523_PIN_BTN_SELECT, INPUT);
    gpioExpander.setGPIOMode(AW9523_PIN_BTN_UP);
    gpioExpander.setGPIOMode(AW9523_PIN_BTN_DOWN);
    gpioExpander.setGPIOMode(AW9523_PIN_BTN_LEFT);
    gpioExpander.setGPIOMode(AW9523_PIN_BTN_RIGHT);
    gpioExpander.setGPIOMode(AW9523_PIN_BTN_SELECT);
    
    Serial.println("AW9523 initialized - buttons and speaker control available");
  } else {
    aw9523Available = false;
    Serial.println("WARNING: AW9523 not found - button/speaker control unavailable");
    Serial.println("         (SOS will still work via OK button on GPIO 11)");
  }
  
  // OK button on ESP32 GPIO
  pinMode(OK_BUTTON_PIN, INPUT_PULLUP);
}

void initSpeaker() {
  ledcSetup(PWM_CHANNEL, TONE_FREQ, PWM_RES);
  ledcAttachPin(SPEAKER_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);  // Start silent
  
  // Keep speaker disabled until needed (SOS)
  if (aw9523Available) {
    gpioExpander.digitalWrite(AW9523_PIN_SPEAKER_EN, LOW);
  }
  Serial.println("Speaker initialized (disabled until SOS)");
}

// ==================== Help Detection Functions ====================

// Check if any help button (up, down, left, right) is pressed
bool checkHelpButton() {
  if (!aw9523Available) return false;
  
  uint8_t up = HIGH, down = HIGH, left = HIGH, right = HIGH;
  
  // Protect I2C access with mutex
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    up = gpioExpander.digitalRead(AW9523_PIN_BTN_UP);
    down = gpioExpander.digitalRead(AW9523_PIN_BTN_DOWN);
    left = gpioExpander.digitalRead(AW9523_PIN_BTN_LEFT);
    right = gpioExpander.digitalRead(AW9523_PIN_BTN_RIGHT);
    xSemaphoreGive(i2cMutex);
  } else {
    return false;  // Couldn't get mutex, skip this read
  }
  
  // Debug: print button states every 2 seconds, or immediately if any pressed
  static unsigned long lastDebug = 0;
  bool anyPressed = (up == LOW || down == LOW || left == LOW || right == LOW);
  
  if (anyPressed || (millis() - lastDebug > 2000)) {
    lastDebug = millis();
    Serial.printf("[BTN] UP=%d DOWN=%d LEFT=%d RIGHT=%d (0=pressed)%s\n", 
                  up, down, left, right,
                  anyPressed ? " <-- PRESSED" : "");
  }
  
  return anyPressed;
}

// Check if Select button is pressed (to stop SOS)
bool checkSelectButton() {
  if (!aw9523Available) return false;
  
  uint8_t select = HIGH;
  
  // Protect I2C access with mutex
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    select = gpioExpander.digitalRead(AW9523_PIN_BTN_SELECT);
    xSemaphoreGive(i2cMutex);
  } else {
    return false;  // Couldn't get mutex, skip this read
  }
  
  // Debug: print select state periodically  
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 2000) {
    lastDebug = millis();
    Serial.printf("[BTN] SELECT=%d (0=pressed)\n", select);
  }
  
  // Select button is active LOW
  return (select == LOW);
}

// Trigger help mode
void triggerHelp(const char* reason) {
  if (xSemaphoreTake(helpStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (!helpActive) {
      helpActive = true;
      sosPlaying = true;
      
      Serial.printf("!!! HELP TRIGGERED: %s !!!\n", reason);
      
      // Publish help status to MQTT
      if (mqttClient.connected()) {
        char helpMsg[64];
        snprintf(helpMsg, sizeof(helpMsg), "HELP:%s", reason);
        mqttClient.publish(TOPIC_HELP_STATUS, helpMsg);
      }
      
      // Auto-send ECG data
      ecgDataRequested = true;
    }
    xSemaphoreGive(helpStateMutex);
  }
}

// Stop help mode (when OK or Select is pressed)
void stopHelp() {
  if (xSemaphoreTake(helpStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (helpActive) {
      helpActive = false;
      sosPlaying = false;
      
      // Signal SOS task to stop
      xSemaphoreGive(sosStopSemaphore);
      
      // Disable speaker
      if (aw9523Available) {
        gpioExpander.digitalWrite(AW9523_PIN_SPEAKER_EN, LOW);
      }
      ledcWrite(PWM_CHANNEL, 0);  // Ensure PWM is off
      
      // Stop timers and reset counters
      xTimerStop(lowBpmTimer, 0);
      xTimerStop(fallResetTimer, 0);
      consecutiveFallCount = 0;
      lowBpmConditionMet = false;
      lowBpmStartTime = 0;
      
      Serial.println("Help mode deactivated - OK/Select button pressed");
      
      if (mqttClient.connected()) {
        mqttClient.publish(TOPIC_HELP_STATUS, "HELP_CANCELLED");
      }
    }
    xSemaphoreGive(helpStateMutex);
  }
}

// Play SOS pattern (blocking, but checks for stop signal)
void playSOS() {
  if (aw9523Available) {
    gpioExpander.digitalWrite(AW9523_PIN_SPEAKER_EN, HIGH);
  }
  
  // S: . . .
  for (int i = 0; i < 3 && sosPlaying; i++) {
    ledcSetup(PWM_CHANNEL, TONE_FREQ, PWM_RES);
    ledcWrite(PWM_CHANNEL, 128);
    vTaskDelay(pdMS_TO_TICKS(DOT_DURATION));
    ledcWrite(PWM_CHANNEL, 0);
    if (i < 2) vTaskDelay(pdMS_TO_TICKS(SYMBOL_GAP));
  }
  
  if (!sosPlaying) return;
  vTaskDelay(pdMS_TO_TICKS(LETTER_GAP));
  
  // O: - - -
  for (int i = 0; i < 3 && sosPlaying; i++) {
    ledcSetup(PWM_CHANNEL, TONE_FREQ, PWM_RES);
    ledcWrite(PWM_CHANNEL, 128);
    vTaskDelay(pdMS_TO_TICKS(DASH_DURATION));
    ledcWrite(PWM_CHANNEL, 0);
    if (i < 2) vTaskDelay(pdMS_TO_TICKS(SYMBOL_GAP));
  }
  
  if (!sosPlaying) return;
  vTaskDelay(pdMS_TO_TICKS(LETTER_GAP));
  
  // S: . . .
  for (int i = 0; i < 3 && sosPlaying; i++) {
    ledcSetup(PWM_CHANNEL, TONE_FREQ, PWM_RES);
    ledcWrite(PWM_CHANNEL, 128);
    vTaskDelay(pdMS_TO_TICKS(DOT_DURATION));
    ledcWrite(PWM_CHANNEL, 0);
    if (i < 2) vTaskDelay(pdMS_TO_TICKS(SYMBOL_GAP));
  }
}

// ==================== New Tasks ====================

void Button_Task(void *pvParameters) {
  Serial.println("Button Task started");
  
  // Wait for system and AW9523 to fully stabilize
  vTaskDelay(pdMS_TO_TICKS(3000));
  
  // Read and discard initial button states to avoid startup glitches
  if (aw9523Available) {
    for (int i = 0; i < 10; i++) {
      gpioExpander.digitalRead(AW9523_PIN_BTN_UP);
      gpioExpander.digitalRead(AW9523_PIN_BTN_DOWN);
      gpioExpander.digitalRead(AW9523_PIN_BTN_LEFT);
      gpioExpander.digitalRead(AW9523_PIN_BTN_RIGHT);
      gpioExpander.digitalRead(AW9523_PIN_BTN_SELECT);
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
  
  Serial.println("Button Task active - buttons now monitored");
  
  const TickType_t delayTicks = pdMS_TO_TICKS(50);  // 50ms polling
  
  static uint8_t lastOkState = HIGH;
  static unsigned long okPressStartTime = 0;
  static bool okLongPressTriggered = false;
  static uint8_t lastSelectState = HIGH;
  static unsigned long lastSelectDebounce = 0;
  static unsigned long lastHelpButtonPress = 0;
  static bool helpButtonWasPressed = false;
  static int helpButtonDebounceCount = 0;  // Require multiple consecutive reads
  
  const unsigned long OK_LONG_PRESS_MS = 3000;  // 3 seconds to trigger SOS via OK button
  
  while(1) {
    // Check OK button - short press stops SOS, long press (3s) starts SOS
    int okReading = digitalRead(OK_BUTTON_PIN);
    
    if (okReading == LOW) {
      // Button is pressed
      if (lastOkState == HIGH) {
        // Just pressed
        okPressStartTime = millis();
        okLongPressTriggered = false;
      } else {
        // Still held - check for long press
        if (!okLongPressTriggered && (millis() - okPressStartTime) >= OK_LONG_PRESS_MS) {
          okLongPressTriggered = true;
          // Long press - trigger SOS if not already active
          if (!helpActive) {
            Serial.println("OK Button long press (3s) - triggering SOS!");
            triggerHelp("OK_LONG_PRESS");
          }
        }
      }
    } else {
      // Button released
      if (lastOkState == LOW && !okLongPressTriggered) {
        // Short press - stop SOS if active
        if (helpActive) {
          Serial.println("OK Button short press - stopping SOS");
          stopHelp();
        }
      }
      okLongPressTriggered = false;
    }
    lastOkState = okReading;
    
    // Check Select button for stopping help/SOS
    bool selectReading = checkSelectButton();
    uint8_t selectState = selectReading ? LOW : HIGH;
    if (selectState != lastSelectState) {
      lastSelectDebounce = millis();
    }
    if ((millis() - lastSelectDebounce) > 50) {
      static uint8_t stableSelectState = HIGH;
      if (selectState != stableSelectState) {
        stableSelectState = selectState;
        if (stableSelectState == LOW) {
          Serial.println("Select Button pressed - stopping SOS");
          if (helpActive) {
            stopHelp();
          }
        }
      }
    }
    lastSelectState = selectState;
    
    // Check help buttons (up, down, left, right) - trigger SOS directly
    // Require 3 consecutive LOW reads to confirm button press (debounce)
    bool helpButtonPressed = checkHelpButton();
    
    if (helpButtonPressed) {
      helpButtonDebounceCount++;
      if (helpButtonDebounceCount >= 3 && !helpButtonWasPressed) {
        // Confirmed button press (3 consecutive reads)
        unsigned long now = millis();
        if ((now - lastHelpButtonPress) > 1000) {  // 1 second debounce between presses
          lastHelpButtonPress = now;
          helpButtonWasPressed = true;
          
          // Trigger help directly when any help button is pressed
          if (!helpActive) {
            Serial.println("Help button (UP/DOWN/LEFT/RIGHT) pressed - triggering SOS!");
            triggerHelp("BUTTON_PRESSED");
          }
        }
      }
    } else {
      helpButtonDebounceCount = 0;
      helpButtonWasPressed = false;
    }
    
    vTaskDelay(delayTicks);
  }
}

void SOS_Task(void *pvParameters) {
  Serial.println("SOS Task started");
  
  const TickType_t delayTicks = pdMS_TO_TICKS(100);
  
  while(1) {
    // Check if SOS should be playing
    bool shouldPlay = false;
    if (xSemaphoreTake(helpStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      shouldPlay = sosPlaying;
      xSemaphoreGive(helpStateMutex);
    }
    
    if (shouldPlay) {
      playSOS();
      
      // Check if stop signal received
      if (xSemaphoreTake(sosStopSemaphore, 0) == pdTRUE) {
        ledcWrite(PWM_CHANNEL, 0);  // Stop sound
        Serial.println("SOS stopped");
      } else {
        // Gap between SOS patterns
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    } else {
      vTaskDelay(delayTicks);
    }
  }
}

void Help_Monitor_Task(void *pvParameters) {
  Serial.println("Help Monitor Task started");
  
  const TickType_t delayTicks = pdMS_TO_TICKS(1000);  // Check every second
  static bool lowBpmTimerRunning = false;
  static const char* lastActivity = "idle";
  
  while(1) {
    // Monitor BPM for low heart rate condition
    float currentBpm = g_beatsPerMinute;
    
    if (xSemaphoreTake(helpStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (currentBpm > 0 && currentBpm < LOW_BPM_THRESHOLD) {
        // BPM is low - start timer if not already running
        if (!lowBpmTimerRunning) {
          xTimerStart(lowBpmTimer, 0);
          lowBpmTimerRunning = true;
          Serial.printf("Low BPM detected: %.1f (timer started)\n", currentBpm);
        }
      } else {
        // BPM normal or no reading - stop timer and reset
        if (lowBpmTimerRunning) {
          xTimerStop(lowBpmTimer, 0);
          lowBpmTimerRunning = false;
          lowBpmConditionMet = false;
          Serial.printf("BPM normalized: %.1f (timer stopped)\n", currentBpm);
        }
      }
      xSemaphoreGive(helpStateMutex);
    }
    
    // Monitor fall detection count
    const char* activity = fallDetector.getLastActivity();
    if (xSemaphoreTake(helpStateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (strcmp(activity, "fall") == 0) {
        // Restart the fall reset timer on each fall detection
        xTimerReset(fallResetTimer, 0);
        consecutiveFallCount++;
        Serial.printf("Fall detected! Count: %d/%d\n", consecutiveFallCount, CONSECUTIVE_FALL_THRESHOLD);
        
        if (consecutiveFallCount >= CONSECUTIVE_FALL_THRESHOLD && !helpActive) {
          xSemaphoreGive(helpStateMutex);
          triggerHelp("FALL_DETECTED");
        } else {
          xSemaphoreGive(helpStateMutex);
        }
      } else {
        // Activity changed - timer will handle reset after timeout
        if (strcmp(activity, lastActivity) != 0 && consecutiveFallCount > 0) {
          Serial.printf("Activity changed to '%s' - fall reset timer running\n", activity);
        }
        xSemaphoreGive(helpStateMutex);
      }
      lastActivity = activity;
    }
    
    vTaskDelay(delayTicks);
  }
}