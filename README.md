# Neuroband - Smart Safety Armband

A comprehensive wearable health monitoring system built on ESP32-S3 using FreeRTOS. The system continuously monitors vital signs (heart rate, ECG, movement) and can automatically trigger an emergency alert (SOS) if critical conditions are detected.

## Hardware
- **Board:** Adafruit Memento Board
- **Sensors/Libraries:** LIS3DH accelerometer, MAX3010x pulse-oximeter, OLED/TFT via Adafruit GFX & ST7735, AW9523 I/O expander, PubSubClient for MQTT.

## Build & Flash (PlatformIO)
1) Install PlatformIO  
   - VS Code extension **or** CLI: `pip install -U platformio`
2) Clone the repo and install libs (PlatformIO will auto-fetch from `platformio.ini`):  
   ```bash
   git clone https://github.com/athern27/Neuroband.git
   cd Neuroband
   ```
3) Update credentials/broker in `src/main.cpp` near the top (`ssid`, `password`, `mqtt_server`) to match your environment.
4) Build firmware:  
   ```bash
   pio run
   ```
5) Connect the board via USB, then upload:  
   ```bash
   pio run -t upload
   ```
6) Open serial monitor (115200 baud):  
   ```bash
   pio device monitor -b 115200
   ```
   
## Project Overview 📋

Neuroband is a multi-sensor health monitoring wearable that combines real-time vital sign acquisition with intelligent emergency detection. The device communicates with a Node-RED server via MQTT to display data and handle remote monitoring.

**Key Capabilities:**
- Real-time heart rate monitoring with beat detection
- Continuous ECG recording (20-second circular buffer)
- Fall detection using accelerometer
- Emergency SOS alert system with speaker
- WiFi + MQTT for cloud connectivity
- Multi-tasking on dual-core ESP32-S3 with FreeRTOS
- OLED display for local feedback
- Hardware button controls

---

## Hardware Architecture 🧰

### Microcontroller
- **ESP32-S3** (Adafruit Camera variant)
- Dual-core processor (Core 0 & Core 1)
- I2C (GPIO 34 SDA, GPIO 33 SCL)
- SPI for display

### Sensors
| Sensor | Purpose | Interface | Details |
|--------|---------|-----------|---------|
| **MAX30105** | Heart Rate / Oximetry | I2C (0x57) | Beat detection, BPM calculation |
| **LIS3DH** | Accelerometer | I2C (0x19) | Fall detection (4G range, 50 Hz) |
| **ECG Sensor** | Cardiac Monitoring | Analog (ADC) | 125 Hz sampling |

### Peripherals
| Device | Purpose | Interface | Details |
|--------|---------|-----------|---------|
| **AW9523** | GPIO Expander | I2C (0x58) | 16 I/O pins for buttons & speaker control |
| **Speaker** | Audio Alert | PWM (GPIO 46) | SOS pattern playback, enabled via AW9523 P0_0 |
| **OLED Display** | Status Display | SPI | Real-time vital signs feedback |

### Button Layout (AW9523)
| Button | Port/Bit | Pin | Function |
|--------|----------|-----|----------|
| UP | P1_5 | 13 | Start SOS |
| DOWN | P1_7 | 15 | Start SOS |
| LEFT | P1_6 | 14 | Start SOS |
| RIGHT | P1_4 | 12 | Start SOS |
| SELECT | P0_1 | 1 | Stop SOS |
| OK | P1_3 (also GPIO 11) | 11 | Short press = stop SOS, Long press (3s) = start SOS |

---

## Software Architecture 🏗️

### FreeRTOS Task Breakdown

```
┌─────────────────────────────────────────────────────────────┐
│                    ESP32-S3 (Dual Core)                      │
├──────────────────────┬──────────────────────────────────────┤
│      Core 0          │           Core 1                      │
├──────────────────────┼──────────────────────────────────────┤
│ • ECG_Analog_Task    │ • Accelerometer_Task                 │
│   (125 Hz sampling)  │   (Fall detection)                   │
│                      │                                      │
│ • Oximeter_I2C_Task  │ • Button_Task                        │
│   (Heart rate BPM)   │   (Button polling, debouncing)      │
│                      │                                      │
│ • Display_Task       │ • SOS_Task                           │
│   (OLED updates)     │   (SOS pattern playback)             │
│                      │                                      │
│ • UART_Debug_Task    │ • Help_Monitor_Task                  │
│   (Serial output)    │   (BPM threshold, fall counter)      │
│                      │                                      │
│ • MQTT_Send_Task     │                                      │
│ • MQTT_ECG_Task      │                                      │
│ • main loop()        │                                      │
└──────────────────────┴──────────────────────────────────────┘
```

### Task Priorities & Responsibilities

**Core 0 (Sensor Reading & Communication):**
1. **ECG_Analog_Task** (Priority 2) - Samples ECG at 125 Hz, filters, stores in circular buffer
2. **Oximeter_I2C_Task** (Priority 2) - Reads MAX30105 every 20ms, calculates BPM with EMA filter
3. **Display_Task** (Priority 1) - Updates OLED every 100ms with vital signs
4. **UART_Debug_Task** (Priority 1) - Serial output for debugging
5. **MQTT_Send_Task** (Priority 1) - Publishes BPM, movement status, help status every 1s
6. **MQTT_ECG_Task** (Priority 1) - Sends 20s ECG buffer on demand

**Core 1 (Real-time Events & Control):**
1. **Accelerometer_Task** (Priority 1) - Fall detection inference
2. **Button_Task** (Priority 2) - Monitors buttons, triggers help/SOS
3. **SOS_Task** (Priority 1) - Plays SOS pattern with stop checks
4. **Help_Monitor_Task** (Priority 1) - Monitors BPM thresholds and fall counter

### Data Flow

```
Sensors (MAx30105, LIS3DH, ECG)
        ↓
    I2C Mutex
        ↓
┌───────────────────────────────────┐
│  Processing Tasks                 │
│  • BPM calculation (EMA filter)   │
│  • Fall counter (debounced)       │
│  • ECG buffer (circular)          │
└───────────────────────────────────┘
        ↓
┌─────────────────────────────────────────────┐
│  Help Detection Logic                       │
│  • Low BPM < 60 for 20 seconds?            │
│  • 5+ consecutive falls?                    │
│  • User button press?                       │
└─────────────────────────────────────────────┘
        ↓
   SOS Triggered
        ├─→ Speaker plays SOS pattern
        ├─→ MQTT publishes HELP_ACTIVE
        └─→ Sends ECG data to server
```

---

## Emergency Detection Logic 🚨

The system triggers **HELP mode** (SOS alert) when:

1. **User Button Press** (PRIMARY)
   - Any direction button (UP/DOWN/LEFT/RIGHT) → Immediate SOS
   - SELECT button or OK long press (3s) → Immediate SOS

2. **Low Heart Rate** (SECONDARY)
   - BPM < 60 for 20+ seconds → Waits for button confirmation

3. **Fall Detection** (SECONDARY)
   - 5+ consecutive falls within 3 seconds → Automatic SOS

**SOS Pattern:**
- Morse code ... --- ... (3 dots, 3 dashes, 3 dots)
- Repeats continuously until user presses SELECT/OK to stop
- During playback, buttons are checked every 10ms for immediate response

**Button Controls:**
- **Direction buttons** (UP/DOWN/LEFT/RIGHT) → **START** SOS
- **SELECT or OK** (short press) → **STOP** SOS
- **OK** (long press 3s) → **START** SOS

---

## Communication Protocols 📡

### WiFi & MQTT

**WiFi Configuration:**
```
SSID: TP-Link_6391
Password: AKMR@159
MQTT Server: 192.168.1.130:1883
```

**MQTT Topics:**

| Topic | Publisher | Message | Frequency |
|-------|-----------|---------|-----------|
| `esp32/movement_status` | MQTT_Send_Task | Activity (fall/idle/etc) | Every 1s |
| `esp32/blood_pressure_value` | MQTT_Send_Task | BPM value (float) | Every 1s |
| `esp32/help_status` | Multiple | HELP_ACTIVE / HELP_OK / HELP:reason | On change |
| `esp32/ask_ecg_data` | Node-RED | 1 = request data | On demand |
| `esp32/recieve_ecg_data` | MQTT_ECG_Task | 2500 ECG samples (CSV) | On request |

### Serial Debug Output

**Baud Rate:** 115200

**Sample Output:**
```
Neuroband ESP32-S3 FreeRTOS Project
I2C initialized (SDA=34, SCL=33)
AW9523: Found device at 0x58
AW9523 initialized - buttons and speaker control available
MAX30105 sensor initialized
LIS3DH accelerometer initialized
All FreeRTOS tasks created successfully

[BTN] UP=1 DOWN=1 LEFT=1 RIGHT=1 (0=pressed)
[BTN] SELECT=1 (0=pressed)
IR=65000, BPM=72
Help button pressed - triggering SOS!
SOS started
Help mode deactivated - OK button pressed
```

---

## Vital Signs Processing 💓

### Heart Rate (BPM)

**Algorithm:**
1. MAX30105 IR LED detects blood perfusion
2. `checkForBeat()` detects peaks in IR signal
3. Beat interval → BPM calculation: `60000 / interval_ms`
4. **Median filter** (3 samples) removes outliers
5. **Exponential Moving Average (EMA)** smoothing
   - EMA_ALPHA = 0.4 (40% new, 60% history)
   - Stable response without lag

**Valid Range:** 45–180 BPM  
**Update Rate:** Every beat detected + every 1 second display refresh

### ECG Signal

**Processing:**
- **Sampling Rate:** 125 Hz (8ms per sample)
- **Filtering:** High-pass filter to remove drift
- **Storage:** Circular buffer (2500 samples = 20 seconds)
- **Transmission:** Sent to Node-RED on request (CSV format)

### Fall Detection

**Algorithm:**
- LIS3DH accelerometer in 4G range, 50 Hz sampling
- Edge ML inference for fall pattern recognition
- Debounced: counts consecutive detections within 3-second window
- **Threshold:** 5 consecutive falls → Automatic help trigger
- **Reset:** Counter resets 3 seconds after last fall

---

## Synchronization & Thread Safety 🔐

### Mutexes

| Mutex | Protected Resource | Used By |
|-------|-------------------|---------|
| `i2cMutex` | I2C bus (sensors + expander) | Oximeter_Task, Accelerometer_Task, Button_Task |
| `ecgBufferMutex` | ECG circular buffer | ECG_Task, MQTT_ECG_Task |
| `helpStateMutex` | Help state variables | All help-related tasks |

### Semaphores

| Semaphore | Purpose | Used By |
|-----------|---------|---------|
| `sosStopSemaphore` | Signal to stop SOS playback | Button_Task → SOS_Task |

### Software Timers

| Timer | Duration | Purpose | Behavior |
|-------|----------|---------|----------|
| `lowBpmTimer` | 20 seconds | Threshold for low BPM condition | One-shot, resets if BPM normalizes |
| `mqttReconnectTimer` | 5 seconds | Attempt MQTT reconnection | Periodic (auto-reload) |
| `fallResetTimer` | 3 seconds | Clear fall counter after inactivity | One-shot, restarted on each fall |

---

## Build & Deploy 🚀

### Requirements
- PlatformIO CLI or VS Code extension
- ESP32-S3 board (Adafruit Camera variant)
- USB cable for programming

### Build
```bash
pio run -e adafruit_camera_esp32s3
```

### Upload & Monitor
```bash
pio run -t upload -e adafruit_camera_esp32s3
pio device monitor -b 115200
```

### Configuration
Edit `src/main.cpp`:
```cpp
// WiFi
const char* ssid = "TP-Link_6391";
const char* password = "AKMR@159";
const char* mqtt_server = "192.168.1.130";

// Thresholds
#define LOW_BPM_THRESHOLD 60
#define LOW_BPM_DURATION_MS 20000  // 20 seconds
#define CONSECUTIVE_FALL_THRESHOLD 5
```

---

## Performance Metrics ⚡

| Metric | Value |
|--------|-------|
| **Core 0 Load** | ~30% (sensor polling + MQTT) |
| **Core 1 Load** | ~20% (button polling + fall inference) |
| **ECG Buffer Size** | 2500 samples = 20 seconds |
| **I2C Bus Speed** | 400 kHz (FAST) |
| **Display Refresh** | 100ms (10 FPS) |
| **BPM Update** | Per beat + 1 second sync |
| **MQTT Publish Rate** | 1 second |
| **Memory Used** | ~70% of available SRAM |

---

## Troubleshooting 🛠️

### No Serial Output
- Check baud rate: must be 115200
- Verify USB cable connection
- Try different USB port

### Sensors Not Found
- Verify I2C wiring (SDA=34, SCL=33)
- Check pull-up resistors (should be 4.7kΩ)
- Use `pio device monitor` with `-f debug` filter

### Buttons Not Responding
- Use `p` command in serial to poll AW9523
- Check AD1/AD0 pins are grounded for address 0x58
- Verify button mappings with `r` (dump registers)

### WiFi/MQTT Connection Issues
- Verify SSID and password are correct
- Check MQTT server IP and port (1883)
- Ensure board has internet connectivity
- Check firewall rules on MQTT server

### Low BPM False Triggers
- Ensure finger is properly placed on sensor
- Clean sensor lens
- Adjust `LOW_BPM_THRESHOLD` and `LOW_BPM_DURATION_MS` if needed

### Fall False Positives
- Calibrate accelerometer threshold
- Increase `CONSECUTIVE_FALL_THRESHOLD` value
- Ensure sensor is securely mounted

---

## Files Overview 📁

| File | Purpose |
|------|---------|
| `src/main.cpp` | Main FreeRTOS tasks and help detection logic |
| `src/aw9523.cpp` | AW9523 GPIO expander driver |
| `src/ecg.cpp` | ECG signal processing and filtering |
| `src/oled.cpp` | OLED display driver and UI |
| `src/oximeter.cpp` | Heart rate processing with EMA |
| `src/uart.cpp` | Serial communication utilities |
| `src/fall_detection.cpp` | Fall detection with edge ML |
| `include/aw9523.h` | AW9523 register definitions and API |
| `lib/fall_detection/` | Edge ML fall detection library |

---

## Future Enhancements 🔮

- [ ] Bluetooth connectivity (BLE) for mobile app
- [ ] Local data storage (SD card)
- [ ] Advanced activity classification (walking, running, sitting)
- [ ] Temperature monitoring
- [ ] Blood oxygen (SpO2) tracking
- [ ] Voice alerts in addition to audio SOS
- [ ] Battery monitoring and low power mode
- [ ] OTA firmware updates via WiFi

---

## License & Credits 📝

**Author:** Neuroband Team  
**Platform:** ESP32-S3 with Arduino Framework  
**Libraries Used:**
- PubSubClient (MQTT)
- Adafruit MAX30105
- Adafruit LIS3DH
- Edge ML for fall detection
- FreeRTOS (built into ESP32 Arduino framework)

---

## Contact & Support 📧

For issues or questions, check:
1. Serial debug output for error messages
2. MQTT topics for real-time status
3. OLED display for sensor feedback
4. Button functionality (confirm hardware is responding)

---

