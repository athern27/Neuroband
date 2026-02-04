# Neuroband

Smart safety armband that monitors motion, pulse oximetry, ECG, and displays data on a small TFT, backed by MQTT connectivity.

## Hardware
- **Board:** Adafruit ESP32-S3 Reverse TFT (adafruit_camera_esp32s3)
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

## Notes
- Default PlatformIO environment: `adafruit_camera_esp32s3`.
- Ensure the board has PSRAM enabled (required by this environment).
- If using VS Code, select the environment and run **Build** / **Upload** from the PlatformIO sidebar.
