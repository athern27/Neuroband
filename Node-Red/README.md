# Neuroband Node-RED Flow

Use this README to import and configure the `Neuroband.json` dashboard flow for monitoring the ESP32-based armband.

## Requirements
- Node-RED with the **Dashboard** palette (`node-red-dashboard`, version 3.6.6 is referenced in the flow).
- An MQTT broker reachable from Node-RED.

## Importing the flow
1. Open Node-RED and choose **Menu → Import → Select a file to import**.
2. Browse to `Node-Red/Neuroband.json` in this repo and import.
3. Deploy the flow.

## MQTT setup
- The flow uses the broker defined as **espClient** (default: `192.168.1.130:1883`).
- Update the broker host/port to match your environment (double-click any MQTT node or the config node to edit).

### Topics
- **Inbound (from device)**
  - `esp32/movement_status` → dashboard text “MOVEMENT”
  - `esp32/help_status` → dashboard text “Status” + toast notification when payload ≠ `HELP_OK`
  - `esp32/blood_pressure_value` → dashboard text “Blood Pressure”
  - `esp32/receive_ecg_data` (the flow file currently names this topic `esp32/recieve_ecg_data`; align with your firmware) → ECG CSV/array converted to an SVG preview
- **Outbound (to device)**
  - `esp32/ask_ecg_data` → triggered by **Get ECG data** dashboard button

## Dashboard overview
- Displays patient info (name, gender, age, ID) and static photo (update the URL in the “Display Patient Photo” template if needed).
- Shows live movement, help status, blood pressure, current date/time.
- Renders received ECG samples as an SVG image on the dashboard.

## Customization
- Replace the patient image URL in the `Display Patient Photo` template node.
- Adjust the MQTT topics if your firmware uses different names.
- The `Debug: Generate ECG wave` inject/function can be enabled during development to visualize test ECG data.
