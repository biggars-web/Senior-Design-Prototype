# Senior-Design-Prototype
This repository contains the firmware, documentation, and hardware design files for the first-stage prototype of a wearable motion-tracking module for lower-limb rehabilitation. Each module uses an ESP32 Feather V2 microcontroller with two LSM6DSOX IMUs to measure fused pitch and roll angles at one joint. The system includes LED posture feedback, pushbutton control, and Bluetooth Low Energy (BLE) transmission for real-time monitoring.

This prototype represents the foundation for a full three-segment wearable device (hip, thigh, and shin), each equipped with its own ESP32 and dual-IMU PCB.

**Features**
- Dual-IMU orientation sensing (two LSM6DSOX units)
- Complementary filter for fused pitch and roll angle estimation
-BLE wireless output (tested using the LightBlue app)
-LED posture-feedback system
-Green: good alignment
-Yellow: borderline alignment
-Red: poor alignment
-Pushbutton interface
  -Start system
  -Stop system
  -Recalibration
-LED animations for each state
-Modular architecture for scaling to hip, thigh, and shin PCBs
-Breadboard prototype used to validate electrical, firmware, and communication behavior

**System Architecture**
The final wearable system will use 3 independent PCBs:
  - Hip PCB: ESP32 + IMU A + IMU B
  - Thigh PCB: ESP32 + IMU A + IMU B
  - Shin PCB: ESP32 + IMU A + IMU B
  
Each PCB computes its own fused pitch and roll values and transmits data vie BLE.

**Repository Structure**
```
Senior-Design-Prototype/
│
├── Source Code/
│   └── EE497_Arduino.ino # Full Arduino source code
│
├── Images/
│   ├── SystemBlockDiagram.png
│   ├── PCB Block Diagram.png
│   ├── breadboard implementation.png
│   └── physical breadboard.jpeg
│
├── Simulations/
│   └── SENIOR DESIGN FINAL SIM.asc
│
└── README.md
```
**Hardware Used**
- Adafruit ESP32 Feather V2
- 2 LSM6DSOX IMUs (addresses 0x6A and 0x6B)
- 3 LEDs (green, yellow, red) with 220 ohm resistors
- Pushbutton for system control
- Breadboard and jumper wiring
- I^2C Cable
- USB-C cable for power and programming

**How to Use the Firmware**
1. Install Required Libraries
   In Arduino IDE:

     - Adafruit LSM6DXS
     - Adafruit Unified Sensor
     - ESP32 BLE Arduino
2. Select the Board
   Tools → Board → ESP32 → Adafruit ESP32 Feather V2
3. Upload the Code
   Open the .ino file and click Upload
4. View BLE Data
   Use the LightBlue app on IOS or Android:

     - Connect to IMU System Boot
     - Open the characteristic
     - Change format from Hex to UTF-8 String and Save
     - Press Subscribe to receive real-time data and notifications
     - View streamed fused pitch, roll, and status

**BLE Output Format**
BLE characteristic sends a text string such as:
  Pitch: 12.5  Roll: -3.1  Status: GOOD

**User Feedback (LED & Button Behavior)**
LED Colors
  - Green: Good posture
  - Yellow: Borderline posture
  - Red: Poor posture

Button Modes
  - Short press: Start system
  - Second press: Stop system
  - Hold button for 2 seconds or longer: Recalibrate IMUs and set new starting point

LED Animations
  - System startup: Red → Yellow → Green
  - Recalibration: All 3 colors flash 3 times
  - Shutdown: Green → Yellow → Red

**Testing Summary**
- Both IMUs initialize reliably at 0x6A and 0x6B
- Complementary filter runs at real-time speed
- Fused pitch/roll values stable during movement
- LightBlue succesfully receives BLE packets
- LED feedback accurately reflects posture thresholds
- Pushbutton correctly controls system states

**Future Work**
  - Build and test the full hip-thigh-shin PCB set
  - Implement true knee and hip joint-angle calculations
  - Expand BLE interface into a mobile app
  - Integrate vibration motors for haptic feedback
  - Complete gait-analysis algorithms
  - Battery-power integration for long-term wear
  - Mechanical enclosure and final brace mounting

**Source Code**
All sourced code is included in the Source Code/ folder
