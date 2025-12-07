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
