# Environment Telemetry Module

## Description
Modified Meshtastic environment telemetry module with support for a GPIO rain gauge and a fixed low-power telemetry schedule.

## Features
- Rain gauge (reed switch) on GPIO 4  
- Interrupt-based pulse counting  
- Rainfall calculation in mm/h over 5 minutes  
- Telemetry sent every 5 minutes  
- Sleep scheduling between transmissions  
- GPIO wake-up enabled for rain detection  
- Rain displayed on screen  

## Configuration
#define RAIN_PIN 4  
#define RAIN_RESOLUTION_MM 0.2794f  
#define RAIN_WINDOW_MS 300000UL  

## Telemetry timing
- Period: 300 s  
- First send offset: 305 s  
- Guard time: 200 ms  

## Rain encoding
Rainfall is stored in the Meshtastic `iaq` field (no native rain field):

rain_mm_per_hour = iaq / 100.0  

## Notes
Simple and efficient implementation to add rainfall monitoring without modifying Meshtastic protobuf. Suitable for low-power environmental sensing deployments.
