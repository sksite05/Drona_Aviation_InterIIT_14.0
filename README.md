# Drona_Aviation_InterIIT_14.0
Our work on the low prep problem statement by Drona Aviation at Inter IIT Tech meet 14.0 Patna.
<img width="904" height="663" alt="image" src="https://github.com/user-attachments/assets/d187f376-2b2b-4692-b76c-cb41513670ef" />

# **MagisV2-Firmware-Team24**
### *Inter IIT Tech Meet 14.0 – Stabilization System*

## **1. Project Summary**

This project delivers an on-board stabilization and position-hold system for the Pluto nano-drone using the MagisV2 firmware.

Key capabilities achieved:

- Stable autonomous hover  
- Optical-flow–based horizontal drift reduction  
- Accurate altitude hold up to **2 meters**  
- Consistent behavior across varying textures and indoor conditions  

The solution uses only the sensors and APIs specified in the problem statement.

## **3. Sensor Fusion Summary**

### **Altitude Fusion**

- VL53L1X ToF prioritized when valid  
- Barometer used as fallback  
- Tilt-compensated height  
- Filtered vertical velocity and altitude estimates  

### **Optical Flow Fusion**

- PAW3903 motion deltas quality-checked with SQUAL  
- Scaled using ToF height  
- Low-pass filtering + deadbanding for noise suppression  
- Outputs horizontal velocities and integrated X/Y position  

This produces stable state estimates required for low-drift hovering.

## **7. Acknowledgments**

Developed by **Team 24** for Inter IIT Tech Meet 14.0, using the Pluto platform and the MagisV2 firmware environment provided by **Drona Aviation**.

