  

---

# Peltier-Based Liquid Cooling System for Temperature Calibration  

This project was developed as part of the **Lam Research Challenge (Phase 2)**. It focuses on precise temperature calibration within the **10°C to 25°C range**, addressing the gap in systems capable of calibrating below ambient temperatures. By leveraging a **Peltier module**, the system efficiently manages heat extraction and cooling, offering an accurate, cost-effective, and user-friendly solution.  

## Features  
- **Specialized Temperature Range:** Operates within **10°C to 25°C**, ideal for applications requiring precise low-temperature calibration.  
- **PID Control System:** Ensures highly accurate temperature stabilization with a resolution of **1°C**, using feedback from a DS18B20 sensor.  
- **Efficient Heat Extraction:** Utilizes a **CPU cooler block** for heat dissipation and a **liquid cooling block** for effective cooling.  
- **User-Friendly Interface:** Features an OLED display with real-time updates and a rotary encoder for easy temperature adjustments.  
- **Closed Cooling Circuit:** Prevents contamination and ensures consistent performance by circulating coolant in a sealed system.  

## System Overview  
1. **Cooling Mechanism:**  
   - The Peltier module transfers heat from the cold side (connected to the cooling block) to the hot side (dissipated by a CPU cooler).  
   - A copper pipe wound around a copper tumbler, connected by silicon tubing, forms a closed coolant circuit powered by a submersible pump.  
2. **Temperature Control:**  
   - Feedback from the DS18B20 sensor is processed by a PID control loop to modulate the Peltier module and pump operation, ensuring precise temperature regulation.  
3. **Interface and Feedback:**  
   - Users can set and monitor the target temperature on an **OLED display**.  
   - The system supports manual adjustments and pre-saved programs for quick configuration.  

## Components  
### Electronics  
- Peltier Module  
- DS18B20 Temperature Sensor  
- Submersible DC Pump  
- I2C OLED Display  
- Rotary Encoder  
- Raspberry Pi Pico W  
- 12V @ 10A Power Supply  

### Mechanical  
- Copper Tumbler  
- Copper Tubes  
- Liquid Cooling Block  
- Silicon Pipes  
- CPU Cooler Block  

For a detailed breakdown of components and costs, refer to the **[Bill of Materials (BoM)]**.  

## Process Flow  
1. Wind copper tubing tightly around the tumbler for efficient heat transfer.  
2. Connect the liquid cooling block to the cold side of the Peltier module and the CPU cooler block to the hot side.  
3. Form a sealed circuit by connecting the tubing to the pump.  
4. Introduce coolant into the system.  
5. Use the rotary encoder to set the desired temperature.  
6. The PID controller modulates the Peltier module and pump to maintain the set temperature.  
7. Monitor the real-time temperature on the OLED display.  

## Technical Documents
 - It contains the **wiring diagram** , **pseudo code**, **Bill of Material** and **mechanical drawing** for further clarifications

## Expected Outcomes  
- **Accurate Calibration:** Maintains temperatures with **1°C resolution** and safety controls for over/under-temperature conditions.  
- **Efficient Cooling:** Demonstrates effective heat extraction from the Peltier module to maintain sub-ambient temperatures.  
- **User Accessibility:** Provides a simple and intuitive interface for setting and monitoring temperatures.  
- **Cost-Effectiveness:** Developed within a budget of Rs. 4300, utilizing readily available components.  

## Future Enhancements  
- Integration of Wi-Fi for remote monitoring and control.  
- Data logging capabilities for extended calibration sessions.    

## Team Members
- Dev Garg
- Eniyan E
- Shrut Jain
- Jash Muni

---

