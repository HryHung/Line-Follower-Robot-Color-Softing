# Line Following Robot (Color-Based Distribution)

Mobile robot designed for line tracking and automatic goods sorting based on color.  
This project demonstrates practical applications of control theory, embedded systems, and mechatronics design.

---

## 🚀 Overview

This project focuses on building a line-following robot with:

- Embedded firmware for real-time control  
- Line tracking using IR sensor array  
- Color detection for decision making  
- Autonomous navigation and delivery  

---

## 🧠 System Architecture

### Hardware

- Differential drive mobile platform  
- DC motors with encoders  
- IR sensor array (line detection)  
- Color sensor (Red / Blue detection)  
- Motor driver (TB6612FNG)  
- Microcontroller (STM32 / ESP32)  
- Battery + power management  

### Software Stack

```
[ High-Level Logic ]
        ↓
[ Decision Making (Color Detection) ]
        ↓
[ Line Tracking Algorithm ]
        ↓
[ PID Motor Control ]
        ↓
[ Hardware Drivers (PWM / ADC / GPIO) ]
```

---

## ⚙️ Features

- Accurate line following (±3 mm error)  
- Autonomous color-based sorting  
- PID-based motor control  
- Modular hardware & software design  
- Suitable for robotics learning and R&D  

---

## 📂 Repository Structure

```
/firmware        → MCU code (control, sensors, PID)
/hardware        → PCB, schematics, CAD design
/simulation      → MATLAB / Simulink models
/docs            → Reports and documentation
```

---

## 🔧 Technologies

- Embedded C / C++  
- STM32 / ESP32  
- PWM / ADC / GPIO  
- SolidWorks (mechanical design)  
- MATLAB / Simulink (modeling & control)  

---

## 🧠 Control Concept

- Line position estimated using weighted sensor readings  
- PID controller used for trajectory correction  
- State machine handles robot behavior:

```
Idle → Follow Line → Detect Load → Detect Color
     → Navigate → Stop at Destination
```

---

## 📊 Specifications

- Speed ≥ 0.1 m/s  
- Line width: 26 mm  
- Tracking error: ±3 mm  
- Stopping error: ±5 mm  
- Load capacity: ~1 kg  

---

## 📌 Roadmap

- Improve control with Fuzzy / Adaptive PID  
- Add wireless monitoring (IoT)  
- Upgrade to vision-based system  
- Implement SLAM navigation (AMR level)  

---

## 🤝 Contribution

This is an R&D-oriented project. Contributions, improvements, and ideas are welcome.
