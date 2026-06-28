# RTOS-Factory-Safety-Controller

This project is a high-reliability industrial automation simulation designed for the **NUCLEO-F429ZI** development board, utilizing the **uC/OS-III Real-Time Operating System (RTOS)**.

The system implements a fail-secure safety architecture for dual-conveyor processes, focusing on real-time vibration monitoring, human-proximity safety, and intelligent maintenance management.

---

## 1. System Architecture

The system employs a preemptive multitasking architecture, ensuring that safety-critical interrupts (Emergency Stop, Proximity Alerts) are handled with deterministic latency.

### Task Priority & Scheduling

Task Name,Priority,Trigger,Responsibility
AppTaskEmergency,2,ISR (Semaphore),High-priority safety shutdown & interlock
AppTaskServo,3,Semaphore,Fail-secure mechanical door control
AppTaskButton,4,Periodic,User input & state management
AppTaskSensor,5,Periodic,I2C-based vibration acquisition (ADXL345)
AppTaskControl,6,Event-driven,State machine orchestration
AppTaskConveyorA/B,8-9,Periodic,Real-time motion control
AppTaskUltrasonic,11,Periodic,Proximity detection (HC-SR04)
AppTaskMaintenance,12,Periodic,Machine health & economic modeling

## 2. Technical Highlights

* **Unified Safety Routine (`ForceServo90Deg`):** A centralized structural bottleneck function that ensures the mechanical safety door is locked before electrical power is cut from the conveyors.
* **Two-Tier Emergency Evacuation:** An intelligent protocol that dynamically adjusts conveyor speeds (overclocking) to flush materials out of the danger zone during critical vibration states, preventing mechanical jamming.
* **RTOS Concurrency:** Utilizes `OS_SEM` for asynchronous event handling and `OS_MUTEX` for protected resource access (UART/Logging) to prevent priority inversion.
* **Deterministic Logic:** Designed to comply with industrial safety standards, ensuring minimal response time for critical failures.

---

## 3. Hardware Integration

* **Controller:** STM32 NUCLEO-F429ZI
* **Sensors:** * ADXL345 (I2C) - Vibration Analysis
* HC-SR04 (GPIO) - Ultrasonic Person Detection
* 10k Potentiometer (ADC) - Maintenance Level Selection

* **Actuators:**
* SG90 Servo (PWM) - Safety Door/Gate
* RGB LED & Passive Buzzer - Status/Alarm Signaling

## 4. Documentation & Demo

* **Project Report:** See the `/docs` folder for the full technical analysis and state-machine diagrams.
* **Demonstration Video:** (https://drive.google.com/file/d/1w6lk_oMkiXUfYjyfHsjD6W5wp_quuKwD/view?usp=share_link)


*Developed for the 2026 Embedded Systems Term Project.*
