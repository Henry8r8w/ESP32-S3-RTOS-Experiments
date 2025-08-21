# ESP32-S3 RTOS Experiments

A collection of small FreeRTOS/Arduino examples for the ESP32‑S3 assignments I have in school. Each sketch explores a specific concept such as basic scheduling, queues, semaphores, and timer-driven tasks.

## Requirements
- Arduino IDE (or PlatformIO) with the ESP32 board package installed  
- ESP32‑S3 development board  
- USB cable for flashing and serial monitoring  

## Repository layout
- Queue_RTOS.cpp # Traffic-light demo using a FreeRTOS queue
- Semaphore_RTOS.cpp # Light sensor + LCD with semaphore synchronization
- Timer_Register.cpp # LED blinking driven by software timers
- Round-Robin_OS.cpp # Simple round‑robin scheduler
- SRTF_Scheduling.cpp # Shortest-remaining-time-first scheduler
- Task-Controller-Block_OS.cpp # Basic task control block demonstration


## Getting started
1. Open the desired `.cpp` file in Arduino IDE or copy it into a new sketch
   directory.
2. Select **ESP32‑S3 Dev Module** (or your board variant) 
3. Connect the board and choose the appropriate serial port.
4. Click **Upload** to compile and flash.
5. Open Tools -> Serial Monitor at 115200 baud to view output.
