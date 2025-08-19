
# Real-Time Sensor Management System Project Overview

This repository, written by Daniel Luchian, contains the software for an in-progress real-time monitoring and management sensor system.


## Table of Contents

- [Features](#features)
  - [RTOS Module](#rtos-module)
  - [Sensor Module](#sensor-module)
  - [File Module](#file-module)
- [Project Hardware and Tools](#project-hardware-and-tools)
  - [Hardware Currently in Use](#hardware-currently-in-use)
  - [MCU Pinout](#mcu-pinout)
  - [Development Environment](#development-environment)
  - [Libraries and Frameworks](#libraries-and-frameworks)


## Features

This project has three core aspects:

  1. RTOS Module: A real-time operating system written in C for the STM32F4 controller. 
  2. Sensor Module: A sensor module written in C++, responsible for configuring, running, and communicating with select sensors.
  3. File Module: A file module written in C, responsible for writing and reading data to a file system (currently a Micro-SD Card)

### RTOS Module

The RTOS module supports:

- **Task Management**: The RTOS can create a thread with its own stack. It also supports pre-emptive and cooperative multitasking. The RTOS manages threads and will trigger an interrupt if the thread reaches its time limit or another thread pre-empts it; however, the user also has the option to yield their threads. 
- **Task Scheduling**: The RTOS currently uses round-robin to schedule threads; however, a high-priority task is to switch round-robin scheduling with a priority scheduling system.
- **Interrupt Handling**: The RTOS currently supports interrupt handling for specific interrupts mainly related to context switching and thread handling. Further development of this RTOS will involve slowly configuring and handling more interrupts. 
- **Synchronization**: Critical sections support from interrupt disabling/enabling is under construction
- **Inter-task Communication**: Task communication library is implemented but is also waiting on crit sections for thread safe integation

The RTOS currently does not support but aims to implement the following (highest priority at the top):

- **Memory Management**: The RTOS does not manage all memory and still relies on functions such as malloc for initialization and uses the memory handling produced by STM's code generation. I aim to slowly move away from STM-generated functionality and implement my system (most likely a binary buddy system) for memory allocation.
- **File System**: I am currently writing the file module (using FATFS Middleware) to support reading and writing from a Micro-SD card, which will then be integrated into the RTOS for file I/O functionality. 

### Sensor Module

The sensor module is responsible for all sensor-related functionality in the project. Currently, I have integrated a [BME280 Sensor](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/) through a purchased [interface](https://www.amazon.ca/BME280-Environmental-Sensor-Temperature-Atmospheric/dp/B088HJHJXG/ref=sr_1_6?crid=2P6LHTNXB7MS7&dib=eyJ2IjoiMSJ9.nbJjv--6WYS78X8ff65UB6wZeHXS2zOL3lRl6T7gppDvdfSCt8PWlh_vcuOCkyxDrY6U0wH-uh1ebdEA9iCWzvALp6icDX7lxZX6YoQlz3J2rkvotRJ28XqutnHnupKFv-3W6cNgOxppK0-YmdqL3mCojUX-xlv1kB-TCwchqSTS_Dkcrses1KrXQbx7mxq2r3-PTeofHwormdyZjQKbflom8UT90wBD_paq_rS_cmxmQMYyvDprBEuAoAYiVRJm0rKr8EaCIX22jWD7Af-dRoUFtP_YVthEXHLyCGOZxhYElcJE7_t8e21WoypatdSL5zhlW4MKKTK7H0I4v7P51cd77qGA9wA0ONiB2Y_MpAIEiE3B50xRHVduFXh5Dhw642jBMuzxt5BWM5g2GvRTT8ALM_2wQ4jVTWe8D6eks0WV2T-I-cRomwlY_erGEYnM.hIuiaYNz8NxEupRgd7acwuxQaRP9F4H_6e3OFbskpm0&dib_tag=se&keywords=bme280&qid=1736553513&sprefix=bme280%2Caps%2C135&sr=8-6), using I2C communication. The environmental sensor's interface is written in C++ using object-oriented principles, which allowed me to explore embedded C++ and better understand how C++ interacts with C in an embedded systems context. A C interface for the environmental sensor class was created so my RTOS (written in C) could support its functionality. Once I finish the remaining tasks for my RTOS, I plan to add more sensors to the sensor module and, using inheritance, create a more generic and easy-to-use system to add sensors. As of now, I can successfully use my RTOS to create sensor tasks and use these tasks to configure, read, and display data from my environmental sensor. 

### File Module

The file module uses the FAT file system (with the FatFs library) to format, configure and interface with an SPI-connected [Micro-SD Card](https://www.amazon.ca/CANADUINO%C2%AE-Micro-SD-Adapter-Arduino-Converter/dp/B09TY9L76L/ref=sr_1_5?crid=3A6RL8QGT985C&dib=eyJ2IjoiMSJ9.SIrVYSov3s7t7n5hnBUjPzDeW0VITudyJrj1i__FlHJForNvKYTDMdU0XJuUHdqjcLfKYL-_lZ0Di8aSQOWHJ3KJ-dVkVW0l2XM1BIDGqTV_xwFpRqXwHDj1_6laR7p_XXTTDQOBgtkK-a1ISolZ_lZ3-n-Z7yyNq5NPrLU6MIxIv1cYWPf6LEJ-D4mE5xaPpm35lszV4ah9Mc0tuqc4c9Cw3odwG3YrRYt59LCofjzlqLt7CaAZAgVOKVcR2QL4AKLy8K7t26Ka-YNlqy7iWtdi7FXLTyCszXcGnunksP6dGDjH5QK6JRicWb9m0Uq-3ay9wbrLaZOOlh27KvT2zXSafqCgXlxmSmFBtqPKNxW-iWkMaaKIlOa5AXZEFj_D5nzq1NdN6pRUL0kdAQ0bPE4Qxno4F0b_NrvY2zSOJURFmZXOXdd-RKXwGiFwkZhT.VaFX_wPsMXRTJHI07sh_Vxkpso4LqKE9dCEW5wRyR7o&dib_tag=se&keywords=sd+card+stm32&qid=1736557535&sprefix=sd+card+stm3%2Caps%2C102&sr=8-5). Currently the fatfs diskio driver is under development, using the [SD Specifications Part 1 Physical Layer Simplified Specification] (https://academy.cba.mit.edu/classes/networking_communications/SD/SD.pdf). 


## Project Hardware and Tools

### Hardware Currently in Use

| Component                      | Manufacturer               | Description                                                                 | Link                                                                                                                       |
|---------------------------------|----------------------------|-----------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------|
| **NUCLEO-F401RE**               | STMicroelectronics         | A development board featuring the STM32F401RE microcontroller and its pinouts. | [Nucleo 64 F401RE on STMicroelectronics](https://www.st.com/en/evaluation-tools/nucleo-f401re.html) |
| **BME280 Environmental Sensor** | Interface by Bicool, sensor Bosch | A sensor interface used to measure temperature, humidity, and atmospheric pressure. | [BME280 Sensor on Amazon](https://www.amazon.ca/BME280-Environmental-Sensor-Temperature-Atmospheric/dp/B088HJHJXG) |
| **MicroSD Card Adapter**        | UNIVERSAL-SOLDER Electronics | A board used to interface a MicroSD card with the STM32 microcontroller.      | [MicroSD Adapter on Amazon](https://www.amazon.ca/CANADUINO%C2%AE-Micro-SD-Adapter-Arduino-Converter/dp/B09TY9L76L) |

### MCU Pinout

![STM32 MCU Pinout](https://github.com/DanielLuchian2909/RealTimeSensorManagementSystem/blob/main/Development%20Docs/MCUPinoutSTM32Cube.png)

### Development Environment

- **STM32CubeIDE**: Version 1.14.0
- **STM32CubeMX**: Version 6.11.0
- **GNU Arm Embedded Toolchain**: Version 11.3.rel1

### Libraries and Frameworks

- **CMSIS**: Version 5.0.5
- **HAL (Hardware Abstraction Layer)**: Version 1.8.3
- **FatFs**: Version R0.12c
