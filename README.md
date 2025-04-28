# BE_Cpp_2024 - Real-Time Mobile Robot Project

## Overview

This project was developed during the second semester of the 4th year of the AE-SE program at INSA Toulouse, as part of the "Real-Time Systems" course.  
The objective is to design and implement a **real-time multi-tasking system** for mobile robot control, using **C++** and **Xenomai**.

The system manages multiple components — robot communication, movement control, battery monitoring, camera image processing, and communication with a monitoring application — by leveraging **parallelism**, **inter-task synchronization** (semaphores, mutexes), and **message queues**.

---

## Features

- 🚀 Real-time multitasking architecture using Xenomai
- 📡 Server-client communication with a remote monitor
- 🤖 Serial communication with a mobile robot
- 🔋 Battery status monitoring and reporting
- 🛠️ Robot movement control (with and without watchdog)
- 📷 Camera integration for arena detection and robot localization
- 🔄 Synchronization using mutexes and semaphores
- 📑 Modular and scalable system design

---

## Project Architecture

Each functionality is managed by a dedicated real-time task:

| Task | Function |
|:----:|:---------|
| `ServerTask` | Starts a server and accepts monitor connection |
| `SendToMonTask` | Sends messages to the monitor |
| `ReceiveFromMonTask` | Receives and handles messages from the monitor |
| `OpenComRobot` | Opens robot serial communication |
| `StartRobotTask` | Starts the robot without a watchdog |
| `StartRobotWDTask` | Starts the robot with a watchdog |
| `MoveTask` | Periodically sends movement commands |
| `BatteryLevelTask` | Sends battery level updates |
| `CameraTask` | Processes camera images for arena detection and robot localization |
| `CommunicationCheckTask` | Monitors robot communication and handles disconnections |
| `MonitorError` | Manages monitor disconnection events |

---

## Technologies Used

- **C++** for all development
- **Xenomai** real-time framework
- **Linux** operating system
- **Microcontroller-based robot** for movement control
- **Camera** for environment processing

---

## How to Run

1. Ensure that **Xenomai** is installed and configured on your Linux machine.
2. Connect the mobile robot and the camera to your system.
3. Build the project:


## Launch the application:
sudo ./supervisor

## Project Structure
make
/src        # Source code files (C++ tasks)
/include    # Header files
/build      # Build files (after compiling)
/docs       # Project documentation (architecture diagrams, reports)
/scripts    # Useful scripts for setup or testing

## Deliverables
Full source code implementation.

Logical and physical architecture diagrams.

Technical documentation (task descriptions, synchronization justification).

Test logs validating real-time behavior and system performance.

## Contributor
Giovanna Grigolon
```bash
