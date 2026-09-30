# STM32 Hardware-In-The-Loop (HIL) 6DoF Flight Simulator

An advanced Hardware-in-the-Loop (HIL) simulation framework architected for evaluating and tuning quadcopter flight dynamics. This project bridges a real-time deterministic physics engine running *bare-metal* on an **STM32 Microcontroller** with a custom high-performance **PyQt6 Ground Control Station (GCS)** over a high-speed, non-blocking USB CDC virtual COM port.

---

## Graphical User Interface (GCS Overview)

The custom-built PyQt6 Ground Control Station provides a fully integrated industrial dark aerospace environment for real-time telemetry monitoring, PID tuning, and tactical flight control:

![GCS Dashboard Overview](Docs/Assets/GCS_GUI.png)

---

### Real-Time Telemetry & 3D Digital Twin

| 3D OpenGL Digital Twin Viewer | Tactical Top-Down Radar (X-Y Plane) |
| :---: | :---: |
| ![3D Viewer](Docs/Assets/3Dmodel_viewer.gif) | ![Tactical Radar](Docs/Assets/XY_viewer.gif) |
| *Real-time 6DoF wireframe rendering with active 3D trajectory history.* | *Dynamic auto-centering X-Y radar with locked aspect ratio.* |

---

## 🚀 Live Flight Simulation Demo

Below is a live demonstration of the manual flight mode in action. Notice the seamless real-time synchronization between keyboard inputs (`W, A, S, D`), the STM32 6DoF physics calculations, the 3D digital twin rotation, and the tactical radar tracking:

![Flight Demo GIF](Docs/Assets/GUI_Example.gif)

---

## Key Engineering Features

* **Real-Time Dual-Loop Control Architecture (STM32):**
  * **Inner Attitude Loop (1000 Hz):** Handles high-frequency PID stabilization for vehicle orientation using hardware timer interrupts (`TIM6`).
  * **Outer Navigation Loop (100 Hz):** Computes spatial positioning and velocity vectors relative to coordinate setpoints.
* **Rigid-Body Newton-Euler Physics Engine (`physics.c`):** 
  * Implements deterministic rigid-body dynamics, resolving mass moments of inertia, tilt-based horizontal coupling ($X, Y, Z$ accelerations derived from thrust and attitude vectors), and simulated aerodynamic drag.
* **Asynchronous Key-Value Communication Protocol:**
  * Employs a robust, non-blocking string protocol (`KEY:val,KEY:val...\n`) over USB CDC, ensuring jitter-free telemetry downlinking and parameter uplinking.
* **High-Performance PyQt6 GCS (Ground Control Station):**
  * **Tactical Top-Down Trajectory Radar ($X-Y$ plane):** Features dynamic auto-centering and real-time path tracking using `PyQtGraph`.
  * **3D OpenGL Digital Twin Viewer:** Real-time spatial wireframe rendering mitigating gimbal lock via explicit $4x4$ rigid transformation matrices (`QMatrix4x4`).

---

## System Architecture

```text
+-----------------------+              USB CDC (Virtual COM)              +---------------------------+
|                       |  Telemetry (R, P, Z, X, Y,, Yaw, Ref)           |                           |
|   STM32 Controller    | ----------------------------------------------> |      PyQt6 GCS (PC)       |
|  (Firmware / C)       |                                                 |  - Tactical X-Y Radar     |
|                       |  Commands (SET:x,y,z / Key Input)               |  - 3D OpenGL Twin Viewer  |
| - 1000Hz Attitude PID | <---------------------------------------------- |  - Real-time Plotting     |
| - 100Hz Navigation    |                                                 |                           |
+-----------------------+                                                 +---------------------------+
```

## Tech Stack & Tools

* **Hardware:** STM32G474 WeAct Studio Mini core Board.
* **Embedded Firmware:** C, STM32 HAL / Low-Level Drivers, CMSIS, Hardware Timers.
* **Ground Control Station:** Python 3.10+, PyQt6, PyQtGraph, PyOpenGL, NumPy, PySerial.
* **Communication:** USB Device Library (CDC class, Virtual COM port).

---

## Project Structure

```text
├── firmware_hil_drone/              # STM32 Embedded Firmware (STM32CubeIDE)
│   ├── Core/
│   │   ├── Inc/
│   │   │   ├── config.h             # System parameters & physical constants
│   │   │   ├── control.h            # Control loop headers
│   │   │   ├── physics.h            # 6DoF physics engine definitions
│   │   │   ├── pid.h                # PID controller structures
│   │   │   └── telemetry.h          # Serial protocol parser and builder
│   │   └── Src/
│   │       ├── config.c             # Configuration loading & defaults
│   │       ├── control.c            # Cascaded control loops & interrupt handlers
│   │       ├── physics.c            # Rigid-body 6DoF equations & aerodynamics
│   │       ├── pid.c                # PID mathematics implementation
│   │       └── telemetry.c          # Asynchronous CDC command parser
│   └── USB_Device/                  # USB CDC Virtual COM port implementation
│
└── desktop_application/             # PyQt6 Ground Control Station (GCS)
    ├── views/
    │   ├── main_window.py           # Main application layout & UI orchestration
    │   ├── plot_manager.py          # pyqtgraph telemetry & tactical X-Y radar
    │   └── viewer_3d.py             # PyOpenGL 6DoF digital twin renderer
    ├── gcs_layout.ui                # Qt Designer layout file
    ├── main.py                      # Application entry point
    └── telemetry_core.py            # Serial communication thread & parser
```

## Getting Started

### 1. Firmware Setup
1. Open the `firmware_hil_drone` project in **STM32CubeIDE**.
2. Build and flash the firmware onto your target STM32 board (optimized for the STM32G4 series).
3. Ensure the USB Device CDC peripheral is active in the project configuration.

### 2. Ground Control Station Setup
1. Navigate to the desktop application directory:
```
   cd desktop_application
```
  * Install the required dependencies:
```
pip install pyqt6 pyqtgraph pyopengl numpy pyserial
```
  * Run the GCS application:
```
python main.py
```

## Manual Flight Mode Operation

* **W / S:** Adjust Forward / Backward spatial target ($X$-axis).
* **A / D:** Adjust Left / Right spatial target ($Y$-axis).
* **R / F:** Adjust Up / Down altitude target ($Z$-axis).
* **Q / E:** Adjust X-Y Orientation target (Yaw).

## License

Distributed under the MIT License. See **LICENSE** for more information.
