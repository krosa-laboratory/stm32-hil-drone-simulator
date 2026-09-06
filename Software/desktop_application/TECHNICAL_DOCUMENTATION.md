# Ground Control Station (GCS) - Technical Documentation

## 1. Overview
The Ground Control Station (GCS) is a custom-built desktop application developed using Python and PyQt6, designed to interface directly with the STM32-based flight controller via a high-speed virtual COM port (USB CDC). It serves as the primary command, telemetry monitoring, and visualization hub for the Hardware-in-the-Loop (HIL) quadcopter simulator.

The architecture emphasizes a clean separation of concerns, routing asynchronous serial data packets into real-time multi-axis time-series charts, a top-down tactical trajectory radar, and a 6-Degrees-of-Freedom (6DoF) OpenGL digital twin viewer.

## 2. Software Architecture & Directory Structure

The GCS application resides inside the desktop_application/ directory and is organized into modular components to ensure maintainability and high frame rates during real-time telemetry rendering:
```
desktop_application/
├── views/
│   ├── __init__.py
│   ├── main_window.py       # Main application layout, UI orchestration, and keyboard hooks
│   ├── plot_manager.py      # pyqtgraph telemetry charts and X-Y tactical radar
│   └── viewer_3d.py         # PyOpenGL 6DoF digital twin renderer
├── gcs_layout.ui            # Qt Designer layout definition file
├── main.py                  # Application entry point & QApplication initialization
└── telemetry_core.py        # Asynchronous serial communication thread & packet parser
```

## 3. Core Subsystems

### 3.1 Asynchronous Telemetry Parser

* **Protocol:** Utilizes a non-blocking key-value string protocol (*KEY:val,KEY:val...\n*) transmitted over USB CDC.
* **Payload Structure:** Processes continuous downlinked strings containing actual states and setpoints:
```
R:1.20,R_ref:0.00,P:-0.45,P_ref:0.00,Z:1.50,U1:9.81,X:0.20,X_ref:0.00,Y:-0.10,Y_ref:0.00\r\n
```
* **Threading:** Operates on a background worker thread via pyserial to prevent UI freezing during high-frequency data ingestion.

### 3.2 Telemetry & Tactical Radar Manager

Built on top of PyQtGraph for high-performance rendering:

* Time-Series Plots: Real-time curves tracking Roll (R, R_ref), Pitch (P, P_ref), and Z-Altitude (Z).
* Tactical X-Y Radar (Top-Down View):
  * Displays a 2D Cartesian plane tracking the actual flight path (magenta trace with a bright cyan current-position marker) versus target waypoints (yellow cross).
  * Features enforced aspect ratio locking to prevent geometric distortion during window resizing.
  * Implements dynamic auto-centering around the vehicle's active coordinates to maintain a smooth, floating tracking window.

### 3.3 6DoF OpenGL Digital Twin Viewer

Powered by PyOpenGL:

* **Rigid-Body Transformation:** Avoids gimbal lock and matrix distortion by applying explicit 4x4 rigid transformations (QMatrix4x4). Rotations are computed locally around the vehicle's dynamic center (X, Y, Z) rather than the global origin.
* **Wireframe Geometry:** Renders an X-configured drone frame complete with color-coded orientation indicators (cyan frame arms with a red front-heading marker).
* **Dynamic Ground Grid & 3D Trail:** Includes a moving ground plane (GLGridItem) that synchronizes its horizontal position with the drone, alongside a 3D historical tail (GLLinePlotItem) tracking past trajectory coordinates in space.

## 4. Manual Flight Control Interaction

The GCS translates user inputs into discrete spatial offset commands that are sent to the STM32 firmware:

* **Key Bindings:**
  * W / S: Adjusts forward/backward spatial target along the $X$-axis.
  * A / D: Adjusts left/right spatial target along the $Y$-axis.
  * R / F: Adjusts vertical altitude target along the $Z$-axis.
* **Command Transmission:** Intercepts key events when Manual Mode is active, formatting target updates into lightweight asynchronous frames (SET:x,y,z) sent downstream to actuate the firmware's navigation loops.

## 5. UI Styling & Industrial Design

* **Theme:** "Industrial Dark Aerospace" aesthetic utilizing custom QSS (Qt Style Sheets) dark palettes (#1a1a1e backgrounds with #00ffcc cyan accents).
* **Framework Integration:** Forces the native Fusion style at startup to guarantee consistent cross-platform widget rendering and clean visual feedback.
