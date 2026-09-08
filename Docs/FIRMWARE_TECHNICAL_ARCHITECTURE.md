# Technical Architecture & Core Subsystems

This document provides an in-depth technical breakdown of the core embedded subsystems and mathematical models implemented in the **STM32 HIL Quadcopter Simulator**.

---

## 1. Real-Time Dual-Loop Control Architecture

To ensure deterministic flight control behavior, the firmware separates attitude stabilization from spatial navigation into two distinct execution rates managed by hardware timer interrupts (`TIM6`):

```text
[ Hardware Timer Interrupt (TIM6 @ 1000Hz) ]
       │
       ├──> Physics Engine Update (6DoF rigid-body simulation)
       │
       ├──> Inner Attitude Loop (1000Hz)
       │      └─ Computes Roll, Pitch, Yaw torques (U2, U3, U4) via PID
       │
       ├──> Outer Navigation Loop (100Hz - Every 10 ticks)
       │      └─ Computes desired Z-thrust and X-Y tilt setpoints via PID
       │
       └──> Motor Mixer & Telemetry Transmission
```

* **Inner Loop (1000 Hz / $dt = 1\text{ ms}$):** Executed inside the primary timer interrupt (TIM6_DAC_IRQHandler). It handles high-frequency error correction for the vehicle's attitude (Roll, Pitch, Yaw) using dedicated PID controllers and feeds the motor mixer.
* **Outer Loop (100 Hz / $dt = 10\text{ ms}$):** Triggered via a decimation counter (NAV_DIVIDER = 10). It processes spatial navigation errors ($X, Y, Z$) and dynamically updates the orientation setpoints (desire_state.roll, desire_state.pitch) and collective thrust (desire_U1).

## 2. Rigid-Body 6DoF Physics Engine

The firmware features an internal simulation engine that models a complete 6-Degrees-of-Freedom rigid-body dynamic system. Instead of treating the drone as a point mass, it resolves rotational accelerations based on physical inertia tensors and aerodynamic drag.

### Rotational Dynamics

Angular accelerations ($\alpha$) are derived from applied control torques ($u_2, u_3, u_4$) divided by the corresponding principal moments of inertia:

$$\alpha_{\text{roll}} = \frac{u_2}{I_{xx}}, \quad \alpha_{\text{pitch}} = \frac{u_3}{I_{yy}}, \quad \alpha_{\text{yaw}} = \frac{u_4}{I_{zz}}$$

Simulated air friction and angular damping are applied recursively to prevent numerical instability:

$$\omega_{\text{axis}} \leftarrow \omega_{\text{axis}} \cdot (1.0 - k_{\text{damping}} \cdot dt)$$

### Translational Dynamics & Tilt Coupling

Linear accelerations ($a_x, a_y, a_z$) project the collective thrust vector ($u_1$) across global coordinates based on the current body attitude (Euler angles) and account for aerodynamic drag coefficients ($f_{\text{aero}}$):

$$a_z = \frac{u_1}{m} \cos(\phi)\cos(\theta) - g - \frac{f_{\text{aero}}}{m} v_z$$

Where:
* $m$ = Vehicle mass (config.weight_kg)
* $\phi, \theta$ = Roll and Pitch angles
* $g$ = Gravity ($9.81\text{ m/s}^2$)

## 3. Asynchronous Key-Value Communication Protocol

Communication between the STM32 and the PyQt6 GCS operates over a virtual COM port via USB CDC using a non-blocking asynchronous packet structure.

* **Telemetry Downlink (STM32 $\rightarrow$ PC):** 

Transmits real-time states and reference targets at high frequency using compact comma-separated key-value pairs:
```
R:1.20,R_ref:0.00,P:-0.45,P_ref:0.00,Z:1.50,U1:9.81,X:0.20,X_ref:0.00,Y:-0.10,Y_ref:0.00\r\n
```

* **Command Uplink (PC $\rightarrow$ STM32):** 

Key inputs from the GCS modify PID parameters Kd, Ki and Kd in the firmware via lightweight character polling.
