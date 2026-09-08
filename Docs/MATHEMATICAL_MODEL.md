# Mathematical Modeling & Control Architecture

This document outlines the mathematical equations, kinematics, dynamics, and cascaded PID control architecture implemented in the 6-Degrees-of-Freedom (6DoF) STM32 Hardware-in-the-Loop (HIL) quadcopter simulator.

## 1. Kinematics & Coordinate Frames
The quadcopter's state vector is defined relative to an inertial earth-fixed frame ($E$) and a body-fixed frame ($B$). The attitude is represented by standard Euler angles using the 3-2-1 Tait-Bryan convention (Yaw $\psi$, Pitch $\theta$, Roll $\phi$):
* **$\phi$:** Roll (rotation about body $x$-axis)
* **$\theta$:** Pitch (rotation about body $y$-axis)
* **$\psi$:** Yaw (rotation about body $z$-axis)

### Kinematic Differential Equations (Euler Rates)
The relationship between body angular velocities $(p, q, r)$ and the time derivatives of the Euler angles $(\dot{\phi}, \dot{\theta}, \dot{\psi})$ is modeled as:

$$\dot{\phi} = p + q \sin(\phi)\tan(\theta) + r \cos(\phi)\tan(\theta)$$

$$\dot{\theta} = q \cos(\phi) - r \sin(\phi)$$

$$\dot{\psi} = \frac{q \sin(\phi)}{\cos(\theta)} + \frac{r \cos(\phi)}{\cos(\theta)}$$

## 2. Rigid-Body 6DoF Dynamics Engine
The translational and rotational motions are governed by rigid-body mechanics, accounting for inertia tensors, total vehicle mass ($m = 1.2\text{ kg}$), gravity ($g = 9.81\text{ m/s}^2$), actuator dynamics, and linear aerodynamic drag ($f_{\text{aero}} = 0.1$).

### A. Rotational Dynamics (Euler's Equations)
Assuming a symmetrical quadcopter frame where cross-products of inertia are negligible, the angular accelerations $(\dot{p}, \dot{q}, \dot{r})$ are computed from control torques $(U_2, U_3, U_4)$ and principal moments of inertia ($I_{xx}, I_{yy}, I_{zz}$):

$$\dot{p} = \frac{U_2 + (I_{yy} - I_{zz})q r}{I_{xx}}$$

$$\dot{q} = \frac{U_3 + (I_{zz} - I_{xx})p r}{I_{yy}}$$

$$\dot{r} = \frac{U_4 + (I_{xx} - I_{yy})p q}{I_{zz}}$$

### B. Translational Dynamics & Tilt Coupling
Linear accelerations $(a_x, a_y, a_z)$ project total collective thrust ($U_{1,\text{real}}$) onto the global frame based on the current attitude orientation, combined with aerodynamic damping:

$$a_x = \frac{U_{1,\text{real}}}{m} \left( \cos(\phi)\sin(\theta)\cos(\psi) + \sin(\phi)\sin(\psi) \right) - \frac{f_{\text{aero}}}{m} v_x$$

$$a_y = \frac{U_{1,\text{real}}}{m} \left( \cos(\phi)\sin(\theta)\sin(\psi) - \sin(\phi)\cos(\psi) \right) - \frac{f_{\text{aero}}}{m} v_y$$

$$a_z = \frac{U_{1,\text{real}}}{m} \left( \cos(\phi)\cos(\theta) \right) - g - \frac{f_{\text{aero}}}{m} v_z$$

Linear velocities and spatial coordinates are integrated iteratively using Euler integration with a fixed time step ($\Delta t = 0.01\text{ s}$):

$$v_x(t+\Delta t) = v_x(t) + a_x \Delta t, \quad x(t+\Delta t) = x(t) + v_x \Delta t$$

## 3. Cascaded Control Architecture (PID Loops)
The flight controller relies on a dual-loop hierarchical structure separating fast attitude stabilization from outer spatial navigation.

### A. Outer Navigation Loop (Position & Altitude)
* **Altitude Control ($Z$):** Computes total thrust corrections relative to altitude target ($z_{\text{ref}} = 5\text{ m}$):

$$e_z = z_{\text{ref}} - z_{\text{filtered}}$$

$$U_{1,\text{desired}} = m g + \left( K_{p,z} e_z + K_{i,z} \int e_z dt + K_{d,z} \frac{de_z}{dt} \right)$$

* **Horizontal Position Control ($X$ and $Y$):** Spatial tracking errors generate virtual attitude setpoints (Pitch $\theta_{\text{des}}$ and Roll $\phi_{\text{des}}$) bounded by a safety saturation limit ($\theta_{\text{max}} = 0.3\text{ rad} \approx 28^\circ$):

$$\theta_{\text{desired}} = \text{clamp}\left( K_{p,\text{xy}} e_x + K_{d,\text{xy}} \frac{de_x}{dt}, -\theta_{\text{max}}, \theta_{\text{max}} \right)$$

$$\phi_{\text{desired}} = \text{clamp}\left( -(K_{p,\text{xy}} e_y + K_{d,\text{xy}} \frac{de_y}{dt}), -\theta_{\text{max}}, \theta_{\text{max}} \right)$$

### B. Inner Attitude Loop (Stabilization)
Compares desired attitude targets against current Euler angles to yield body torques ($U_2, U_3, U_4$):

$$e_\phi = \phi_{\text{desired}} - \phi$$

$$U_2 = K_{p,\text{att}} e_\phi + K_{i,\text{att}} \int e_\phi dt + K_{d,\text{att}} \frac{de_\phi}{dt}$$

## 4. Actuator Dynamics & Sensor Filtering
* **Motor Lag Model:** Actuator physical response delay ($\tau_{\text{motor}} = 0.05\text{ s}$) is modeled via a discrete low-pass filter update:

$$\alpha = \frac{\Delta t}{\tau_{\text{motor}} + \Delta t}$$

$$U_{1,\text{real}} \leftarrow U_{1,\text{real}} + \alpha (U_{1,\text{desired}} - U_{1,\text{real}})$$

* **Sensor Noise & Estimation:** Simulated sensor measurements incorporate additive white Gaussian noise combined with a moving-average / exponential filter ($\beta = 0.1$) for altitude estimation:

$$z_{\text{measured}} = z + \mathcal{N}(0, 0.2)$$

$$z_{\text{filtered}} = (1 - \beta) z_{\text{filtered}} + \beta z_{\text{measured}}$$
