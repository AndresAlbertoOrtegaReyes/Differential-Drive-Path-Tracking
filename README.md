# Differential Drive Robot: Path Tracking & Planning

![Platform](https://img.shields.io/badge/Hardware-ESP32%2FArduino-blue) ![Language](https://img.shields.io/badge/Code-C%2B%2B-orange) ![Control](https://img.shields.io/badge/Control-Kinematic-green) ![Status](https://img.shields.io/badge/Status-Completed-success)

## 📖 Overview
This repository contains the implementation of **Autonomous Navigation and Kinematic Control** algorithms for a differential drive mobile robot. The project solves the trajectory tracking problem using **Odometry** (Dead Reckoning) and control laws based on **Feedback Linearization**.

The system is designed to execute three distinct navigation strategies, implemented in C++ for microcontrollers (ESP32/Arduino).

## ✨ Key Features

### 1. Parametric Trajectory Tracking (`3trayectorias.ino`)
The robot tracks complex mathematical functions defined in time $x_d(t), y_d(t)$.
* **Circular Trajectory:** Generates sinusoidal references.
* **Sawtooth Wave:** Linear motion in $X$ with triangular oscillation in $Y$.
* **Root-Spiral (Constant Velocity):** Implements a spiral where the radius grows proportional to the square root of time ($\sqrt{t}$) to maintain a constant linear velocity.

### 2. Point-to-Point Planning (`puntoapunto.ino`)
Generates smooth velocity profiles to navigate from an initial pose $A$ to a final target $B$ within a specific time $t_f$.
* **Cubic Polynomials:** Interpolates position to ensure velocity continuity (avoiding acceleration spikes).
* **Automatic Coefficient Calculation:** Solves for coefficients ($a_0, a_1, a_2, a_3$) based on boundary conditions ($x_0, x_f, v_0=0, v_f=0$).

### 3. Waypoint Navigation (`splines.ino`)
The robot navigates through a sequence of 6 predefined control points using **Linear Splines** [cite: 7-10].
***Segmented Path:** Logic switches between linear equations based on accumulated time intervals ($t_{01}, t_{12}, \dots$).

## ⚙️ Mathematical Model
The control logic is based on the kinematic model of a differential robot located at $(x, y)$ with orientation $\theta$.

### Odometry (Estimation)
Position is estimated by integrating encoder pulses:
$$\Delta d = \frac{\Delta d_R + \Delta d_L}{2}, \quad \Delta \theta = \frac{\Delta d_R - \Delta d_L}{2L}$$
$$x_{k+1} = x_k + \Delta d \cos(\theta), \quad y_{k+1} = y_k + \Delta d \sin(\theta)$$

### Control Law (Feedback Linearization)
To avoid singularities in the differential drive model, a **Handel Point** $(x_h, y_h)$ is placed at a distance $D$ ahead of the wheel axis. The Proportional control is applied to this point:

$$u_x = \dot{x}_d + k_x (x_d - x_h)$$
$$u_y = \dot{y}_d + k_y (y_d - y_h)$$

The final linear ($v$) and angular ($\omega$) velocities are computed by inverting the robot's Jacobian:
$$v = u_x \cos(\theta) + u_y \sin(\theta)$$
$$\omega = \frac{-u_x \sin(\theta) + u_y \cos(\theta)}{D}$$

## 🛠️ Hardware Specifications
Derived from the codebase configuration:
* **Microcontroller:** ESP32 (inferred from GPIOs 32, 25, 21, etc.).
* **Actuators:** DC Motors with Gearbox and Quadrature Encoders.
* **Driver:** PWM-controlled H-Bridge (e.g., L298N).
* **Physical Parameters**:
    * Wheel Radius ($R$): **0.0335 m**
    * Track Width ($2L$): **0.25 m** ($L=0.125$)
    * Encoder Resolution ($N$): **572 PPR**
