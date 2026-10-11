# 🤖 Intelligent Differential Drive Robot Simulation (P, PD & PID Control)
![Python](https://img.shields.io/badge/Python-3.10%2B-blue?logo=python&logoColor=white)
![Pygame](https://img.shields.io/badge/Pygame-2.x-green?logo=python&logoColor=white)
![Matplotlib](https://img.shields.io/badge/Matplotlib-Plots-orange)

## ✨ Key Features

* **Realistic Non-Ideal Plant Model:** Incorporates wheel velocity saturation ($\pm 200\text{ px/s}$), first-order electro-mechanical motor lag ($\tau = 0.35\text{ s}$), and a $4\%$ right-wheel speed mismatch to test steady-state disturbance rejection.
* **Three Switchable Control Architectures:**
  * **P Control (`1`):** Proportional distance + Proportional heading control (demonstrates inertia-induced target overshoot).
  * **PD Control (`2`):** Proportional-Derivative distance braking + Proportional heading control (eliminates terminal overshoot via virtual damping).
  * **PID Control (`3`):** PD distance control + full PID heading control with anti-windup clamping and an integral activation zone (eliminates steady-state heading drift and suppresses angular swing).
* **Real-Time Pygame Visualisation:** Live trajectory polyline rendering, heading vector indicator, start/target tolerance markers, final stopping cross, and a real-time telemetry HUD.
* **Automated Benchmarking & Plotting:** Headless execution scripts (`compare_controllers.py`) that run repeatable multi-scenario experiments and generate publication-ready comparative telemetry plots and gain-tuning sweeps.

---

## 📐 Mathematical Model & Control Architecture

### 1. Forward & Inverse Kinematics
Given wheel track separation $L$ and left/right ground wheel speeds $(v_L, v_R)$, the linear velocity $v$ and angular velocity $\omega$ of the robot chassis are governed by:

$$v = \frac{v_R + v_L}{2}, \qquad \omega = \frac{v_R - v_L}{L}$$

Controller velocity commands $(v, \omega)$ are mapped back to individual wheel speed commands via the inverse kinematics:

$$v_L = v - \frac{\omega L}{2}, \qquad v_R = v + \frac{\omega L}{2}$$

### 2. Mid-Point Odometry Integration
For global pose $[x, y, \theta]^T$ over a fixed simulation timestep $\Delta t = \frac{1}{60}\text{ s}$:

$$x_{k+1} = x_k + v_k \cos\left(\theta_k + \frac{\omega_k \Delta t}{2}\right)\Delta t$$

$$y_{k+1} = y_k + v_k \sin\left(\theta_k + \frac{\omega_k \Delta t}{2}\right)\Delta t$$

$$\theta_{k+1} = \theta_k + \omega_k \Delta t$$

### 3. Error Formulation & Feedback Control Laws
For a target at $(x_t, y_t)$, the Euclidean distance error $e_d$ and wrapped heading error $e_\theta \in (-\pi, \pi]$ are defined as:

$$e_d = \sqrt{(x_t - x)^2 + (y_t - y)^2}, \qquad e_\theta = \text{atan2}\big(\sin(\theta_d - \theta),\, \cos(\theta_d - \theta)\big)$$

where $\theta_d = \text{atan2}(y_t - y, x_t - x)$.

| Controller | Linear Velocity Law ($v$) | Angular Velocity Law ($\omega$) | Dynamic Behaviour |
| :--- | :--- | :--- | :--- |
| **P** | $K_{pd}\, e_d \max(0, \cos e_\theta)$ | $K_{p\theta}\, e_\theta$ | Fast approach, but under-damped ($\zeta = 0.49$); overshoots target and exhibits steady-state heading bias. |
| **PD** | $(K_{pd}\, e_d + K_{dd}\, \dot{e}_d) \max(0, \cos e_\theta)$ | $K_{p\theta}\, e_\theta$ | Critically damped braking ($\zeta \approx 0.98$); smooth stop with $0\text{ px}$ overshoot. |
| **PID** | $(K_{pd}\, e_d + K_{dd}\, \dot{e}_d) \max(0, \cos e_\theta)$ | $K_{p\theta}\, e_\theta + K_{i\theta} \int e_\theta\,dt + K_{d\theta}\, \dot{e}_\theta$ | Smooth stop + eliminates $4\%$ wheel-mismatch drift, reducing mean cruise heading error by $84\%$. |

---
