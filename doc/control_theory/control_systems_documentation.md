# Control Systems Documentation
## Brake Pedal Simulation: PID, Fuzzy Logic, and Sliding Mode Control

---

## Table of Contents

1. [Plant Model](#1-plant-model)
2. [PID Controller](#2-pid-controller)
3. [Fuzzy Logic Controller](#3-fuzzy-logic-controller)
4. [Sliding Mode Controller](#4-sliding-mode-controller)
5. [Simulation Setup](#5-simulation-setup)
6. [Results and Comparison](#6-results-and-comparison)

---

## 1. Plant Model

### 1.1 Second-Order Linear System

The brake hydraulic system is modeled as a **second-order linear plant**. This is a standard approximation for systems with inertia and damping, such as hydraulic actuators. The transfer function in the Laplace domain is:

$$G(s) = \frac{K \omega_n^2}{s^2 + 2\psi\omega_n s + \omega_n^2}$$

where:

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| Static gain | $K$ | 1.0 | DC gain of the system |
| Natural frequency | $\omega_n$ | 3.0 rad/s | Frequency of undamped oscillation |
| Damping ratio | $\psi$ | 0.8 | Ratio of actual to critical damping |

### 1.2 State-Space Representation

The transfer function is converted to state-space form for time-domain simulation:

$$\dot{x}_1 = x_2$$
$$\dot{x}_2 = -\omega_n^2 x_1 - 2\psi\omega_n x_2 + K\omega_n^2 u$$
$$y = x_1$$

where $x_1$ is the output (brake pressure), $x_2$ is its rate of change, and $u$ is the control input.

### 1.3 Numerical Integration

The states are integrated using the **forward Euler method** with time step $\Delta t = 0.01$ s:

$$x_1[k+1] = x_1[k] + \Delta t \cdot x_2[k]$$
$$x_2[k+1] = x_2[k] + \Delta t \cdot (-\omega_n^2 x_1[k] - 2\psi\omega_n x_2[k] + K\omega_n^2 u[k])$$

### 1.4 Damping Characteristics

The damping ratio $\psi$ determines the transient behavior:

| Condition | Value | Behavior |
|-----------|-------|----------|
| Underdamped | $\psi < 1$ | Oscillatory response |
| Critically damped | $\psi = 1$ | Fastest non-oscillatory response |
| Overdamped | $\psi > 1$ | Slow, no overshoot |

With $\psi = 0.8$ (slightly underdamped), the brake model exhibits a realistic response: fast pressure build-up with minor overshoot, physically representative of a hydraulic brake system.

### 1.5 Setpoint Profile

The driver input is modeled as a piecewise ramp, simulating a realistic brake pedal press and release:

$$r(t) = \begin{cases} 0 & t < 0.5 \\ t - 0.5 & 0.5 \leq t < 1.5 \\ 1.0 & 1.5 \leq t < 3.0 \\ 1.0 - (t - 3.0) & 3.0 \leq t < 4.0 \\ 0 & t \geq 4.0 \end{cases}$$

---

## 2. PID Controller

### 2.1 Theory

The **Proportional-Integral-Derivative (PID)** controller is the most widely used controller in industrial applications. It computes a control signal $u(t)$ based on the error $e(t) = r(t) - y(t)$ between the setpoint $r(t)$ and the plant output $y(t)$:

$$u(t) = K_p \, e(t) + K_i \int_0^t e(\tau)\, d\tau + K_d \frac{de(t)}{dt}$$

Each term has a distinct role:

- **Proportional** $K_p e(t)$: Reacts to the current error. Higher $K_p$ increases speed of response but can cause overshoot.
- **Integral** $K_i \int e \, dt$: Eliminates steady-state error by accumulating past errors. High $K_i$ causes windup and sluggish release.
- **Derivative** $K_d \frac{de}{dt}$: Anticipates future error by reacting to its rate of change. Adds damping and reduces overshoot.

### 2.2 Discrete Implementation

For digital implementation with time step $\Delta t$, the continuous terms are approximated:

$$u[k] = K_p \, e[k] + K_i \sum_{j=0}^{k} e[j] \cdot \Delta t + K_d \frac{e[k] - e[k-1]}{\Delta t}$$

### 2.3 Output Clamping (Anti-Windup)

Since a brake can only apply pressure (never pull), the control output is clamped to a unipolar range:

$$u[k] = \text{clamp}(u[k],\ u_{min},\ u_{max}) = \text{clamp}(u[k],\ 0,\ 10)$$

This also provides implicit anti-windup: the integrator cannot accumulate error beyond what the actuator can produce.

### 2.4 Tuned Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| $K_p$ | 10.0 | High proportional gain for fast ramp tracking |
| $K_i$ | 1.2 | Moderate to reduce windup during ramp-down |
| $K_d$ | 0.5 | Sufficient damping to offset high $K_p$ |
| $u_{min}$ | 0.0 | Unipolar — brakes only apply |
| $u_{max}$ | 10.0 | Maximum actuator authority |

### 2.5 Limitations

- **Integrator windup**: During the ramp phase the integrator accumulates a large value, causing delayed response on release. This is the primary limitation visible in the simulation.
- **Linear assumption**: PID is derived assuming a linear plant. Performance degrades if the plant is highly nonlinear.
- **Fixed gains**: A single set of gains cannot be optimal across all operating conditions.

---

## 3. Fuzzy Logic Controller

### 3.1 Theory

**Fuzzy Logic Control (FLC)** is a rule-based, nonlinear control strategy that mimics human reasoning. Instead of precise mathematical models, it uses linguistic rules such as "if error is large positive and error is increasing, apply large positive control."

The controller operates in three stages:

1. **Fuzzification**: Convert crisp inputs into fuzzy membership degrees
2. **Inference**: Evaluate fuzzy rules using the membership degrees
3. **Defuzzification**: Convert the fuzzy output back to a crisp control value

### 3.2 Membership Functions

A **triangular membership function** is defined by three points $(a, b, c)$:

$$\mu(x) = \max\left(0,\ \min\left(\frac{x-a}{b-a},\ \frac{c-x}{c-b}\right)\right)$$

It returns 1 at the center $b$ and 0 at the edges $a$ and $c$, linearly interpolating between them.

### 3.3 Linguistic Variables

#### Error $e = r - y$

Universe of discourse: $[-1.5,\ 1.5]$

| Label | $a$ | $b$ | $c$ | Meaning |
|-------|-----|-----|-----|---------|
| NB | -1.5 | -1.0 | -0.5 | Negative Big |
| NS | -1.0 | -0.5 |  0.0 | Negative Small |
| ZE | -0.3 |  0.0 |  0.3 | Zero |
| PS |  0.0 |  0.5 |  1.0 | Positive Small |
| PB |  0.5 |  1.0 |  1.5 | Positive Big |

#### Derivative of Error $\dot{e}$ (normalized)

The raw derivative is normalized before fuzzification to prevent saturation:

$$\dot{e}_{norm} = \text{clamp}\left(\frac{\dot{e}_{raw}}{D_{max}},\ -1,\ 1\right), \quad D_{max} = 10.0$$

Universe of discourse: $[-1.0,\ 1.0]$

| Label | $a$ | $b$ | $c$ | Meaning |
|-------|-----|-----|-----|---------|
| NG | -1.0 | -0.5 | 0.0 | Negative |
| ZE | -0.3 |  0.0 | 0.3 | Zero |
| PG |  0.0 |  0.5 | 1.0 | Positive |

#### Control Output $u$

Universe of discourse: $[-10.0,\ 10.0]$

| Label | $a$ | $b$ | $c$ | Meaning |
|-------|-----|-----|-----|---------|
| NB | -10.0 | -7.0 | -3.0 | Negative Big |
| NS |  -5.0 | -2.5 |  0.0 | Negative Small |
| ZE |  -1.0 |  0.0 |  1.0 | Zero |
| PS |   0.0 |  2.5 |  5.0 | Positive Small |
| PB |   3.0 |  7.0 | 10.0 | Positive Big |

### 3.4 Rule Base

The rule table encodes the control strategy. The key insight is **damping**: when the error is large but already shrinking fast (negative $\dot{e}$), the control effort should ease off to avoid overshoot.

| $e$ \ $\dot{e}$ | NG | ZE | PG |
|-----------------|----|----|-----|
| **PB** | PS | PB | PB |
| **PS** | ZE | PS | PB |
| **ZE** | NS | ZE | PS |
| **NS** | NB | NS | ZE |
| **NB** | NB | NB | NS |

Reading example: "If error is PS (small positive) and $\dot{e}$ is NG (error decreasing), apply ZE (zero) control" — the output is nearly at setpoint and approaching it, so hold steady.

### 3.5 Inference (Mamdani)

For each rule with antecedents $A_1, A_2$ and consequent $C$, the firing strength is computed using the **minimum** T-norm:

$$\alpha = \min(\mu_{A_1}(e),\ \mu_{A_2}(\dot{e}))$$

The output fuzzy set is clipped at height $\alpha$:

$$\mu_C'(u) = \min(\alpha,\ \mu_C(u))$$

All activated output sets are **aggregated** by taking the maximum at each point:

$$\mu_{agg}(u) = \max_i \mu_{C_i}'(u)$$

### 3.6 Defuzzification (Centroid)

The crisp output is computed as the **center of gravity** of the aggregated output set:

$$u^* = \frac{\int u \cdot \mu_{agg}(u)\, du}{\int \mu_{agg}(u)\, du}$$

In discrete form over the universe of discourse sampled at $N$ points:

$$u^* = \frac{\sum_{j=1}^{N} u_j \cdot \mu_{agg}(u_j)}{\sum_{j=1}^{N} \mu_{agg}(u_j)}$$

### 3.7 Unipolar Clamping

As with the PID, the fuzzy output is clamped to positive values only:

$$u_{fuzzy} = \max(0,\ u^*)$$

---

## 4. Sliding Mode Controller

### 4.1 Theory

**Sliding Mode Control (SMC)** is a nonlinear, robust control technique from Variable Structure Control theory. The core idea is to define a **sliding surface** $s(t) = 0$ in the state space such that, once the system trajectory reaches this surface, it "slides" along it toward the desired equilibrium.

SMC is inherently robust to matched uncertainties and disturbances — a major advantage over PID in systems with parameter variation (e.g., brake pad wear, temperature effects on hydraulic fluid viscosity).

### 4.2 Sliding Surface

For a second-order system with error $e = r - y$, the sliding surface is defined as:

$$s = e + \lambda \dot{e}$$

where $\lambda > 0$ is a design parameter that sets the slope of the surface in the phase plane $(e, \dot{e})$. When $s = 0$, the error dynamics satisfy:

$$\dot{e} = -\frac{1}{\lambda} e$$

which is a stable first-order system with time constant $\lambda$. A smaller $\lambda$ gives faster convergence.

### 4.3 Reaching Condition

To guarantee that trajectories outside the surface move toward it, the **reaching condition** (Lyapunov stability criterion) must be satisfied:

$$s \dot{s} < 0$$

This is achieved by the **relay control law**:

$$u = K_{smc} \cdot \text{sign}(s)$$

where $K_{smc}$ must be large enough to overcome disturbances and plant uncertainty.

### 4.4 Discrete Implementation

The derivative $\dot{e}$ is approximated with a backward difference:

$$s[k] = e[k] + \lambda \cdot \frac{e[k] - e[k-1]}{\Delta t}$$

The control law with unipolar clamping:

$$u[k] = \text{clamp}\left(K_{smc} \cdot \text{sign}(s[k]),\ 0,\ 10\right)$$

### 4.5 Chattering

The relay function $\text{sign}(s)$ causes **chattering** — high-frequency switching of the control signal near the sliding surface. This is visible in the simulation as a ripple in the output during the hold phase.

In practice, chattering is mitigated by replacing the sign function with a smooth approximation. The **hyperbolic tangent** (boundary layer approach):

$$u[k] = \text{clamp}\left(K_{smc} \cdot \tanh\left(\frac{s[k]}{\varepsilon}\right),\ 0,\ 10\right)$$

where $\varepsilon$ is the boundary layer thickness. A larger $\varepsilon$ reduces chattering at the cost of less precise sliding.

### 4.6 Tuned Parameters

| Parameter | Symbol | Value | Description |
|-----------|--------|-------|-------------|
| SMC gain | $K_{smc}$ | 5.0 | Switching amplitude |
| Surface slope | $\lambda$ | 1.5 | Error/derivative weighting |
| Boundary layer | $\varepsilon$ | 0.1 | Chattering reduction (tanh) |

### 4.7 Advantages and Limitations

**Advantages:**
- Robust to parameter uncertainty and disturbances
- Finite-time convergence to the sliding surface
- Simple to implement — only two tuning parameters

**Limitations:**
- Chattering can excite mechanical resonances in a real brake system
- Pure relay control is unipolar-incompatible without clamping
- No integral action — small steady-state error possible under constant disturbance

---

## 5. Simulation Setup

### 5.1 Parameters Summary

| Parameter | Value |
|-----------|-------|
| Time step $\Delta t$ | 0.01 s |
| Simulation duration | 5.0 s |
| Plant $K$ | 1.0 |
| Plant $\omega_n$ | 3.0 rad/s |
| Plant $\psi$ | 0.8 |

### 5.2 Software Architecture

The simulation is implemented in C++ with the following components:

- `SecondOrderPlant`: Euler-integrated second-order state-space model
- `PIDController`: Discrete PID with output clamping
- `FuzzyController`: Mamdani inference engine with centroid defuzzification
- SMC: Inline implementation in the simulation loop
- `Logger`: CSV and stdout logging via sink pattern
- Gnuplot: Real-time plotting via pipe

### 5.3 Real-Time Constraints

Each simulation step sleeps for 10 ms to match the $\Delta t = 0.01$ s step, providing visual real-time behavior in gnuplot. The gnuplot buffer is limited to 500 points (sliding window) to maintain rendering performance.

---

## 6. Results and Comparison

### 6.1 Qualitative Comparison

| Criterion | PID | Fuzzy | SMC |
|-----------|-----|-------|-----|
| Ramp tracking | Good | Good | Excellent |
| Steady-state error | Small (Ki limited) | Small | None |
| Overshoot | Minimal | Minimal | None (clamped) |
| Release lag | Moderate (windup) | Moderate | Fast |
| Chattering | None | None | Visible ripple |
| Tuning effort | Low | High | Low |
| Robustness | Low | Medium | High |

### 6.2 Key Observations

**PID**: Well-tuned parameters produce a response very close to the setpoint during the hold phase. The main weakness is integrator windup during the release phase, causing a lag of approximately 0.5–1.0 s before the output tracks the ramp-down.

**Fuzzy**: Closely matches the PID response once properly tuned, validating the rule base design. The nonlinear nature provides smooth transitions and natural damping. The main challenge is the sensitivity of performance to the normalization constants $E_{max}$ and $D_{max}$.

**SMC**: Fastest ramp tracking due to full-authority switching. Chattering during hold is physically interpretable as ABS-like pressure cycling. The unipolar clamp effectively converts the bipolar switching law to a brake-appropriate signal.

### 6.3 Physical Interpretation

In a real brake system, these controllers correspond to different ECU strategies:

- **PID** maps to classical hydraulic brake boosters with electronic pressure feedback
- **Fuzzy** maps to modern ABS/ESC systems that use heuristic control tables derived from driving data
- **SMC** maps to early ABS systems, where wheel lock detection triggered on/off solenoid valve switching — exactly the chattering behavior observed in simulation
