# Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM

## Overview
This project implements Indirect Sensored Field-Oriented Control (FOC) for a Three-Phase AC Induction Motor (ACI). It utilizes a rotor position sensor, Clarke and Park Transforms, Proportional-Integral (PI) Controllers, and Space Vector Pulse Width Modulation (SVPWM) to achieve precise speed and torque control. The goal is to ensure optimal motor performance with reduced harmonic distortion, improved dynamic response, and better efficiency.

## ⚙️ What Is Field-Oriented Control (FOC)?

**Field-Oriented Control (FOC)**, also known as **vector control**,  is an advanced technique used for the precise and high-performance control of AC motors. It enables independent regulation of motor torque and flux—similar to the control strategy used in DC motors—thereby ensuring smooth and efficient operation over a wide range of speeds and load conditions.

FOC is applicable to multiple motor types:

- 🌀 Induction Machines (IM)  
- 🧲 Permanent Magnet Synchronous Machines (PMSM)  
- 🔁 Brushless DC Motors (BLDC)

For applications requiring speeds beyond the rated value, FOC is often combined with **field weakening control** to extend the operating range.

### 🧩 Key Components of FOC Architecture

- **🔧 Current Controller:**  
  Uses two PI controllers to regulate d-axis (flux) and q-axis (torque) current components in the rotating frame.

- **📈 Speed Controller (Optional):**  
  An outer-loop PI controller generates current references based on the speed error.

- **🧮 Clarke and Park Transforms:**  
  - **Clarke Transform** – Converts 3-phase currents to 2-phase stationary frame (α-β).  
  - **Park Transform** – Converts α-β values to a rotating d-q frame aligned with rotor position.

- **🔄 Inverse Park Transform:**  
  Converts control outputs from d-q back to α-β frame for PWM generation.

- **📊 Space Vector Modulation (SVM):**  
  Converts α-β voltage commands into optimized PWM signals for efficient inverter switching signals applied to stator windings.

- **🛡️ Protection and Logic Blocks:**  
  Implements safety mechanisms like overcurrent, overvoltage, and startup/shutdown handling.

- **🧠 Rotor Position Estimator (Optional for Sensorless):**  
  Estimates rotor angle and speed when no physical sensor is used.

## 📊 Block Diagram

```txt
        Speed Ref      Measured Speed
            |               |
            v               |
     +---------------------------+
     |   Velocity Controller     | (optional)
     +---------------------------+
            | Iq_ref
            v
     +---------------------------+
     |   Current Controller      |
     |   (PI controllers)        |
     +---------------------------+
           | Id_ref, Iq_ref
           v
     +---------------------------+
     |     Park Transform        | --> dq frame
     +---------------------------+
           | Vd, Vq
           v
     +---------------------------+
     | Inverse Park Transform    | --> αβ frame
     +---------------------------+
           | Vα, Vβ
           v
     +---------------------------+
     | Space Vector Modulation   |
     +---------------------------+
           | PWM signals
           v
        Inverter -> Motor

  (Clarke Transform applied to input currents for feedback)

```

---------

# Quadrature Decoder

The **Quadrature Decoder** module calculates the angular position of a rotating shaft using signals from a quadrature encoder. It supports position output in degrees, radians, or per-unit, based on encoder counts.

This decoder is essential in motion control systems, particularly for precise rotor position measurement in applications like Field-Oriented Control (FOC), servo control, and motor feedback systems.

---

<p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/9a5dc663665fb4e69fff79570d258ed58f25713f/Photos%20of%20Encoder/Diagram%20of%20eQEP.png" width="500">
</p>  


In this example, the timer driven by the QEP increments by four for each slit:

<p align="center"><b>Figure 1:</b> Basic Diagram of eQEP</p>  

<p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/9a5dc663665fb4e69fff79570d258ed58f25713f/Photos%20of%20Encoder/Waveform%20of%20eQEP.png" width="500">
</p>  

<p align="center"><b>Figure 2:</b> Genral Waveform of eQEP</p>  

## 📌 Overview

A **quadrature encoder** provides two square-wave signals (QEPA and QEPB) 90 degrees out of phase and an optional **index pulse (QEPI)** once per revolution. The decoder uses these inputs to determine the rotor position.

### Supported Modes:
- With **Index Pulse**: Accurate absolute position per revolution.
- Without **Index Pulse**: Relative position based only on encoder counts.

---

## 🧩 How It Works

### Inputs:
- **Cnt**: Current encoder counter value.
- **Idx**: Encoder count value at the last index pulse (optional).
- **Counts Per Revolution**: Total counts per mechanical rotation.

---

# 🧭 Quadrature Encoder Angular Position Calculation

This module computes the **angular position** (in counts or angle) from a quadrature encoder using the `Cnt` (counter) and `Idx` (index) signals.

---

## 🧮 Position Count Calculation

The position count is computed based on the values of `Cnt` and `Idx`.

### Case 1: When `Cnt ≥ Idx`

```math
\text{Position} {\text{count}} = \text{Cnt} - \text{Idx}
```

### 📈 Case 2: When `Cnt < Idx`

If the encoder counter value `Cnt` is less than the index position `Idx`, the position count is calculated as:

```math
\text{Position} {\text{count}} = \text{Counts} {\text{per revolution}} + (\text{Cnt} - \text{Idx})
```
When Cnt (an unsigned integer) exceeds the maximum value of the selected counter size, the block adds the necessary compensation internally.

When the **External Index Count** checkbox is **unchecked**, only the counter `Cnt` is used to compute the position:

```math
\text{Position} {\text{count}} = \text{Cnt}
```

### Where,

- **Position_count**: The angular position of the quadrature encoder in counts.
- **Counts per revolution**: The total number of counts in one mechanical revolution of the encoder:

```math
\text{Counts} {\text{per revolution}} = \text{Encoder} {\text{slits}} \times \text{Encoder} {\text{counts per slit}}
```

### 📐 Angular Output Calculation (θₘ)

The angular output (θₘ) is calculated using the following formula:

```math
\theta_m = \text{MaxPosition} \times \left( \frac{\text{Position count}} {\text{Encoder slits} \times \text{Encoder counts per slit}} \right)
```

### 🔁 Compact Angular Output Expression

Or more compactly, the angular output (θₘ) can be written as:

```math
\theta_m = \text{MaxPosition} \times \left( \frac{\text{Position count}} {\text{Counts per revolution}} \right)
```

### Where, 
- MaxPosition = 360 (degrees) or 2π (radians) or 1 (per-unit), based on the selected value of the Position unit parameter.


# Modeling of the Induction Machine

The dynamic modeling of a three-phase Induction Machine (IM) in the **rotating reference frame (RRF)** using the **dq0 transformation**. This modeling is commonly used in simulation and control of induction motors in vector control or field-oriented control applications.

By transforming the stator and rotor quantities to a synchronously rotating reference frame, we convert sinusoidal variables into DC quantities in steady-state, simplifying analysis and control design.

---

## 1. Stator Voltage Equation

The stator voltage equation in the rotating reference frame (dq frame) is [4]:

$V_{dqs} = R_s I_{dqs} + \frac{d\Psi_{dqs}}{dt} + j\omega_r \Psi_{dqs} \quad (1)$

### Where,

* $V_{dqs}$ is the Stator voltage vector in the dq frame, composed of d-axis and q-axis components.
* $R_s$ is the Stator resistance, representing the ohmic loss in the stator windings.
* $I_{dqs}$ is the Stator current vector in the dq frame.
* $\Psi_{dqs}$ is the Stator flux linkage vector, which includes the effect of stator and rotor currents.
* $\omega_r$ is the Electrical angular frequency of the rotating reference frame.

The equation consists of three terms: ohmic voltage drop $R_s I_{dqs}$, time derivative of flux linkage $\frac{d\Psi_{dqs}}{dt}$, and the rotational EMF induced due to the rotating reference frame $j\omega_r \Psi_{dqs}$.

---

## 2. Rotor Voltage Equation

For squirrel-cage induction motors (where rotor voltages are zero), the rotor voltage equation is [4]:

$0 = R_r I_{dqr} + \frac{d\Psi_{dqr}}{dt} + j(\omega_r - \omega_m) \Psi_{dqr} \quad (2)$

### Where,

* $R_r$ is the Rotor resistance, representing the copper loss in rotor windings.
* $I_{dqr}$ is the Rotor current vector in the dq frame.
* $\Psi_{dqr}$ is the Rotor flux linkage vector.
* $\omega_m$ is the Mechanical angular speed of the rotor.
* $\omega_r$ is the Reference frame electrical angular frequency.

This equation models the induced voltage in the rotor. The difference $\omega_r - \omega_m$ defines the slip frequency, indicating how far the rotor is from synchronous speed.

---

## 3. Flux Linkage Equations

The flux linkage in the stator and rotor windings is expressed as [4]:

$\Psi_{dqs} = L_s I_{dqs} + L_m I_{dqr} \quad (3)$

$\Psi_{dqr} = L_r I_{dqr} + L_m I_{dqs} \quad (4)$

### Where,

* $L_s$ is the Total stator inductance (includes leakage and magnetizing components).
* $L_r$ is the Total rotor inductance.
* $L_m$ is the Mutual inductance between stator and rotor.

These equations reflect the magnetic coupling between stator and rotor circuits. The mutual inductance $L_m$ allows energy transfer from the stator to the rotor via the rotating magnetic field.

---

## 4. Electromagnetic Torque Equation

The torque produced by the motor is given by [4]:

$T_{em} = \frac{3}{2} p (\Psi_{ds} I_{qs} - \Psi_{qs} I_{ds}) \quad (5)$

### Where,

* $T_{em}$ is the Electromagnetic torque developed by the motor.
* $p$ is the Number of pole pairs.
* $\Psi_{ds}, \Psi_{qs}$ is the d- and q-axis stator flux linkages.
* $I_{ds}, I_{qs}$ is the d- and q-axis stator currents.

This torque expression resembles the cross-product of the stator flux and current vectors, highlighting that torque is maximized when these vectors are orthogonal.

This equation forms the basis for Field-Oriented Control (FOC), where control is achieved by aligning the rotor flux with one axis and controlling torque through the orthogonal axis.

---

## 5. Steady-State Equivalent Circuit (Per Phase)

The steady-state behavior of the induction machine can be analyzed using its equivalent circuit as shown in the figure below. This circuit is widely used to study machine performance parameters such as power factor, torque, and efficiency.

<p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/9a5dc663665fb4e69fff79570d258ed58f25713f/Photos%20for%20ACIM/Steady-State%20Equivalent%20Circuit%20of%20a%20Squirrel%20Cage%20Induction%20Machine.png" width="500">
</p>  

<p align="center"><b>Figure 3:</b> Steady-State Equivalent Circuit of a Squirrel Cage Induction Machine</p>  

### Where,

* $V_s$ is the Stator voltage
* $R_s$ is the Stator resistance
* $j\omega L_{\gamma s}$ is the Stator leakage reactance
* $R_{fe}$ is the Core (iron) loss resistance
* $j\omega L_m$ is the Magnetizing reactance
* $R_r \frac{1-s}{s}$ is the Rotor resistance referred to stator side, accounting for slip $s$
* $j\omega L_{\gamma r}$ is the Rotor leakage reactance

The magnetizing branch represents the core magnetization and iron loss components. In practice, the iron loss component ($I_{fe}$) is often neglected due to its relatively small value.

The total rotor resistance in the circuit is modeled as $R_r \frac{1-s}{s}$, where $s$ is the slip:

$R_{r\_eq} = R_r \left(\frac{1-s}{s}\right)$

---

### Parameter Definitions:

| Parameter                 | Symbol         |
| ------------------------- | -------------- |
| Stator resistance         | $R_s$          |
| Stator inductance         | $L_s$          |
| Stator leakage inductance | $L_{\gamma s}$ |
| Mutual inductance         | $L_m$          |
| Rotor resistance          | $R_r$          |
| Rotor inductance          | $L_r$          |
| Rotor leakage inductance  | $L_{\gamma r}$ |
| Iron resistance           | $R_{fe}$       |

Note: Rotor parameters are referred to the stator side.

---

## 6. Rotor Current Expression

From the previously derived equation (4), the rotor current $I_{dqr}$ can be isolated and written in terms of rotor flux linkage $\Psi_{dqr}$, magnetizing inductance $L_m$, and stator current $I_{dqs}$ as:

$I_{dqr} = \frac{1}{L_r} \left(\Psi_{dqr} - L_m I_{dqs}\right) \quad (6)$

This expression is essential in the modeling of a squirrel cage induction machine because the rotor current cannot be directly measured. By expressing $I_{dqr}$ in terms of measurable or computable quantities, we can eliminate the rotor current from the machine model, allowing further analysis and control design to rely solely on stator-side variables.

---

## 7. Stator Flux Equation after Eliminating Rotor Current

To remove the inaccessible rotor current $I_r$ from the stator flux equation, we substitute equation (6) into equation (3). This yields the following expression for the stator flux linkage:

$\Psi_{dqs} = L_\sigma I_{dqs} + \frac{L_m}{L_r} \Psi_{dqr} \quad \text{with} \quad L_\sigma = L_s - \frac{L_m^2}{L_r} \quad (7)$

Here, $L_\sigma$ represents the total leakage inductance, which accounts for both stator and rotor leakage effects. This formulation simplifies the flux linkage expression by separating the effects of stator leakage and mutual coupling.

---

## 8. Rotor Circuit Equation after Eliminating Rotor Current

Similarly, we can substitute equation (6) into the rotor circuit dynamic equation (originally labeled as equation (2)) to eliminate the rotor current $I_r$. This results in the following differential equation for rotor flux dynamics:

$\frac{d \Psi_{dqr}}{dt} = \frac{L_m}{T_r} I_{dqs} - \left( j(\omega_r - \omega_m) + \frac{1}{T_r} \right) \Psi_{dqr} \quad \text{with} \quad T_r = \frac{L_r}{R_r} \quad (8)$

This equation captures the evolution of rotor flux linkage $\Psi_{dqr}$ over time, incorporating the slip frequency $(\omega_r - \omega_m)$ and the rotor time constant $T_r$. The term $\frac{L_m}{T_r} I_{dqs}$ signifies the contribution of stator current to rotor flux, while the remaining term governs the decay and dynamic interaction of rotor flux in the rotating reference frame.

# Rotor Flux Control

The next step is to establish how to control the rotor flux. In an induction machine with a squirrel cage, there is no direct access to the rotor circuit, and there are no permanent magnets on the rotor. As a result, the rotor must be magnetized indirectly through the stator currents.

Equation (8) from the previous section provides insight into how rotor flux evolves over time. It suggests that the rotor flux can be influenced by the stator current. Assuming that the rotating reference frame (RRF) is correctly aligned with the rotor flux (i.e., the d-axis aligns with the rotor flux vector), equation (8) can be split into real and imaginary components:

$\frac{d \Psi_r}{dt} = \frac{L_m}{T_r} I_{ds} - \frac{1}{T_r} \Psi_r \quad (9)$

$0 = \frac{L_m}{T_r} I_{qs} - (\omega_r - \omega_m) \Psi_r \quad (10)$

From equation (9), in the steady state $\frac{d\Psi_r}{dt} = 0$, the rotor flux $\Psi_r$ becomes directly proportional to the d-axis component of the stator current:

$\Psi_r = L_m I_{ds} \quad (11)$

This shows that rotor flux control is achieved via the d-axis current. Maintaining a constant $I_{ds}$ leads to a constant rotor flux in steady-state.

---

# Torque Control

Once the rotor is magnetized (i.e., once $\Psi_r$ is established), the machine is capable of producing torque. Equation (5) describes torque as a function of the stator current and the flux linkage. Assuming that the RRF remains aligned with the rotor flux, we revisit the flux expressions from equation (7), separating them into their d- and q-axis components:


$$
\begin{cases}
\Psi_{ds} = L_\sigma I_{ds} + \frac{L_m}{L_r} \Psi_r \\
\Psi_{qs} = L_\sigma I_{qs} 
\end{cases} \quad (12)
$$

Now, substituting equation (12) into the torque expression (originally equation (5)) gives us an expression for torque in terms of $\Psi_r$ and $I_{qs}$:

$T_{em} = \frac{3}{2} p \frac{L_m}{L_r} \Psi_r I_{qs} \quad (13)$ 

This shows that torque is directly proportional to the q-axis stator current $I_{qs}$, as long as $\Psi_r$ is kept constant through d-axis current control.

> ⚠️ **Note:** Unlike Permanent Magnet Synchronous Machines (PMSMs), where torque is typically produced only by the q-axis current component, an Induction Machine (IM) requires the d-axis current component as well to first magnetize the rotor. Thus, both d-axis and q-axis components of stator current play essential roles in torque generation.


# Current Controller for Rotor Field-Oriented Control (RFOC)

As previously discussed, the rotor flux and electromagnetic torque are directly proportional to the d-axis and q-axis components of the stator current, respectively. Therefore, controlling these current components enables precise regulation of the rotor flux and torque via a stator current controller.

The stator circuit is described by (1). By injecting (7) into (1), it is possible to replace the stator flux with the rotor flux.

---

## 1. Stator Circuit Modeling

The stator voltage equation in the **d-q synchronous reference frame** is:
 
$V_{dqs} = R_s I_{dqs} + L_σ \frac{dI_{dqs}}{dt} + jω_r L_σ I_{dqs} + \frac{L_m}{L_r} \frac{dψ_{dqr}}{dt} + jω_r \frac{L_m}{L_r} ψ_{dqr} \quad (14)$

### Where,
- $R_s$ is the stator resistance  
- $L_σ$ is the stator leakage inductance  
- $L_m$ is the magnetizing inductance  
- $L_r$ is the rotor inductance  
- $I_{dqs}$ is the stator current in the d–q reference frame  
- $ψ_{dqr}$ is the rotor flux linkage in the d–q reference frame  
- $ω_r$ is the rotor angular speed  
- $ω_s$ is the stator angular speed  
- $V_{dqs}$ is the stator voltage in the d–q reference frame

---

## 2. Decoupling the d-q Axes

By aligning the rotor reference frame (RRF) along the rotor flux, the system equations separate into **real** (d-axis) and **imaginary** (q-axis) components (14):

$$
\begin{cases}
V_{ds} = R_s I_{ds} + L_σ \frac{dI_{ds}}{dt} - ω_r L_σ I_{qs} + \frac{L_m}{L_r} \frac{dψ_r}{dt} \\
V_{qs} = R_s I_{qs} + L_σ \frac{dI_{qs}}{dt} + ω_r L_σ I_{ds} + ω_r \frac{L_m}{L_r} ψ_r
\end{cases} \quad (15)
$$

The **cross terms** $ω_r L_σ I_{qs}$ and $ω_r L_σ I_{ds}$ introduce coupling between d and q axes, making independent control challenging.  
To handle this, a **decoupling network** is introduced to cancel out the cross-coupling terms.

---

## 3. Current Control Transfer Function in the Synchronous Reference Frame

The **first two terms** of the equations in (15) express the relationship between the **stator voltage** and **stator current**. This relationship can be reformulated as a transfer function, which is the same on both the \( d \)- and \( q \)-axes:

$H_d(s) = H_q(s) = \frac{I_s(s)}{V_s(s)} = \frac{1 / R_s}{1 + s L_\sigma / R_s} = \frac{K_1}{1 + s T_1} \quad (16)$

This transfer function expresses how the **current** reacts to a change in the **voltage input** generated by the inverter. With a **PI controller**, it is possible to control the stator currents. PI tuning is discussed in the next section.

The **third term** in equation (15) corresponds to the **coupling of the axes**.  As explained a **decoupling network** can be used to enable **truly independent control** of the \( d \)- and \( q \)-axes.

The **last term** relates to the effect of the **rotor flux** on the stator circuit. To simplify torque control, it is desirable to **keep the rotor flux constant**.

Since:
$\frac{d \Psi_r}{dt} = 0$

Only the \( q \)-axis is influenced by the rotor flux.  
The term:
$\omega_r \frac{L_m}{L_r} \Psi_r$

Can be computed by the controller and **added to the \( q \)-axis voltage reference**.

> ⚠️ **Note:**  It might be confusing to see $\omega_r$ in equation (15), since the current controller operates on the **stator side**. It would seem that $\omega_s$ should be used instead.

However, in the **stationary reference frame**, both the stator and rotor fluxes rotate at the same speed.  
Therefore:
$\omega_r = \omega_s
$

---

# Controller Tuning and Current Reference Strategy

This section describes how the PI controller is tuned using the *magnitude optimum* criterion and how current references are derived for Field Oriented Control (FOC) of AC machines.

---

## 1. Tuning of the Controller

As discussed in [5], the magnitude optimum criterion provides an effective method for tuning a PI controller, particularly when the plant’s transfer function follows the structure given in equation (16).

The corresponding PI controller parameters are given by:

$$
\begin{cases}
T_n &= T_1 \\
T_i &= 2K_1 T_d \\
K_p &= \frac{T_n}{T_i} \\
K_i &= \frac{1}{T_i}
\end{cases} \quad (17)
$$

- $T_d$ is the **total delay** in the system, which includes all small delays such as:
  - Computational delays
  - PWM update delays
  - Inverter delays

This tuning method provides good stability and performance, particularly in real-time control applications such as motor drives.

---

## 2. Current References

To operate the machine under nominal conditions, the rotor flux must also be nominal.

From the rotor flux equation:

$I_{ds}^* = \frac{\Psi_r^*}{L_m} = \frac{\Psi_m}{L_m} = I_{dsn} \quad (18)$

### Where,
- $I_{ds}^*$ is the D-axis current reference  
- $\Psi_r^*$ is the Reference rotor flux linkage  
- $L_m$ is the Mutual inductance

The q-axis current reference, which controls torque, is derived as (13):

$I_{qs}^* = \frac{T_e^*}{\frac{3}{2}p \cdot \frac{L_m}{L_r} \cdot \Psi_r} \quad (19)$

### Where,
- $T_e^*$ is the Desired electromagnetic torque  
- $p$ is the Number of pole pairs  
- $L_r$ is the Rotor inductance

---

There is, however, a minor inconvenience at startup: the machine **cannot be magnetized instantaneously**. As a result, while the flux $\psi_r$ is still building up, the current consumption will be high, as described by equation (19).

If the high current consumption at startup is a concern, one common approach is to **assume the flux to be constant and equal to its nominal value**. In this case, the machine will not be able to fully produce the desired torque reference, since the rotor flux is being overestimated. However, this discrepancy only occurs during the short startup transient. Therefore, its impact on the overall control performance is minimal.

> ℹ️ The estimation of $\Psi_r$ is discussed further in the **RRF orientation** section.

# Calculation of the Nominal d-axis Current

If the nominal rotor flux is unknown in equation (18), the nominal d-axis current can be calculated from the nominal voltage, the nominal current, and the equivalent circuit of the machine.

## Equivalent Circuit

From the equivalent circuit:

$$V_s = R_s I_s + j \omega_s (L_s - L_m) I_s + V_m \quad (20)$$

Taking $\arg(V_s) = 0 \, \text{rad}$ as the reference phase, the real and imaginary parts of $V_m$ are:

$$
\begin{cases}
\text{Re}(V_m) = V_s - R_s I_s \cos(\phi) + \omega_s (L_s - L_m) I_s \sin(\phi) \\
\text{Im}(V_m) = -R_s I_s \sin(\phi) - \omega_s (L_s - L_m) I_s \cos(\phi)
\end{cases} \quad (21)
$$

Where $\cos(\phi)$ is the power factor of the machine.

## d-axis Current Calculation

If the rotating reference frame (RRF) is correctly aligned on the rotor flux, the magnetizing current is equal to the d-axis component of the stator current. Assuming that iron losses can be neglected:

$$V_m = \sqrt{\text{Re}(V_m)^2 + \text{Im}(V_m)^2} = \omega_s L_m I_m = \omega_s L_m \left(\frac{I_{ds}}{\sqrt{2}}\right) \quad (22)$$

From equation (22), the d-axis current is:

$$I_{ds} = \frac{\sqrt{2} V_m}{\omega_s L_m} \quad (23)$$

The nominal d-axis stator current is found by solving equations (21) to (23) using the nominal voltage  $V_{sn}$ and current $I_{sn}$ of the IM. The power factor is usually available in the machine datasheet.

---

# Dynamic Saturation

The output of the current controller is a voltage reference to generate with the voltage-source inverter (VSI). However, the maximal output voltage of the VSI is limited by the DC bus voltage $V_{dc}$. In the rotating reference frame (RRF), this saturation limit corresponds to a circle of radius $\frac{V_{dc}}{\sqrt{3}}$. As long as the reference voltage is inside that circle, it can be generated by the VSI:

<p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/9a5dc663665fb4e69fff79570d258ed58f25713f/Photos%20for%20ACIM/Dynamic_Saturation.png" width="300">
</p>  

<p align="center"><b>Figure 4:</b> Dynamic Saturation in the RRF</p>  

The current controller must take the saturation of the VSI into account with a proper anti-windup strategy. Since there is one PI controller per axis, saturation limits should be defined on a per-axis basis.

A first option is to use the same limits for both axes. In the figure above, $V_1^*$ represents the extreme case where the full DC bus voltage is utilized, and both components have identical saturation limits. In this case, the limits are set to:

$$
\pm \frac{V_{dc}}{\sqrt{6}}
$$

While this approach is simple to implement, it does not fully utilize the potential of the DC bus: $V_2^*$ and $V_3^*$ are both valid reference vectors even if they lie outside the $\pm \frac{V_{dc}}{\sqrt{6}}$ boundaries.

A second option is then to compute the saturation limits _dynamically_. According to (19), the IM cannot produce a torque if the rotor is not magnetized beforehand. Therefore, the d-axis PI current controller should have priority. With this logic, $V_{ds}$ can use whatever it needs from the DC bus voltage, and $V_{qs}$ takes whatever is left:

$$
\begin{cases}
|V_{ds}| \le \displaystyle\frac{V_{dc}}{\sqrt{3}},\\
|V_{qs}| \le \displaystyle\sqrt{\frac{V_{dc}^2}{3} - V_{ds}^2}
\end{cases} \quad (24)
$$

The saturation limits are then expressed as:

$$
\begin{cases}
V_{ds,\mathrm{sat}} = \pm\ \dfrac{V_{dc}}{\sqrt{3}},\\
V_{qs,\mathrm{sat}} = \pm\ \sqrt{\dfrac{V_{dc}^2}{3} - \min\bigl(V_{ds}^2,\;\tfrac{V_{dc}^2}{3}\bigr)}
\end{cases} \quad (24)
$$

---

# RRF Orientation for Rotor Field-Oriented Control

In the previous sections, the RRF was assumed to be oriented on the rotor flux. The question is then: how to orient the RRF correctly?  
To align the RRF on the rotor flux, the position of the flux must be known. In [6] and [7], the authors present a FOC variant called *Indirect Field-Oriented Control (IFOC)*.  
This method is **indirect** because the rotor flux is estimated from the model of the machine.

## 1. Rotor Flux Estimation from d-axis Current
Let us assume that the RRF is indeed oriented on the rotor flux. In this case, from (9), using the d-axis stator current $I_{ds}$, the rotor flux $\Psi_r$ can be estimated as:

$$\Psi_r = \frac{L_m}{s T_r + 1} I_{ds} \quad (26)$$

### Where,
- $L_m$ = mutual inductance,
- $T_r = \frac{L_r}{R_r}$ = rotor time constant,
- $s$ = Laplace operator (in frequency domain),
- $I_{ds}$ = d-axis component of stator current.

This equation is typically implemented as a first-order low-pass filter in a digital system.

## 2. Slip Frequency Calculation from q-axis Current
According to (10), From the q-axis component of the stator current $I_{qs}$, the **slip angular frequency** $\omega_{slip}$ is derived using:

$$\omega_{\text{slip}} = \omega_r - \omega_m = \frac{L_m}{T_r \Psi_r} I_{qs} \quad (27) $$

### Where,
- $\omega_r$ = electrical rotor speed,
- $\omega_m$ = mechanical rotor speed,
- $\omega_{slip}$ = slip frequency.

## 3. Computing Rotor Electrical Speed
Then, Rearranging equation (27), the electrical speed of the rotor becomes:

$$\omega_r = \omega_m + \omega_{\text{slip}} \quad (28)$$

### Where,
- $\omega_m$ can be **measured from an encoder** or **estimated using observers**.

The rotor mechanical speed $\omega_m$ is either measured or estimated.  

## 4. Integrating Rotor Speed to Compute RRF Angle
Finally, the RRF position $\theta_r$ is obtained by **integrating the rotor electrical speed**:

$$\theta_r = \int_0^t \omega_r dt \quad (29)$$

This angle is then used in the **Park and Inverse Park transformations** to rotate the measured stator quantities between stationary and rotating frames.

> ⚠️ Since RFOC aligns the RRF on the rotor flux, the angle $\theta_r$ must be used for the Park transform.  
> Even if $\omega_r = \omega_s$, $\theta_r \ne \theta_s$  because there is a phase difference between the stator and rotor fluxes.

---





<!-- 
# ⚙️ ACIM Torque Estimator

Estimate **electromechanical torque** and **power** for an Induction Motor using d-q axis current and rotor speed inputs.  

---

## 📖 Description

The **ACIM Torque Estimator** block calculates the **electromechanical torque** and **power output** of an induction motor based on feedback signals. It assumes constant motor parameters and is most accurate when paired with a physical torque sensor.

**Inputs**:
- d-axis stator current (`isd`)
- q-axis stator current (`isq`)
- mechanical rotor speed (`ωm`)

**Outputs**:
- Electromechanical Torque (`Te`)
- Estimated Power (`Pe`)

---

## 📐 Equations Used

When Per-Unit (PU) is selected in the input settings, the block automatically converts all input values from PU to SI units before performing any internal computations. Once the outputs are calculated, they are converted back to PU for output.

### Torque Equation

```math
T_e = \frac{3}{2}p\left(\frac{L_m}{L_r}\right)\lambda_{rd}i_{sq}
```

### Where,
- `Te` is the Electromagnetic torque (Nm)
- `p` is the Number of pole pairs in the motor                    
- `Lm` is the Magnetizing inductance (H)                           
- `Lr` is the Rotor inductance (H)                                 
- `λrd` is the d-axis rotor flux linkage (Wb)                       
- `isq` is the q-axis stator current (A)                            

### Power Equation

```math
P_e = T_e \cdot \omega_m
```

### Where,
- `Pe` is the Mechanical output power (W)

- `ωm` is the Mechanical angular speed (rad/s)

---


# ⚡ ACIM Feed Forward Control

Decouple **d-axis** and **q-axis** currents to eliminate disturbances and improve dynamic performance  

---

## 📖 Description

The **ACIM Feed Forward Control** block decouples the **d-axis** and **q-axis** stator currents and calculates the feed-forward voltage gains for **Field-Oriented Control (FOC)** of an induction motor.

**Inputs**:
- d-axis stator current (`isd`)
- q-axis stator current (`isq`)
- mechanical rotor speed (`ωm`)

**Outputs**:
- Feed-forward voltage gains `V_{sd}^{FF}`, `V_{sq}^{FF}`

If **Per-Unit (PU)** is selected for inputs, values are internally converted to SI before computation and then reconverted to PU for output.

---

## 📐 Equations Used

### The machine inductances and stator flux are represented as,

```math
L_s = L_{ls} + L_m
```

```math
L_r = L_{lr} + L_m
```

```math
\sigma = 1 - \left( \frac{L_m^2}{L_s L_r} \right)
```

### Where, 

- `Ls` is the Stator inductance (H)
- `Lls` is the Stator leakage inductance (H)                         
- `Llr` is the Rotor leakage inductance (H)                          
- `Lm` is the Magnetizing inductance (H)                            
- `Lr` is the Rotor inductance (H)                                                                   
- `σ` is the Total leakage factor of the motor                     
  
### Flux Linkage

```math
\lambda_{sd} = \frac{L_m}{L_r} \lambda_{rd} + \sigma L_s i_{sd}
``` 

```math
\lambda_{sq} = \sigma L_s i_{sq}
```

### Where, 
                   
- `λsd` is the d-axis stator flux linkage (Wb)                       
- `λsq` is the q-axis stator flux linkage (Wb)
- `λrd` is the d-axis rotor flux linkage (Wb)                                    
- `isd` is the d-axis stator currents (A)
- `isq` is the q-axis stator currents (A)
  
---

### These equations describe how the block computes the feed-forward gain.

```math
V_{sd}^{FF} = \omega_e \lambda_{sq}
```

```math
V_{sq}^{FF} = -\omega_e \lambda_{sd}
```

### Where, 

- `ωe` is the Electrical speed corresponding to stator voltage frequency (rad/s)                   
- `λsd` is the d-axis stator flux linkage (Wb)                       
- `λsq` is the q-axis stator flux linkage (Wb)                                     

---


# ⚙️ ACIM Control Reference

Compute **d-axis** and **q-axis** reference currents for field-oriented control of induction motor  

---

## 📖 Description

The **ACIM Control Reference** block calculates the **d-axis** and **q-axis** current references used in **Field-Oriented Control (FOC)** and field-weakening operations of an induction motor.

**Functionality**:
- Accepts **reference torque** and **feedback mechanical speed** as inputs
- Outputs the corresponding **d- and q-axis current references**

The block internally computes the reference current values using analytical relationships based on the SI unit system.  
If **Per-Unit (PU)** input is selected, signals are converted to SI before processing and converted back to PU for output.

---

## 📐 Mathematical Model of Induction Motor

These equations describe induction motor dynamics in the **rotor flux reference frame**:

### The machine inductances are represented as,

```math
L_s = L_{ls} + L_m
``` 

```math
L_r = L_{lr} + L_m
```

```math
\sigma = 1 - \left( \frac{L_m^2}{L_s L_r} \right)
```

---

### Stator voltages are represented as,

```math
v_{sd} = R_s i_{sd} + \sigma L_s \frac{di_{sd}}{dt} + \frac{L_m}{L_r} \frac{d\lambda_{rd}}{dt} - \omega_e \sigma L_s i_{sq}
```

```math
v_{sq} = R_s i_{sq} + \sigma L_s \frac{di_{sq}}{dt} + \frac{L_m}{L_r} \omega_e \lambda_{rd} + \omega_e \sigma L_s i_{sd}
```

### Where,

- `v_sd`, — Stator d-axis voltages *(Volts)*
- `v_sq` — Stator q-axis voltages *(Volts)*    
- `R_s` — Stator phase winding resistance *(Ohms)*  
- `R_r` — Rotor resistance referred to stator *(Ohms)*  
- `L_ls` — Stator leakage inductance *(Henry)*  
- `L_lr` — Rotor leakage inductance *(Henry)*  
- `L_s` — Stator inductance *(Henry)*  
- `L_m` — Magnetizing inductance *(Henry)*  
- `L_r` — Rotor inductance referred to stator *(Henry)*  
- `σ` — Total leakage factor of the induction motor  

---

### In the preceding equations, the **flux linkages** can be represented as

```math
\lambda_{sd} = \frac{L_m}{L_r} \lambda_{rd} + \sigma L_s i_{sd}
```  

```math
\lambda_{sq} = \sigma L_s i_{sq}
```

```math
\tau_r \frac{d\lambda_{rd}}{dt} + \lambda_{rd} = L_m i_{sd}
```

### Where

- `λ_sd` — d-axis stator flux linkage *(Weber)*  
- `L_m` — Magnetizing inductance *(Henry)*  
- `L_r` — Rotor inductance referred to stator *(Henry)*  
- `λ_rd` — d-axis rotor flux linkage *(Weber)*  
- `σ` — Total leakage factor of the induction motor  
- `L_s` — Stator inductance *(Henry)*  
- `i_sd` — d-axis stator current *(Amperes)*  
- `i_sq` — q-axis stator current *(Amperes)*  
- `τ_r` — Rotor time constant *(seconds)*  
- `dλ_rd/dt` — Derivative of the d-axis rotor flux linkage with respect to time *(Weber/second)*

---

### If we **keep the rotor flux constant** and align the **d-axis to the rotor flux reference frame**, then:

```math
\lambda_{rd} = L_m i_{sd}
```  

```math
\lambda_{rq} = 0
```

---

### These equations describe the mechanical dynamics,

The electromagnetic torque and mechanical dynamics are given by:

```math
T_e = \frac{3}{2} p \left( \frac{L_m}{L_r} \right) \lambda_{rd} i_{sq}
```

```math
T_e - T_L = J \frac{d\omega_m}{dt} + B \omega_m
```

### Where

- `T_e` — Electromechanical torque produced by the motor *(Nm)*  
- `T_L` — Load torque *(Nm)*  
- `p` — Number of pole pairs  
- `L_m` — Magnetizing inductance *(Henry)*  
- `L_r` — Rotor inductance referred to stator *(Henry)*  
- `λ_rd` — d-axis rotor flux linkage *(Weber)*  
- `i_sq` — Final q-axis current reference after saturation *(Amperes)*  
- `J` — Rotor inertia *(kg·m²)*  
- `B` — Damping coefficient *(N·m·s)*  
- `ω_m` — Mechanical speed of the rotor *(radians/sec)*  
- `dω_m/dt` — Derivative of mechanical speed with respect to time *(radians/sec²)*

---

### These equations describe the slip speed,

Define rotor time constant:

```math
\tau_r = \left( \frac{L_r}{R_r} \right)
```

Slip angular speed:

```math
\omega_{e\_slip} = \left( \frac{L_m \cdot i_{sq}^{ref}}{\tau_r \cdot \lambda_{rd}} \right)
```

### Where

- `τ_r` — Rotor time constant *(seconds)*  
- `L_r` — Rotor inductance referred to stator *(Henry)*  
- `R_r` — Rotor resistance referred to stator *(Ohms)*  
- `ω_e_slip` — Electrical slip speed of the rotor *(radians/sec)*  
- `L_m` — Magnetizing inductance *(Henry)*  
- `i_sq^ref` — Final q-axis reference current *(Amperes)*  
- `λ_rd` — d-axis rotor flux linkage *(Weber)*

Total synchronous angular speed:

```math
\omega_e = \omega_r + \omega_{e\_slip}
```

Electrical angle:

```math
\theta_e = \int \omega_e \cdot dt
```
```math
= \int (\omega_r + \omega_{e\_slip}) \cdot dt
```

```math
= \theta_r + \theta_{slip}
```

### Where

- `ω_e` — Electrical speed *(radians/sec)*  
- `ω_r` — Rotor electrical speed *(radians/sec)*  
- `ω_e_slip` — Electrical slip speed of the rotor *(radians/sec)*  
- `θ_e` — Electrical angle *(radians)*  
- `θ_r` — Rotor electrical angle *(radians)*  
- `θ_slip` — Slip angle *(radians)*  
- `∫` — Represents the integration over time

  
These equations are critical for **Field-Oriented Control (FOC)** of AC Induction Motors, ensuring decoupled control of torque and flux.

---


## Reference Current Computation

These equations show computation of the **reference currents**:

```math
i_{sd\_0} = \frac{\lambda_{rd}}{L_m}
```

```math
i_{sq\_req} = \frac{T^{ref}}{\frac{3}{2}p\left(\frac{L_m}{L_r}\right)\lambda_{rd}}
```

### Where

- `i_sd_0` — Reference d-axis current (magnetizing current) *(Amperes)*  
- `i_sq_req` — Required q-axis reference current for torque generation *(Amperes)*  
- `λ_rd` — d-axis rotor flux linkage *(Weber)*  
- `L_m` — Magnetizing inductance *(Henry)*  
- `L_r` — Rotor inductance referred to stator *(Henry)*  
- `p` — Number of pole pairs  
- `T^ref` — Reference torque *(Nm)*

---

#### The reference currents are computed **differently based on motor speed** – distinguishing between below base speed and field weakening regions:

- **If** ωₘ ≤ ωᵣₐₜₑd:

```math
i_{sd\_sat} = \min(i_{sd\_0}, i_{max})
```

- **If** ωₘ > ωᵣₐₜₑd:

```math
i_{sd\_fw} = i_{sd\_0} \left( \frac{\omega_{rated}}{\omega_m} \right)
```

```math
i_{sd\_sat} = \min(i_{sd\_fw}, i_{max})
```

### Where

- `i_sd_fw` — d-axis current under field weakening *(Amperes)*  
- `i_sd_0` — Reference d-axis current (magnetizing current) *(Amperes)*  
- `ω_rated` — Rated mechanical speed of the motor *(radians/sec)*  
- `ω_m` — Actual mechanical speed of the rotor *(radians/sec)*  
- `i_sd_sat` — Saturated d-axis current *(Amperes)*  
- `i_max` — Maximum allowable phase current *(Amperes)*

---

### These equations indicate the **q-axis current** computation:

```math
i_{sq\_lim} = \sqrt{i_{max}^2 - i_{sd\_sat}^2}
```

```math
i_{sq} = sat(i_{sq\_lim}, i_{sq\_req})
```

### Where

- `i_sq_lim` — Maximum allowable q-axis current limit under current constraint *(Amperes)*  
- `i_max` — Maximum allowable phase current *(Amperes)*  
- `i_sd_sat` — Saturated d-axis current *(Amperes)*  
- `i_sq` — Final q-axis current reference after saturation *(Amperes)*  
- `i_sq_req` — Required q-axis current for torque generation *(Amperes)*  
- `sat()` — Saturation function that limits `i_sq_req` within ±`i_sq_lim`


---

### The block outputs the following values:

```math
i_{sd}^{ref} = i_{sd\_sat}
```

```math
i_{sq}^{ref} = i_{sq}
```

### Where

- `i_sd^ref` — Final d-axis reference current *(Amperes)*  
- `i_sq^ref` — Final q-axis reference current *(Amperes)*  
- `i_sd_sat` — Saturated d-axis current *(Amperes)*  
- `i_sq` — Final q-axis current reference after saturation *(Amperes)*

This logic ensures safe and efficient motor operation in both base and field weakening regions, maintaining the current magnitude within limits.

---

-->


# 💡 Field-Oriented Control (FOC) – Theory and Implementation

**Field-Oriented Control (FOC)** is a high-performance technique used to control AC motors such as **Permanent Magnet Synchronous Motors (PMSM)** and **Induction Motors (IM)**. It provides **independent control of torque and flux** by transforming motor variables into a rotating reference frame aligned with the rotor magnetic field.

---

# ⚙️ Key Concepts and Transformation Equations

# 🧮 Clarke Transformation (ABC → αβ)

## Description

The **Clarke Transform** block computes the Clarke transformation of balanced three-phase components in the `abc` reference frame and outputs the balanced two-phase orthogonal components in the stationary `αβ` reference frame.

Alternatively, the block can compute Clarke transformation of three-phase components `a`, `b`, and `c` and output the components `α`, `β`, and `0`. For a balanced system, the zero component is equal to zero.

Use the **Number of inputs** parameter to use either two or three inputs.

When using two-input configuration, the block accepts two signals out of the three phases (`abc`), automatically calculates the third signal, and outputs the corresponding components in the `αβ` reference frame.

For example, the block accepts either `a` and `b` input values or the multiplexed input value `abc`, where the phase-a axis aligns with the `α`-axis.

### Magnetic Axis Representation

* The figure below shows the direction of the magnetic axes of the stator windings in the `abc` reference frame and the stationary `αβ` reference frame:

<p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/9a5dc663665fb4e69fff79570d258ed58f25713f/Clarke%20Transformation/The%20Stator%20Windings%20in%20the%20ABC%20Reference%20Frame%20and%20the%20Stationary%20AlphaBeta%20Reference%20Frame.png" width="200">
</p>  

<p align="center"><b>Figure 5:</b> The Stator Windings in the ABC Reference Frame and the Stationary AlphaBeta Reference Frame</p>  


* The figure below shows the equivalent `α` and `β` components in the stationary `αβ` reference frame:

 <p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/9a5dc663665fb4e69fff79570d258ed58f25713f/Clarke%20Transformation/Equivalent%20Alpha%20and%20Beta%20Components%20in%20the%20Stationary%20AlphaBeta%20Reference%20Frame.png" width="200">
</p>  

<p align="center"><b>Figure 6:</b> Equivalent Alpha and Beta Components in the Stationary AlphaBeta Reference Frame</p>  


* The time-response of the individual components of equivalent balanced `abc` and `αβ` systems.

 <p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/64b8573f64bbaab2b215a50492f673aace25fb89/Clarke%20Transformation/The%20Time-Response%20of%20the%20Individual%20Components%20of%20Equivalent%20Balanced%20ABC%20and%20AlphaBeta%20Systems.png" width="550">
</p>  

<p align="center"><b>Figure 7:</b> The Time-Response of the Individual Components of Equivalent Balanced ABC and AlphaBeta Systems</p>  


---

## Equations (Three-Phase to Alpha-Beta-Zero)
The following equation describes the Clarke transform computation:

```math
\begin{bmatrix}
f_\alpha \\
f_\beta \\
f_0
\end{bmatrix}
=
\left( \frac{2}{3} \right)
\begin{bmatrix}
1 & -\frac{1}{2} & -\frac{1}{2} \\
0 & \frac{\sqrt{3}}{2} & -\frac{\sqrt{3}}{2} \\
\frac{1}{2} & \frac{1}{2} & \frac{1}{2}
\end{bmatrix}
\begin{bmatrix}
f_a \\
f_b \\
f_c
\end{bmatrix}
```

---

## Simplified Clarke Transform (Using Only Two Phases)

For balanced systems like motors, the zero sequence component calculation is always zero. For example, the currents of the motor can be represented as,

```math
i_a + i_b + i_c = 0
```

Therefore, you can use only two current sensors in three-phase motor drives, where you can calculate the third phase as,

```math
i_c = -(i_a + i_b)
```

Thus, the simplified Clarke transform using only two phases becomes:

```math
\begin{bmatrix}
f_\alpha \\
f_\beta
\end{bmatrix} =
\begin{bmatrix}
1 & 0 \\
\frac{1}{\sqrt{3}} & \frac{2}{\sqrt{3}}
\end{bmatrix}
\begin{bmatrix}
f_a \\
f_b
\end{bmatrix}
```

---

### Where,

- `fₐ`, `f_b`, `f_c` are the balanced three-phase components in the **abc reference frame** (natural frame).

- `f_α`, `f_β` are the balanced two-phase orthogonal components in the **stationary αβ reference frame** (obtained using Clarke transformation).

- `f₀` is the **zero-sequence component** in the stationary αβ reference frame, representing any unbalance in the three-phase system.

---

# 🧮 Park Transformation (αβ → dq)

## Description

The Park Transform block computes the Park transformation of two-phase orthogonal components (`α, β`) or multiplexed `αβ0` components in a stationary `αβ` reference frame.

The block accepts the following inputs:

* Either `α-β` axes components or multiplexed components `αβ0` in the stationary reference frame. Use the **Number of inputs** parameter to use either two or three inputs.

* Sine and cosine values of the corresponding angles of transformation.

When using two-input configuration, it outputs orthogonal direct (`d`) and quadrature (`q`) axis components in the rotating `dq` reference frame. When using three-input configuration, it outputs multiplexed components `dq0`.

For a balanced system, the zero component is equal to zero.

You can configure the block to align either the `d`- or the q-axis with the α-axis at time `t = 0`.

The figures show the `α-β` axes components in an `αβ` reference frame and a rotating dq reference frame for when:

* The `d`-axis aligns with the `α`-axis.

<p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/88d267bc8549baf1331a0f4b34dad696946cbcbf/Park%20Transformation/The%20Alpha-Beta%20Axes%20Components%20in%20an%20AlphaBeta%20Reference%20Frame.png" width="200">
</p>  

<p align="center"><b>Figure 8:</b> The Alpha-Beta Axes Components in an AlphaBeta Reference Frame</p>  


* The `q`-axis aligns with the `α`-axis.

 <p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/88d267bc8549baf1331a0f4b34dad696946cbcbf/Park%20Transformation/The%20Alpha-Beta%20Axes%20Components%20a%20Rotating%20DQ%20Reference%20Frame.png" width="200">
</p>  

<p align="center"><b>Figure 9:</b> The Alpha-Beta Axes Components a Rotating DQ Reference Frame</p>  

In both cases, the angle `θ = ωt`, where:

* `θ` is the angle between the `α`- and `d`-axes for the `d`-axis alignment or the angle between the `α`- and `q`-axes for the `q`-axis alignment. It indicates the angular position of the rotating dq reference frame with respect to the `α`-axis.

* `ω` is the rotational speed of the `d-q` reference frame.

* `t` is the time, in seconds, from the initial alignment.

The figures show the time-response of the individual components of the `αβ` and `dq` reference frames when:

* The `d`-axis aligns with the `α`-axis.

<p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/88d267bc8549baf1331a0f4b34dad696946cbcbf/Park%20Transformation/The%20Time-Response%20of%20the%20Individual%20Components%20of%20the%20AlphaBeta%20%20and%20DQ%20Reference%20Frames.png" width="550">
</p>  

<p align="center"><b>Figure 10:</b> The D-Axis Aligns with the Alpha-Axis</p>  

* The q-axis aligns with the α-axis.

<p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/da6ae1cd867bdef593fa53a7d2959e6d16bf077e/Park%20Transformation/Q-Axis%20Aligns%20with%20the%20Alpha-Axis.png" width="550">
</p>  

<p align="center"><b>Figure 11:</b> The Q-Axis Aligns with the Alpha-Axis</p>  

---

## Equations (αβ → dq)

The Park transformation is mathematically implemented as follows:

- **When the d-axis aligns with the α-axis**:
  
  ```math
  \begin{bmatrix}
  f_d \\
  f_q
  \end{bmatrix}
  = 
  \begin{bmatrix}
  \cos(\theta) & \sin(\theta) \\
  -\sin(\theta) & \cos(\theta)
  \end{bmatrix}
  \begin{bmatrix}
  f_\alpha \\
  f_\beta
  \end{bmatrix}
  ```
  
  ### Where,
 - `f_α`, and `f_β` are the two-phase orthogonal components in the stationary αβ reference frame.
 - `f_d`, and `f_q` are the direct and quadrature axis components in the rotating dq reference frame.
 - `θ` is the angle between the α- and q-axes.

- **When the q-axis aligns with the α-axis**:
  
  ```math
  \begin{bmatrix}
  f_d \\
  f_q
  \end{bmatrix}
  = 
  \begin{bmatrix}
  \sin(\theta) & -\cos(\theta) \\
  \cos(\theta) & \sin(\theta)
  \end{bmatrix}
  \begin{bmatrix}
  f_\alpha \\
  f_\beta
  \end{bmatrix}
   ```
  
  ### Where,
  - `f_α`, and `f_β` are the two-phase orthogonal components in the stationary αβ reference frame.
  - `f_d`, and `f_q` are the direct and quadrature axis components in the rotating dq reference frame.
  - `θ` is the angle between the α- and q-axes.

---


### 🎯 Speed PI Controller

The **Speed PI Controller** compares the actual rotor speed \( \omega \) with the reference speed \( \omega_{\text{ref}} \). The error in speed \( e_\omega \) is calculated as:

```math
e_\omega = \omega_{\text{ref}} - \omega
```

This error is fed into a **Proportional-Integral (PI) controller** to produce the desired torque current \( i_q^* \), which is used to control the motor's torque production. The control law for \( i_q^* \) is given by:

```math
i_q^* = K_p^\omega \cdot e_\omega + K_i^\omega \cdot \int e_\omega \, dt
```

### Where,
- \( K_p^\omega \) is the **proportional gain**.
- \( K_i^\omega \) is the **integral gain**.
- \( \int e_\omega \, dt \) is the integral of the speed error over time, which helps eliminate steady-state error.

The **Proportional** term \( K_p^\omega \) reacts to the current speed error, while the **Integral** term \( K_i^\omega \) ensures that the steady-state error is driven to zero, leading to precise speed regulation over time.

---

### 🔁 Current Control PI Loops

The **Current Control PI Loops** are designed to independently control the d-axis and q-axis stator currents, \( i_d \) and \( i_q \), using PI controllers. These currents are essential in motor control because they directly influence the motor's flux and torque.

#### 📘 Direct-Axis (d-axis) Control

The **d-axis** current \( i_d \) is responsible for controlling the flux in the motor, often linked to the magnetizing current. The error in the d-axis current \( e_d \) is given by:

```math
e_d = i_d^* - i_d
```

### Where,
- \( i_d^* \) is the reference direct-axis current.
- \( i_d \) is the actual direct-axis current measured from the motor.

The PI control law for the d-axis voltage \( v_d \) is:

```math
v_d = K_p^d \cdot e_d + K_i^d \cdot \int e_d \, dt
```

### Where,
- \( K_p^d \) is the **proportional gain** for the d-axis.
- \( K_i^d \) is the **integral gain** for the d-axis.

#### 📙 Quadrature-Axis (q-axis) Control

The **q-axis** current \( i_q \) is responsible for controlling the torque in the motor, and the error in the q-axis current \( e_q \) is:

```math
e_q = i_q^* - i_q
```

### Where,
- \( i_q^* \) is the reference quadrature-axis current.
- \( i_q \) is the actual quadrature-axis current measured from the motor.

The PI control law for the q-axis voltage \( v_q \) is:

```math
v_q = K_p^q \cdot e_q + K_i^q \cdot \int e_q \, dt
```

### Where,
- \( K_p^q \) is the **proportional gain** for the q-axis.
- \( K_i^q \) is the **integral gain** for the q-axis.

The **Proportional** term for both axes adjusts the voltage based on the current error, while the **Integral** term eliminates steady-state errors by integrating the current error over time.

---

### Control System Behavior

- The **PI controllers** are used to eliminate steady-state errors and ensure smooth tracking of the reference signals.
- The **d-axis control** aims to maintain the desired flux, typically set to zero for surface Permanent Magnet Synchronous Motors (PMSM) or adjusted for different motor types.
- The **q-axis control** directly governs the torque production, where the current \( i_q \) is typically proportional to the torque generated by the motor.

In combination, these controllers allow for the **decoupling** of the flux and torque components of the motor, providing **independent control** of each, which is crucial in **Field-Oriented Control (FOC)**.

---

## Notation

| Symbol         | Description                                         |
|----------------|-----------------------------------------------------|
| \( \omega \)        | Actual rotor speed                              |
| \( \omega_{\text{ref}} \) | Reference rotor speed                        |
| \( i_d, i_q \)      | Actual d-axis and q-axis currents               |
| \( i_d^*, i_q^* \)  | Reference d-axis and q-axis currents             |
| \( v_d, v_q \)      | Control voltages for d-axis and q-axis           |
| \( K_p, K_i \)      | Proportional and Integral gains for the controllers |
| \( e_d, e_q, e_\omega \) | Error terms for d-axis, q-axis, and speed     |

---


# 🧮 Inverse Park Transformation (dq → αβ)

## Description

The Inverse Park Transform block computes the inverse Park transformation of the orthogonal direct (`d`) and quadrature (`q`) axes components or the multiplexed `dq0` components in the rotating `dq` reference frame.

You can configure the block to align either the `d`- or `q`-axis with the `α`-axis at time `t = 0`.

The block accepts the following inputs:

* Either `d-q` axes components or multiplexed components `dq0` in the rotating reference frame. Use the Number of inputs parameter to use either two or three inputs.

* Sine and cosine values of the corresponding angles of transformation.

When using two-input configuration, it outputs the two-phase orthogonal components in the stationary `αβ` reference frame. When using three-input configuration, it outputs multiplexed components `αβ0`.

For a balanced system, the zero component is equal to zero.

The figures show a rotating `dq` reference frame and the `α-β` axes components in an `αβ` reference frame for when:

* The `d`-axis aligns with the `α`-axis.

<p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/88d267bc8549baf1331a0f4b34dad696946cbcbf/Inverse%20Park%20Transformation/A%20Rotating%20DQ%20Reference%20Frame%20%20in%20an%20AlphaBeta%20reference%20frame.png" width="200">
</p>  

<p align="center"><b>Figure 12:</b> A Rotating DQ Reference Frame  in an AlphaBeta reference frame</p>  


* The `q`-axis aligns with the `α`-axis.

 <p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/88d267bc8549baf1331a0f4b34dad696946cbcbf/Inverse%20Park%20Transformation/The%20Alpha-Beta%20Axes%20Components%20in%20an%20AlphaBeta%20Reference%20Frame.png" width="200">
</p>  

<p align="center"><b>Figure 13:</b> The Alpha-Beta Axes Components in an AlphaBeta Reference Frame</p>  

In both cases, the angle `θ = ωt`, where:

* `θ` is the angle between the `α`- and `d`-axes for the `d`-axis alignment or the angle between the `α`- and `q`-axes for the `q`-axis alignment. It indicates the angular position of the rotating `dq` reference frame with respect to the `α`-axis.

`ω` is the rotational speed of the `d-q` reference frame.

t is the time, in seconds, from the initial alignment.

The figures show the time-response of the individual components of the `αβ` and `dq` reference frames when:

* The `d`-axis aligns with the `α`-axis.


<p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/88d267bc8549baf1331a0f4b34dad696946cbcbf/Inverse%20Park%20Transformation/Time-Response%20of%20the%20Individual%20Components%20of%20the%20AlphaBeta%20and%20DQ%20Reference%20Frames.png" width="550">
</p>  

<p align="center"><b>Figure 14:</b> The D-Axis Aligns with the Alpha-Axis</p>  

* The `q`-axis aligns with the `α`-axis.

<p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/3fec55d18bbd49a4fadcc077230cd2b6980892a4/Inverse%20Park%20Transformation/The%20Q-Axis%20Aligns%20with%20the%20Alpha-Axis..png" width="550">
</p>  

<p align="center"><b>Figure 15:</b> The Q-Axis Aligns with the Alpha-Axis</p>  

---

## Inverse Park Transform Equation (dq → αβ)

```math
\begin{bmatrix}
f_\alpha \\
f_\beta
\end{bmatrix}
=
\begin{bmatrix}
\cos(\theta) & -\sin(\theta) \\
\sin(\theta) & \cos(\theta)
\end{bmatrix}
\begin{bmatrix}
f_d \\
f_q
\end{bmatrix}
```

Where `θ` is the angle of the rotating reference frame, typically provided by a rotor position sensor or PLL.

---

## Park Transform Equation (αβ → dq)

```math
\begin{bmatrix}
f_d \\
f_q
\end{bmatrix}
=
\begin{bmatrix}
\cos(\theta) & \sin(\theta) \\
-\sin(\theta) & \cos(\theta)
\end{bmatrix}
\begin{bmatrix}
f_\alpha \\
f_\beta
\end{bmatrix}
```

This is the forward Park Transform, typically applied before control logic.

---

### Where,

- `f_α` and `f_β` are the two-phase orthogonal components in the stationary `αβ` reference frame.
- `f_d`, `f_q` a are the direct and quadrature axis orthogonal components in the rotating `dq` reference frame.
- `θ` is the **electrical angle** between the `α`-axis and the `d`-axis in the rotating `dq` frame.

---

# 🧮 Inverse Clarke Transformation (αβ → ABC)

## Description

The Inverse Clarke Transform block computes the Inverse Clarke transformation of balanced, two-phase orthogonal components in the stationary `αβ` reference frame and outputs the balanced, three-phase components in the stationary `abc` reference frame. 

Alternatively, the block can compute Inverse Clarke transformation of the components `α, β`, and `0` to output the three-phase components `a, b`, and `c`. 

For a balanced system, the zero component is equal to zero. 

Use the **Number of inputs** parameter to use either two or three inputs.

The block accepts the `α-β` axis components as inputs and outputs the corresponding three-phase signals, where the phase-`a` axis aligns with the `α`-axis.

* The `α` and `β` input components in the `αβ` reference frame.

<p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/88d267bc8549baf1331a0f4b34dad696946cbcbf/Inverse%20Clarke%20Transformation/The%20Alpha%20and%20Beta%20Input%20Components%20in%20the%20AlphaBeta%20%20Reference%20Frame.png" width="200">
</p>  

<p align="center"><b>Figure 16:</b> The Alpha and Beta Input Components in the AlphaBeta  Reference Frame</p>  


* The direction of the equivalent `a, b`, and `c` output components in the `abc` reference frame and the `αβ` reference frame.

 <p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/88d267bc8549baf1331a0f4b34dad696946cbcbf/Inverse%20Clarke%20Transformation/The%20Equivalent%20A%2C%20B%2C%20and%20C%20Components%20in%20the%20ABC%20Reference%20Frame%20and%20the%20AlphaBeta%20%20Reference%20Frame.png" width="200">
</p>  

<p align="center"><b>Figure 17:</b> The Equivalent A, B, and C Components in the ABC Reference Frame and the AlphaBeta  Reference Frame</p>  

The time-response of the individual components of equivalent balanced `αβ` and `abc` systems.

<p align="center">
  <img src="https://github.com/vandemataram15aug1947/Indirect_Sensor_Based_FOC_of_Three_Phase_ACIM_Using_SVPWM/blob/88d267bc8549baf1331a0f4b34dad696946cbcbf/Inverse%20Clarke%20Transformation/The%20Time-Response%20of%20the%20Individual%20Components%20of%20Equivalent%20Balanced%20AlphaBeta%20%20and%20ABC%20systems.png" width="550">
</p>  

<p align="center"><b>Figure 18:</b> The Time-Response of the Individual Components of Equivalent Balanced AlphaBeta  and ABC systems</p>  

---

## Equations (αβ → abc)

The following equation describes the **Inverse Clarke Transform** computation:

```math
\begin{bmatrix}
f_a \\
f_b \\
f_c \\
\end{bmatrix}
=
\begin{bmatrix}
1 & 0 & 1 \\
-\frac{1}{2} & \frac{\sqrt{3}}{2} & 1 \\
-\frac{1}{2} & -\frac{\sqrt{3}}{2} & 1 \\
\end{bmatrix}
\begin{bmatrix}
f_α \\
f_β \\
f₀ \\
\end{bmatrix}
```

For balanced systems like motors, the zero-sequence component calculation is always zero:

```math
i_a + i_b + i_c = 0
```

Therefore, you can use only two current sensors in three-phase motor drives, where you can calculate the third phase as:

```math
i_c = -(i_a + i_b)
```

By using these equations, the block implements the **Inverse Clarke Transform** as:

```math
\begin{bmatrix}
f_a \\
f_b \\
f_c \\
\end{bmatrix}
=
\begin{bmatrix}
1 & 0 \\
-\frac{1}{2} & \frac{\sqrt{3}}{2} \\
-\frac{1}{2} & -\frac{\sqrt{3}}{2} \\
\end{bmatrix}
\begin{bmatrix}
f_α \\
f_β \\
\end{bmatrix}
```

### Where,

- `f_α` and `f_β` are the balanced two-phase orthogonal components in the **stationary `αβ` reference frame**.
- `f₀` is the **zero-sequence component** in the stationary `αβ` reference frame.
- `f_a`, `f_b`, and `f_c` are the balanced three-phase components in the **`abc` reference frame**.

### 🔲 7. Space Vector PWM (SVPWM)

Generates PWM switching signals from the three-phase voltages to apply to the motor via the inverter.  
SVPWM:
- Maximizes bus voltage usage
- Reduces harmonic distortion

---


## Code Explanation

### Include Required Header Files

```c
#include "stdint.h"         /* Standard integer types */
#include "math.h"           /* Standard math library */
#include "IQmathLib.h"      /* TI's fixed-point math library */

#include "device.h"         /* Device-specific configuration */
#include "ADC.h"            /* ADC configuration and data handling */
#include "eQEP.h"           /* Quadrature encoder interface */

#include "CLARKE.h"         /* Clarke transformation (abc → αβ) */
#include "PARK.h"           /* Park transformation (αβ → dq) */
#include "iPARK.h"          /* Inverse Park transformation (dq → αβ) */
#include "iCLARKE.h"        /* Inverse Clarke transformation (αβ → abc) */
#include "SVPWM.h"          /* Space Vector PWM generation */
```

## Headers and Functionality

Here is a list of the headers used in this project and their specific functionality:

- **`"stdint.h"`**: Standard integer types (such as `int32_t`, `uint16_t`) for improved portability across platforms.
- **`"math.h"`**: Standard math library providing functions like `sin()`, `cos()`, and `atan2()` for performing trigonometric calculations.
- **`"IQmathLib.h"`**: TI's fixed-point math library, enabling efficient fixed-point mathematical operations.
- **`"device.h"`**: Device-specific configuration, such as microcontroller settings, peripheral initialization, and clock configuration.
- **`"ADC.h"`**: ADC driver functions for configuring and handling Analog-to-Digital Conversion (ADC) to acquire motor currents and rotor speed.
- **`"eQEP.h"`**: Quadrature Encoder Pulse (eQEP) interface, used for acquiring encoder data to monitor rotor position and speed.
- **`"CLARKE.h"`**: Clarke transformation (abc → αβ), used to convert a 3-phase system to a 2-phase system in motor control.
- **`"PARK.h"`**: Park transformation (αβ → dq), which is used to convert the αβ frame to the dq frame for motor control, aligned with the rotor.
- **`"PI.h"`**: Contains the implementation of Proportional-Integral (PI) controllers, which are used for feedback control in motor control systems. The PI controller calculates the error between the reference and measured values (e.g., motor speed or position), and adjusts the control signal to minimize this error, ensuring stable motor operation.
- **`"iPARK.h"`**: Inverse Park transformation (dq → αβ), which converts the dq frame back to the αβ frame.
- **`"iCLARKE.h"`**: Inverse Clarke transformation (αβ → abc), which converts the 2-phase system back to the 3-phase system for motor drive.
- **`"SVPWM.h"`**: Space Vector Pulse Width Modulation (SVPWM) generation, used to control motor voltage using space vector modulation techniques.

---


## 🔁 FOC Control Loop Summary

### Code Implementation

#### Global Variables
```c
/* Global Variables */
float i_alpha, i_beta;      /* Alpha-beta current components */
float i_d, i_q;             /* Direct-quadrature currents */
float v_d = 0, v_q = 0;     /* Direct-quadrature voltage components */
float v_alpha, v_beta;      /* Inverse Clarke transform results */
float theta;                /* Rotor position from encoder */
float omega_actual;         /* Actual rotor speed */
float omega_ref = 1500;     /* Reference speed in RPM */
float i_d_ref = 0.0f;       /* d-axis current reference */
float i_q_ref = 2.0f;       /* q-axis current reference */
```

#### Encoder Read (Position Feedback)
```c
/* Encoder Read Function */
/* Reads the current rotor position from encoder */
float encoder_read() {
    return get_encoder_position();  /* Read encoder feedback */
}
```

#### Clarke Transform (ABC to αβ)
```c
/* Clarke Transform Function */
/* Converts 3-phase current (i_a, i_b, i_c) to 2-phase orthogonal (i_alpha, i_beta) */
void clarke_transform(float i_a, float i_b, float i_c) {
    i_alpha = i_a;
    i_beta = (i_a + 2.0f * i_b) / sqrtf(3.0f);  /* Assuming balanced system */
}
```

#### Park Transform (αβ → dq)
```c
/* Park Transform Function */
/* Converts (i_alpha, i_beta) into rotating reference frame components (i_d, i_q) */
void park_transform(float theta) {
    float sin_theta = sinf(theta);  /* Calculate sin(theta) */
    float cos_theta = cosf(theta);  /* Calculate cos(theta) */
    
    i_d = i_alpha * cos_theta + i_beta * sin_theta;    /* d-axis current */
    i_q = -i_alpha * sin_theta + i_beta * cos_theta;   /* q-axis current */
}
```

#### Speed PI Controller
```c
/* Speed PI Controller */
/* Implements a basic PI controller for speed regulation */
void speed_control(float omega_actual) {
    static float integral = 0;           /* Integral term storage */
    float error = omega_ref - omega_actual;  /* Speed error */
    float Kp = 0.01f, Ki = 0.005f;       /* PI controller gains */
    
    integral += error * Ki;              /* Integrate error */
    integral = fminf(fmaxf(integral, -1.0f), 1.0f);  /* Anti-windup: limit integral */
    
    v_q = (Kp * error) + integral;       /* Output q-axis voltage command */
}

/* Current PI Controller */
/* Controls i_d and i_q to track reference values */
void current_pi_control() {
    static float int_d = 0, int_q = 0;   /* Integral terms */
    float Kp = 0.2f, Ki = 0.01f;         /* PI gains for current loops */
    
    float error_d = i_d_ref - i_d;       /* Error in d-axis current */
    float error_q = i_q_ref - i_q;       /* Error in q-axis current */
    
    int_d += error_d * Ki;               /* Integrate d error */
    int_q += error_q * Ki;               /* Integrate q error */
    
    int_d = fminf(fmaxf(int_d, -1.0f), 1.0f);  /* Anti-windup for d */
    int_q = fminf(fmaxf(int_q, -1.0f), 1.0f);  /* Anti-windup for q */
    
    v_d = (Kp * error_d) + int_d;        /* Output d-axis voltage */
    v_q = (Kp * error_q) + int_q;        /* Output q-axis voltage */
}
```

#### Inverse Park Transform (dq → αβ)
```c
/* Inverse Park Transform Function */
/* Converts (v_d, v_q) from rotating to stationary reference frame (v_alpha, v_beta) */
void inverse_park_transform(float theta) {
    float sin_theta = sinf(theta);      /* Calculate sin(theta) */
    float cos_theta = cosf(theta);      /* Calculate cos(theta) */
    
    v_alpha = v_d * cos_theta - v_q * sin_theta;  /* Alpha-axis voltage */
    v_beta = v_q * cos_theta + v_d * sin_theta;   /* Beta-axis voltage */
}
```

#### SVPWM Generation
```c
#include "driverlib.h"
#include "math.h"

/* SVPWM Generation */
/* Generates PWM duty cycles based on v_alpha and v_beta */
void generate_svpwm_pulses(float v_alpha, float v_beta) {
    float v_ref = sqrtf(v_alpha * v_alpha + v_beta * v_beta);  /* Magnitude of voltage vector */
    float angle = atan2f(v_beta, v_alpha);                     /* Angle of voltage vector */
    
    if (angle < 0) angle += 2.0f * M_PI;                       /* Wrap angle into [0, 2π] */

    int sector_num = (int)(angle / (M_PI / 3.0f));             /* Determine sector (0 to 5) */
    float T = 1.0f;                                            /* Normalized PWM period */
    float t1 = 0, t2 = 0, t0 = 0;                              /* Time durations */
    float sqrt3 = sqrtf(3.0f);

    /* Calculate t1 and t2 based on sector */
    switch (sector_num) {
        case 0:
            t1 = sqrt3 * v_ref * sinf(M_PI / 3.0f - angle);
            t2 = sqrt3 * v_ref * sinf(angle);
            break;
        case 1:
            angle -= M_PI / 3.0f;
            t1 = sqrt3 * v_ref * sinf(M_PI / 3.0f - angle);
            t2 = sqrt3 * v_ref * sinf(angle);
            break;
        case 2:
            angle -= 2 * M_PI / 3.0f;
            t1 = sqrt3 * v_ref * sinf(M_PI / 3.0f - angle);
            t2 = sqrt3 * v_ref * sinf(angle);
            break;
        case 3:
            angle -= M_PI;
            t1 = sqrt3 * v_ref * sinf(M_PI / 3.0f - angle);
            t2 = sqrt3 * v_ref * sinf(angle);
            break;
        case 4:
            angle -= 4 * M_PI / 3.0f;
            t1 = sqrt3 * v_ref * sinf(M_PI / 3.0f - angle);
            t2 = sqrt3 * v_ref * sinf(angle);
            break;
        case 5:
            angle -= 5 * M_PI / 3.0f;
            t1 = sqrt3 * v_ref * sinf(M_PI / 3.0f - angle);
            t2 = sqrt3 * v_ref * sinf(angle);
            break;
    }

    t0 = T - t1 - t2;  /* Zero vector time */

    /* Calculate duty cycles for phases A, B, C */
    float Ta = (t1 + t2 + t0 / 2.0f) / T;
    float Tb = (t2 + t0 / 2.0f) / T;
    float Tc = t0 / 2.0f / T;

    /* Set PWM duty cycles (assumes API exists for this) */
    PWM_setDutyCycle(PWM1_BASE, PWM_OUT_1, Ta);  /* Phase A */
    PWM_setDutyCycle(PWM1_BASE, PWM_OUT_2, Tb);  /* Phase B */
    PWM_setDutyCycle(PWM1_BASE, PWM_OUT_3, Tc);  /* Phase C */
}
```

#### Main Loop
```c
/* Main Entry Point */
int main() {
    /* Initialize encoder and peripherals */
    encoder_init();     /* Encoder setup */
    pwm_init();         /* PWM peripheral setup */
    adc_init();         /* ADC initialization for i_a, i_b, i_c */
    
    while (1) {
        /* Step 1: Read actual rotor position */
        theta = encoder_read();

        /* Step 2: Read 3-phase currents */
        float i_a = read_adc_current_A();
        float i_b = read_adc_current_B();
        float i_c = read_adc_current_C();

        /* Step 3: Clarke and Park transforms */
        clarke_transform(i_a, i_b, i_c);
        park_transform(theta);

        /* Step 4: Speed PI controller (if speed control is used) */
        omega_actual = calculate_speed_from_encoder();
        speed_control(omega_actual);  /* Updates v_q */

        /* Step 5: Current PI controller */
        current_pi_control();         /* Updates v_d and v_q */

        /* Step 6: Inverse Park Transform */
        inverse_park_transform(theta);

        /* Step 7: SVPWM signal generation */
        generate_svpwm_pulses(v_alpha, v_beta);
    }

    return 0;
}

```

## 5. Conclusion
This project provides a comprehensive approach to Field-Oriented Control (FOC) of an Induction Motor using Space Vector PWM (SVPWM). Let me know if you need refinements, hardware details, or optimizations! 🚀

## 📚 References

1. B. Robyns, B. François, P. Degobert, and J.-P. Hautier, *[Vector Control of Induction Machines](https://isbnsearch.org/isbn/9781447123045)*, Springer, 2012.  
   [ISBN: 978-1-4471-2304-5](https://isbnsearch.org/isbn/9781447123045)

2. Nguyen Phung Quang and Jörg-Andreas Dittrich, *[Vector Control of Three-Phase AC Machines](https://isbnsearch.org/isbn/9783662469149)*, Springer, 2015.  
   [ISBN: 978-3-662-46914-9](https://isbnsearch.org/isbn/9783662469149)

3. E. Y. Y. Ho and P. C. Sen, “[Decoupling control of induction motor drives](https://doi.org/10.1109/41.3078),” *IEEE Transactions on Industrial Electronics*,  
   Vol. 35, No. 2, May 1988, pp. 253–262.  
   [DOI: 10.1109/41.3078](https://doi.org/10.1109/41.3078)

4. Slobodan N. Vukosavic, *[Electrical Machines](https://isbnsearch.org/isbn/9781447140370)*, Springer, 2013.  
   [ISBN: 978-1-4471-4037-0](https://isbnsearch.org/isbn/9781447140370)

5. Karl J. Åström and Tore Hägglund, *[Advanced PID Control](https://isbnsearch.org/isbn/9781556175169)*, ISA—The Instrumentation, Systems, and Automation Society, 1995.  
   [ISBN: 978-1-55617-516-9](https://isbnsearch.org/isbn/9781556175169)

6. A. K. Akkarapaka and D. Singh, “[The IFOC based speed control of induction motor fed by a high performance Z-source inverter](https://doi.org/10.1109/ICRERA.2014.7016440),”  
   *2014 International Conference on Renewable Energy Research and Application (ICRERA)*, Milwaukee, WI, 2014, pp. 539–543.  
   [DOI: 10.1109/ICRERA.2014.7016440](https://doi.org/10.1109/ICRERA.2014.7016440)

7. I. Ferdiansyah, L. P. S. Raharja, D. S. Yanaratri, and E. Purwanto, “[Design of PID Controllers for Speed Control of Three Phase Induction Motor Based on Direct-Axis Current (Id) Coordinate Using IFOC](https://doi.org/10.1109/ICITISEE48480.2019.9003971),”  
   *2019 4th International Conference on Information Technology, Information Systems and Electrical Engineering (ICITISEE)*, Yogyakarta, Indonesia, 2019, pp. 369–372.  
   [DOI: 10.1109/ICITISEE48480.2019.9003971](https://doi.org/10.1109/ICITISEE48480.2019.9003971)

8. Slobodan N. Vukosavić, *[Digital Control of Electrical Drives](https://isbnsearch.org/isbn/9780387259857)*, Springer, 2007.  
   [ISBN: 978-0-387-25985-7](https://isbnsearch.org/isbn/9780387259857)
   

## 📚 Journal and Conference Papers
 
1. B. Bose, *Modern Power Electronics and AC Drives*, Prentice Hall, 2001. ISBN: [0-13-016743-6](https://isbnsearch.org/isbn/0130167436)

2. Lorenz, Robert D., Thomas Lipo, and Donald W. Novotny. "[Motion control with induction motors](https://doi.org/10.1109/5.301680)," *Proceedings of the IEEE*, Vol. 82, Issue 8, August 1994, pp. 1215–1240.

3. W. Leonhard, *Control of Electrical Drives*, 3rd ed., Springer-Verlag, New York, 2001. [ISBN: 978-3-540-41293-7](https://isbnsearch.org/isbn/9783540412937)

4. Briz, Fernando, Michael W. Degner, and Robert D. Lorenz. "[Analysis and design of current regulators using complex vectors](https://doi.org/10.1109/28.845069)," *IEEE Transactions on Industry Applications*, Vol. 36, Issue 3, May/June 2000, pp. 817–825.

5. Briz, Fernando, et al. "[Current and flux regulation in field-weakening operation of induction motors](https://doi.org/10.1109/28.903143)," *IEEE Transactions on Industry Applications*, Vol. 37, Issue 1, Jan/Feb 2001, pp. 42–50.

6. R. M. Prasad and M. A. Mulla, "[A novel position-sensorless algorithm for field oriented control of DFIG with reduced current sensors](https://doi.org/10.1109/TSTE.2018.2878452)," *IEEE Transactions on Sustainable Energy*, Vol. 10, No. 3, July 2019, pp. 1098–1108.

📬 Contact
Author: [Your Name] \\
Email: [your.email@example.com] \\
LinkedIn: linkedin.com/in/yourprofile
