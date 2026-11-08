# PMSM Dynamic Model in MATLAB

## Overview

This repository contains a MATLAB implementation of the discrete-time dynamic model of a **Permanent Magnet Synchronous Motor (PMSM)** in the rotating \(dq\) reference frame.

The model describes the electrical and mechanical dynamics of a PMSM using a state-space formulation with coupled inductances. The differential equations are numerically solved using the **Forward Euler integration method**.

The function is designed to calculate the motor's electrical and mechanical states at each simulation time step.

## Features

- Discrete-time PMSM dynamic modeling.
- Electrical dynamics in the rotating \(dq\) reference frame.
- Consideration of direct-axis and quadrature-axis inductances.
- Inclusion of cross-coupling inductances (\(L_{dq}\) and \(L_{qd}\)).
- Electromagnetic torque calculation.
- Mechanical speed dynamics.
- Electrical and mechanical rotor position estimation.
- Forward Euler numerical integration.

## Mathematical Model

### 1. Electrical Dynamics

The electrical dynamics are formulated in the rotating \(dq\) reference frame, considering the stator resistance, inductances, permanent magnet flux linkage, and rotor electrical speed.

The inductance matrix is defined as:

$$
\mathbf{L} =
\begin{bmatrix}
L_d & L_{dq} \\
L_{qd} & L_q
\end{bmatrix}
$$

Its determinant is:

$$
\Delta_L = L_d L_q - L_{dq} L_{qd}
$$

The current derivatives are calculated using the coupled voltage equations and the inverse of the inductance matrix.

### 2. Electromagnetic Torque

The electromagnetic torque is calculated using the following expression implemented in the MATLAB function:

$$
T_e = \frac{3}{2}\frac{P}{2}
\left[
\Lambda_m I_{qs}
+ (L_d-L_q)I_{qs}I_{ds}
+ L_{dq}I_{qs}^{2}
- L_{qd}I_{ds}^{2}
\right]
$$

### 3. Mechanical Dynamics

The mechanical dynamics are described by:

$$
J\frac{d\omega_m}{dt}
= T_e - T_L - B\omega_m
$$

where \(J\) is the rotor and load inertia, \(B\) is the viscous friction coefficient, and \(T_L\) is the load torque.

### 4. Numerical Integration

The electrical currents and mechanical speed are updated using the Forward Euler method:

$$
x(k+1) = x(k) + T_s\dot{x}(k)
$$

The rotor electrical and mechanical angles are also updated using discrete-time integration.

## Function Description

The main MATLAB function is:

`pmsm.m`

### Input Parameters

| Input | Description | Unit |
|---|---|---|
| `Lq_in` | Quadrature-axis inductance | mH |
| `Ld_in` | Direct-axis inductance | mH |
| `Lqd_in` | Cross-coupling inductance | mH |
| `Ldq_in` | Cross-coupling inductance | mH |
| `Lambda_d` | Direct-axis flux linkage input | Wb |
| `Vds` | Direct-axis stator voltage | V |
| `Vqs` | Quadrature-axis stator voltage | V |
| `Iqs_prev` | Previous quadrature-axis current | A |
| `Ids_prev` | Previous direct-axis current | A |
| `T_L` | Load torque | N.m |
| `omega_m_prev` | Previous mechanical angular speed | rad/s |
| `theta_e_prev` | Previous electrical rotor angle | rad |
| `theta_m_prev` | Previous mechanical rotor angle | rad |

**Note:** The inductance inputs are provided in millihenries (mH) and converted internally into henries (H).

### Output Parameters

| Output | Description | Unit |
|---|---|---|
| `Iqs` | Updated quadrature-axis current | A |
| `Ids` | Updated direct-axis current | A |
| `Te` | Electromagnetic torque | N.m |
| `omega_m` | Updated mechanical angular speed | rad/s |
| `theta_e` | Electrical rotor position | rad |
| `theta_m` | Mechanical rotor position | rad |

## Model Parameters

The following parameters are currently defined internally in the MATLAB function:

| Parameter | Value | Unit |
|---|---:|---|
| Stator resistance (\(R_s\)) | 0.22 | Ω |
| Number of poles (\(P\)) | 8 | - |
| Sampling time (\(T_s\)) | 5 × 10⁻⁶ | s |
| Rotor and load inertia (\(J\)) | 0.003617 | kg.m² |
| Viscous friction (\(B\)) | 9.444 × 10⁻⁵ | N.m.s |

The number of pole pairs is \(P/2=4\).

## Usage

The function can be called at each simulation time step by providing the motor parameters, stator voltages, previous electrical states, load torque, and previous mechanical states.

Example:

```matlab
[Iqs, Ids, Te, omega_m, theta_e, theta_m] = ...
    pmsm(Lq, Ld, Lqd, Ldq, Lambda_d, ...
         Vds, Vqs, Iqs_prev, Ids_prev, ...
         T_L, omega_m_prev, theta_e_prev, theta_m_prev);
```

The calculated outputs can then be used as the previous states for the next simulation step.

## Applications

This PMSM dynamic model can be used as a foundation for:

- PMSM dynamic simulation and analysis.
- Electrical machine modeling.
- Motor drive simulation.
- Fault diagnosis and condition monitoring research.
- Development and testing of data-driven fault detection algorithms.

## Repository Structure

```text
PMSM-Dynamic-Model/
│
├── pmsm.m
└── README.md
```

## Author

**Hamed Soltani**

M.Sc. Control Engineering  
University of Tehran

GitHub: [hamedsoltani2000](https://github.com/hamedsoltani2000)

---

*This repository is intended for research, educational, and engineering applications involving permanent magnet synchronous motor modeling.*
