# Complete Equation Reference — PMSM Simulator

Every equation implemented in the interactive PMSM simulator, organized by category. The state vector is **x** = [i_d, i_q, ω_m, θ_e] and the inputs are v_d, v_q, and T_L.

---

## 1 · Electrical Angular Velocity

Links mechanical and electrical domains via the number of pole pairs:

```
ω_e = N_p · ω_m
```

## 2 · Electrical Rotor Angle

The fourth state variable, integrated from electrical speed:

```
dθ_e / dt = ω_e = N_p · ω_m
```

---

## 3 · Stator Voltage Equations (dq Frame)

The core electrical dynamics of the PMSM in the rotor-synchronous reference frame.

### 3.1 — d-axis voltage equation

```
v_d = R_s · i_d + L_d · (di_d / dt) − ω_e · L_q · i_q
```

Rearranged as a state derivative for integration:

```
di_d / dt = (1 / L_d) · (v_d − R_s · i_d + ω_e · L_q · i_q)
```

### 3.2 — q-axis voltage equation

```
v_q = R_s · i_q + L_q · (di_q / dt) + ω_e · L_d · i_d + ω_e · λ_m
```

Rearranged as a state derivative for integration:

```
di_q / dt = (1 / L_q) · (v_q − R_s · i_q − ω_e · L_d · i_d − ω_e · λ_m)
```

---

## 4 · Flux Linkage Equations

Computed as derived outputs at each time step.

### 4.1 — d-axis flux linkage

```
λ_d = L_d · i_d + λ_m
```

### 4.2 — q-axis flux linkage

```
λ_q = L_q · i_q
```

---

## 5 · Back-EMF Components

The speed-dependent voltage terms that oppose the applied stator voltages.

### 5.1 — d-axis back-EMF

```
e_d = −ω_e · L_q · i_q
```

### 5.2 — q-axis back-EMF

```
e_q = ω_e · L_d · i_d + ω_e · λ_m
```

---

## 6 · Electromagnetic Torque

Derived from the co-energy of the machine. Contains two components:

```
T_e = (3/2) · N_p · [ λ_m · i_q + (L_d − L_q) · i_d · i_q ]
         \_____________/   \__________________________/
          magnet torque          reluctance torque
```

- **Magnet torque**: `(3/2) · N_p · λ_m · i_q` — present in all PMSMs.
- **Reluctance torque**: `(3/2) · N_p · (L_d − L_q) · i_d · i_q` — nonzero only for salient machines (L_d ≠ L_q).

---

## 7 · Mechanical Dynamics

Newton's second law applied to the rotor.

### 7.1 — Speed equation (torque balance)

```
J · (dω_m / dt) = T_e − T_L − B · ω_m
```

Rearranged as a state derivative for integration:

```
dω_m / dt = (1 / J) · (T_e − T_L − B · ω_m)
```

---

## 8 · Inverse Park Transform (dq → abc)

Converts rotating dq-frame quantities back to the stationary three-phase abc frame. Applied to both currents (i_d, i_q → i_a, i_b, i_c) and voltages (v_d, v_q → v_a, v_b, v_c).

### 8.1 — Phase A

```
f_a = f_d · cos(θ_e) − f_q · sin(θ_e)
```

### 8.2 — Phase B

```
f_b = f_d · cos(θ_e − 2π/3) − f_q · sin(θ_e − 2π/3)
```

### 8.3 — Phase C

```
f_c = f_d · cos(θ_e + 2π/3) − f_q · sin(θ_e + 2π/3)
```

where `f` represents either voltage or current, and `θ_e` is the electrical rotor angle.

---

## 9 · Electrical Power

Instantaneous electrical power input to the motor, computed in the dq frame using the power-invariant form:

```
P_e = (3/2) · (v_d · i_d + v_q · i_q)
```

---

## 10 · Speed Unit Conversion

Converts mechanical angular velocity from rad/s to revolutions per minute:

```
RPM = ω_m · 60 / (2π)
```

---

## 11 · Complete State-Space Summary

Collecting all four state derivatives into a single system:

```
dx/dt = f(x, u)

where  x = [ i_d, i_q, ω_m, θ_e ]ᵀ
       u = [ v_d, v_q, T_L ]ᵀ

┌              ┐
│  di_d  / dt  │ = (1/L_d) · (v_d − R_s·i_d + ω_e·L_q·i_q)
│  di_q  / dt  │ = (1/L_q) · (v_q − R_s·i_q − ω_e·L_d·i_d − ω_e·λ_m)
│  dω_m  / dt  │ = (1/J)   · (T_e − T_L − B·ω_m)
│  dθ_e  / dt  │ = N_p · ω_m
└              ┘

with:
  ω_e = N_p · ω_m
  T_e = (3/2) · N_p · [λ_m·i_q + (L_d − L_q)·i_d·i_q]
```

---

## 12 · Parameter Definitions

| Symbol | Description                        | Default Value | Unit      |
| ------ | ---------------------------------- | ------------- | --------- |
| R_s    | Stator resistance per phase        | 1.2           | Ω         |
| L_d    | d-axis stator inductance           | 0.008         | H         |
| L_q    | q-axis stator inductance           | 0.008         | H         |
| λ_m    | Peak permanent magnet flux linkage | 0.175         | Wb        |
| N_p    | Number of pole pairs               | 4             | —         |
| J      | Rotor moment of inertia            | 0.0008        | kg·m²     |
| B      | Viscous friction coefficient       | 0.001         | N·m·s/rad |
| v_d    | d-axis input voltage               | 0             | V         |
| v_q    | q-axis input voltage               | 24            | V         |
| T_L    | Load torque                        | 0             | N·m       |

---

## 13 · Equation Count

| Category                    | Count                        |
| --------------------------- | ---------------------------- |
| State derivatives (ODEs)    | 4                            |
| Electromagnetic torque      | 1                            |
| Flux linkages               | 2                            |
| Back-EMF components         | 2                            |
| Electrical angular velocity | 1                            |
| Inverse Park (per quantity) | 3                            |
| Inverse Park applications   | ×2 (currents + voltages) = 6 |
| Electrical power            | 1                            |
| Unit conversion (RPM)       | 1                            |
| **Total unique equations**  | **18**                       |
