# NeuroMPC: Neural System Identification & Fast Surrogate MPC

Methodology and results of **Physics-Informed Neural System Identification** and **Surrogate Model Predictive Control (Explicit Policy Distillation)** for non-linear dynamical systems.

---

## Summary & Core Objectives

Traditional Model Predictive Contro solves a constrained optimization problem at every sampling step. This solver introduces significant computational latency on resource-constrained embedded system. 

**NeuroMPC** addresses this computational cost through a two-stage data-driven method:
1. **System Identification:** Approximating non-linear cart-pole dynamics using both data-driven (Black-Box MLP) and physics involved (PINN) neural networks.
2. **Surrogate MPC:** Offloading the heavy implicit MPC optimization solver into a neural policy network capable of microsecond-level explicit inference via a single forward pass.

---

## Hypotheses & Benchmarking Goals

This repository is built to benchmark and evaluate two core hypotheses:

1. **Modeling Accuracy:**  
   $$\mathcal{H}_1: \text{PINN} \succ \text{Black Box MLP}$$  
   *Integrating physical differential equation residuals into the neural network loss function prevents state drift and ensures generalization outside training bounds.*

2. **Control Latency**  
   $$\mathcal{H}_2: \text{Surrogate Neural MPC} \succ \text{Online MPC}$$  
   *Surrogate policy networks achieve 100x–1000x latency reduction compared to online implicit optimization solvers with minimal loss in control optimality.*

---

## Mathematical Foundations

### 1. Non-linear Cart-Pole Dynamics
The system state vector is defined as $x = [p, \dot{p}, \theta, \dot{\theta}]^T$ and the control input $u = F$.

The equations of motion derived via Euler-Lagrange mechanics:

$$\ddot{\theta} = \frac{g \sin\theta - \cos\theta \left( \frac{u + m_p l \dot{\theta}^2 \sin\theta}{m_c + m_p} \right)}{l \left( \frac{4}{3} - \frac{m_p \cos^2\theta}{m_c + m_p} \right)}$$

$$\ddot{p} = \frac{u + m_p l \left( \dot{\theta}^2 \sin\theta - \ddot{\theta} \cos\theta \right)}{m_c + m_p}$$

Where:
* $m_c$: Cart mass
* $m_p$: Pendulum mass
* $l$: Pendulum half-length
* $g$: Gravitational acceleration ($9.81 \text{ m/s}^2$)

### 2. Discrete-Time Numerical Integration (RK4)
Continuous dynamics $\dot{x} = f(x, u)$ are discretized with a sampling period $T_s$ using 4th-Order Runge-Kutta integration:

$$k_1 = f(x_k, u_k)$$
$$k_2 = f\left(x_k + \frac{T_s}{2} k_1, u_k\right)$$
$$k_3 = f\left(x_k + \frac{T_s}{2} k_2, u_k\right)$$
$$k_4 = f\left(x_k + T_s k_3, u_k\right)$$
$$x_{k+1} = x_k + \frac{T_s}{6} (k_1 + 2k_2 + 2k_3 + k_4)$$
