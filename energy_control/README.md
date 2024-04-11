# Energy Control Examples

This folder contains MATLAB examples focusing specifically on Energy Control methods.

## Files in this Subfolder

- `example_01.m`: Energy control of a simple point-mass pendulum.
- `systems/`: Directory containing supporting functions and utilities for model systems, which includes the simple pendulum model.

## Example Descriptions

### Example 01: Energy Control of a Pendulum

Example file: `example_01.m`

The following example demonstrates the concept of energy control applied to a pendulum system. Our goal is to design an energy-based control strategy that aims to regulate the pendulum at its upright position, given constraints on the input.

Consider a simple point mass pendulum with the dynamics

$$
m g l \ddot{\theta} = m g l \sin(\theta) + u
$$

Let $x_1 = \theta$ represents the pendulum angle where $x_1 = 0$ corresponds to the upward configuration and measured counter clockwise, according to the right hand rule. $x_2 = \dot{\theta}$ denotes the angular velocity. Assuming $m=l=g=1$, the state space representation of the system is

$$
\begin{aligned}
    \dot{x}_1 &= x_2,\\
    \dot{x}_2 &= \sin(x_1) + u.
\end{aligned}
$$

The total mechanical energy of the system is given by

$$
E = \frac{1}{2}\dot{\theta}^2 + \cos(\theta) = \frac{1}{2} x_2^2 + \cos(x_1) \implies \dot{E} = x_2 u.
$$

Considering the energy at the upright position $E(\mathbf{x}=\mathbf{0}) = 1$, we introduce a Lyapunov function for the control system as

$$
V = \frac{1}{2}(E - 1)^2 \implies \dot{V} = (E-1) u x_2.
$$

To achieve control over the system's energy, we define the control $u_E$ as a function of the system state, such that $\dot{V}$ is negative definite. Accordingly, for some $ k > 0 $, we have

$$
u_E(\mathbf{x}) := - k (E(\mathbf{x}) - 1) \text{sgn}(x_2) \implies \dot{V} = -k(E(\mathbf{x}) - 1)^2|x_2| \leq 0.
$$

For stabilization at $x_1 = x_2 = 0$, let's introduce a secondary control function $u_S$. Note that the linear approximation of the system at $x_1 = x_2 = u = 0$ is

$$
\begin{bmatrix}
\dot{x}_1 \\
\dot{x}_2
\end{bmatrix} = 
\begin{bmatrix}
0 & 1 \\ 
1 & 0
\end{bmatrix}
\begin{bmatrix}
{x}_1 \\
{x}_2
\end{bmatrix} + 
\begin{bmatrix}
0 \\ 
1
\end{bmatrix} u.
$$

Choosing $u = -k_1 x_1 - k_2 x_2$ leads to linear approximation of the closed-loop system at the equilibrium point $x_1 = x_2 = 0$ as

$$
\begin{bmatrix}
\dot{x}_1 \\
\dot{x}_2
\end{bmatrix} = 
\begin{bmatrix}
0 & 1 \\
1 - k_1 & -k_2
\end{bmatrix}
\begin{bmatrix}
x_1 \\
x_2
\end{bmatrix},
$$

which is Hurwitz for $k_1, k_2 > 0$. Accordingly, we can define a hybrid control function as

$$
u(\mathbf{x}) := \text{proj}_{[-w,w]}
\begin{cases}
    u_S(\mathbf{x}) = -k_1x_1 - k_2x_2, & \text{if}\quad 1 - \cos(x_1) + x_2^2  \leq \epsilon,\\
    u_E(\mathbf{x}) = k (E - 1) \text{sgn}(x_2), & \text{otherwise},
\end{cases}
$$

where we have used

$$
\frac{1}{2}(\cos(x_1) - 1)^2 + \frac{1}{2}\sin^2(x_1) + x_2^2 = 1 - \cos(x_1) + x_2^2,
$$

as a measure of distance and for given $w > 0$ as the limit of actuator output torque, we have

$$
\text{proj}_{[-w, w]} u  =  
\begin{cases} 
u , & |u | < w,\\
\text{sgn}(u )\cdot w, &  |u | \geq w. 
\end{cases}
$$
