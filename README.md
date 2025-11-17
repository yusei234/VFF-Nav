# VFF-Nav: Autonomous Drone Navigation using Virtual Force Fields

![Python](https://img.shields.io/badge/Python-3.10-blue.svg)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy_Jalisco-green.svg)
![Gazebo](https://img.shields.io/badge/Gazebo-11-orange.svg)
![PX4](https://img.shields.io/badge/PX4-1.14-lightgrey.svg)

A research project implementing Virtual Force Fields for autonomous drone navigation and obstacle avoidance, developed in partnership with CITI-USP.

## Research paper

https://github.com/miguel-usp/tcc-scanair-doc


## Overview

ScanAir addresses the fundamental challenge of autonomous obstacle avoidance for UAVs using a Virtual Force Field (VFF) approach. The algorithm creates attractive forces toward goals and repulsive forces from obstacles, enabling real-time navigation in complex environments.

**Key Features:**
- Real-time autonomous navigation and obstacle avoidance 
- Integration with ROS 2 and Gazebo simulation
- Support for PX4 autopilot
- Search of optimized parameters for the force field

## ⚡ Quick Start

### Prerequisites
- Ubuntu 22.04+ or 24.04+
- ROS 2 Jazzy Jalisco
- Gazebo Fortress or Garden
- PX4 Autopilot
- Python 3.10

### Installation

1. **Clone the repository:**
```bash
**placeholder**
```

2. **Setup the simulation environment:**

3. **Build the workspace:**
```bash
colcon build
source install/setup.bash
```

### Running the Simulation

1. **Launch the simulation:**
```bash
ros2 launch
```
2. **Run the VFF algorithm:**
   In an other terminal:
   ```bash
   ros2 run
   ```

## Center of Mass Guidance Model - Electrostatic Analogy

This model uses an analogy to electrostatic forces to guide a drone towards a goal while avoiding obstacles. The drone is treated as a charged particle influenced by attractive and repulsive forces.

### Definitions

-   $d\vec{p}$: Position vector of the drone
-   $d\vec{v} = \dfrac{d\vec{p}}{dt}$: Velocity vector of the drone
-   \vec{g}: Position vector of the goal
-   $\vec{o_i}$: Position vector of the center of obstacle $i$ (assumed spherical)
-   $n_{obs}$: Number of obstacles
-   $\vec{r}_g = \vec{p} - \vec{g}$ (goal to drone)
-   $\vec{r}_i = \vec{p} - \vec{o_i}$ (obstacle to drone)
-   $\forall \vec{v} \neq \vec{0}, \hat{v} = \dfrac{\vec{v}}{\lVert\vec{v}\rVert}$. $\hat{0} = \vec{0}$. $\hat{v}$ is dimensionless, even if $\vec{v}$ has units.

### Total force

$
\vec{F}_{total} = \vec{F}_{goal} + \vec{F}_{\text{obs, squash}} + \vec{F}_{damping}
$

### Goal force


\vec{F}_{goal} = \left( -F_{\text{goal, const}} + \frac{k_e \cdot Q_{drone} \cdot Q_{\text{goal}}}{\lVert\vec{r}_g\rVert^2 + \varepsilon_{\text{goal}}^2} \right) \hat{r}_g


> **Notes:**
>
> -   The goal force is always attractive, pulling the drone towards the goal, due to $Q_{goal} < 0$.
> -   $k_e = 1 \left(N \cdot m^2 \cdot C^{-2}\right)$, a Coulomb's constant analog, is included for dimensional consistency. Since its value is 1, it does not affect the force magnitude and can be omitted in calculations.
> -   $Q_{drone} = 1 \left(C\right)$ is the drone's charge, which is constant. It is included for dimensional consistency. Since its value is 1, it does not affect the force magnitude and can be omitted in calculations.

#### Obstacle force

$$
\vec{F}_{\text{obs},i} = \left( \frac{ k_e \cdot Q_{drone} \cdot Q_i }{ \lVert\vec{r}_i\rVert^2 + \varepsilon_{\text{obs}}^2 } \right) \hat{r}_i
$$

> **Notes:**
>
> -   Each obstacle exerts a repulsive force on the drone, pushing it away from the obstacle, due to $Q_i > 0$.
> -   $k_e = 1 \left(N \cdot m^2 \cdot C^{-2}\right)$, a Coulomb's constant analog, is included for dimensional consistency. Since its value is 1, it does not affect the force magnitude and can be omitted in calculations.
> -   $Q_{drone} = 1 \left(C\right)$ is the drone's charge, which is constant. It is included for dimensional consistency. Since its value is 1, it does not affect the force magnitude and can be omitted in calculations.

##### Raw and squashed obstacle force

$$
\vec{F}_{\text{obs, raw}} = \sum_{i=1}^{n_{\text{obs}}}{\vec{F}_{\text{obs},i}}
$$

$$
\vec{F}_{\text{obs, squash}} = F_{\text{fac,squash}} \cdot \tanh\left( \frac{ \lVert \vec{F}_{\text{obs, raw}} \rVert }{ F_{\text{sat}} } \right) \cdot \hat{F}_{\text{obs, raw}}
$$

> **Notes:**
>
> -   $\forall x \in \mathbb{R}, \tanh(x) \in [-1, 1]$
> -   $\forall x \in [-0.3, 0.3], \lvert \tanh(x) - x \rvert < 0.01$ - This fact is used to skip squashing when the raw obstacle force is small enough, improving performance without significant loss of accuracy.

#### Damping force

$$
\vec{F}_{damping} = -K_{damping} \cdot \vec{v}
$$

### System invariants

-   Coulomb's constant (analog): $k_e = 1 \left(N \cdot m^2 \cdot C^{-2}\right)$
-   Drone mass: $m_{drone} = 1 \left(kg\right)$
-   Drone charge: $Q_{drone} = 1 \left(C\right)$
-   Obstacle charges: $Q_i \in [0, +1] \left(C\right)$

### Parameters for calibration

These are going to be determined by the simulation calibration program.

| Parameter               | Description                           | Unit  | Range          |
| ----------------------- | ------------------------------------- | ----- | -------------- |
| $Q_{goal}$              | Goal charge                           | C     | $]-\infty, 0[$ |
| $F_{goal, const}$       | Goal constant attraction              | N     | $[0, +\infty[$ |
| $\varepsilon_{goal}$    | Goal softening factor                 | m     | $[0, +\infty[$ |
| $\varepsilon_{obs}$     | Obstacle softening factor             | m     | $[0, +\infty[$ |
| $F_{\text{sat}}$        | Obstacle total saturation scale       | N     | $]0, +\infty[$ |
| $F_{\text{fac,squash}}$ | Obstacle total factor after squashing | N     | $[0, +\infty[$ |
| $K_{damping}$           | Damping constant                      | N·s/m | $[0, +\infty[$ |

### Notes

-   **No singularities:** both goal and obstacles use $\varepsilon$ to avoid $1/r^2$ blow-ups.
-   **Dimensionless tanh:** $\dfrac{\lVert \vec{F}_{\text{obs, raw}} \rVert}{F_{\text{sat}}}$ is unitless; $F_{\text{fac,squash}}$ carries the resulting Newton scale.


## Results

### Performance Metrics and Optimal parameters

### Demo

**links para video de demonstration**


## Team

This project was developed by:
-   Gavril Loyer (16101509)
-   Miguel Velasques Abilio Piola Alves (11807601)
-   Vitor Chinaglia (11912363)

Supervision:
-   Prof. Dr. Bruno Albertini
-   In partnership with CITI-USP

## Citation

If you use this work in your research, please cite:
```bibtex
@misc{scanair2024,
  title={ScanAir: Autonomous Drone Navigation using Virtual Force Fields},
  author={Loyer, G. P. L. and Alves, M. V. A. P. and Chinaglia, V.},
  year={2025},
  publisher={GitHub},
  howpublished={\url{https://github.com/miguel-usp/tcc-scanair-doc}}
}
```

```<div align="center">
Developed at Escola Politécnica da USP
Department of Computer Engineering and Digital Systems

</div> ```
