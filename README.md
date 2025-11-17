# ScanAir: Autonomous Drone Navigation using Virtual Force Fields

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

# Virtual Force Field Algorithm

This model uses an analogy to electrostatic forces to guide a drone towards a goal while avoiding obstacles. The drone is treated as a charged particle influenced by attractive and repulsive forces.

## Definitions

- **p**: Position vector of the drone
- **v = dp/dt**: Velocity vector of the drone  
- **g**: Position vector of the goal
- **oᵢ**: Position vector of the center of obstacle i (assumed spherical)
- **n_obs**: Number of obstacles
- **r_g = p - g** (goal to drone)
- **rᵢ = p - oᵢ** (obstacle to drone)
- **v̂** = unit vector in direction of v

## Total Force

**F_total = F_goal + F_obs,squash + F_damping**

## Goal Force

**F_goal = [ -F_goal,const + (k_e · Q_drone · Q_goal) / (‖r_g‖² + ε_goal²) ] · r̂_g**

**Notes:**
- The goal force is always attractive, pulling the drone towards the goal, due to Q_goal < 0
- k_e = 1 (N·m²·C⁻²), Coulomb's constant analog for dimensional consistency
- Q_drone = 1 (C), drone's charge for dimensional consistency

## Obstacle Force

**F_obs,i = [ (k_e · Q_drone · Q_i) / (‖rᵢ‖² + ε_obs²) ] · r̂ᵢ**

**Raw and Squashed Obstacle Force:**

**F_obs,raw = Σ F_obs,i**

**F_obs,squash = F_fac,squash · tanh( ‖F_obs,raw‖ / F_sat ) · F̂_obs,raw**

**Notes:**
- Each obstacle exerts a repulsive force (Q_i > 0)
- tanh(x) ∈ [-1, 1] for all real x
- Squashing can be skipped when ‖F_obs,raw‖ is small for performance

## Damping Force

**F_damping = -K_damping · v**

## System Invariants

- Coulomb's constant analog: k_e = 1 (N·m²·C⁻²)
- Drone mass: m_drone = 1 (kg)
- Drone charge: Q_drone = 1 (C)
- Obstacle charges: Q_i ∈ [0, +1] (C)

## Parameters for Calibration

| Parameter | Description | Unit | Range |
|-----------|-------------|------|-------|
| Q_goal | Goal charge | C | ]-∞, 0[ |
| F_goal,const | Goal constant attraction | N | [0, +∞[ |
| ε_goal | Goal softening factor | m | [0, +∞[ |
| ε_obs | Obstacle softening factor | m | [0, +∞[ |
| F_sat | Obstacle total saturation scale | N | ]0, +∞[ |
| F_fac,squash | Obstacle total factor after squashing | N | [0, +∞[ |
| K_damping | Damping constant | N·s/m | [0, +∞[ |

## Notes

- **No singularities:** both goal and obstacles use ε to avoid 1/r² blow-ups
- **Dimensionless tanh:** ‖F_obs,raw‖/F_sat is unitless; F_fac,squash carries the Newton scale

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
