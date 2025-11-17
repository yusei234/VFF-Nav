# C++ Charged Spheres Calibration Simulation

A simulation of charged spheres using C++ for autonomous flight calibration. Searches for optimal parameters to guide a drone to a goal while avoiding obstacles.

# Macro Architecture & Purpose (C++)

The C++ portion inside `inc/` and `src/` is designed as a lightweight simulation lab for calibrating electrostatic Center-of-Mass navigation. The executable (`src/main.cpp`) repeatedly instantiates a navigation controller, randomizes its parameters, builds a universe of charged obstacles, and runs a physics simulation until a successful trial is found or the budget is exhausted. Everything is organised so that new navigation modules, samplers, or presenters can be swapped in without touching unrelated code.

## Layout and layering decisions

-   **Headers vs. sources:** Public headers live under `inc/`, keeping template-heavy code available to downstream projects, while `src/` contains the concrete implementations (`lin_algebra`, `domain/navigation/COM`, `adapters`, `driven_ports`, and the executable). This split keeps build artifacts small and enables reuse of the domain layer in other binaries.
-   **Domain building blocks:** `inc/domain/lin_algebra/{point,vector}.hpp` and `src/lin_algebra/*.cpp` provide the POD math types used everywhere. Physics abstractions such as `mechanics::CenterOfMass` and `eletro::ChargedSphere` live under `inc/domain/physics/**` and stay header-only so controllers can inline their calculations.
-   **Navigation contracts:** `inc/domain/navigation/controller/*.hpp`, `drone.hpp`, `parameters.hpp`, and `modules.hpp` define the common interfaces. The COM module (`inc/domain/navigation/COM/**` plus `src/domain/navigation/COM/*.cpp`) specialises those contracts with `SphereDrone`, `ElectricCOMController`, and `COMResultCode`, keeping controller logic decoupled from the simulator.
-   **Simulation utilities:** `inc/domain/simulation/core/{simulator.hpp,sim_probe.hpp}` implement the time-stepping harness, while the `inc/domain/simulation/searcher/**` tree hosts higher-level orchestration (queues, planners, workers, facades, and evaluators). These headers are template-only so tests and tools can embed the searcher without linking extra objects.

## Navigation modules and controllers

-   `com_nav::ElectricCOMController` (`inc/domain/navigation/COM/controller.hpp`, `src/domain/navigation/COM/controller.cpp`) derives from `navigation::ParamController` and exposes bounded parameter vectors for search. It owns a `SphereDrone`, obstacles (`eletro::ChargedSphere`), and a goal, encapsulating all force computations (goal, obstacle, damping) plus stopping conditions.
-   `SphereDrone` (`inc/domain/navigation/COM/point_drone.hpp`, `src/domain/navigation/COM/point_drone.cpp`) adapts the generic `navigation::Drone` contract to a point-mass model backed by `mechanics::CenterOfMass`, ensuring every controller state transition is deterministic and unit-testable.
-   `navigation::modules::COM` (`inc/domain/navigation/modules.hpp`) bundles the controller, domain obstacle, presenter specs, and success enum behind a `NavigationModule` concept so the simulator/searcher code can stay generic.

## Simulation harness and parameter search

-   `simulation::Simulator` (`inc/domain/simulation/core/simulator.hpp`) is a thin loop that calls `Controller::update()`, then checks for termination and optionally forwards each step to a probe. It purposefully keeps no policy logic so it can be embedded in benchmarks or tooling.
-   `simulation::Searcher` (`inc/domain/simulation/searcher/engine/searcher.hpp`) owns the macro flow: sample parameters, seed a universe via `UniverseFactory`, spin up per-trial presenters, and run trials (optionally in parallel) until a success code surfaces. Precomputing `TrialConfig` instances makes the search deterministic for a fixed RNG seed.
-   Parameter exploration is pluggable through `DefaultParamSampler` and `GridParamSampler` (`inc/domain/simulation/support/param_sampler.hpp`). Controllers expose their min/max via `navigation::ParamController`, letting the sampler stay generic.
-   Environment generation is abstracted by `UniverseFactory` (`inc/domain/simulation/support/universe_factory.hpp`), which clears/creates obstacles and goals given an enum option. The COM implementation (`COMUniverseFactory`) demonstrates both handcrafted and random obstacle layouts, keeping world-building separate from controller logic.
-   `NavigationTraits` (`inc/domain/simulation/core/navigation_traits.hpp`) ties a navigation module to its default search collaborators (sampler, universe options, config presenters), making it easy to slot new modules into the same pipeline.

## Driven ports and adapters

-   Driven ports (`inc/domain/driven_ports/*.hpp`) define passive interfaces such as `SimConfigPresenter` and `SimulatorStepPresenter`. They deliberately avoid mentioning filesystems or formats so the domain layer remains pure.
-   Concrete adapters live in `inc/adapters/presenter/*.hpp` and `src/adapters/presenter/*.cpp`. `CSVPresenter` buffers per-step state and writes `out/trial_<n>.csv` on demand, while `SimConfigSaver` emits a `sim_config.yaml` compatible with the Python visualizer. Both implement the driven ports without leaking IO concerns into the simulator.
-   Specifications under `inc/domain/driven_ports/specs/com.hpp` (with serialization in `src/driven_ports/specs/com_charged_sphere.cpp`) translate domain obstacles into presenter-friendly structs, ensuring IO stays decoupled from physics representations.

## Composition entry point

`src/main.cpp` wires everything together: it seeds RNGs, fills a universe through `COMUniverseFactory`, chooses a sampler (`DefaultParamSampler` or `GridParamSampler`), installs presenters, and kicks off a `simulation::Searcher` run. The entry point also owns the output directory lifecycle so individual components remain oblivious to filesystem policy. Swapping controllers or presenters only requires changing the aliases near the top of `main.cpp`, showcasing the benefit of the layered design.

# Units

For simplicity, the simulation uses the SI unit system on all quantities.

# Center of Mass Guidance Model - Electrostatic Analogy

This model uses an analogy to electrostatic forces to guide a drone towards a goal while avoiding obstacles. The drone is treated as a charged particle influenced by attractive and repulsive forces.

## Definitions

-   $\vec{p}$: Position vector of the drone
-   $\vec{v} = \dfrac{d\vec{p}}{dt}$: Velocity vector of the drone
-   $\vec{g}$: Position vector of the goal
-   $\vec{o_i}$: Position vector of the center of obstacle $i$ (assumed spherical)
-   $n_{obs}$: Number of obstacles
-   $\vec{r}_g = \vec{p} - \vec{g}$ (goal to drone)
-   $\vec{r}_i = \vec{p} - \vec{o_i}$ (obstacle to drone)
-   $\forall \vec{v} \neq \vec{0}, \hat{v} = \dfrac{\vec{v}}{\lVert\vec{v}\rVert}$. $\hat{0} = \vec{0}$. $\hat{v}$ is dimensionless, even if $\vec{v}$ has units.

### Total force

$$
\vec{F}_{total} = \vec{F}_{goal} + \vec{F}_{\text{obs, squash}} + \vec{F}_{damping}
$$

### Goal force

$$
\vec{F}_{goal} = \left( -F_{\text{goal, const}} + \frac{k_e \cdot Q_{drone} \cdot Q_{\text{goal}}}{\lVert\vec{r}_g\rVert^2 + \varepsilon_{\text{goal}}^2} \right) \hat{r}_g
$$

> **Notes:**
>
> -   The goal force is always attractive, pulling the drone towards the goal, due to $Q_{goal} < 0$.
> -   $k_e = 1 \left(N \cdot m^2 \cdot C^{-2}\right)$, a Coulomb's constant analog, is included for dimensional consistency. Since its value is 1, it does not affect the force magnitude and can be omitted in calculations.
> -   $Q_{drone} = 1 \left(C\right)$ is the drone's charge, which is constant. It is included for dimensional consistency. Since its value is 1, it does not affect the force magnitude and can be omitted in calculations.

### Obstacle force

$$
\vec{F}_{\text{obs},i} = \left( \frac{ k_e \cdot Q_{drone} \cdot Q_i }{ \lVert\vec{r}_i\rVert^2 + \varepsilon_{\text{obs}}^2 } \right) \hat{r}_i
$$

> **Notes:**
>
> -   Each obstacle exerts a repulsive force on the drone, pushing it away from the obstacle, due to $Q_i > 0$.
> -   $k_e = 1 \left(N \cdot m^2 \cdot C^{-2}\right)$, a Coulomb's constant analog, is included for dimensional consistency. Since its value is 1, it does not affect the force magnitude and can be omitted in calculations.
> -   $Q_{drone} = 1 \left(C\right)$ is the drone's charge, which is constant. It is included for dimensional consistency. Since its value is 1, it does not affect the force magnitude and can be omitted in calculations.

#### Raw and squashed obstacle force

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

### Damping force

$$
\vec{F}_{damping} = -K_{damping} \cdot \vec{v}
$$

## System invariants

-   Coulomb's constant (analog): $k_e = 1 \left(N \cdot m^2 \cdot C^{-2}\right)$
-   Drone mass: $m_{drone} = 1 \left(kg\right)$
-   Drone charge: $Q_{drone} = 1 \left(C\right)$
-   Obstacle charges: $Q_i \in [0, +1] \left(C\right)$

## Parameters for calibration

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

## Notes

-   **No singularities:** both goal and obstacles use $\varepsilon$ to avoid $1/r^2$ blow-ups.
-   **Dimensionless tanh:** $\dfrac{\lVert \vec{F}_{\text{obs, raw}} \rVert}{F_{\text{sat}}}$ is unitless; $F_{\text{fac,squash}}$ carries the resulting Newton scale.

# Drone orientation control

The orientation of the drone is done via the generation of torques that rotate it towards the desired orientation. The desired orientation is defined by the direction of the velocity vector of the drone projected on the horizontal plane and an upwards direction aligned with the world z axis.

## Definitions

-   $\vec{g}$: Position vector of the goal (world frame)
-   $\vec{p}$: Position vector of the drone (world frame)
-   $\vec{v} = \dfrac{d\vec{p}}{dt}$: Velocity vector of the drone (world frame)
-   $\vec{\omega} = (\omega_x, \omega_y, \omega_z)$: Angular velocity vector of the drone in body frame
-   $\hat{e}_x$ : Unit vector in the x direction (world frame)
-   $\hat{e}_y$ : Unit vector in the y direction (world frame)
-   $\hat{e}_z$ : Unit vector in the z direction (world frame)
-   $\hat{u}_x = (1, 0, 0)$: Unit vector in the x direction (forward) in the body frame
-   $\hat{u}_y = (0, 1, 0)$: Unit vector in the y direction (right) in the body frame
-   $\hat{u}_z = (0, 0, 1)$: Unit vector in the z direction (upwards) in the body frame
-   $q$: Orientation quaternion of the drone
-   Rotations:

$$
Rot_q(\vec{v}) = q \, \left(0, \vec{v}\right) \, q^{-1}
$$

-   $\hat{f} = Rot_q(\hat{u}_x)$: Forward direction unit vector of the drone in world frame
-   $\hat{r} = Rot_q(\hat{u}_y)$: Right direction unit vector of the drone in world frame
-   $\hat{u} = Rot_q(\hat{u}_z)$: Upwards direction unit vector of the drone in world frame

## Desired heading

The desired heading is computed as the unit vector of the projection on the horizontal plane of the velocity vector of the drone.

$$
\vec{v}_{horizontal} = \vec{v} - (\vec{v} \cdot \hat{e}_z) \cdot \hat{e}_z
$$

> **Note:** In the code, this is computed by zeroing the vertical component of the velocity vector and normalizing the result.

$$
\hat{f}_{desired} = \frac{\vec{v}_{horizontal}}{\lVert \vec{v}_{horizontal} \rVert}
$$

> **Note:** if $\lVert \vec{v}_{horizontal} \rVert \approx 0$, then no actions are taken to change the orientation of the drone.

This is a unit vector in the horizontal plane representing the desired forward direction of the drone. That means that its information can be reduced to a single angle $\psi$ (yaw) around the vertical axis $\hat{e}_z$. To calculate this angle, we can use the dot product between the desired forward direction and the world forward direction:

$$
\psi_{\text{des}} = \operatorname{atan2} \left( \hat{f}_{desired} \cdot \hat{e}_y , \hat{f}_{desired} \cdot \hat{e}_x \right)
$$

With this angle, we can calculate a quaternion representing the desired orientation of the drone:

$$
q_{desired} = \left[ \cos\left(\frac{\psi_{\text{des}}}{2}\right), 0, 0, \sin\left(\frac{\psi_{\text{des}}}{2}\right) \right]
$$

Thus, the needed alignment rotation to reach the desired orientation is given by:

$$
q_{align} = q_{desired} \cdot q^{-1}
$$

$q_align$ represents the rotation needed to align the drone's current orientation with the desired heading. Naming its axis and angle as:

-   $\hat{u}_{align}$: Unit vector representing the axis of alignment rotation (in world frame)
-   $\theta_{align}$: Angle of alignment rotation

We obtain $\hat{u}_{align}$ and $\cos(\theta_{align}/2)$ directly from the quaternion components. Then, we can compute $\cos(\theta_{align})$ as:

$$
\cos(\theta_{align}) = 2 \cdot \cos^2\left(\frac{\theta_{align}}{2}\right) - 1
$$

## Alignment torque

The direction of the alinment torque (expressed in the world frame) is $\hat{\tau}_{align} = \hat{u}_{align}$. The magnitude of the alignment torque is computed based on the angle $\theta_{align}$ as:

$$
\boxed{
\lVert \vec{\tau}_{align} \rVert = K_{\text{align}} \cdot (K_{cardioid} - \cos(\theta_{align}))
}
$$

## Parameters for calibration

These are going to be determined by the simulation calibration program.

| Parameter          | Description             | Unit          | Range          |
| ------------------ | ----------------------- | ------------- | -------------- |
| $K_{\text{align}}$ | Alignment torque factor | N·m           | $[0, +\infty[$ |
| $K_{cardioid}$     | Cardioid shape factor   | Dimensionless | $[1, +\infty[$ |

## Notes

-   **Anti-parallel threshold:** $\epsilon_{align}$ should be a small value, e.g., $10^{-6}$, to avoid numerical instability when determining if the vectors are anti-aligned. This value is not subject to calibration and can be hardcoded in the implementation.

# Solid Body Dynamics

## Quaternion Integration and Angular Motion

Quaternions are used to represent the drone’s orientation in 3D space without gimbal lock.  
A unit quaternion

$$
q = (w, x, y, z)
$$

can be written as

$$
q = [\,\cos(\tfrac{\theta}{2}),\, \mathbf{u}\sin(\tfrac{\theta}{2})\,],
$$

where:

-   $\theta$ is the rotation angle
-   $\mathbf{u}$ is the unit axis of rotation

This quaternion represents a rotation of $\theta$ radians around the axis $\mathbf{u}$.

### Rotating a Vector

To rotate a vector $\mathbf{v}$ by quaternion $q$, the quaternion form of $\mathbf{v}$ is  
$[0, \mathbf{v}]$, and the rotated vector is obtained by:

$$
\mathbf{v}' = q \, [0, \mathbf{v}] \, q^{-1}
$$

The quaternion multiplication (Hamilton product) follows:

$$
(a_1, \mathbf{b}_1)(a_2, \mathbf{b}_2)
= (a_1 a_2 - \mathbf{b}_1 \cdot \mathbf{b}_2,\;
   a_1 \mathbf{b}_2 + a_2 \mathbf{b}_1 + \mathbf{b}_1 \times \mathbf{b}_2)
$$

This rule defines how scalar and vector parts interact — dot products generate scalars, while cross products handle the vector rotation direction.

---

## Quaternion Time Evolution

Given the body’s angular velocity $\boldsymbol{\omega} = (\omega_x, \omega_y, \omega_z)$, the time derivative of the orientation quaternion is:

$$
\dot{q} = \tfrac{1}{2} [0, \boldsymbol{\omega}]\, q
$$

This means the quaternion’s rate of change is half the quaternion product between $q$ and the angular-velocity quaternion.

Expanding component-wise gives:

$$
\dot{q} =
\frac{1}{2}
\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z \\
\omega_x & 0 & \omega_z & -\omega_y \\
\omega_y & -\omega_z & 0 & \omega_x \\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
\begin{bmatrix}
w \\ x \\ y \\ z
\end{bmatrix}
$$

### Meaning of the Ω(ω) Matrix

The matrix above, often denoted $\Omega(\boldsymbol{\omega})$, is simply a compact way to represent all the sign changes that naturally appear in the Hamilton product when multiplying by $[0, \boldsymbol{\omega}]$.  
It doesn’t introduce new math — it’s just a shorthand that expands to:

$$
[0, \boldsymbol{\omega}] q =
(-\omega_x x - \omega_y y - \omega_z z,\;
 w \omega_x + \omega_y z - \omega_z y,\;
 w \omega_y + \omega_z x - \omega_x z,\;
 w \omega_z + \omega_x y - \omega_y x)
$$

This is exactly the same as performing the quaternion multiplication manually; the matrix form simply makes it linear in $q$.

---

## Integration Schemes

### Euler Approximation

For small time steps, an explicit Euler step gives:

$$
q_{t+\Delta t} \approx q_t + \tfrac{1}{2}[0, \boldsymbol{\omega}] q_t \, \Delta t
$$

Afterward, $q$ must be normalized to maintain unit length.

### Exponential Map (Exact for Constant Angular Velocity)

If $\boldsymbol{\omega}$ is approximately constant during $\Delta t$, define:

$$
\theta = \|\boldsymbol{\omega}\| \, \Delta t, \quad
\mathbf{u} = \frac{\boldsymbol{\omega}}{\|\boldsymbol{\omega}\|}
$$

Then, the incremental rotation (delta quaternion) is:

$$
\delta q =
[\cos(\tfrac{\theta}{2}),\, \mathbf{u} \sin(\tfrac{\theta}{2})]
$$

and the orientation updates as:

$$
q_{t+\Delta t} =
\begin{cases}
\delta q \, q_t, & \text{if } \boldsymbol{\omega} \text{ is in world frame} \\
q_t \, \delta q, & \text{if } \boldsymbol{\omega} \text{ is in body frame}
\end{cases}
$$

This “exponential map” update is numerically stable and corresponds to an exact rotation when angular velocity is constant over the time step.

---

## Angular Dynamics

For an isotropic rigid body (inertia scalar $I$):

$$
\frac{d\boldsymbol{\omega}}{dt} = \frac{\boldsymbol{\tau}}{I}
$$

which integrates as:

$$
\boldsymbol{\omega}_{t+\Delta t} = \boldsymbol{\omega}_t + \frac{\boldsymbol{\tau}}{I} \Delta t
$$

and then feeds into the quaternion integration to update orientation.

---

## Summary

| Concept               | Formula                                                         | Implementation                         |
| --------------------- | --------------------------------------------------------------- | -------------------------------------- |
| Vector rotation       | $\mathbf{v}' = q[0,\mathbf{v}]q^{-1}$                           | `Quaternion::rotate()`                 |
| Quaternion derivative | $\dot{q} = \frac{1}{2}[0,\boldsymbol{\omega}]q$                 | In integrator                          |
| Ω(ω) matrix           | Encodes Hamilton-product sign pattern                           | `Ω(ω) * q` form                        |
| Euler integration     | $q' = q + \dot{q}\Delta t$                                      | Simple but drifty                      |
| Exponential map       | $\delta q = [\cos(\tfrac{θ}{2}), \mathbf{u}\sin(\tfrac{θ}{2})]$ | Accurate and stable                    |
| Frame choice          | Left: world, Right: body                                        | `integrateWorld()` / `integrateBody()` |

> **In short:**  
> The Ω matrix is just a clean, matrix-friendly way to express the “weird sign stuff” that naturally appears in quaternion multiplication — no magic, just algebra in disguise.

# Software architecture

This simulation is implemented in C++20, using modern programming practices for clarity and performance. The code is organized into classes representing the drone, obstacles, and the simulation environment. Currently, the domain of the software is the only part developed. Later, it will be decided whether to integrate it with an external GUI or visualization tool, which will result in additional architectural complexity. Alternatively, the results could be exported for visualization in external software like Python or MATLAB.

## Software layout

The layout chosen is a simple layered architecture, with the following layers:

-   **Optimizer**: Responsible for searching the parameter space to find optimal calibration parameters.
-   **Navigation**: Implements the guidance model and drone control logic.
-   **Physics**: Handles the needed dynamics, kinematics and physics simulation.
-   **Linear Algebra**: Provides math utilities for all above layers such as vectors, points and quaternions.

Not all of those layers are fully implemented yet. More details can be found below, as each layer is described in its own section.

## Layer descriptions

This description will both provide the intended functionality of each layer as well as the current implementation status. The layers are described from the bottom (Linear Algebra) to the top (Optimizer).

### Linear Algebra

This layer provides basic linear algebra utilities such as vectors, points and quaternions. Each class only provides the necessary operations needed by the upper layers, avoiding unnecessary complexity. The classes provided are intended to closely resemble mathematical objects, making the code more readable and easier to understand. To the current needs, the following classes are implemented:

-   `Vector`: Represents a 3D vector with basic operations such as addition, subtraction, and dot product.
-   `Point`: Represents a 3D point, essentially a vector with a fixed origin.
-   `Quaternion`: Represents a quaternion for 3D rotation, with operations for multiplication and normalization.

### Physics

This layer provides physics utilities for the simulation of the drone and obstacles. It implements the solid body kinematics and dynamics using quaternions for orientation representation, plus basic electric field calculations. To the current needs, the following classes and modules are implemented:

-   `CenterOfMass`: Class that implements a point mass with position, velocity, and methods to apply forces and update its state.
-   `SolidBody`: Class that represents a rigid body in the simulation, with properties such as mass, orientation, and angular velocity. It provides methods for updating the body's state based on applied forces and torques;
-   `SO3`: Module that provides functions for quaternion operations relevant for solid body kinematics;
-   `ChargedSphere`: Class that represents a charged sphere obstacle in the simulation, with properties such as position and charge, and methods to compute the electrostatic field it generates.

### Navigation

This layer implements the guidance model and drone control logic. It is responsible for processing sensor data, planning trajectories, and controlling the drone's movement. Some prototypes of these modules have been implemented but the most recent architectural decisions have not been implemented yet. The current design features a modular approach, with an interface `INavigator<T>` templated to a type of obstacle. Different navigation strategies can be implemented by realizing this interface in one class of a fully cohesive module, focused on a single navigation strategy, including its model for the drone itself and the action generation logic.

#### COM Navigation

As an example, the `COM Navigation` module implements the Center of Mass guidance model described above. It does not concern itself with the alignment of the drone, nor with the sensors used to detect obstacles. Instead, it assumes perfect knowledge of the environment and focuses solely on computing the forces acting on the drone based on the positions of the goal and obstacles. It uses the `CenterOfMass` class from the Physics layer to represent the drone and compute its movement based on the electrostatic analogy.

#### Sonar Navigation

An intended improvement on the navigation is the `Sonar Navigation` module, conceptualized but currently at early stages of development. It assumes the drone is equipped a sonar sensor at its front, which can measure the distance to obstacles in its line of sight, which forms a plane circular sector in space aligned to its orientation. This module will implement a different navigation strategy, using logic such as:

-   Obstacles will be registered in the universe as 3D solids such as spheres or boxes, the sonar will simulate ray casting to detect obstacles in its field of view;
-   The navigation logic will use the sonar readings to position virtual charged spheres in space, which will be continuously added to an internal memory of obstacles;
-   The navigation logic will use the internal memory of obstacles to compute the electrostatic forces acting on the drone, similar to the COM Navigation module.
-   Possibly, the navigation logic will also implement a simple obstacle forgetting mechanism, to avoid the memory growing indefinitely.
-   The navigation logic will use the `SolidBody` class from the Physics layer to represent the drone, allowing it to control both its position and orientation.
-   The navigation logic will implement the alignment torque computation to control the drone's orientation towards the goal.

### Simulation

This layer provides the core infrastructure for running physics-based simulations to evaluate navigation parameter sets. It consists of two main components:

-   **Core Simulator**: A lightweight time-stepping loop that advances the simulation state by repeatedly calling the controller's update method, checking termination conditions, and optionally forwarding each step to a probe for data collection. The simulator is policy-agnostic and can be embedded in benchmarks or higher-level search algorithms.

-   **Composition Framework**: Provides parameter space region abstractions and the search engine that bridges A\* tree search with simulation evaluation. Each search node represents a region of parameter space with methods to split into subregions and sample representative parameter sets for evaluation.

The simulation layer evaluates parameter quality by running multiple randomized trials (universes) with different obstacle configurations and averaging their heuristic scores, providing robust performance estimates that guide the planning layer's search.

### Planning

The planning layer implements a multi-phase, covariance-guided parameter space search that combines global exploration with adaptive local refinement. It orchestrates multiple independent A\* tree search runs to discover optimal calibration parameters for the navigation model, using simulation heuristics to evaluate candidate parameter sets.

The algorithm executes in two main phases:

#### Phase 1: Orthogonal Exploration

The search begins with **8 independent orthogonal search runs**, each exploring the parameter space using axis-aligned rectangular regions. This phase provides broad coverage of the parameter space and establishes an initial set of promising candidates.

**Search mechanics:**

-   Each A\* tree node represents a hyperrectangular region defined by min/max bounds for each parameter
-   **Region splitting:** At expansion, a node splits along each parameter axis independently, creating $2^n$ child regions for $n$ parameters (binary subdivision per dimension)
-   **Parameter sampling:** Each region samples a representative "check point" by drawing from a gaussian centered at the region midpoint with a predetermined standard deviation proportional to the region size
-   **Evaluation:** The heuristic function averages simulation outcomes across multiple randomized universes, measuring how well the parameters guide the drone to the goal
-   **Priority queue:** The frontier is ordered by heuristic scores (for implementing best-first search, costs are set to 0), with the best nodes expanded first

**Interest collection:**

The search maintains two thresholds:

-   `maxGoal`: Parameters achieving heuristics below this threshold are considered successful solutions
-   `maxInterest`: Parameters below this threshold are recorded as "interesting" candidates for later analysis, regardless of whether they reach the goal

All promising parameter sets and their scores are collected in a global pool for statistical analysis.

**Termination conditions:**

-   A parameter set meeting the goal threshold is found
-   Maximum nodes evaluated is reached

#### Phase 2: Principal Axis Refinement

After collecting initial candidates, the algorithm performs **3 refinement phases**, each running **8 independent searches** guided by the statistical structure of previously successful parameter sets.

**Adaptive region definition:**

1. **Covariance analysis:** Compute weighted covariance matrix from all collected parameter sets, weighting by inverse heuristic score (better solutions contribute more)
2. **Eigendecomposition:** Extract principal components (eigenvectors) and their variances (eigenvalues) to identify the most important directions of variation in parameter space
3. **Principal axis regions:** Instead of axis-aligned splits, the search now partitions space along the principal components, focusing computational budget on directions where good parameters tend to vary most

**Splitting strategy:**

Each principal axis region splits by:

-   Selecting a principal direction (eigenvector)
-   Creating child regions offset along that direction by distances proportional to $\sigma \sqrt{\lambda}$, where $\lambda$ is the eigenvalue and $\sigma$ controls the search radius
-   Starting with $\sigma = 3.0$ and decaying by 0.8× each phase to progressively narrow the search

**Adaptive threshold decay:**

Both thresholds decay by 0.8× after each phase:

-   Orthogonal phase → Principal phase 1: $4.0 \to 3.2$
-   Phase 1 → Phase 2: $3.2 \to 2.56$
-   Phase 2 → Phase 3: $2.56 \to 2.05$

This progressive tightening forces the algorithm to refine solutions rather than accept mediocre parameters found early.

#### Parallel Heuristic Evaluation

To accelerate search, heuristic computation is parallelized across **16 worker threads**. When a node is expanded and generates child regions, all children's heuristics are evaluated concurrently before being inserted into the priority queue. This is safe because heuristic evaluation was purposefully designed to be side-effect free, allowing multiple simulations to run in parallel without interference.

#### Final Output

After all phases complete, the algorithm reports the best parameter set found along with all "interesting" candidates collected during the search.

# Requirements

-   g++ compiler with C++20 support
-   make utility

# Sequence Diagram

-   g++ compiler with C++20 support
-   make utility
