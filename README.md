# Spacecraft-Robotics-System
This folder contains matlab material to simulate a spacecraft equipped with a 3DoF robotic arm.

## Model Introduction

To derive the equation of motion of the chaser spacecraft equipped with a robotic arm, we first define the generalized system coordinates that can be expressed as

```math
q(t) = \begin{bmatrix} q^\top_c(t),& q^\top_r(t) \end{bmatrix}^\top,
```
where 
```math
q^\top_c(t) = \begin{bmatrix} x_c(t),\;y_c(t),\;z_c(t),\;\phi_c(t),\;\theta_c(t),\;\psi_c(t)\end{bmatrix}
```
 contains the generalized coordinates of the chaser (i.e. the base), $x_c(t), y_c(t), z_c(t)$ are the position coordinates of the CoM of the spacecraft while its orientation is described by 
$\phi_c(t), \theta_c(t), \psi_c(t)$. 

The generalized coordinates of the robotic arm are 
```math
q^\top_r(t) = \begin{bmatrix}q_1(t),...,q_{nr}(t) \end{bmatrix}$ 
```
where $nr$ is the number of DoFs (or joints) of the arm.
The equation of the dynamics of the Spacecraft Manipulator can be expressed as
```math
    \begin{bmatrix}
        M_c(q(t))&M_{c,r}(q(t))\\M^\top_{c,r}(q(t))&M_r(q(t))
    \end{bmatrix}\begin{bmatrix}
        \ddot{q}^\top_c(t)\\\ddot{q}^\top_r(t)
    \end{bmatrix}+\begin{bmatrix}
        C_c(q(t),\dot{q}(t))\\C_r(q(t),\dot{q}(t)) 
    \end{bmatrix}= \begin{bmatrix}
         u_c(t) \\u_r(t)   
        \end{bmatrix},        
```
where $M_c(q(t))\in \mathbb R^{6+nr} \rightarrow \mathbb R^{6+nr}$ is the chaser inertia matrix, $M_r(q(t))\in \mathbb R^{6+nr} \rightarrow \mathbb R^{nr+nr}$ is the spacecraft inertia matrix, and $M_{c,r}(q(t))\in \mathbb R^{6+nr} \rightarrow \mathbb R^{6+nr}$ is the interaction inertia matrix between the chaser and the robotic arm. The vectors $C_c(q(t),\dot{q}(t)) \in \mathbb R^{6+nr} \times \mathbb R^{6+nr} \rightarrow \mathbb R^{6}$ and $C_r(q(t),\dot{q}(t)) \in \mathbb R^{6+nr} \times \mathbb R^{6+nr} \rightarrow \mathbb R^{nr}$ represent the Coriolis and centrifugal components of the chaser and the arm, respectively. Moreover, $u_c(t) \in \mathbb R^6$ and $u_r(t) \in \mathbb R^{nr}$, are the inputs for the chaser and for the robot, respectively. More precisely, $u_c(t)$ contains the force and torques applied on to chaser using reaction wheels and thrusters, while $u_r(t)$ represents the torque applied to each joint of the manipulator.

For a 3DoFs robotic arm mounted on a spacecraft base, the generalized coordinates are 

```math
    q_c(t) = \begin{bmatrix}
        x_c(t),y_c(t),\psi_c(t)
    \end{bmatrix}^\top, \quad q_r(t) = \begin{bmatrix}
        q_1(t),q_2(t),q_3(t)
    \end{bmatrix}^\top.
```
### Trajectory Planning and Control Design ###

**Trajectory Planning**. The trajectories to be executed are pre-planned using an assigned initial condition and a desired reference. A quintic (5th order) polynomial is used to obtain the desired trajectory. In this example we plan five different trajectories for each of the states (position) of the system.

```math
    q_{c,r(t)} = \begin{bmatrix}
        x_{c,r}(t),y_{c,r}(t),\psi_{c,r}(t)
    \end{bmatrix}^\top, \quad q_{r,r}(t) = \begin{bmatrix}
        q_{1,r}(t),q_{3,r}(t),q_{3,r}(t)
    \end{bmatrix}^\top.
```

**Control Design.** We use a Proportional-Derivative (PD) controller to drive each state (position) to the desired referece. Note that this is not the best approach to perform trajectory tracking but can help to familiarize with the dynamic model of system. Each of the five PD can be expressed in the form:

```math
 u = K_p (x_d - x) - K_d\dot{x} ,
```
where $K_p$ is the proportional gain, $K_d$ is the derivative gain, $x_d$ is the desired reference (i.e $x_{c,r}(t),y_{c,r}(t),\psi_{c,r}(t),q_{1,r}(t),q_{3,r}(t),q_{3,r}(t)$ ), $x$ is the current measured state (e.g $x_c(t),y_c(t),\psi_c(t),q_1(t),q_3(t),q_r(t)$ ), and $\dot{x}$ the current velocity. 

## List of file in this folder
param_dyn.m = definition of the dynamic parameters. (this script will generate the param.mat)

TrajectoryFollowingExample3DoF.m = main script to run the simulation.

FFS_dynamic_model.m = this script contains the dynamic model of the system.

poly_traj.m = this script is the same function of the jtraj that is in the Robotic Toolboox by using ploy_traj.m you don't need to install the toolbox.

print_system_config.m = animation to print the motion of the system.

## How to use these scripts

**First Step.** Using the script param_dyn.m to define the dynamic paraments. This script will generate the param.mat file
**Second Step.** Using the script TrajectoryFollowingExample3DoF.m to perform the simululation (NOTE: the results and the animation can be enabled or disabled using appropriate flags in the script).

## References
