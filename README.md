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
$q^\top_r(t) = \begin{bmatrix}q_1(t),...,q_{nr}(t) \end{bmatrix}$ 
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

## List of file in this folder
param_dyn.m = definition of the dynamic parameters. (this script will generate the param.mat)

TrajectoryFollowingExample3DoF.m = main script to run the simulation.

FFS_dynamic_model.m = this script contains the dynamic model of the system.

poly_traj.m = this script is the same function of the jtraj that is in the Robotic Toolboox by using ploy_traj.m you don't need to install the toolbox.

print_system_config.m = animation to print the motion of the system.

## How to use these scripts

Lunch before etc etc
