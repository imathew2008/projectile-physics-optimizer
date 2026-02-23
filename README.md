# projectile-physics-optimizer
A 3D ballistic simulation and discrete trajectory optimizer with quadratic drag, Magnus lift, and real-time visualization.

## Overview

This project models realistic ballistic motion including:

- Gravity  
- Quadratic aerodynamic drag  
- Magnus lift from spin  
- Two-roller shooter exit dynamics  

It includes a discrete optimizer that adjusts launch velocity to minimize:

- Distance to target plane  
- Entry angle error  

A Compose Desktop visualization renders side-view (X–Y) trajectories and a top-down (X–Z) inset for lateral deviation analysis.

## Physics

Forces modeled:

- Gravity  
- F_d = -1/2 ρ C_d A |v| v  
- F_m ∝ ω × v  

Spin is derived from differential wheel speeds in a two-roller shooter.

## Optimization

The optimizer performs iterative discrete search with adaptive step reduction until convergence.

Cost function:

cost = distance_error + 0.8 * angle_error

## Purpose

Built as a physics simulation for autonomous projectile targeting for FRC.

<img width="360" height="201" alt="trajectory-ui" src="https://github.com/user-attachments/assets/f1260121-eade-4fa0-a53a-3e6d955bd2a3" />
