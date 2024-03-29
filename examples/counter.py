#!/usr/bin/env python
"""
@author Jesse Haviland
"""

import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as sg
import swift
import numpy as np
import time

# Configurations
q0 = [-0.8452,  0.0813, -1.8312, -1.555,  2.6911, -0.8326,  2.068]
q1 = [-0.4134, -0.238, -3.6504, -1.1768,  2.7225, -1.2706, -2.3709]
q2 = [-0.9547,  0.3356, -2.3151, -0.9126,  1.8166, -0.8724, -3.1287]
q3 = [-0.496, -0.2946, -2.626, -1.5671, -0.9644, -0.5307,  0.5828]
q4 = [-0.978, -0.235, -1.3629, -1.282, -2.2903, -0.4913,  0.9081]
q5 = [-0.3043, -0.1995,  0.4997, -0.9161, -3.0128, -1.2772, -0.4844]
q6 = [-0.0826, -0.3115, -0.7685, -1.0468, -2.8332, -1.2915,  0.7087]
q7 = [-0.9493, -0.2259, -1.2924, -1.2902, -2.2911, -0.5655, -2.1449]
q8 = [-0.0077, -0.1813, -1.2825, -0.2072, -2.475, -0.3674, -2.5659]
qs = [q0, q1, q2, q3, q4, q5, q6, q7, q8]

# Collisions
s1 = sm.Cuboid(
    scale=[0.60, 0.71, 0.02],
    base=sm.SE3(0.30, 0.355, 1.50))

s2 = sm.Cuboid(
    scale=[0.70, 0.02, 0.90],
    base=sm.SE3(0.65, 0.01, 1.06))

s3 = sm.Cuboid(
    scale=[0.59, 0.70, 0.02],
    base=sm.SE3(0.30, 0.35, 1.23))

s4 = sm.Cuboid(
    scale=[0.59, 0.70, 0.02],
    base=sm.SE3(0.30, 0.35, 0.92))

s5 = sm.Cuboid(
    scale=[0.59, 0.70, 0.02],
    base=sm.SE3(0.30, 0.35, 0.60))

s6 = sm.Cuboid(
    scale=[0.02, 0.70, 0.60],
    base=sm.SE3(0.60, 0.35, 0.30))

s7 = sm.Cuboid(
    scale=[0.07, 0.03, 0.03],
    base=sm.SE3(0.63, 0.20, 0.55))

s8 = sm.Cuboid(
    scale=[0.07, 0.03, 0.03],
    base=sm.SE3(0.63, 0.50, 0.55))

s9 = sm.Cuboid(
    scale=[0.03, 0.30, 0.03],
    base=sm.SE3(0.65, 0.35, 0.55))

s10 = sm.Cuboid(
    scale=[0.60, 0.02, 1.5],
    base=sm.SE3(0.30, 0.01, 0.75))

s11 = sm.Cuboid(
    scale=[0.60, 0.02, 1.5],
    base=sm.SE3(0.30, 0.70, 0.75))

s12 = sm.Cuboid(
    scale=[0.02, 0.70, 1.5],
    base=sm.SE3(0.02, 0.35, 0.75))

s13 = sm.Cuboid(
    scale=[0.60, 0.60, 0.02],
    base=sm.SE3(0.30, 1.01, 0.75))

s14 = sm.Cuboid(
    scale=[0.60, 0.52, 0.02],
    base=sm.SE3(0.30, 2.14, 0.75))

s15 = sm.Cuboid(
    scale=[0.60, 0.60, 0.02],
    base=sm.SE3(0.30, 1.60, 0.50))

s16 = sm.Cuboid(
    scale=[0.60, 0.02, 0.25],
    base=sm.SE3(0.30, 1.30, 0.625))

s17 = sm.Cuboid(
    scale=[0.60, 0.02, 0.25],
    base=sm.SE3(0.30, 1.90, 0.625))

s18 = sm.Cuboid(
    scale=[0.02, 2.4, 2.0],
    base=sm.SE3(0.01, 1.20, 1.0))

s19 = sm.Cuboid(
    scale=[0.02, 0.62, 0.76],
    base=sm.SE3(0.03, 1.60, 0.38))

s20 = sm.Cuboid(
    scale=[0.02, 0.64, 0.76],
    base=sm.SE3(0.60, 1.60, 0.38))

s21 = sm.Cuboid(
    scale=[0.08, 0.60, 0.02],
    base=sm.SE3(0.04, 1.60, 0.75))

s22 = sm.Cuboid(
    scale=[0.03, 0.03, 0.06],
    base=sm.SE3(0.04, 1.70, 0.78))

s23 = sm.Cuboid(
    scale=[0.03, 0.03, 0.06],
    base=sm.SE3(0.04, 1.50, 0.78))

s24 = sm.Cuboid(
    scale=[0.03, 0.03, 0.30],
    base=sm.SE3(0.04, 1.60, 0.90))

s25 = sm.Cuboid(
    scale=[0.25, 0.03, 0.03],
    base=sm.SE3(0.15, 1.60, 1.05))

s26 = sm.Cuboid(
    scale=[0.02, 0.02, 0.05],
    base=sm.SE3(0.265, 1.60, 1.025))

s27 = sm.Cuboid(
    scale=[0.40, 0.02, 0.70],
    base=sm.SE3(0.20, 0.74, 1.55))

s28 = sm.Cuboid(
    scale=[0.40, 0.02, 0.70],
    base=sm.SE3(0.20, 2.39, 1.55))

s29 = sm.Cuboid(
    scale=[0.40, 0.02, 0.70],
    base=sm.SE3(0.20, 1.62, 1.55))

s30 = sm.Cuboid(
    scale=[0.40, 1.67, 0.02],
    base=sm.SE3(0.20, 1.565, 1.20))

s31 = sm.Cuboid(
    scale=[0.40, 1.67, 0.02],
    base=sm.SE3(0.20, 1.565, 1.90))

s32 = sm.Cuboid(
    scale=[0.40, 1.67, 0.02],
    base=sm.SE3(0.20, 1.565, 1.55))

s33 = sm.Cuboid(
    scale=[0.60, 0.02, 0.76],
    base=sm.SE3(0.30, 1.29, 0.38))

s34 = sm.Cuboid(
    scale=[0.60, 0.02, 0.76],
    base=sm.SE3(0.30, 0.72, 0.38))

s35 = sm.Cuboid(
    scale=[0.02, 0.6, 0.75],
    base=sm.SE3(0.02, 1.00, 0.37))

s36 = sm.Cuboid(
    scale=[0.60, 0.6, 0.02],
    base=sm.SE3(0.30, 1.00, 0.35))

s37 = sm.Cuboid(
    scale=[0.60, 0.6, 0.02],
    base=sm.SE3(0.30, 1.00, 0.01))

s38 = sm.Cuboid(
    scale=[0.04, 0.04, 0.24],
    base=sm.SE3(0.43, 1.92, 0.87))

s39 = sm.Cuboid(
    scale=[0.05, 0.05, 0.16],
    base=sm.SE3(0.45, 1.97, 0.81))

s40 = sm.Cuboid(
    scale=[0.06, 0.06, 0.32],
    base=sm.SE3(0.36, 1.94, 0.92))

s41 = sm.Cuboid(
    scale=[0.60, 0.02, 0.75],
    base=sm.SE3(0.30, 2.39, 0.375))

s42 = sm.Cuboid(
    scale=[0.60, 0.02, 0.75],
    base=sm.SE3(0.30, 1.91, 0.375))

s43 = sm.Cuboid(
    scale=[0.02, 0.48, 0.16],
    base=sm.SE3(0.60, 2.16, 0.08))

s44 = sm.Cuboid(
    scale=[0.02, 0.48, 0.08],
    base=sm.SE3(0.61, 2.16, 0.72))

s45 = sm.Cuboid(
    scale=[0.02, 0.48, 0.74],
    base=sm.SE3(0.02, 2.15, 0.35))

s46 = sm.Cuboid(
    scale=[0.60, 0.48, 0.02],
    base=sm.SE3(0.90, 2.16, 0.16))

s47 = sm.Cuboid(
    scale=[0.56, 0.48, 0.02],
    base=sm.SE3(0.30, 2.15, 0.10))

s48 = sm.Cuboid(
    scale=[0.03, 0.03, 0.07],
    base=sm.SE3(1.10, 2.00, 0.12))

s49 = sm.Cuboid(
    scale=[0.03, 0.03, 0.07],
    base=sm.SE3(1.10, 2.3, 0.12))

s50 = sm.Cuboid(
    scale=[0.03, 0.33, 0.03],
    base=sm.SE3(1.10, 2.15, 0.08))

s51 = sm.Cuboid(
    scale=[0.2, 0.03, 0.26],
    base=sm.SE3(0.38, 2.34, 0.89))

s52 = sm.Cuboid(
    scale=[0.2, 0.02, 0.3],
    base=sm.SE3(0.41, 2.37, 0.92))

s = [s1, s2, s3, s4, s5, s6, s7, s8, s9,
     s10, s11, s12, s13, s14, s15, s16, s17, s18, s19, 
     s20, s21, s22, s23, s24, s25, s26, s27, s28, s29,
     s30, s31, s32, s33, s34, s35, s36, s37, s38, s39,
     s40, s41, s42, s43, s44, s45, s46, s47, s48, s49,
     s50, s51, s52]

# Launch Simulator
env = swift.Swift()
env.launch()

# Make PR2 and set joint angles
r = rtb.models.PR2()
r.q = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.16825, 0.0,
    0.0, 0.0, -0.8451866735633127, 0.08128204585620136, -1.8311522093787793,
    -1.5550420276338315, 2.6911049983477024, -0.8325789594278508,
    2.067980015738538, 2.248201624865942e-15, 0.0, 0.0, 0.0, 0.0, 0.0,
    -1.1102230246251565e-16, 1.5301977050596074, -0.1606585554274158, 1.122,
    -2.1212501013292435, 2.5303972717977263, -0.7028113314291433,
    1.925250846169634, -7.494005416219807e-15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0]

R = sm.base.q2r([0.03074371707199644, -0.0, -0.0, -0.9995273002077517])
T = np.eye(4)
T[:3, :3] = R
T = sm.SE3(T)
r.base = sm.SE3(1.0746809244155884, 1.500715732574463, 0.0) * T

# Add everything to the simulator
env.add(r)
for col in s:
    env.add(col)

# Leave open for 5 seconds then close the sim
time.sleep(5)
