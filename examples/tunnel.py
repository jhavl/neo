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
q0 = [-0.3728,  1.0259, -0.9876, -1.2082,  2.0677, -0.563, -1.6563]
q1 = [0.2993,  0.1747, -1.7835, -0.8593,  3.1042, -1.2145, -1.3626]
q2 = [0.3537,  0.2079, -1.7168, -0.2937,  3.0224, -0.7083, -1.3286]
q3 = [0.3398, -0.2349, -0.0415, -1.5042,  2.7647, -1.7995,  3.0539]
qs = [q0, q1, q2, q3]

# Collisions
s1 = sm.Cuboid(
    scale=[0.6, 0.04, 0.6],
    base=sm.SE3(0.65, -0.15, 0.43))

s2 = sm.Cuboid(
    scale=[0.6, 0.04, 0.6],
    base=sm.SE3(0.65, -0.15, 1.22))

s3 = sm.Cuboid(
    scale=[0.3, 0.04, 0.2],
    base=sm.SE3(0.8, -0.15, 0.83))

s = [s1, s2, s3]

# Launch Simulator
env = swift.Swift()
env.launch()

# Make PR2 and set joint angles
r = rtb.models.PR2()
r.q = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.16825, 0.0, 0.0, 0.0, -0.37279882212870064, 1.0259015008778194,
    -0.9875997438281771, -1.208229229103619, 2.0676739431952065,
    -0.5630237954661839, -1.6563473012595384, 2.248201624865942e-15, 0.0, 0.0,
    0.0, 0.0, 0.0, -1.1102230246251565e-16, 1.1473791555597268,
    -0.2578419004077155, 0.5298918609954418, -2.121201719392923,
    2.198118788614387, -1.4189668927954484, 2.1828521334438378,
    -1.2961853812498703e-14, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

R = sm.base.q2r([0.9999523740218402, 0.0, 0.0, 0.00975959466810749])
T = np.eye(4)
T[:3, :3] = R
T = sm.SE3(T)
r.base = sm.SE3(-0.013043247163295746, -0.2435353398323059, 0.0) * T

# Add everything to the simulator
env.add(r)
for col in s:
    env.add(col)

# Leave open for 5 seconds then close the sim
time.sleep(5)
