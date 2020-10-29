#!/usr/bin/env python
"""
@author Jesse Haviland
"""

import roboticstoolbox as rp
import spatialmath as sm
import time

# Configurations
q0 = [-0.5653, -0.1941, -1.2602, -0.7896, -2.3227, -0.3919, -2.5173]
q1 = [-0.1361, -0.1915, -1.2602, -0.8652, -2.8852, -0.7962, -2.039]
q2 = [0.2341, -0.2138, -1.2602, -0.4709, -3.0149, -0.7505, -2.0164]
q3 = [0.1584,  0.3429, -1.2382, -0.9829, -2.0892, -1.6126, -0.5582]
q4 = [0.3927,  0.1763, -1.2382, -0.1849, -1.96, -1.4092, -1.0492]
q5 = [-0.632,  0.5012, -1.2382, -0.8353, 2.2571, -0.1041,  0.3066]
q6 = [0.1683,  0.7154, -0.4195, -1.0496, 2.4832, -0.6028, -0.6401]
q7 = [-0.1198,  0.5299, -0.6291, -0.4348, 2.1715, -1.6403,  1.8299]
q8 = [0.2743,  0.4088, -0.5291, -0.4304, 2.119, -1.9994, 1.7162]
q9 = [0.2743,  0.4088, -0.5291, -0.4304, -0.9985, -1.0032, -1.7278]
qs = [q0, q1, q2, q3, q4, q5, q6, q7, q8, q9]

# Collisions
s1 = rp.Box(
    scale=[0.60, 1.1, 0.02],
    base=sm.SE3(0.95, 0, 0.20))

s2 = rp.Box(
    scale=[0.60, 1.1, 0.02],
    base=sm.SE3(0.95, 0, 0.60))

s3 = rp.Box(
    scale=[0.60, 1.1, 0.02],
    base=sm.SE3(0.95, 0, 1.00))

s4 = rp.Box(
    scale=[0.60, 1.1, 0.02],
    base=sm.SE3(0.95, 0, 1.40))

s5 = rp.Box(
    scale=[0.60, 0.02, 1.40],
    base=sm.SE3(0.95, 0.55, 0.7))

s6 = rp.Box(
    scale=[0.60, 0.02, 1.40],
    base=sm.SE3(0.95, -0.55, 0.7))

s = [s1, s2, s3, s4, s5, s6]

# Launch Simulator
env = rp.backend.Swift()
env.launch()

# Make PR2 and set joint angles
r = rp.models.PR2()
r.q = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.16825, 0.0, 0.0, 0.0, -0.5652894131595758, -0.1940789551546196,
    -1.260201738335192, -0.7895653603354864, -2.322747882942366,
    -0.3918504494615993, -2.5173485998351066, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 2.1347678577910827, 0.05595277251194286, 0.48032314980402596,
    -2.0802263633096487, 1.2294916701952125, -0.8773017824611689,
    2.932954218704465, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Add everything to the simulator
env.add(r)
for col in s:
    env.add(col)

# Leave open for 5 seconds then close the sim
time.sleep(5)