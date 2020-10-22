---
title: NEO
description: A Novel Expeditious Optimisation Algorithm for Reactive Motion Control of Manipulators
---

### _Jesse Haviland and Peter Corke_

**Under Review**

**[Preprint Avaliable Here](https://arxiv.org/abs/2010.08686)**

![Cover Image](/images/cover_lite.svg)

We present NEO, a fast and purely reactive motion controller for manipulators which can avoid static and dynamic obstacles while moving to the desired end-effector pose. Additionally, our controller maximises the manipulability of the robot during the trajectory, while avoiding joint position and velocity limits. NEO is wrapped into a strictly convex quadratic programme which, when considering obstacles, joint limits, and manipulability on a 7 degree-of-freedom robot, is generally solved in a few ms. While NEO is not intended to replace state-of-the-art motion planners, our experiments show that it is a viable alternative for scenes with moderate complexity while also being capable of reactive control. For more complex scenes, NEO is better suited as a  reactive local controller, in conjunction with a global motion planner. We compare NEO to motion planners on a standard benchmark in simulation and additionally illustrate and verify its operation on a physical robot in a dynamic environment. We provide an open-source library which implements our controller.

<br>

<iframe width="560" height="315" src="https://www.youtube.com/embed/jSLPJBr8QTY" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

<br>


* * *

## How do I use it?

We have incorporated required functionality into the [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python) which allows our algorithm to be used an any robot. See [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python) for installation instructions. We use the library [qpsolvers](https://pypi.org/project/qpsolvers/) to solve the optimisation function. However, you can use whichever solver you wish.


### Position-Based Servoing Example
```python
soon
```

### Acknowledgements

This research was supported by the Australian Centre for Robotic Vision (ACRV) and the Queensland Universilty of Technology Centre for Robotics (QCR).

![thanks](/images/acrvqut.png)
