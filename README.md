# Development and Testing of a Moblie Manipulation Robot

The graduation project for Bachelor of Science, from the class of 2024 of the [_Department of Electromechanical Engineering_](https://www.fst.um.edu.mo/eme/), _University of Macau_

<p align="center">by Cheney, Dechao Jiang and Bobby, Tong Zhang</p>
<p align="center">Project Supervisor: PhD, FASME, Professor Qingsong Xu</p>

Connect with us!

**Cheney**: dcjiang@berkeley.edu / dechao.jiang@connect.um.edu.mo

**Bobby**: TZHANG043@e.ntu.edu.sg / bobby.zhang@connect.um.edu.mo

## Introduction
### Libot: Autonomous Mobile Manipulation Robot for Libraries
In this final year project, we proposed Libot. Libot is a mobile manipulation robot designed to automate tasks in library environments. Combining the capabilities of the UR5 robotic arm and MiR250 mobile platform, enhanced with a Realsense 435i depth camera, Libot autonomously navigates and interacts within library settings.

### Feature
- **Book Detection System:** Utilizes the YOLO v8 deep learning model to recognize book spine labels from RGB images, facilitating efficient book cataloging.
- **Deep-learning-based Object grasping:** Integrated with the [MoveIt](https://moveit.ros.org/) platform through move group API to optimize book handling through precise pick-and-place operations. Deep-learning algorithm [GPD](https://github.com/atenpas/gpd?tab=readme-ov-file) by [Andreas](https://www.khoury.northeastern.edu/home/atp/) is used for grasp generation.
- **SLAM Mapping and Navigation:** Enhanced with Simultaneous Localization and Mapping (SLAM) to adapt to dynamic library layouts, improving navigation and operational efficiency. Mapping through hectoring mapping by [TeamHector](https://www.teamhector.de/) at TU Darmstadt.

## Testing and Results


## Experimental Design
### Structure of Libot
### Simulation Environment
### Control Methodology

**Dependencies:**
Make sure the official repository of Universal Robot is in your overlay workspace





