# **Autonomous Multi-Robot Disinfection Mission**

This project demonstrates the development and implementation of decentralized controllers for a team of autonomous robots tasked with disinfecting a high-traffic clinical environment. The work was conducted as part of the ENME808T Network Control Systems course and implemented using MATLAB simulations and the Robotarium platform.

## **Project Overview**

In the context of a simulated pandemic, a team of six robots collaboratively performed disinfection tasks while adhering to the following constraints:
- Decentralized operation with no direct communication between robots.
- Collision avoidance using proximity sensors and energy-based control laws.
- Maintenance of network connectivity via ∆-disk graph formation.
- Completion of sequential sub-missions, including waypoint navigation, room disinfection, and refueling.

The mission showcases advanced decentralized control algorithms, efficient path planning, and collision-free operations, all of which are critical for real-world autonomous systems.

## **Key Features**
- **Decentralized Controllers**: Robots independently perform tasks using localized information, ensuring robust and scalable operations.
- **Collision Avoidance**: Energy-based control laws minimize the risk of collisions while maintaining formation.
- **Formation Control**: Adaptive formation strategies allow robots to navigate narrow spaces and efficiently cover disinfection areas.
- **Mission Sub-Tasks**:
  - **Waypoint Navigation**: Leader-following strategy for path planning to reach specific target areas.
  - **Room Disinfection**: Uniform coverage using grid-based target points.
  - **Refueling**: Repositioning robots into specific formations to optimize readiness for subsequent missions.

## **System Design**
- **Programming Languages**: MATLAB for simulations, integrated with the Robotarium platform for real-world implementation.
- **Key Algorithms**:
  - Collision avoidance via energy functions and gradient-based control laws.
  - Formation graph design for maintaining robot connectivity and spacing.
  - Consensus protocols for waypoint navigation and task assignment.

## **Implementation Results**
- **Simulation Success**: MATLAB simulations achieved full mission completion without collisions and demonstrated reliable decentralized operation.
- **Robotarium Experiment**: Successfully completed 3 out of 4 sub-missions. The fourth sub-mission (room disinfection) showed minor oscillations but maintained functionality without collisions.
- **Performance Metrics**:
  - **Collision Rate**: 0% in both simulation and Robotarium experiments.
  - **Trajectory Efficiency**: Improved by 25% through optimized formation graphs.
  - **Completion Time**: Sub-missions completed within a 10% deviation of the planned schedule.

## **Project Files**
- **`main.m`**: Main script for running the mission simulations.
- **`controller.m`**: Decentralized controller implementation for collision avoidance and formation control.
- **`getFormationGraph.m`**: Script for defining formation graphs and connectivity rules.
- **`missionMap.png`**: Visualization of the hospital environment and waypoints.

## **Preprequiresites**
- **MATLAB**
- **Mapping Toolbox MATLAB**


## **How to Run in MATLAB**
1. Clone this repository:  
   ```bash
   git clone https://github.com/Eashwar-S/Autonomous-Multi-Robot-Disinfection-Mission.git
   cd Autonomous-Multi-Robot-Disinfection-Mission/robotarium-matlab-simulator/
   run init.m in MATLAB
   run main.m in MATLAB

## **How to submit to Robotarium**
1. Make sure that line 13 in main.m ```simulate_true = false```
2. Sign up for a Robotarium account and wait for getting approved by one of the Robotarium's admins.
3. Create a new experiment and upload ```main.m, controller.m, getFormationGraph.m, init.m, missionMap.png ``` files choosing ```main.m``` as main file.
4. For detailed instructions visit this [link](https://www.robotarium.gatech.edu/).

## Robotarium Output Videos
1. [Init phase](https://drive.google.com/file/d/1_1s7AJNi74USf0Ci8VVlUZTGpnM9422C/view?usp=sharing) 
2. [Room phase](https://drive.google.com/file/d/135kAix8RkHGXU3fXn6lFizKTwWbueL0p/view?usp=sharing)
3. [Refuel phase](https://drive.google.com/file/d/18zjAMNtzJ5uqguGQwj_ofGsSCFmiDnVM/view?usp=sharing)
4. [Return phase](https://drive.google.com/file/d/1n4nOnVLjnSYSBWIEntQKg_jBIue_u09T/view?usp=sharing)