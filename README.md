# A Threat Predictor System of Dynamic Environments by using a Multi-Robot fleet
3D Simulation of a set of robots used to patrol and and secure dynamic environments.
This software system is composed of ROS packages, topics, and services. The scenarios are simulated using the Gazebo 3D 
simulator and the Pedsim pedestrian simulator.
<p align="center"> 
<img src=https://github.com/amattag/mrs_predictor/blob/master/images/Patrolling-01.png width=800 align="center"/>
</p>

### Features
- 3D Simulation the Robotnik Summit-XL mobile robot and all its sensors in Gazebo.
- Coordination of a multi-robot system using Finite State Machines.
- Pedestrian walking simulation using social force models.
- Threats detection and prediction based on cooperative Growing Hidden Markov Models (GHMMs).
- Extensive and detailed visualization using Rviz.

### Requirements
- ROS with the Navigation Stack (currently tested on ROS Indigo and Ubuntu 14.04)
- Gazebo 2.2
- C++11 compiler
- Python
- NumPy

### Installation
```
cd [workspace]/src
git clone https://github.com/amattag/mrs_predictor.git
cd ..
catkin_make
```
### Sample usage
```
roslaunch mrs_predictor mrs_predictor.launch $world_file_name
```
### Contributors
* Antonio Matta-Gómez
* Antonio Barrientos Cruz

The meta-package is a **work in progress**, currently it is being used for testing in a Phd Thesis. It holds a BSD license.
