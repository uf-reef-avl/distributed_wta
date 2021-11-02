# Decentralized Weapon-Target Assignment under Asynchronous Communications
This repository contains a ROS package which implements the [DACOA](https://github.com/kathendrickson/DACOA) [algorithm](https://arxiv.org/pdf/2004.05142.pdf) to solve 
a weapon target assignment problem in real-time on both hardware and simulated systems.

### Installation
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git  clone https://github.com/uf-reef-avl/distributed_wta
cd distributed_wta
git submodule update --init --recursive 
```

### Overview
The ROS Package works both in Gazebo, hardware or a combination of Gazebo and hardware agents. The pacakge also comes with an visualization module to visualize
agents, targets and communications on RViz. 

### Dependencies
  geometry_msgs
  rospy
  std_msgs
  tf2
  message_generation
  visualization_msgs
  
Tested on ROS Melodic with Python 2.7 

### Citation
TBA

Please contact Katherine Hendrikson or Prashant Ganesh for questions.
