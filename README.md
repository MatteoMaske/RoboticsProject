# RoboticsProject
Introduction to robotics project 2022/2023

# Authors
##	@StefanoSacchet
##	@MatteoMascherin
##	@AmirGheser

@Description
	Project for the course of Introduction to Robotics, University of Trento 2022/2023

	Overall, the task of the project is to develop the code for a robotic arm capable of moving Megablocks of different classes to precise locations despite their orientation. Each of the 10 block types must be recognized and classified properly even if it’s placed upside down or lying on its side. The final task requires blocks to be moved around and piled up to build a hardcoded building such as a castle, a tower or more.

	This project relies on the locosim repository (https://github.com/mfocchi/locosim) that manages the simulation of the robot and the environment. It spawns the blocks and the environment and manages the gripper. The locosim repository is a submodule of this repository, so make sure you have it installed.

# Installation
## Requirements
- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- Python 3.8
- OpenCV 4.5.2
- Numpy 1.19.5
- Scipy 1.7.1
- Scikit-learn 0.24.2
- Scikit-image 0.18.3
- Matplotlib 3.4.2
- Pandas 1.3.2
- Tensorflow 2.5.0
- Keras 2.5.0
- Pytorch 1.9.0
- Torchvision 0.10.0
- CUDA 11.2
- CUDNN 8.1.1

## Installation
1. Clone the repository
2. Clone the submodules, make sure you have locosim installed
locosim: https://github.com/mfocchi/locosim
```
git submodule update --init --recursive
```
3. Install the requirements
4. Build the workspace
5. Source the workspace

# Usage
## Define your alias for ur5 script
1. Open your .bashrc file
```
nano ~/.bashrc
```
2. Add the following line
```
alias ur5='python3 -i ~/${your_catkin_ws}/src/locosim/ur5'
```
3. Save and exit
4. Source the .bashrc file
```
source ~/.bashrc
```

## Launch the simulation
### Preliminary steps and configuration

### Running the simulation
1. Open another terminal and launch the ur5 script from locosim, we've used an alias to launch the script
The ur5 will launch roscore, if you're looking to test the single nodes separately, you must launch roscore
```
ur5
```
2. Open another terminal and launch the move node
```
roslaunch cpp_publisher move
```
3. Open another terminal and launch the planner
```
roslaunch cpp_publisher planner
```
4. Open another terminal and launch the vision node
```
roslaunch py_publisher vision
```
The rviz and gazebo simulation should start and the robot should start moving

## Launch on the real robot
### Preliminary steps and configuration

### Running the script
1. Open another terminal and launch the ur5 script from locosim, we've used an alias to launch the script.
Roscore isn't necessary as ur5 will launch it
```
ur5
```
2. Open another terminal and launch the move node
```
roslaunch cpp_publisher move
```
3. Open another terminal and launch the planner
```
roslaunch cpp_publisher planner
```
4. Open another terminal and launch the vision node
```
roslaunch py_publisher vision
```

# Documentation
## Move node
The move node is responsible for moving the robot in the simulation, it's written in C++ and it's based on the ur5 script from locosim. The move node is launched by '''rosrun cpp_publisher move'''. The move node publishes the current position of the robot on the topic /ur5/position and it subscribes to the topic /ur5/goal to receive the goal position of the robot.

## Planner node
The planner node is responsible for planning the path of the robot, it's written in C++ and it's based on the ur5 script from locosim. The planner node is launched by '''rosrun cpp_publisher planner'''. The planner node subscribes to the topic /ur5/position to receive the current position of the robot and it publishes the goal position of the robot on the topic /ur5/goal.

## Vision node
The vision node is responsible for detecting the blocks in the simulation, it's written in Python. The vision node is launched by '''rosrun py_publisher vision'''. The vision node subscribes to the topic /ur5/zed_node/left/image_rect_color to receive the image from the camera and it publishes the position of the blocks on the topic vision/vision_detection.

# Sources
-[Report](https://docs.google.com/document/u/2/d/e/2PACX-1vQiZPfs2Z4FkhYv-KPZE-VaQdFag_Jqy1Be6Zwl1rrErOfBeSZTSVxRmRH1eSXyPTvBu4t7OcXht-1Q/pub)

-[YOLOv5](https://github.com/ultralytics/yolov5)

-[locosim](github.com/mfocchi/locosim)
