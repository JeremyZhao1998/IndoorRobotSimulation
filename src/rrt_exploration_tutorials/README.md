# rrt_exploration_tutorials
This package is a complementary package for the [rrt_exploration](https://github.com/hasauino/rrt_exploration) ROS package. It provides all the needed Gazebo simulation files to bring up Kobuki robots equipped with laser scanners and ready for the [rrt_exploration](https://github.com/hasauino/rrt_exploration) package. 

## 1. Requirements
The package has been tested on both ROS Kinetic and ROS Indigo, it should work on other distributions like Jade. The following requirements are needed before installing the package:

1- You should have installed a ROS distribution (indigo or later. Recommended is either indigo or kinetic).

2- Created a workspace.

3- Installed the "gmapping" ROS package: on Ubuntu, and if you are running ROS Kinectic, you can do that by typing the following command in the terminal:

```sh
$ sudo apt-get install ros-kinetic-gmapping
```
4- Install ROS navigation stack. You can do that with the following command (assuming Ubuntu, ROS Kinetic):
```sh
$ sudo apt-get install ros-kinetic-navigation
```
5- Install Kobuki robot packages:
```sh
sudo apt-get install ros-kinetic-kobuki ros-kinetic-kobuki-core
sudo apt-get install ros-kinetic-kobuki-gazebo
```
## 2. Installation
Download the package and place it inside the ```/src``` folder in your workspace. And then compile using ```catkin_make```.

## 3. Example
```sh
roslaunch rrt_exploration_tutorials single_simulated_house.launch
```