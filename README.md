# Map My World

Submission for project #4 within the Udacity Robotics Software Engineer nanodegree.

## Description

A demonstration of a robot using the Real-Time Appearance-Based Mapping (RTB-Map) algorithm to perform simultaneous localization and mapping (SLAM) within an unknown environment. A simple two-wheeled robot uses an RGB-D camera and laser rangefinder sensor to build a map of its surroundings as it drives around. The robot is built using the ROS framework and simulated using the Gazebo simulator.

## Getting Started
#### Clone repo into catkin workspace:
```
$ git clone git@github.com:MattCotton20/MapMyWorld ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
#### Initialise the robot and environment:
```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
This will start an instance of Gazebo containing the robot housed within a building interior.

#### Switch on the RTB-Map algorithm:
In a second terminal, run:
```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot mapping.launch
```
This will start an instance of RTAB-Map Viz, ready to demonstrate the algorithm at work.

#### Control the robot:
In a third terminal, run:
```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot teleop.launch
```
With this terminal highlighted, the robot can now be driven within Gazebo (see prompts for key bindings). As you drive the robot around, the RTAB-Map Viz window will show the map being built, along with identified features and loop closures. Once complete, close the RTAB-Map terminal to save the data to `~/.ros/rtabmap.db`

#### View the generated map:
Run:
```
$ rtabmap-databaseViewer ~/.ros/rtabmap.db
```
Then enable:
* View -> Constraint View
* View -> Graph View

#### Localize the robot using the generated map:
Repeat the steps as before, except in the second terminal, instead run:
```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot localization.launch
```
Instead of generating a new map, the robot will use the previous map as the 'truth' and localise itself against that.

## Author

* Matt Cotton
