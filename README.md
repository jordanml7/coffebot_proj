# Coffeebot

## Installation

Currently supports ROS Kinetic only.

First, create a workspace and clone the source repositories:
```
$ source /opt/ros/$ROS_DISTRO/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/jordanml7/coffebot_proj.git
```

Next, install basic dependencies:
```
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

Then, build everything:
```
$ catkin_make
$ source devel/setup.bash
```


## Upload changes

Navigate to workspace and into coffeebot_proj folder
```
$ git init
$ git add .
$ git commit -m "comments describing this commit"
$ git remote add origin https://github.com/jordanml7/coffebot_proj.git
$ git push origin master
```


## Launch a TurtleBot2

Must have tufts_service_robots repo installed in same workspace.

In the first terminal, do:

```
$ roslaunch tbot2_launch tbot2_lidar.launch OR tbot2.launch
```

In a second terminal, do:

```
$ roslaunch tbot2_launch amcl_navigation.launch
```

After rviz shows up, provide the robot's initial location. Now you can launch any app or node you wrote which moves the robot around.

In a third terminal, do:
```
$ rosrun [folder containing node] [node name], like...
$ rosrun navigation_test navigation_test
```


