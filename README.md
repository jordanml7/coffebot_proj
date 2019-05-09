# Coffeebot

## Installation

Currently supports ROS Kinetic only.

First, create a workspace and clone the source repositories:
```
$ source /opt/ros/$ROS_DISTRO/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/jordanml7/coffeebot_proj.git
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


## Upload & Download changes

To upload changes, navigate to workspace and into coffeebot_proj folder
```
$ git init
$ git add .
$ git commit -m "comments describing this commit"
$ git remote add origin https://github.com/jordanml7/coffebot_proj.git
$ git push origin master
```

To download latest version
```
$ git pull
```


## Launch a TurtleBot2

Must have tufts_service_robots repo installed in same workspace.

In the first terminal, do:

```
$ roslaunch coffeebot_launch coffeebot_launch
```

After rviz shows up, provide the robot's initial location. Now you can launch any app or node you wrote which moves the robot around.

In a second terminal, do:
```
$ rosrun [folder containing node] [node name], like...
$ rosrun navigation_test navigation_test
```

Modify the above launch file with any additional nodes


