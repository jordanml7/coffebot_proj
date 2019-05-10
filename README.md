# The Rise and Fall of Coffeebot
## Jordan Lueck and Marlow Fawn

This workspace contains three main divisions:
	The CoffeeBot code, found in `coffeebot`, for single-floor coffee retrieval.
	The Elevator navigation code, found in `open_door_detector`.
	The combined multi-floor coffee retrieval code, found in `multifloor_coffeebot`.

Also include are all relevant launch files, in 'coffeebot_launch', and 'prelim_tests', which includes all the code for early stage testing of various functionalities
Descriptions of how to install the code and run each division can be found below.

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

Must also have tufts_service_robots repo and all relevant turtlebot and ros packages installed.


## Running CoffeeBot (single-floor)

Place CoffeeBot at any location on the second floor of Halligan. Then...

In the first terminal, do:
```
$ roslaunch coffeebot_launch coffeebot_minimal.launch
```
Note that the minimal launch file is used because face and leg detection functionalities proved relatively ineffective. To use these, instead run 'coffeebot.launch' and uncomment the relevant marked sections of the coffeebot code.

Once rViz has launched, provide an initial pose and orientation estimate for CoffeeBot.

Then, in a second terminal, do:
```
$ rosrun coffeebot coffeebot
```

To activate CoffeeBot. Follow CoffeeBot's prompts. If CoffeeBot cannot be heard speaking, ensure the computer volume is at maximum.

Note that the 

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


