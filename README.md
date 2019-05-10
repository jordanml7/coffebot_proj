# The Rise and Fall of Coffeebot
## Jordan Lueck and Marlow Fawn

This workspace contains two main codes:

	The single floor coffee retrieval code, found in `coffeebot`.

	The combined multi-floor coffee retrieval code, found in `multifloor_coffeebot`.

Also included are all relevant launch files, in `coffeebot_launch`, and `prelim_tests`, which contain the code for early stage testing of various functionalities.
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

Must also have `tufts_service_robots` repo and all relevant Turtlebot and ROS packages installed.


## Running CoffeeBot (single-floor)

Place CoffeeBot at any location on the second floor of Halligan. Then...

In the first terminal, run:
```
$ roslaunch coffeebot_launch coffeebot_minimal.launch
```
Note that the minimal launch file is used because face and leg detection functionalities proved relatively ineffective. To use these, instead run 'coffeebot.launch' and uncomment the relevant marked sections of the coffeebot code.

Once rViz has launched, provide an initial pose and orientation estimate for CoffeeBot.

Then, in a second terminal, run:
```
$ rosrun coffeebot coffeebot
```
To activate CoffeeBot. Follow CoffeeBot's prompts. If CoffeeBot cannot be heard speaking, ensure the computer volume is at maximum.

Note that the "coffeeshop" is located at the top of the Halligan main stairs. Someone will need to be there to hand over the coffee to CoffeeBot. Note also that the sensor and Arduino have been removed they belong to Jordan, but the Arduino code used can be found in the `sensor_code` folder in the `coffeebot` package.


## Running CoffeeBot (multi-floor)

Place CoffeeBot at any location on the first floor of Halligan. Then...

In the first terminal, run:
```
$ roslaunch coffeebot_launch multifloor_coffeebot.launch
```
Note that this launch file also does not run any person detection functionalities.

Once rViz has launched, provide an initial pose and orientation estimate for CoffeeBot.

Then, in a second terminal, run:
```
$ rosrun multifloor_coffeebot multifloor_coffeebot
```

To activate CoffeeBot. Follow CoffeeBot's prompts. If CoffeeBot cannot be heard speaking, ensure the computer volume is at maximum.

Note that the "coffeeshop" is located at the top of the Halligan main stairs. Someone will need to be there to hand over the coffee to CoffeeBot. Someone will also need to be at the elevator when CoffeeBot arrives to use it.


