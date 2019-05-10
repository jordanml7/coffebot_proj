#include <ros/ros.h>
#include <tf/tf.h>
#include "std_msgs/Bool.h"

#include <vector>
#include <iostream>
#include <string>
#include <stdio.h>

using namespace std;

bool coffee;

void sleepok(int, ros::NodeHandle &);
void detect_coffee(const std_msgs::Bool& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_client");
    ros::NodeHandle n;
    
    // subscriber for sensor msgs
    ros::Subscriber coffee_detect = n.subscribe("/coffee_topic",1,detect_coffee);
    //sleep for a bit to make sure the sub will work
    sleepok(2,n);
	
  
    while (ros::ok()) {
        
        ros::spinOnce();
        if (coffee)
			cout << "Yep!" << endl;
		else
			cout << "Nope!" << endl;
			
		sleep(1);
    }
    
    return 0;

}

void sleepok(int t, ros::NodeHandle &nh)
{
    if (nh.ok())
        sleep(t);
}

void detect_coffee(const std_msgs::Bool& msg)
{
	coffee = msg.data;
}
