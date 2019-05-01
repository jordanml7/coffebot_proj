#include <ros/ros.h>
#include <tf/tf.h>
#include "std_msgs/Bool.h"

#include <vector>
#include <iostream>
#include <string>
#include <stdio.h>

using namespace std;

void sleepok(int, ros::NodeHandle &);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_client");
    ros::NodeHandle n;
    
	
  
    while (ros::ok()) {
        
        ros::spinOnce();
        
        
        break;
    }
    
    return 0;

}

void sleepok(int t, ros::NodeHandle &nh)
{
    if (nh.ok())
        sleep(t);
}
