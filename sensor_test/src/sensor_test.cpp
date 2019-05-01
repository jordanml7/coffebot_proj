#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <sound_play/sound_play.h>
#include <sound_play/SoundRequest.h>

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
