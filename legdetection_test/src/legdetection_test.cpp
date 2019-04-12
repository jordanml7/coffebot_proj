#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "people_msgs/PositionMeasurementArray.h"

#include <vector>
#include <iostream>
#include <string>
#include <stdio.h>

using namespace std;

void sleepok(int, ros::NodeHandle &);
int move_turtle_bot (double, double, double);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_client");
    ros::NodeHandle n;
    
    // this will be reset based on starting location
    double home_location[3] = {21.8,13.9,0.0};
    
    // coffee shop is currently right outside kitchenette
    double coffee_shop[3] = {5.8,13.9,0.0};
  
    while (ros::ok()) {
                
        cout << "Traveling to: " << coffee_shop[0] << ", " << coffee_shop[1] << endl;   
        move_turtle_bot(coffee_shop[0],coffee_shop[1],coffee_shop[2]);
        sleepok(2,n);
        
        break;
    }
    
    return 0;

}

void sleepok(int t, ros::NodeHandle &nh)
{
    if (nh.ok())
        sleep(t);
}

int move_turtle_bot (double x, double y, double yaw)
{

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    ac.waitForServer();
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "/map";
    
    //set relative x, y, and angle
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    //send the goal
    ac.sendGoal(goal);
    
    //block until the action is completed
    ac.waitForResult();
    
    return 0;
}