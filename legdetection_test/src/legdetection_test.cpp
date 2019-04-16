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

double curr_loc[3];

void sleepok(int, ros::NodeHandle &);
void get_turtle_bot_loc(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& sub_amcl);
void get_person_locs(const people_msgs::PositionMeasurementArray::ConstPtr& ppl_locs);
int move_turtle_bot (double, double, double);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_client");
    ros::NodeHandle n;
    
    // subscriber for position
    ros::Subscriber ppl_meas = n.subscribe("/people_tracker_measurements",1,get_person_locs);
    //sleep for a bit to make sure the sub will work
    sleepok(2,n);
    
    ros::Subscriber sub_amcl = n.subscribe("/amcl_pose",1,get_turtle_bot_loc);
    sleepok(2,n);
      
    while (ros::ok()) {
        ros::spinOnce();
    
        cout << "Currently at x: " << curr_loc[0] << ", y: " << curr_loc[1] << endl;
        
        cin.get();
    }
    
    return 0;

}

void sleepok(int t, ros::NodeHandle &nh)
{
    if (nh.ok())
        sleep(t);
}

void get_person_locs(const people_msgs::PositionMeasurementArray::ConstPtr& ppl_locs)
{
    int ppl_meas = ppl_locs->people.size();
    cout << "Counted " << ppl_meas << " people" << endl;
    for(int i = 0; i < ppl_meas; i++) {
        cout << "Person " << i << " recorded at x: " 
            << ppl_locs->people[i].pos.x << ", y: " << ppl_locs->people[i].pos.y 
            << " with reliability: " << ppl_locs->people[i].reliability << endl;
    }
}

void get_turtle_bot_loc(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& sub_amcl)
{
    curr_loc[0] = sub_amcl->pose.pose.position.x;
    curr_loc[1] = sub_amcl->pose.pose.position.y;
    curr_loc[2] = tf::getYaw(sub_amcl->pose.pose.orientation);
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