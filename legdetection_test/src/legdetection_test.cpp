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
void get_person_locs(const people_msgs::PositionMeasurementArray::ConstPtr& ppl_locs);
int move_turtle_bot (double, double, double);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_client");
    ros::NodeHandle n;
    
    // subscriber for position
    ros::Subscriber ppl_meas = n.subscribe("/people_tracker_measurements",3,get_person_locs);
    //sleep for a bit to make sure the sub will work
    sleepok(2,n);
    double reliability = 0.95;
    n.setParam("leg_reliability_limit",reliability);
    
    // this will be reset based on starting location
    double home_location[3] = {21.8,13.9,0.0};
    
    // coffee shop is currently right outside kitchenette
    double coffee_shop[3] = {5.8,13.9,0.0};
  
    while (ros::ok()) {
        ros::spinOnce();
    
        n.getParam("leg_reliability_limit",reliability);
        cout << "Reliability threshold set at " << reliability << endl;
        cin.get();
        
        cout << endl;
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