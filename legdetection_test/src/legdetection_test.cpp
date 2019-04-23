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
#include "people_msgs/PositionMeasurement.h"

#include <vector>
#include <iostream>
#include <string>
#include <stdio.h>

using namespace std;

double curr_loc[3];
double leg_loc[2];
double face_loc[2];

void sleepok(int, ros::NodeHandle &);
void get_turtle_bot_loc(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& sub_amcl);
void get_leg_locs(const people_msgs::PositionMeasurementArray::ConstPtr& leg_locs);
void get_face_locs(const people_msgs::PositionMeasurement::ConstPtr& face_locs);
int move_turtle_bot (double, double, double);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_client");
    ros::NodeHandle n;
    
    // subscriber for leg position
    ros::Subscriber leg_meas = n.subscribe("/people_tracker_measurements",1,get_leg_locs);
    //sleep for a bit to make sure the sub will work
    sleepok(2,n);
    
    // subscriber for face position
    ros::Subscriber face_meas = n.subscribe("/people_tracker_measurements",1,get_face_locs);
    //sleep for a bit to make sure the sub will work
    sleepok(2,n);
    
    ros::Subscriber sub_amcl = n.subscribe("/amcl_pose",1,get_turtle_bot_loc);
    sleepok(2,n);
    double trajectory_legs[2];
    double trajectory_face[2];
      
    while (ros::ok()) {
        cin.get();
        ros::spinOnce();
        cout << "Currently at x: " << curr_loc[0] << ", y: " << curr_loc[1] << endl;
        trajectory_legs[0] = curr_loc[0] + leg_loc[0];
        trajectory_legs[1] = curr_loc[1] + leg_loc[1];
        cout << "Legs detected at x: " << trajectory_legs[0] << ", y: " << trajectory_legs[1] << endl;
        trajectory_face[0] = curr_loc[0] + face_loc[0];
        trajectory_face[1] = curr_loc[1] + face_loc[1];
        cout << "Face detected at x: " << trajectory_face[0] << ", y: " << trajectory_face[1] << endl;
    }
    
    return 0;

}

void sleepok(int t, ros::NodeHandle &nh)
{
    if (nh.ok())
        sleep(t);
}

void get_leg_locs(const people_msgs::PositionMeasurementArray::ConstPtr& leg_locs)
{
    int ppl_meas = leg_locs->people.size();
    if (ppl_meas > 0) {
        int max_i = 0;
        double reliability = leg_locs->people[0].reliability;
        for(int i = 1; i < ppl_meas; i++) {
            if (leg_locs->people[i].reliability > reliability) {
                    reliability = leg_locs->people[i].reliability;
                    max_i = i;
            }
        }
        
        leg_loc[0] = leg_locs->people[max_i].pos.x;
        leg_loc[1] = leg_locs->people[max_i].pos.y;
    }
    else {
        cout << "Nobody detected..." << endl;
        leg_loc[0] = 0.0;
        leg_loc[1] = 0.0;
    }
}

void get_face_locs(const people_msgs::PositionMeasurement::ConstPtr& face_locs)
{
    face_loc[0] = face_locs->pos.x;
    face_loc[1] = face_locs->pos.y;
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