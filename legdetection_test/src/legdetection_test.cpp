#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <people_msgs/PositionMeasurement.h>
#include <std_msgs/Float64MultiArray.h>

#include <vector>
#include <iostream>
#include <string>
#include <stdio.h>

using namespace std;

double curr_loc[3];
vector<double> leg_locs;
vector<double> face_locs;

void sleepok(int, ros::NodeHandle &);
void get_turtle_bot_loc(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& sub_amcl);
void get_leg_locs(const people_msgs::PositionMeasurementArray::ConstPtr& input_legs);
void get_face_locs(const std_msgs::Float64MultiArray::ConstPtr& input_faces);
int find_people();
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
    ros::Subscriber face_meas = n.subscribe("rel_yaw_frac",1,get_face_locs);
    //sleep for a bit to make sure the sub will work
    sleepok(2,n);
    
    ros::Subscriber sub_amcl = n.subscribe("/amcl_pose",1,get_turtle_bot_loc);
    sleepok(2,n);
    double trajectory_legs[2];
    double trajectory_face[2];
      
    while (ros::ok()) {
        cin.get();
        ros::spinOnce();
    }
    
    return 0;

}

void sleepok(int t, ros::NodeHandle &nh)
{
    if (nh.ok())
        sleep(t);
}

void get_leg_locs(const people_msgs::PositionMeasurementArray::ConstPtr& input_legs)
{
    int ppl_meas = input_legs->people.size();
    leg_locs.resize(2*ppl_meas);
    int j = 0;
    for(int i = 0; i < ppl_meas; i++) {
        leg_locs[j] = input_legs->people[i].pos.x;
        leg_locs[j+1] = input_legs->people[i].pos.y;
        j += 2;
    }
}

void get_face_locs(const std_msgs::Float64MultiArray::ConstPtr& input_faces)
{
    int ppl_meas = input_faces->data.size();
    face_locs.resize(ppl_meas);
    for (int i = 0; i < ppl_meas; i++)
        face_locs[i] = input_faces->data[i];
}

int find_people()
{
    
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