#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <vector>
#include <iostream>
#include <stdio.h>

using namespace std;

double curr_loc[3];

void sleepok(int, ros::NodeHandle &);
void get_turtle_bot_loc(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& sub_amcl);
int move_turtle_bot (double, double, double);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_client");
    ros::NodeHandle n;
       
    ros::Subscriber sub_amcl = n.subscribe("/amcl_pose",100,get_turtle_bot_loc);
    // sleep to give subscriber time to open
    sleepok(2,n);
    
    // this will be reset based on starting location
    double home_location[3] = {21.8,13.9,0.0};
    
    // coffee shop is currently right outside kitchenette
    double coffee_shop[3] = {5.8,13.9,0.0};
  
    while (ros::ok()) {
        
        // RECORD STARTING LOCATION INTO HOME_LOCATION
        home_location[0] = curr_loc[0];
        home_location[1] = curr_loc[1];
        home_location[2] = curr_loc[2];
        cout << "Starting at: " << home_location[0] << ", " << home_location[1] << endl;
        // Maybe detect a person in a room and approach them, then record this location?
                
        cout << "Ask for name & order (press Enter)" << endl;
        cin.get();
        
        cout << "To the coffeeshop" << endl;        
        //move_turtle_bot(coffee_shop[0],coffee_shop[1],coffee_shop[2]);
        sleepok(2,n);
        
        cout << "Place order (press Enter)" << endl;
        cin.get();
                
        cout << "Deliver order" << endl;        
        //move_turtle_bot(home_location[0],home_location[1],home_location[2]);
        sleepok(2,n);
        
        cout << "Done." << endl;
        break;
        // Update home_location to next person detected?
    }
    
    return 0;

}

void sleepok(int t, ros::NodeHandle &nh)
{
    if (nh.ok())
        sleep(t);
}

void get_turtle_bot_loc(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& sub_amcl)
{
    curr_loc[0] = sub_amcl->pose.pose.position.x;
    curr_loc[1] = sub_amcl->pose.pose.position.x;
    curr_loc[2] = tf::getYaw(sub_amcl->pose.pose.orientation);
    cout << curr_loc[0];
    cout << curr_loc[1];
}

int move_turtle_bot (double x, double y, double yaw)
{

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    ac.waitForServer();
    move_base_msgs::MoveBaseGoal goal;
    
    cout << "Going to :" << x << ", " << y;

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
    
    //cout<<"\n \n waiting for " << sleep_time<<" seconds \n \n";
    //sleep(sleep_time);
  
    return 0;
}