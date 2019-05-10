#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <open_door_detector/detect_open_door.h>
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

double curr_loc[4];

bool ele_open(ros::NodeHandle);
int move_forward();
void sleepok(int, ros::NodeHandle &);
void get_turtle_bot_loc(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& sub_amcl);
int move_turtle_bot (double, double, double);
void say_phrase(int, char [], char []);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_client");
    ros::init(argc, argv, "open_door_detector_client");

    ros::NodeHandle n;
    //sleep for a bit to make sure the sub will work
    sleepok(2,n);
    // subscriber for position
    ros::Subscriber sub_amcl = n.subscribe("/amcl_pose",100,get_turtle_bot_loc);

    double elevator1[4] = {0.0, 0.0, 1.0, 0.0}; //x, y, z, yaw
    double elevator2[4] = {-2.013, 11.701, 2.0, 0.0};
    double goal[4] = {0.0, 0.0, 1.0, 0.0};
    
    double home_location[4] = {21.8, 13.9, 2.0, 0.0};
    
    while (ros::ok()) {
        //Find self location
/*
        ros::spinOnce();
        home_location[0] = curr_loc[0];
        home_location[1] = curr_loc[1];
        home_location[2] = curr_loc[2];
        home_location[3] = curr_loc[3];
        cout << "Starting at: " << home_location[0] << ", " << home_location[1] << endl;
 */      
        //move_turtle_bot(elevator2)  //move in front of ele
        while(!ele_open(n)) {
            //Vocalize
            sleepok(5,n);
        }
        move_forward();
/*        
switch_map(path, n)
        while(!ele_open) {
            //Vocalize
            sleepok(5,n)
        }
        move_forward();
        move_turtle_bot(goal);
*/
    }
}

bool ele_open(ros::NodeHandle n)
{
    //Start open door detecting service
    ros::ServiceClient doorClient = n.serviceClient<open_door_detector::detect_open_door>("detect_open_door"); 
    open_door_detector::detect_open_door doorSrv;
    doorSrv.request.aperture_angle = 0;
    doorSrv.request.wall_distance = 2;
    doorSrv.request.min_door_width = 0.2;
    
    if(doorClient.call(doorSrv))
    {
        if(doorSrv.response.door_pos.pose.position.x == 0){
            return false;
        }
        else {
            return true; 
        }
    }
    else
    {
        ROS_ERROR("Failed to call service detect_open_door");
    }
}

void sleepok(int t, ros::NodeHandle &nh)
{
    if (nh.ok())
        sleep(t);
}

void get_turtle_bot_loc(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& sub_amcl)
{
    curr_loc[0] = sub_amcl->pose.pose.position.x;
    curr_loc[1] = sub_amcl->pose.pose.position.y;
    curr_loc[3] = tf::getYaw(sub_amcl->pose.pose.orientation);
}

int move_turtle_bot (double pos[4])
{

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    ac.waitForServer();
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "/map";
    
    //set relative x, y, and angle
    goal.target_pose.pose.position.x = pos[0];
    goal.target_pose.pose.position.y = pos[1];
    goal.target_pose.pose.position.z = pos[2];
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pos[3]);

    //send the goal
    ac.sendGoal(goal);
    
    //block until the action is completed
    ac.waitForResult();
    
    return 0;
}

int move_forward()
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    ac.waitForServer();
    move_base_msgs::MoveBaseGoal goal;
    
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "/base_footprint"; //Change this
    
    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.orientation.w = tf::createQuaternionMsgFromYaw(3.14);

    ac.sendGoal(goal);
    ac.waitForResult();

    return 0;
}
/*
void switch_map(string map_path, ros::NodeHandle n)
{
    ros::Publisher mapPub = n.advertise<>("map_server", 1);
    
    std_msgs::String map;
    std::stringstream ss;
    ss << map_path;
    map.data = ss.str();
    mapPub.publish(map);
    ros::spinOnce();
}
*/
void say_phrase(int m, char name[], char coffee[])
{
    sound_play::SoundClient S;

    string startMsg = "Hello, what's your name?";
    char coffeeRqst[100];
    sprintf(coffeeRqst,"Hi there, %s! What coffee can I get for you?",name);
    char coffeeCnfm[100];
    sprintf(coffeeCnfm,"Great, I'll be back with your %s in just a few moments. Wait here.",coffee);
    char coffeeOrdr[100];
    sprintf(coffeeOrdr,"Hi! Could I please get a %s? Please press enter to tell me it's done.",coffee);
    string thankYou = "Thank You!";
    char coffeeRtrn[100];
    sprintf(coffeeRtrn,"Hi %s, here's your %s. Enjoy!",name,coffee);

    string messages[6] = {startMsg,coffeeRqst,coffeeCnfm,coffeeOrdr,thankYou,coffeeRtrn};
    
    S.say(messages[m]);
    cout << messages[m] << endl;
}
