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

double curr_loc[3];

void sleepok(int, ros::NodeHandle &);
void get_turtle_bot_loc(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& sub_amcl);
int move_turtle_bot (double, double, double);
void sayPhrase(int);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_client");
    ros::NodeHandle n;
    
    // subscriber for position
    ros::Subscriber sub_amcl = n.subscribe("/amcl_pose",100,get_turtle_bot_loc);
    //sleep for a bit to make sure the sub will work
    sleepok(2,n);
    
    // publisher for sound
    ros::Publisher sound_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 1);
    //sleep for a bit to make sure the pub will work
    sleepok(2,n);
    
    // this will be reset based on starting location
    double home_location[3] = {21.8,13.9,0.0};
    
    // coffee shop is currently at the top of the stairs
    double coffee_shop[3] = {-0.8737,1.3296,-1.746};
  
    while (ros::ok()) {
        
        sayPhrase(4);
        cin.get();
        cout << "Returning to: " << home_location[0] << ", " << home_location[1] << endl;
        //move_turtle_bot(home_location[0],home_location[1],home_location[2]);
        cin.get();
        
        sayPhrase(5);
        sleepok(2,n);
        
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

void sayPhrase(int m)
{
    sound_play::SoundClient S;

    string startMsg = "Hello. I'm Coffee Bot. What's your name?";
    char coffeeRqst[100];
    sprintf(coffeeRqst,"Hi there, %s! What coffee can I get for you?","jordan"
);
    char coffeeCnfm[100];
    sprintf(coffeeCnfm,"Great, I'll be back with your %s in just a few moments. Wait here.","coffee");
    char coffeeOrdr[100];
    sprintf(coffeeOrdr,"Hi! Could I please get a %s?","coffee");
    string thankYou = "Thank You!";
    char coffeeRtrn[100];
    sprintf(coffeeRtrn,"Hi %s, here's your %s. Enjoy!","Jordan","coffee");

    string messages[6] = {startMsg,coffeeRqst,coffeeCnfm,coffeeOrdr,thankYou,coffeeRtrn};
    
    S.say(messages[m]);
    cout << messages[m] << endl;
}
