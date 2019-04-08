#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"

#include <sound_play/sound_play.h>
#include <sound_play/SoundRequest.h>

#include <vector>
#include <iostream>
#include <string>
#include <stdio.h>

#define PI 3.14159265359

using namespace std;

void sleepok(int, ros::NodeHandle &);
int move_turtle_bot (double, double, double);
void sayPhrase(ros::Publisher, int, string, string);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_client");
    ros::NodeHandle n;

    //publisher for sound
    ros::Publisher sound_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 1);
       
    //sleep for a bit to make sure the pub will work
    sleepok(2,n);
    
    // this will be reset based on starting location
    double home_location[3] = {5.65,13.8,0.0};
    
    // coffee shop is currently right outside kitchenette
    double coffee_shop[3] = {6.09,-3.06,0.0};
  
    while (ros::ok()) {
        
        // RECORD STARTING LOCATION INTO HOME_LOCATION
        get_turtle_bot_loc(home_location);
        // Maybe detect a person in a room and approach them, then record this location?
        
        sayPhrase(sound_pub,0,"","");
	sleepok(2,n);
        string name;
        getline(cin,name);
        sleep(1);
        
        sayPhrase(sound_pub,1,name,"");
        sleepok(2,n);
        string coffee;
        getline(cin,coffee);
        sleep(1);
        
        sayPhrase(sound_pub,2,name,coffee);
        sleepok(2,n);
        sleep(1);
        move_turtle_bot(coffee_shop[0],coffee_shop[1],coffee_shop[2]);
        sleepok(2,n);
        sleep(1);

        sayPhrase(sound_pub,3,name,coffee);
        sleepok(2,n);
        cin.get();
        // SWITCH TO WHEN SENSOR ACTIVATED
        sleep(1);
        
        sayPhrase(sound_pub,4,name,coffee);
        sleepok(2,n);
        sleep(1);
        move_turtle_bot(home_location[0],home_location[1],home_location[2]);
        sleepok(2,n);
        sleep(1);
        
        sayPhrase(sound_pub,5,name,coffee);
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

void get_turtle_bot_loc(double home_location[3])
{
    tf::TransformListener listener;
    
    geometry_msgs::PoseStamped pBase, pMap;
    pBase.header.frame_id = "base_link";
    
    pBase.pose.position.x = 0.0;
    pBase.pose.position.y = 0.0;
    pBase.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    
    ros::Time current_transform = ros::Time::now();
    listener.getLatestCommonTime(pBase.header.frame_id, "map", current_transform, NULL);
    
    pBase.header.stamp = current_transform;
    listener.transformPose("map", pBase, pMap);
    
    home_location[0] = pMap[0];
    home_location[1] = pMap[1];
    home_location[2] = pMap[2];
}

int move_turtle_bot (double x, double y, double yaw)
{

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    ac.waitForServer();
    move_base_msgs::MoveBaseGoal goal;
    
    cout<<"Going to :"<< x  << y;

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

void sayPhrase(ros::Publisher sound_pub, int m, string name, string coffee)
{
    sound_play::SoundRequest S;
    S.sound = -3;
    S.command = 1;
        
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

    string messages[6] = {startMsg,coffeeRqst,coffeeCnfm,CoffeeOrdr,thankYou,coffeeRtrn}
    
    S.arg = messages[m];
    cout << messages[m] << endl;

    sound_pub.publish(S);
}
