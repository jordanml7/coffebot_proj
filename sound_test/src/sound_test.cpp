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

using namespace std;

void sleepok(int, ros::NodeHandle &);
void sayPhrase(ros::Publisher, int, char [], char []);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_client");
    ros::NodeHandle n;

    //publisher for sound
    ros::Publisher sound_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 1);
       
    //sleep for a bit to make sure the pub will work
    sleepok(2,n);
  
    while (ros::ok()) {
        
        sayPhrase(sound_pub,0,NULL,NULL);
	sleepok(2,n);
        char name[100];
        cin.getline(name,100);
        
        sayPhrase(sound_pub,1,name,NULL);
        sleepok(2,n);
        char coffee[100];
        cin.getline(coffee,100);
        
        sayPhrase(sound_pub,2,name,coffee);
        sleepok(2,n);
        cin.get();

        sayPhrase(sound_pub,3,name,coffee);
        sleepok(2,n);
        cin.get();
        
        sayPhrase(sound_pub,4,name,coffee);
        sleepok(2,n);
        cin.get();
        
        sayPhrase(sound_pub,5,name,coffee);
        sleepok(2,n);
        cin.get();
    }
    
    return 0;

}

void sleepok(int t, ros::NodeHandle &nh)
{
    if (nh.ok())
        sleep(t);
}

void sayPhrase(ros::Publisher sound_pub, int m, char name[], char coffee[])
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

    string messages[6] = {startMsg,coffeeRqst,coffeeCnfm,coffeeOrdr,thankYou,coffeeRtrn};
    
    S.arg = messages[m];
    cout << messages[m] << endl;

    sound_pub.publish(S);
}
