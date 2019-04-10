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
void sayPhrase(ros::Publisher, int, string, string);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_client");
    ros::NodeHandle n;

    //publisher for sound
    ros::Publisher sound_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 1);
       
    //sleep for a bit to make sure the pub will work
    sleepok(2,n);
  
    while (ros::ok()) {
        
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

        sayPhrase(sound_pub,3,name,coffee);
        sleepok(2,n);
        cin.get();
        // SWITCH TO WHEN SENSOR ACTIVATED
        sleep(1);
        
        sayPhrase(sound_pub,4,name,coffee);
        sleepok(2,n);
        sleep(1);
        
        sayPhrase(sound_pub,5,name,coffee);
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
