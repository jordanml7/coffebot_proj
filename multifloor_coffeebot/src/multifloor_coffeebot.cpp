#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <vector>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <open_door_detector/detect_open_door.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sound_play/sound_play.h>
#include <sound_play/SoundRequest.h>
#include <std_msgs/Bool.h>

using namespace std;

double curr_loc[4];
bool press_sense;

void sleepok(int, ros::NodeHandle &);
void get_turtle_bot_loc(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& sub_amcl);
int move_turtle_bot (double, double, double);
void detect_coffee(const std_msgs::Bool& msg);
void sayPhrase(int, char[], char[]);
bool ele_open(ros::NodeHandle);
int move_forward();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_client");
    ros::NodeHandle n;
    
    // subscriber for position
    ros::Subscriber sub_amcl = n.subscribe("/amcl_pose",1,get_turtle_bot_loc);
    //sleep for a bit to make sure the sub will work
    sleepok(2,n);
    
    // subscriber for arduino sensor msgs
    ros::Subscriber coffee_detect = n.subscribe("/coffee_topic",1,detect_coffee);
    //sleep for a bit to make sure the sub will work
    sleepok(2,n);
    
    // publisher for sound
    ros::Publisher sound_pub = n.advertise<sound_play::SoundRequest>("/robotsound",1);
    //sleep for a bit to make sure the pub will work
    sleepok(2,n);
    
    // this will be reset based on starting location
    double home_location[4] = {21.8,13.9,1.0, 0.0};
    
    // coffee shop is currently at the top of the stairs
    double coffee_shop[4] = {-0.8662,1.670,2.0, 0.0};
    
    double elevator1[4] = {0.0, 0.0, 1.0, 0.0}; //x, y, z, yaw
    double elevator2[4] = {-2.013, 11.701, 2.0, 0.0};
    double goal[4] = {0.0, 0.0, 1.0, 0.0};
  
    while (ros::ok()) {
        
        ros::spinOnce();
        home_location[0] = curr_loc[0];
        home_location[1] = curr_loc[1];
        home_location[2] = 1.0;
        home_location[3] = curr_loc[3];
        cout << "Starting at: " << home_location[0] << ", " << home_location[1] << endl;

        // Greet and ask for user's name
        sayPhrase(0,NULL,NULL);
		sleepok(2,n);
        char name[100];
        cin.getline(name,100);
        sleepok(2,n);
        
        while(true) {
			sayPhrase(1,name,NULL);
			sleepok(2,n);
			char response[10];
			cin.getline(response,10);
			if(strcmp(response,"No") == 0 | strcmp(response,"no") == 0) {
				sayPhrase(2,name,NULL);
				sleepok(2,n);
				return -1;
			}
			else if (strcmp(response,"Yes") == 0 | strcmp(response,"yes") == 0) {
				sleepok(2,n);
				break;
			}
			sleepok(5,n);
		}
        
        // Ask for user's coffee order
        sayPhrase(3,NULL,NULL);
        sleepok(2,n);
        char coffee[100];
        cin.getline(coffee,100);
        sleepok(2,n);
        
        // Order recieved
        sayPhrase(4,NULL,coffee);
        sleepok(8,n);
        
        // Headed to the coffeeshop!
        cout << "Traveling to: " << coffee_shop[0] << ", " << coffee_shop[1] << endl;   
        
        //Move to 1st floor elevator 
        move_turtle_bot(elevator1[0],elevator1[1],elevator1[3]);
        sleepok(2,n);
        
        //Wait for door to be open
        while(!ele_open(n)) {
            sayPhrase(9, NULL, NULL);
            sleepok(8,n);
        }
        //Enter elevator
        move_forward();
        //Turn around
        
        system("rosrun map_server map_server elevator_maps/2ele");
        
        //Wait for door to be open
        while(!ele_open(n)) {
            sleepok(8,n);
        }

        //Exit elevator
        move_forward();
        
        //Finish navigating to coffee_shop
        move_turtle_bot(coffee_shop[0],coffee_shop[1],coffee_shop[3]);

		// Place coffee order
        sayPhrase(5,NULL,coffee);
        sleepok(4,n);
        
        // Wait for sensor to read that coffee has been given
        int waiting = 0;
        while(!press_sense) {
			ros::spinOnce();
			sleep(1);
			waiting += 1;
			if(waiting % 10 == 0) {
				sayPhrase(6,NULL,NULL);
				sleepok(4,n);
			}
		}
        
        // Thank server for coffee
        sayPhrase(7,NULL,NULL);
        sleepok(2,n);
        
        // Headed back to the user!
        cout << "Returning to: " << home_location[0] << ", " << home_location[1] << endl;
        
        //Move to 1st floor elevator 
        move_turtle_bot(elevator2[0],elevator2[1],elevator2[3]);
        sleepok(2,n);
        
        //Wait for door to be open
        while(!ele_open(n)) {
            sayPhrase(9, NULL, NULL);
            sleepok(8,n);
        }
        //Enter elevator
        move_forward();
        //Turn around
        
	system("rosrun map_server map_server elevator_maps/1ele");
	    
        //Wait for door to be open
        while(!ele_open(n)) {
            sleepok(8,n);
        }

        //Exit elevator
        move_forward();
        
        //Finish navigating home
        move_turtle_bot(home_location[0],home_location[1],home_location[3]);
        sleepok(2,n);
        
        // Deliver coffee
        sayPhrase(8,name,coffee);
        sleepok(2,n);
        
        return -1;
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
    curr_loc[3] = tf::getYaw(sub_amcl->pose.pose.orientation);
}

void detect_coffee(const std_msgs::Bool& msg)
{
	press_sense = msg.data;
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

void sayPhrase(int m, char name[], char coffee[])
{
    sound_play::SoundClient S;

    string startMsg = "Hello. I'm Coffee Bot. What's your name?";
    char coffeeRqst[100];
    sprintf(coffeeRqst,"Hi there, %s! Would you like a coffee?",name);
    string negRspn = "Okay then, have a nice day!";
    string takeOrder = "Great! What coffee can I get for you?";
    char coffeeCnfm[100];
    sprintf(coffeeCnfm,"I'll be back with your %s in just a few moments. Wait here.",coffee);
	char placeOrdr[100];
    sprintf(placeOrdr,"Hi! Could I please get a %s? Please place it in my cupholder when it's ready.",coffee);
    string waitingMsg = "Is everything alright? I've been waiting a while.";
    string thankYou = "Thank You!";
    char coffeeRtrn[100];
    sprintf(coffeeRtrn,"Hi %s, here's your %s. Enjoy!",name,coffee);
    string openDoor = "Please open door"; 

    string messages[10] = {startMsg,coffeeRqst,negRspn, \
		takeOrder,coffeeCnfm,placeOrdr,waitingMsg,thankYou,coffeeRtrn};
    
    S.say(messages[m]);
    cout << messages[m] << endl;
    if(m == 0) {
		cout << "*******************************" << endl;
		cout << "*                             *" << endl;
		cout << "*   Please Enter Your Name:   *" << endl;
		cout << "*                             *" << endl;
		cout << "*******************************" << endl;
	}
	if(m == 1) {
		cout << "*****************************************" << endl;
		cout << "*                                       *" << endl;
		cout << "*   Would you like a coffee? (Yes/No)   *" << endl;
		cout << "*                                       *" << endl;
		cout << "*****************************************" << endl;
	}
	if(m == 3) {
		cout << "***************************************" << endl;
		cout << "*                                     *" << endl;
		cout << "*   Please Enter Your Coffee Order:   *" << endl;
		cout << "*                                     *" << endl;
		cout << "***************************************" << endl;
	}
}
int move_forward()
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    ac.waitForServer();
    move_base_msgs::MoveBaseGoal goal;
    
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "/base_footprint"; //Change this
    
    goal.target_pose.pose.position.x = -1.0;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    ac.sendGoal(goal);
    ac.waitForResult();

    return 0;
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
