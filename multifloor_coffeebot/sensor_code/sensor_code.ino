/*
* Sensor code for coffee detection
* CoffeeBot
* Jordan Lueck
*/
#include <ros.h>
#include <std_msgs/Bool.h>

int fsrAnalogPin = 0; // FSR is connected to analog 0
int fsrReading;      // the analog reading from the FSR resistor divider

ros::NodeHandle nh;
std_msgs::Bool coffee;
ros::Publisher coffee_pub("coffee_topic",&coffee);

void setup() {
  nh.initNode();
  nh.advertise(coffee_pub);
}
 
void loop() {
  fsrReading = analogRead(fsrAnalogPin);
  if (fsrReading > 100)
    coffee.data = true;
  else
    coffee.data = false;
  
  coffee_pub.publish(&coffee);  
  nh.spinOnce();
  delay(100);
}
