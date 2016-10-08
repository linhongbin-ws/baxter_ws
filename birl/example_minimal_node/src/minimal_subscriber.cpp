#include<ros/ros.h>
#include<std_msgs/Float64.h>

void myCallback(const std_msgs::Float64& message_holder)
{
  // the real work is done in this callback function
  // it wakes up every time a new message is published on "topic1"
  // Since this function is prompted by a message event, it does not consume CPU time polling for new data
  // the ROS_INFO() function is like a printf() function, except
  // it publishes its output to the default rosout topic, which prevents
  // slowing down this function for display calls, and it makes the
  // data available for viewing and logging purposes
  ROS_INFO("received value is: %f",message_holder.data);
  // really could do something interesting here with the received data... but all we do is print it.
}

int main(int argc, char**argv)
{
  ros::init(argc,argv,"minimal_subscriber"); 	// name of this node
  ros::NodeHandle n; 				// need to establish communications with our new node. 

  // Create a subsriber object on topic "topic1". 
  // "myCallback" will wake up whenever a new message is published to the topic1. 
  ros::Subscriber my_subscriber_object = n.subscribe("topic1",1,myCallback);
  ros::spin(); 					// this statment is necessary to avoid moving to the return statement in the next line. Instead it allows the code to continue to spin or go around in a loop UNTIL a Ctrl+c statement is issued. 

  return 0; // only called when roscore dies.
}
