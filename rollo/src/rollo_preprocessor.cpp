/*
 * Name: rollo_node.cpp
 * Created by: Ernest Damian Skrzypzyk, Rabbia Asghar
 * Modified on: 16/2/16
 * Details: Filter the raw data from optitrack motion capture system and 
 * publish it for modeling of odomety and the measurement in Kalman Filter
 * 
 * sample command: rosrun rollo rollo_node _rate:=1 _samplesize:=5 _sampling:='0'
 *  _rate is the sampling frquency of the node, default value is 1
 *  _samplesize is the no. of elements that are averaged/subsampled, default value is 10
 *  _sampling selects if the raw data should be subsampled after a certain delay or averaged over a certain period
 *  _sampling 0 represents subsampling and any number other than 0 represents averaging, default is set to subsampling
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/tf.h"
#include <sstream>
#include <iostream>

// Global variables
float x, y, theta;
float x_mm, y_mm, theta_deg;

// SubscriberCallback Function reads position and orientation from optitrack node 
void subscriberCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
	x = msg->x; //raw x is in metres
	x_mm = 1000 * x;
	y = msg->y; //raw y is in metres
	y_mm = 1000 * y;
	theta = msg->theta; //raw theta is in radians
	theta_deg = theta / 3.14159265359 * 180 + 180;
	// ROS_INFO("[Rollo][Sub][X, Y, Theta]: %f, %f, %f", msg->x, msg->y, msg->theta);
	// ROS_INFO("[Rollo][Sub][X, Y, Theta]: %f, %f, %f, %f", msg->x, msg->y, msg->theta, theta_deg);
	ROS_INFO("[Rollo][Sub][X, Y, Theta]: %f, %f, %f", x_mm, y_mm, theta_deg);
}


int main(int argc, char **argv)
{

// Initialization 
ros::init(argc, argv, "rollo_pubsub"); // name of the converter node
ros::start(); // Necessary to be called

// Nodehandle for subscriber and publisher
ros::NodeHandle RolloNode;

// Subscriber
ros::Subscriber PoseSub = RolloNode.subscribe("/Optitrack_Rollo/ground_pose", 1024, subscriberCallback);

// Publisher
ros::Publisher RolloPub = RolloNode.advertise<std_msgs::String>("/Rollo/pose", 1024);

// Publisher arguments using command line
int rate_frequency;
int samplesize;
int sampling; // sampling is either done using subsampling('0') or simple averaging('1') 

// Sample command: rosrun rollo rollo_node _rate:=1 _samplesize:=5 _sampling:='0'
// Initialize node parameters from launch file or command line.
// Use a private node handle so that multiple instances of the node can be run simultaneously
// while using different parameters.
ros::NodeHandle private_node_handle_("~");
private_node_handle_.param("rate", rate_frequency, int(1));
private_node_handle_.param("samplesize", samplesize, int(10));
private_node_handle_.param("sampling", sampling, int(0));

// publishing rate in units of Hz
ros::Rate frequency(rate_frequency); 

// publisher variables for Standard messages
std_msgs::String Message;
std::stringstream StringStream;

std_msgs::String PubRolloPosition; // Declaration of type of message 

// publisher variables to compute average
float sum_x_mm = 0;
float sum_y_mm = 0;
float sum_theta_deg = 0;

float average_x_mm = 0;
float average_y_mm = 0;
float average_theta_deg = 0;
int loopcounter = 0;


int loopcondition = 1;	//for while(1) loop

//loop
do {

 sum_x_mm += x_mm;
 sum_y_mm += y_mm;
 sum_theta_deg += theta_deg;


if (loopcounter >= samplesize )
{	
average_x_mm = sum_x_mm / samplesize;	
average_y_mm = sum_y_mm / samplesize;	
average_theta_deg = sum_theta_deg / samplesize;	
	
//StringStream << "[Rollo][X, Y, Theta]: " << x_mm << ", " << y_mm << ", " << theta_deg << "\n";
//StringStream << "[Rollo][X, Y, Theta]: " << sum_x_mm << ", " << sum_y_mm << ", " << sum_theta_deg << "\n";
//StringStream << "[Rollo]["<<samplesize<<"][X, Y, Theta]: " << average_x_mm << ", " << average_y_mm << ", " << average_theta_deg << "\n";

// prepare data for publishing
StringStream << "[Rollo][X, Y, Theta]: " << average_x_mm << ", " << average_y_mm << ", " << average_theta_deg << "\n";
PubRolloPosition.data = StringStream.str();

//publish
RolloPub.publish(PubRolloPosition);

//reset variables
loopcounter = 0;
sum_x_mm  = 0;
sum_y_mm  = 0;
sum_theta_deg = 0;

//if we are subsampling: sleep for time defined by rate and then read the states from the subscriber callback() without sleep() delay 
if (sampling == 0) frequency.sleep();
}


ros::spinOnce();

// ROS_INFO("[Rollo][Debug][Counter]: %d", loopcounter);

if (! ros::ok()) loopcondition = 0;

//if we are averaging: sleep for time defined by rate before reading states from the subscriber callback()
if (sampling != 0) frequency.sleep();
loopcounter++;
} while (loopcondition);
// end while loop


// ros::shutdown();
return 0;
}
