#include "ros/ros.h" 
#include "geometry_msgs/Twist.h" 	//topic cmd_vel
#include "sensor_msgs/LaserScan.h" 	//topic base_scan
#include <iostream>
#include "second_assignment/UI.h"	//header of the service created for the user interface 
#include "std_srvs/Empty.h"

ros::Publisher pub_velocity;		//publisher used for the velocities
double d_th = 0.7; 					//distance treshold
float acceleration = 0.0;			//velue of the acceleration of the robot initialized to 0

std_srvs::Empty reset_position;  	//service to reset the position of the robot

int check_front(double distance_front_wall[]);
int check_right(double distance_right_wall[]);
int check_left(double distance_left_wall[]);


//Function to check if there is a a wall in front of the robot,
//the only argument is the array of distances of the nearest obstacles
//from 0° to 180° in front of the robot 
int check_front(double distance_front_wall[])
{
	int i;

	//the control is executed between 97.5° (390° value) and 82.5° (330° value),
	//that is +/- 7.5° in front of the robot

	for(i=330; i>=330 && i<=390; i++) 
	{
		if(distance_front_wall[i] < d_th)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
}

//Function to check if there is a a wall on the right wing of the robot,
//the only argument is the array of distances of the nearest obstacles
//from 0° to 180° in front of the robot 
int check_right(double distance_right_wall[]) 
{
	int i;

	//the control is executed between 45° (180° value) and 15° (60° value),
	//that is a span of 30 degrees on the right side of the robot (slightly in the front to 
	//prevent a collision against the wall)

	for(i=60; i>=60 && i<=180; i++)
	{
		if(distance_right_wall[i] < d_th)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
}

//Function to check if there is a a wall on the left wing of the robot,
//the only argument is the array of distances of the nearest obstacles
//from 0° to 180° in front of the robot 
int check_left(double distance_left_wall[])
{
	int i;

	//the control is executed between 165° (660° value) and 135° (540° value),
	//that is a span of 30 degrees on the left side of the robot (slightly in the front to 
	//prevent a collision against the wall)

	for(i=540; i>=540 && i<=660; i++)
	{
		if(distance_left_wall[i] < d_th)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
}

//Callback function to manage the input given by the user
bool uiCallback(second_assignment::UI::Request &req, second_assignment::UI::Response &res)
{
	if(req.command == 'w')		//command to increment the acceleration of the robot
	{
		acceleration = acceleration + 0.5;
	}
	else if(req.command == 's')	//command to decrement the acceleration of the robot
	{
		acceleration = acceleration - 0.5;
	}
	else if(req.command == 'r')	//command to reset the position of the robot
	{
		ros::service::call("/reset_positions", reset_position);
	}
	else if(req.command != 'w' && req.command != 's' && req.command != 'r')	 //if none of the above
	{
		std::cout<<"The command is invalid \n"<<std::endl;
		fflush(stdout);
	}

	res.quantity = acceleration;

	return true;
}


//Callback function to acquire the distances from obstacles and move accordingly to that
void controllerCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	geometry_msgs::Twist robot_vel;
	
	double walls_dist[720];
	int count;
	
	//for cycle to acquire distances
	for (count=0; count<720; count++)
	{
		walls_dist[count] = msg->ranges[count];
	}

	//if construct to drive the robot
	if(check_front(walls_dist) == 0)
	{
		if(check_right(walls_dist) == 0)
		{
			robot_vel.angular.z = 0.95;
			robot_vel.linear.x = 0.25;
		}
		else if(check_left(walls_dist) == 0)
		{
			robot_vel.angular.z = -0.95;
			robot_vel.linear.x = 0.25;
		}
		else
		{
			robot_vel.angular.z = 0.95;
			robot_vel.linear.x = 0.20;
		}

	}
	else if(check_right(walls_dist) == 0)
	{
		robot_vel.angular.z = 0.5;
		robot_vel.linear.x = 0.25;
	}
	else if(check_left(walls_dist) == 0)
	{
		robot_vel.angular.z = -0.5;
		robot_vel.linear.x = 0.25;
	}
	else
	{
		robot_vel.linear.x = 1.5 + acceleration;
		robot_vel.angular.z = 0.0;
		
		if(robot_vel.linear.x <= 0)
		{
			robot_vel.linear.x = 0;
		}
	}

	pub_velocity.publish(robot_vel);	//publishment of the velocity on the cmd_vel topic
}

int main(int argc , char **argv)
{
	//initialization of the node and setup of the NodeHandle 
	//for handling the communication with the ROS system 
	ros::init(argc, argv, "robot_control");
	ros::NodeHandle nh;

	//definition of the server side of the UI service
	ros::ServiceServer ui_service =  nh.advertiseService("/userinterface", uiCallback);
	
	//association of the publisher to the cmd_vel topic
	pub_velocity = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	//definition of the subscriber to the robot's position
	ros::Subscriber sub = nh.subscribe ("/base_scan", 1, controllerCallback);
	
	ros::spin();

	return 0;
}
