#include "ros/ros.h"
#include "geometry_msgs/Twist.h" //topic cmd_vel
#include "sensor_msgs/LaserScan.h" //topic base_scan
#include "std_srvs/Empty.h"
#include "second_assignment/UI.h" //header of the user interface service

ros::ServiceClient UI_client;	//client side of the UI service


//Function to print the "rules" for the user (which commands are feasible) and
//acquire the input give by him 
char GetUserInput()
{
	char command;

	std::cout<<"Here are the commands to control the robot:\n";
	std::cout<<"To increase the robot's speed press the W button\n";
	std::cout<<"To decrease the robot's speed press the S button\n";
	std::cout<<"Too reset the robot's position press the R button\n";
	std::cout<<"Wait for your command (press enter to confirm):  ";
	std::cin>>command;

	return command;
}


//Callback function to send the command chosen by the user (change of velocity/reset of the position) to the server
void interactionCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    second_assignment::UI ui_srv;
    char user_command = GetUserInput();

    ui_srv.request.command = user_command;

	UI_client.waitForExistence();
	UI_client.call(ui_srv);
}

int main(int argc, char **argv)
{
	//initialization of the node and setup of the NodeHandle 
	//for handling the communication with the ROS system 
	ros::init(argc, argv, "UI");
	ros::NodeHandle nh;

	//definition of the subscriber to the robot's position
    ros::Subscriber UI_subscriber = nh.subscribe("/base_scan", 1, interactionCallback);

	//definition of the client side of the UI service
	UI_client = nh.serviceClient<second_assignment::UI>("/userinterface");

	ros::spin();

	return 0;
}
