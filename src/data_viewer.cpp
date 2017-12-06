#include "ros/ros.h"
#include <sstream>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/UInt16.h" // left_speed, right_speed
#include "std_msgs/UInt16MultiArray.h" // IR_raw, US_raw

class Listener
{
public:
	void setLeftSpeed(const std_msgs::UInt16 msg);
	void setRightSpeed(const std_msgs::UInt16 msg);
	void setIRdata(const std_msgs::UInt16MultiArray msg);
	void setUSdata(const std_msgs::UInt16MultiArray msg);
	void printData();
	
private:
	unsigned int leftSpeed;
	unsigned int rightSpeed;
	unsigned int IRdata[8];
	unsigned int USdata[8];
};

void Listener::setLeftSpeed(const std_msgs::UInt16 msg)
{
	leftSpeed = msg.data;
}

void Listener::setRightSpeed(const std_msgs::UInt16 msg)
{
	rightSpeed = msg.data;
}

void Listener::setIRdata(const std_msgs::UInt16MultiArray msg)
{
	for (int i = 0; i < 8; i++)
		IRdata[i] = msg.data[i];
}

void Listener::setUSdata(const std_msgs::UInt16MultiArray msg)
{
	for (int i = 0; i < 8; i++)
		USdata[i] = msg.data[i];
}

void Listener::printData()
{
	system("clear");
	std::cout << "Left Speed:  " << leftSpeed << std::endl;
	std::cout << "Right Speed: " << rightSpeed << std::endl << std::endl;
	
	for (int i = 0; i < 8; i++)
		std::cout << "IR" << i << ": " << IRdata[i] << std::endl;
	
	std::cout << std::endl;
	
	for (int i = 0; i < 8; i++)
		std::cout << "US" << i << ": " << USdata[i] << std::endl;
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "data_viewer");
	
	ros::NodeHandle n;
	ros::Rate loop_rate(20);
	
	Listener listener;
	
	ros::Subscriber leftSub = n.subscribe("left_speed", 1000, &Listener::setLeftSpeed, &listener);
	ros::Subscriber rightSub = n.subscribe("right_speed", 1000, &Listener::setRightSpeed, &listener);
	ros::Subscriber IRSub = n.subscribe("IR_raw", 1000, &Listener::setIRdata, &listener);
	ros::Subscriber USSub = n.subscribe("US_raw", 1000, &Listener::setUSdata, &listener);
	
	while (ros::ok())
	{
		ros::spinOnce();
		listener.printData();
		loop_rate.sleep();
	}
	
	return 0;
}