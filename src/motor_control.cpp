#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <pigpiod_if2.h>
#include <sstream>

const int LEFT_MOTOR_PIN = 23;
const int RIGHT_MOTOR_PIN = 24;

class Listener
{
public:
	int pi;
	
	void setLeftSpeed(const std_msgs::Int16 msg);
	void setRightSpeed(const std_msgs::Int16 msg);
};

void Listener::setLeftSpeed(const std_msgs::Int16 msg)
{
	set_PWM_dutycycle(pi, LEFT_MOTOR_PIN, msg.data);
}

void Listener::setRightSpeed(const std_msgs::Int16 msg)
{
	set_PWM_dutycycle(pi, RIGHT_MOTOR_PIN, msg.data);
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "motor_control");
	
	ros::NodeHandle n;
	
	Listener listener;
	
	ros::Subscriber leftSub = n.subscribe("left_speed", 1000, &Listener::setLeftSpeed, &listener);
	ros::Subscriber rightSub = n.subscribe("right_speed", 1000, &Listener::setRightSpeed, &listener);
	
	// initialize pigpio
	int pi = pigpio_start(NULL, NULL);	
	if (pi < 0)
		ROS_INFO("Error: %s\n", pigpio_error(pi));

	listener.pi = pi;
	
	ros::spin();
	
	return 0;
}