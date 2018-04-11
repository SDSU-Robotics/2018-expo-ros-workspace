#include "ros/ros.h"
#include <pigpiod_if2.h>
#include <sstream>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32.h" // left_speed, right_speed
#include "std_msgs/Int16MultiArray.h" // encoder_count


/* Motor labeling convention:
	Front
	0	1

	2	3
	
	4	5
*/
	
const int MOTOR_QUANTITY = 6;
const int MOTOR_PINS[MOTOR_QUANTITY] = { 22, 23, 24, 25, 26, 27 };

class Motor
{
public:
	void init(int pi, int pin);
	void setSpeed(int dutyCycle);
	
private:
	int _pi;
	int _pin;
};

void Motor::init(int pi, int pin)
{
	_pi = pi;
	_pin = pin;
}

void Motor::setSpeed(int dutyCycle)
{
	set_PWM_dutycycle(_pi, _pin, dutyCycle);
}

class Listener
{
public:
	Listener(int pi);
	
	void setLspeed(const std_msgs::Float32 msg);
	void setRspeed(const std_msgs::Float32 msg);

private:	
	Motor motors[MOTOR_QUANTITY];
};

Listener::Listener(int pi)
{
	// initialize motors
	for (int i = 0; i < MOTOR_QUANTITY; ++i)
	{
		motors[i].init(pi, MOTOR_PINS[i]);
	}
}

void Listener::setLspeed(const std_msgs::Float32 msg)
{
	motors[0].setSpeed(msg.data);
	motors[2].setSpeed(msg.data);
	motors[4].setSpeed(msg.data);
}

void Listener::setRspeed(const std_msgs::Float32 msg)
{
	motors[1].setSpeed(msg.data);
	motors[3].setSpeed(msg.data);
	motors[5].setSpeed(msg.data);
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "motor_control");
	
	ros::NodeHandle n;
	
	// initialize pigpio
	int pi = pigpio_start(NULL, NULL);	
	if (pi < 0)
		ROS_INFO("Error: %s\n", pigpio_error(pi));
	
	Listener listener(pi);
	
	ros::Subscriber leftSub = n.subscribe("left_speed", 1000, &Listener::setLspeed, &listener);
	ros::Subscriber rightSub = n.subscribe("right_speed", 1000, &Listener::setRspeed, &listener);
	
	ros::spin();
	
	return 0;
}