#include "ros/ros.h"
#include <pigpiod_if2.h>
#include <sstream>
#include <pid.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32.h" // left_speed, right_speed
#include "std_msgs/Int16MultiArray.h" // encoder_count


/* Motor labelling convention:
	Front
	0	1

	2	3
*/
	
const int MOTOR_QUANTITY = 4;
const int MOTOR_PINS[MOTOR_QUANTITY] = { 23, 24, 25, 26 };

const int COUNTS_PER_REV = 1440; // encoder counts per revolution

// PID constants
const double DT = 0.1; // loop interval time
const double MAX = 100; // max adjustment to make
const double MIN = -100; // min adjustment to make
const double KP = 0.1; // proportional gain
const double KD = 0.01; // derivative gain
const double KI = 0.5; // integral gain

class Motor
{
public:
	Motor(int pi, int pin);
	void update(double encoderCount);
	void setSetpoint(double setPoint);
	
private:
	PID _pid;
	
	int _pi;
	int _pin;
	int _setPoint; // desired rev/s
	int _lastCount;
}

Motor::Motor(int pi, int pin)
{
	_pi = pi;
	
	_pid = new PID(DT, MAX, MIN, KP, KD, KI);
	
	_pin = pin;
	_setPoint = 0;
	_lastCount = 0;
}

Motor::update(double encoderCount);
{
	int dif = encoderCount - _lastCount;
	int pv = dif / COUNTS_PER_REV / DT; // current rev/s
	
	_lastCount = encoderCount;
	
	set_PWM_dutycycle(pi, _pin, _pid.calculate(_setPoint, pv));
}

class Listener
{
public:
	Listener(int pi);
	
	void setLspeed(const std_msgs::Float32 msg);
	void setRspeed(const std_msgs::Float32 msg);
	void updateEncoders(const std_msgs::Int16MultiArray msg);

private:	
	Motor motors[MOTOR_QUANTITY];
};

Lister::Listener(int pi)
{
	// initialize motors
	for (int i = 0; i < MOTOR_QUANTITY; ++i)
	{
		motors[i] = new Motor(pi, MOTOR_PINS[i]);
	}
}

void Listener::setLspeed(const std_msgs::Int16 msg)
{
	motors[0].setSetpoint(msg.data);
	motors[2].setSetpoint(msg.data);
}

void Listener::setRspeed(const std_msgs::Int16 msg)
{
	motors[1].setSetpoint(msg.data);
	motors[3].setSetpoint(msg.data);
}

void Listener::updateEncoders(const std_msgs::Int16MultiArray msg)
{
	for (int i = 0; i < MOTOR_QUANTITY; ++i)
	{
		motors[i].update(msg.data[i]);
	}
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
	ros::Subscriber encoderSub = n.subscribe("encoder_count", 1000, &Listener::updateEncoders, &listener);
	
	ros::spin();
	
	return 0;
}