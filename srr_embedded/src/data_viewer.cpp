#include "ros/ros.h"
#include <sstream>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32.h" // left_speed, right_speed
#include "std_msgs/UInt16MultiArray.h" // US_raw
#include "std_msgs/Float32MultiArray.h" // IR_raw
#include "std_msgs/Int32MultiArray.h" // encoder_raw
#include "std_msgs/Float64.h" // Current Sensor

class Listener
{
public:
	void setLeftSpeed(const std_msgs::Float32 msg);
	void setRightSpeed(const std_msgs::Float32 msg);
	void setIRdata(const std_msgs::Float32MultiArray msg);
	void setUSdata(const std_msgs::UInt16MultiArray msg);
	void setEncoderData(const std_msgs::Int32MultiArray msg);
	void setIMUdata(const std_msgs::Float32MultiArray msg);
	void setM0Idata(const std_msgs::Float64 msg);
	void setM1Idata(const std_msgs::Float64 msg);
	void setM2Idata(const std_msgs::Float64 msg);
	void printData();
	
private:
	double leftSpeed;
	double rightSpeed;
	unsigned int IRdata[8];
	unsigned int USdata[8];
	int encoderData[6];
	double IMUdata[9];
	float m0_current;
	float m1_current;
	float m2_current;
};

void Listener::setLeftSpeed(const std_msgs::Float32 msg)
{
	leftSpeed = msg.data;
}

void Listener::setRightSpeed(const std_msgs::Float32 msg)
{
	rightSpeed = msg.data;
}

void Listener::setIRdata(const std_msgs::Float32MultiArray msg)
{
	for (int i = 0; i < 8; i++)
		IRdata[i] = msg.data[i];
}

void Listener::setUSdata(const std_msgs::UInt16MultiArray msg)
{
	for (int i = 0; i < 8; i++)
		USdata[i] = msg.data[i];
}

void Listener::setEncoderData(const std_msgs::Int32MultiArray msg)
{
	for (int i = 0; i < 6; i++)
		encoderData[i] = msg.data[i];
}

void Listener::setIMUdata(const std_msgs::Float32MultiArray msg)
{
	for (int i = 0; i < 10; i++)
		IMUdata[i] = msg.data[i];
}

void Listener::setM0Idata(const std_msgs::Float64 msg)
{
	m0_current = msg.data;
}

void Listener::setM1Idata(const std_msgs::Float64 msg)
{
        m1_current = msg.data;
}

void Listener::setM2Idata(const std_msgs::Float64 msg)
{
        m2_current = msg.data;
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
	
	std::cout << std::endl;
	
	for (int i = 0; i < 6; i++)
		std::cout << "Encoder" << i << ": " << encoderData[i] << std::endl;
	
	std::cout << std::endl;
	
	std::cout << "IMU Ax: " << IMUdata[0] << std::endl;
	std::cout << "IMU Ay: " << IMUdata[1] << std::endl;
	std::cout << "IMU Az: " << IMUdata[2] << std::endl;
	std::cout << "IMU Gx: " << IMUdata[3] << std::endl;
	std::cout << "IMU Gy: " << IMUdata[4] << std::endl;
	std::cout << "IMU Gz: " << IMUdata[5] << std::endl;
	std::cout << "IMU Hx: " << IMUdata[6] << std::endl;
	std::cout << "IMU Hy: " << IMUdata[7] << std::endl;
	std::cout << "IMU Hz:  " << IMUdata[8] << std::endl;
	
	std::cout << std::endl;	

	std::cout << "Current Sensor 1: " << m0_current << std::endl;
	std::cout << "Current Sensor 2: " << m1_current << std::endl;
	std::cout << "Current Sensor 3: " << m2_current << std::endl; 
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
	ros::Subscriber encoderSub = n.subscribe("encoder_raw", 1000, &Listener::setEncoderData, &listener);
	ros::Subscriber IMUSub = n.subscribe("IMU_raw", 1000, &Listener::setIMUdata, &listener);
	ros::Subscriber m0ISub = n.subscribe("m0_current", 10, &Listener::setM0Idata, &listener);
        ros::Subscriber m1ISub = n.subscribe("m1_current", 10, &Listener::setM1Idata, &listener);
        ros::Subscriber m2ISub = n.subscribe("m2_current", 10, &Listener::setM2Idata, &listener);

	while (ros::ok())
	{
		ros::spinOnce();
		listener.printData();
		loop_rate.sleep();
	}
	
	return 0;
}
