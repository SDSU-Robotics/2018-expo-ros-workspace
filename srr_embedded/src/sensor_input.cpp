#include "ros/ros.h"
#include "std_msgs/Int16.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h" // ir_raw
#include "std_msgs/Int32MultiArray.h" // encoder_raw

#include "mcp3008Spi.h"
#include "sensors.h"
#include "dn2DistIR.h"

#include <sstream>
#include <pigpiod_if2.h>

const int NUM_IR = 8; // number of IR sensors
const int NUM_ENC = 6; // number of encoders

const int ENCODER_ADDRESS = 40; // address of encoder counter

int main (int argc, char **argv)
{
	ros::init(argc, argv, "sensor_input");
	
	ros::NodeHandle n;
	
	ros::Rate loop_rate(100);

	// initialize pigpio
	int pi = pigpio_start(NULL, NULL);	
	if (pi < 0)
		ROS_INFO("Error: %s\n", pigpio_error(pi));
	
	// create mcp3008Spi objects
	mcp3008Spi adc0("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);
	mcp3008Spi adc1("/dev/spidev0.1", SPI_MODE_0, 1000000, 8);
	
	// IR sensor setup
	ros::Publisher IR_raw_pub = n.advertise<std_msgs::Float32MultiArray>("IR_raw", 1000);
	
	ADCsensor IRsensors[NUM_IR];
	for (int i = 0; i < NUM_IR; i++)
		IRsensors[i].init(&adc0, i);
	
	std_msgs::Float32MultiArray IRmsg;
	IRmsg.data.resize(NUM_IR);
	
	// Encoder setup
	ros::Publisher encoder_raw_pub = n.advertise<std_msgs::Int32MultiArray>("encoder_raw", 1000);
	
	int encoderI2C = i2c_open(pi, 1, ENCODER_ADDRESS, 0); // opens i2c device at address 40
	
	std_msgs::Int32MultiArray encoderMsg;
	encoderMsg.data.resize(NUM_ENC);
	
	while (ros::ok())
	{
		// read IR sensors
		for (int i = 0; i < NUM_IR; i++)
			IRmsg.data[i] = dn2DistIR(IRsensors[i].getValue());
		IR_raw_pub.publish(IRmsg);
		
		
		// read encoders
		char data[24];
		i2c_read_i2c_block_data(pi, encoderI2C, 0, data, 4 * NUM_ENC);
		
		for (int i = 0; i < NUM_ENC; i++)
		{
			encoderMsg.data[i] = 0;
			encoderMsg.data[i] |= (data[4*i] << 24) & 0xFFFFFFFF;
			encoderMsg.data[i] |= (data[4*i + 1] << 16) & 0xFFFFFFFF;
			encoderMsg.data[i] |= (data[4*i + 2] << 8) & 0xFFFFFFFF;
			encoderMsg.data[i] |= data[4*i + 3] & 0xFFFFFFFF;
		}
		encoder_raw_pub.publish(encoderMsg);
		
		
		loop_rate.sleep();
	}
	
	return 0;
}
