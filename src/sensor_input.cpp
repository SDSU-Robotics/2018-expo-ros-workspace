#include "ros/ros.h"
#include "std_msgs/Int16.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h"

#include "mcp3008Spi.h"
#include "sensors.h"
#include "dn2DistIR.h"

#include <sstream>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "sensor_input");
	
	ros::NodeHandle n;
	
	ros::Publisher IR_raw_pub = n.advertise<std_msgs::Float32MultiArray>("IR_raw", 1000);
	
	ros::Rate loop_rate(100);

	// create mcp3008Spi objects
	mcp3008Spi adc0("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);
	mcp3008Spi adc1("/dev/spidev0.1", SPI_MODE_0, 1000000, 8);
	
	ADCsensor IRsensors[8];
	for (int i = 0; i < 8; i++)
		IRsensors[i].init(&adc0, i);
	
	std_msgs::Float32MultiArray IRmsg;
	IRmsg.data.resize(8);
	
	while (ros::ok())
	{
		for (int i = 0; i < 8; i++)
			IRmsg.data[i] = dn2DistIR(IRsensors[i].getValue());
		
		IR_raw_pub.publish(IRmsg);
		
		loop_rate.sleep();
	}
	
	return 0;
}
