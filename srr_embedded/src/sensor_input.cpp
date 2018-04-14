#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h" // ir_raw
#include "std_msgs/Int32MultiArray.h" // encoder_raw

#include "std_msgs/Float64.h" // control_effort

#include "mcp3008Spi.h"
#include "sensors.h"
#include "dn2DistIR.h"

#include "MPU9250.h"
#include <wiringPi.h>

#include <sstream>
#include <pigpiod_if2.h>

const int NUM_IR = 8; // number of IR sensors
const int NUM_ENC = 6; // number of encoders

const int ENCODER_ADDRESS = 40; // address of encoder counter

const int LOOP_FREQUENCY = 10; // Hz

const float WHEEL_DIAMETER = 5; // diameter in cm
const int COUNTS_PER_REVOLUTION = 1440; // encoder counts per revolution

const double PI = 3.14159265;

int main (int argc, char **argv)
{
	float ax, ay, az, gx, gy, gz, hx, hy, hz, t;
		
	int beginStatus;
	MPU9250 IMU(0x68);
	
	ros::init(argc, argv, "sensor_input");
	
	ros::NodeHandle n;
	
	ros::Rate loop_rate(LOOP_FREQUENCY);

	// initialize pigpio
	int pi = pigpio_start(NULL, NULL);	
	if (pi < 0)
		ROS_INFO("Error: %s\n", pigpio_error(pi));
	
	// create mcp3008Spi objects
	mcp3008Spi adc0("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);
	mcp3008Spi adc1("/dev/spidev0.1", SPI_MODE_0, 1000000, 8);
	
	
	// Initialize IMU communication
	beginStatus = IMU.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);

 	if(beginStatus < 0) {
		delay(1000);
		fprintf(stderr, "IMU initialization unsuccessful\n");
		fprintf(stderr, "Check IMU wiring or try cycling power \n");
		delay(10000);
	}
	ros::Publisher IMU_raw_pub = n.advertise<std_msgs::Float32MultiArray>("IMU_raw", 1000);
	std_msgs::Float32MultiArray IMUmsg;
	IMUmsg.data.resize(9);
	
	// IR sensor setup
	ros::Publisher IR_raw_pub = n.advertise<std_msgs::Float32MultiArray>("IR_raw", 1000);
	
	ADCsensor IRsensors[NUM_IR];
	for (int i = 0; i < NUM_IR; i++)
		IRsensors[i].init(&adc0, i);
	
	std_msgs::Float32MultiArray IRmsg;
	IRmsg.data.resize(NUM_IR);
	
	// Encoder setup
	ros::Publisher encoder_raw_pub = n.advertise<std_msgs::Int32MultiArray>("encoder_raw", 1000);
	ros::Publisher m0_rate_pub = n.advertise<std_msgs::Float64>("m0_rate", 1000);
	ros::Publisher m1_rate_pub = n.advertise<std_msgs::Float64>("m1_rate", 1000);
	ros::Publisher m2_rate_pub = n.advertise<std_msgs::Float64>("m2_rate", 1000);
	ros::Publisher m3_rate_pub = n.advertise<std_msgs::Float64>("m3_rate", 1000);
	ros::Publisher m4_rate_pub = n.advertise<std_msgs::Float64>("m4_rate", 1000);
	ros::Publisher m5_rate_pub = n.advertise<std_msgs::Float64>("m5_rate", 1000);
	
	int encoderI2C = i2c_open(pi, 1, ENCODER_ADDRESS, 0); // opens i2c device at address 40
	
	int currentCounts[NUM_ENC] = { 0 };
	int lastCounts[NUM_ENC] = { 0 };
	
	std_msgs::Int32MultiArray encoderMsg;
	encoderMsg.data.resize(NUM_ENC);
	
	std_msgs::Float64 rateMsg[NUM_ENC];
	
	while (ros::ok())
	{
		// read IR sensors
		for (int i = 0; i < NUM_IR; i++)
			IRmsg.data[i] = dn2DistIR(IRsensors[i].getValue());
		IR_raw_pub.publish(IRmsg);
		
		//read IMU
		IMU.getMotion10(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz, &t);
		
		// Assign values to arrays to make life easier
		IMUmsg.data[0] = ax;
		IMUmsg.data[1] = ay;
		IMUmsg.data[2] = az;
		IMUmsg.data[3] = gx;
		IMUmsg.data[4] = gy;
		IMUmsg.data[5] = gz;
		IMUmsg.data[6] = hx;
		IMUmsg.data[7] = hy;
		IMUmsg.data[8] = hz;
		IMUmsg.data[9] = t;
		
 		IMU_raw_pub.publish(IMUmsg);
		
		
		// read encoders
		char data[24];
		i2c_read_i2c_block_data(pi, encoderI2C, 0, data, 4 * NUM_ENC);
		
		for (int i = 0; i < NUM_ENC; i++)
		{
			lastCounts[i] = currentCounts[i];
			
			currentCounts[i] = 0;
			currentCounts[i] |= (data[4*i] << 24) & 0xFFFFFFFF;
			currentCounts[i] |= (data[4*i + 1] << 16) & 0xFFFFFFFF;
			currentCounts[i] |= (data[4*i + 2] << 8) & 0xFFFFFFFF;
			currentCounts[i] |= data[4*i + 3] & 0xFFFFFFFF;
			
			encoderMsg.data[i] = currentCounts[i];
			
			rateMsg[i].data = (currentCounts[i] - lastCounts[i]) / COUNTS_PER_REVOLUTION * LOOP_FREQUENCY * PI * WHEEL_DIAMETER / 100;
		}
		
		encoder_raw_pub.publish(encoderMsg);
		m0_rate_pub.publish(rateMsg[0]);
		m1_rate_pub.publish(rateMsg[1]);
		m2_rate_pub.publish(rateMsg[2]);
		m3_rate_pub.publish(rateMsg[3]);
		m4_rate_pub.publish(rateMsg[4]);
		m5_rate_pub.publish(rateMsg[5]);
		
		
		loop_rate.sleep();
	}
	
	return 0;
}
