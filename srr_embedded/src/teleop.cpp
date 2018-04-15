#include "ros/ros.h"

#include "std_msgs/Float64.h" 
#include <sensor_msgs/Joy.h>
#include <sstream>

class Listener
{
public:
	void update(const sensor_msgs::Joy::ConstPtr& Joy);
	void init(ros::Publisher *, ros::Publisher *, ros::Publisher *, ros::Publisher *, ros::Publisher *, ros::Publisher *);
private:
	bool teleopMode = true;

	ros::Publisher *m0_pub_;
	ros::Publisher *m1_pub_;
	ros::Publisher *m2_pub_;
	ros::Publisher *m3_pub_;
	ros::Publisher *m4_pub_;
	ros::Publisher *m5_pub_;

	std_msgs::Float64 mL_speed_setpoint_msg;
	std_msgs::Float64 mR_speed_setpoint_msg;
};

void Listener::update(const sensor_msgs::Joy::ConstPtr& Joy)
{
	if (Joy->buttons[9] == 1)
	{
		teleopMode = !teleopMode;
		if (teleopMode)
			ROS_INFO("Teleop Mode\n");
		else
			ROS_INFO("Autonomous Mode\n");
	}

	if (teleopMode)
	{
		mL_speed_setpoint_msg.data = Joy->axes[1];
		mR_speed_setpoint_msg.data = Joy->axes[3];

		m0_pub_->publish(mL_speed_setpoint_msg);
		m2_pub_->publish(mL_speed_setpoint_msg);
		m4_pub_->publish(mL_speed_setpoint_msg);
	
		m1_pub_->publish(mR_speed_setpoint_msg);
		m3_pub_->publish(mR_speed_setpoint_msg);
		m5_pub_->publish(mR_speed_setpoint_msg);
	}
}

void Listener::init(ros::Publisher *m0, ros::Publisher *m1, ros::Publisher *m2, ros::Publisher *m3, ros::Publisher *m4, ros::Publisher *m5)
{
	m0_pub_ = m0;
	m1_pub_ = m1;
	m2_pub_ = m2;
	m3_pub_ = m3;
	m4_pub_ = m4;
	m5_pub_ = m5;
}


int main (int argc, char **argv)
{
        ros::init(argc, argv, "teleop");
	
	ros::NodeHandle n;
	
	//ros::Rate loop_rate(20);
	Listener listener;

        ros::Publisher m0_pub = n.advertise<std_msgs::Float64>("m0_speed_control_effort", 1000);
        ros::Publisher m1_pub = n.advertise<std_msgs::Float64>("m1_speed_control_effort", 1000);
        ros::Publisher m2_pub = n.advertise<std_msgs::Float64>("m2_speed_control_effort", 1000);
        ros::Publisher m3_pub = n.advertise<std_msgs::Float64>("m3_speed_control_effort", 1000);
        ros::Publisher m4_pub = n.advertise<std_msgs::Float64>("m4_speed_control_effort", 1000);
        ros::Publisher m5_pub = n.advertise<std_msgs::Float64>("m5_speed_control_effort", 1000);

	listener.init(&m0_pub, &m1_pub, &m2_pub, &m3_pub, &m4_pub, &m5_pub);

	ros::Subscriber joySub = n.subscribe("joy", 10, &Listener::update, &listener);
	
	while (ros::ok())
	{
		ros::spinOnce();
		//loop_rate.sleep();
	}
	
	return 0;
}
