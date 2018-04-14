#include "ros/ros.h"

#include "std_msgs/Float64.h" 
#include <sensor_msgs/Joy.h>
#include <sstream>



class Listener
{
public:
	void update(const sensor_msgs::Joy::ConstPtr& Joy);
//	void setHandle(ros::NodeHandle n) { n_ = n; };
	void init(ros::Publisher *, ros::Publisher *, ros::Publisher *, ros::Publisher *, ros::Publisher *, ros::Publisher *);
private:
	ros::Publisher *m0_pub_;
	ros::Publisher *m1_pub_;
	ros::Publisher *m2_pub_;
	ros::Publisher *m3_pub_;
	ros::Publisher *m4_pub_;
	ros::Publisher *m5_pub_;
};

void Listener::update(const sensor_msgs::Joy::ConstPtr& Joy)
{

//	std::cout << "Test Functionality" << std::endl;
	std_msgs::Float64 mL_speed_setpoint_msg;
	std_msgs::Float64 mR_speed_setpoint_msg;

	
	mL_speed_setpoint_msg.data = Joy -> axes[1];
	
	mR_speed_setpoint_msg.data = Joy -> axes[3];

	m0_pub_->publish(mL_speed_setpoint_msg);
        m2_pub_->publish(mL_speed_setpoint_msg);
        m4_pub_->publish(mL_speed_setpoint_msg);
	
        m1_pub_->publish(mR_speed_setpoint_msg);
        m3_pub_->publish(mR_speed_setpoint_msg);
        m5_pub_->publish(mR_speed_setpoint_msg);

	
	
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

	std::cout << "Successfully Initialized" << std::endl;
	
	ros::NodeHandle n;
	
	std::cout << "Nodehandle Successful" << std::endl;

	std::cout << "Handle Successful" << std::endl;	
	//ros::Rate loop_rate(20);
	Listener listener;
	
//	listener.setHandle(n);
	
	std::cout << "listener Successful" << std::endl;	



        ros::Publisher m0_pub = n.advertise<std_msgs::Float64>("m0_speed_setpoint", 1000);
        ros::Publisher m1_pub = n.advertise<std_msgs::Float64>("m1_speed_setpoint", 1000);
        ros::Publisher m2_pub = n.advertise<std_msgs::Float64>("m2_speed_setpoint", 1000);
        ros::Publisher m3_pub = n.advertise<std_msgs::Float64>("m3_speed_setpoint", 1000);
        ros::Publisher m4_pub = n.advertise<std_msgs::Float64>("m4_speed_setpoint", 1000);
        ros::Publisher m5_pub = n.advertise<std_msgs::Float64>("m5_speed_setpoint", 1000);


	listener.init(&m0_pub, &m1_pub, &m2_pub, &m3_pub, &m4_pub, &m5_pub);


//	listener.setHandle(n);

	std::cout << "Nodehandle Transferred" << std::endl;

	ros::Subscriber joySub = n.subscribe("joy", 10, &Listener::update, &listener);
	
	
	while (ros::ok())
	{
		ros::spinOnce();
		//loop_rate.sleep();
	}
	
	return 0;
}
