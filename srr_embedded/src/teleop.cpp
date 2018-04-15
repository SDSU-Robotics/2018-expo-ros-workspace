#include "ros/ros.h"

#include "std_msgs/Float64.h" 
#include <sensor_msgs/Joy.h>
#include <sstream>

const double ACTUATOR_RATE = 0.001;

const double STARTING_HORZ = 1.0;
const double STARTING_VERT = 0.5;
const double STARTING_CLAW = 0.0;

class Listener
{
public:
	void update(const sensor_msgs::Joy::ConstPtr& Joy);
	void init(ros::Publisher *, ros::Publisher *, ros::Publisher *, ros::Publisher *, ros::Publisher *, ros::Publisher *);
	void getButtonStates(bool buttonStates[]) {  for (int i = 0; i < 12; i++) buttonStates[i] = _buttonStates[i]; };
	bool isTeleop() { return teleopMode; }

private:
	bool teleopMode = true;

	ros::Publisher *_m0_pub;
	ros::Publisher *_m1_pub;
	ros::Publisher *_m2_pub;
	ros::Publisher *_m3_pub;
	ros::Publisher *_m4_pub;
	ros::Publisher *_m5_pub;

	std_msgs::Float64 mL_speed_setpoint_msg;
	std_msgs::Float64 mR_speed_setpoint_msg;

	bool _buttonStates[12] = { 0 };
};


void Listener::update(const sensor_msgs::Joy::ConstPtr& Joy)
{
	for (int i =0 ; i < 12; i++)
		_buttonStates[i] = Joy->buttons[i];

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

		_m0_pub->publish(mL_speed_setpoint_msg);
		_m2_pub->publish(mL_speed_setpoint_msg);
		_m4_pub->publish(mL_speed_setpoint_msg);

		_m1_pub->publish(mR_speed_setpoint_msg);
		_m3_pub->publish(mR_speed_setpoint_msg);
		_m5_pub->publish(mR_speed_setpoint_msg);
	}
}

void Listener::init(ros::Publisher *m0, ros::Publisher *m1, ros::Publisher *m2, ros::Publisher *m3, ros::Publisher *m4, ros::Publisher *m5)
{
	_m0_pub = m0;
	_m1_pub = m1;
	_m2_pub = m2;
	_m3_pub = m3;
	_m4_pub = m4;
	_m5_pub = m5;
}


int main (int argc, char **argv)
{
        ros::init(argc, argv, "teleop");
	
	ros::NodeHandle n;
	
	ros::Rate loop_rate(100);
	Listener listener;

        ros::Publisher m0_pub = n.advertise<std_msgs::Float64>("m0_speed_control_effort", 1000);
        ros::Publisher m1_pub = n.advertise<std_msgs::Float64>("m1_speed_control_effort", 1000);
        ros::Publisher m2_pub = n.advertise<std_msgs::Float64>("m2_speed_control_effort", 1000);
        ros::Publisher m3_pub = n.advertise<std_msgs::Float64>("m3_speed_control_effort", 1000);
        ros::Publisher m4_pub = n.advertise<std_msgs::Float64>("m4_speed_control_effort", 1000);
        ros::Publisher m5_pub = n.advertise<std_msgs::Float64>("m5_speed_control_effort", 1000);
	
	ros::Publisher horz_pub = n.advertise<std_msgs::Float64>("horz_actuator_pos", 1000);
	ros::Publisher vert_pub = n.advertise<std_msgs::Float64>("vert_actuator_pos", 1000);
	ros::Publisher claw_pub = n.advertise<std_msgs::Float64>("claw_pos", 1000);

	listener.init(&m0_pub, &m1_pub, &m2_pub, &m3_pub, &m4_pub, &m5_pub);

	ros::Subscriber joySub = n.subscribe("joy", 10, &Listener::update, &listener);
	
	std_msgs::Float64 horz_pos_msg;
        std_msgs::Float64 vert_pos_msg;
        std_msgs::Float64 claw_pos_msg;

	double horzPos = STARTING_HORZ;
	double vertPos = STARTING_VERT;
	double clawPos = STARTING_CLAW;

	while (ros::ok())
	{
		if (listener.isTeleop())
		{
			bool buttonStates[12];
			listener.getButtonStates(buttonStates);

			// if X is pressed
			if (buttonStates[0])
			{
				// rotate base
			}

			// if B is pressed
			if (buttonStates[2])
			{
				// rotate base
			}

			// if Y is pressed
			if (buttonStates[3])
			{
				vertPos += ACTUATOR_RATE;
				if (vertPos > 1)
					vertPos = 1;
			}

			// if A is pressed
			if (buttonStates[1])
			{
				vertPos -= ACTUATOR_RATE;
				if (vertPos < 0)
					vertPos = 0;
			}

			// if RB is pressed
                        if (buttonStates[5])
                        {
                                horzPos += ACTUATOR_RATE;
                                if (horzPos > 1)
                                        horzPos = 1;
                        }

			// if RT is pressed
                        if (buttonStates[7])
                        {
                                horzPos -= ACTUATOR_RATE;
                                if (horzPos < 0)
                                        horzPos = 0;
                        }


			horz_pos_msg.data = horzPos;
			vert_pos_msg.data = vertPos;
			claw_pos_msg.data = clawPos;

			horz_pub.publish(horz_pos_msg);
			vert_pub.publish(vert_pos_msg);
			claw_pub.publish(claw_pos_msg);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
