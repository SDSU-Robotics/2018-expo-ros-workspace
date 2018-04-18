#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include <sensor_msgs/Joy.h>
#include <sstream>

const double ACTUATOR_RATE = 0.001;
const double CLAW_RATE = 0.01;

const double STARTING_HORZ = 1.0;
const double STARTING_VERT = 0.5;
const double STARTING_CLAW = 0.0;

enum Mode { TELEOP, AUTONOMOUS };
const int NUM_MODES = 2;

const int DEFAULT_MODE = AUTONOMOUS;


class Listener
{
public:
	void joyListener(const sensor_msgs::Joy::ConstPtr& Joy);
	void getJoyVals(bool buttons[], double axes[]) const;

private:
    bool _buttons[12] = { 0 };
	double _axes[6] = { 0 };
};


void Listener::joyListener(const sensor_msgs::Joy::ConstPtr& Joy)
{
	for (int i = 0 ; i < 12; i++)
		_buttons[i] = Joy->buttons[i];

    for (int i = 0; i < 6; i++)
        _axes[i] = Joy->axes[i];
}

void Listener::getJoyVals(bool buttons[], double axes[]) const
{
    for (int i = 0; i < 12; i++)
        buttons[i] = _buttons[i];

    for (int i = 0; i < 6; i++)
        axes[i] = _axes[i];
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

	ros::Subscriber joySub = n.subscribe("joy", 100, &Listener::joyListener, &listener);

	std_msgs::Float64 horz_pos_msg;
    std_msgs::Float64 vert_pos_msg;
    std_msgs::Float64 claw_pos_msg;

	double horzPos = STARTING_HORZ;
	double vertPos = STARTING_VERT;
	double clawPos = STARTING_CLAW;

	int mode = DEFAULT_MODE;

	while (ros::ok())
	{
	    bool buttons[12];
	    double axes[6];

	    std_msgs::Float64 mL_speed_setpoint_msg;
        std_msgs::Float64 mR_speed_setpoint_msg;

        listener.getJoyVals(buttons, axes);

        // if Start is pressed, cycle through modes
        if (buttons[9])
        {
            mode++;
            if (mode >= NUM_MODES)
                mode = 0;

            switch (mode)
            {
            case TELEOP:
                ROS_INFO("Teleop Mode\n");
                break;
            case AUTONOMOUS:
                ROS_INFO("Autonomous Mode\n");
                break;
            }
        }

		switch (mode)
		{
        case TELEOP:
            mL_speed_setpoint_msg.data = axes[1];
            mR_speed_setpoint_msg.data = axes[3];

			// if X is pressed
			if (buttons[0])
			{
				// rotate base
			}

			// if B is pressed
			if (buttons[2])
			{
				// rotate base
			}

			// if Y is pressed
			if (buttons[3])
			{
				vertPos += ACTUATOR_RATE;
				if (vertPos > 1)
					vertPos = 1;
			}

			// if A is pressed
			if (buttons[1])
			{
				vertPos -= ACTUATOR_RATE;
				if (vertPos < 0)
					vertPos = 0;
			}

			// if RB is pressed
            if (buttons[5])
            {
                horzPos += ACTUATOR_RATE;
                if (horzPos > 1)
                    horzPos = 1;
                }

			// if RT is pressed
            if (buttons[7])
            {
                horzPos -= ACTUATOR_RATE;
                if (horzPos < 0)
                    horzPos = 0;
            }

            // if LB is pressed
            if (buttons[4])
            {
                clawPos += CLAW_RATE;
                if (clawPos > 1)
                    clawPos = 1;
            }

            // if LT is pressed
            if (buttons[6])
            {
                clawPos -= CLAW_RATE;
                if (clawPos < 0)
                    clawPos = 0;
            }

			horz_pos_msg.data = horzPos;
			vert_pos_msg.data = vertPos;
			claw_pos_msg.data = clawPos;

			horz_pub.publish(horz_pos_msg);
			vert_pub.publish(vert_pos_msg);
			claw_pub.publish(claw_pos_msg);

			break;

        case AUTONOMOUS:


            break;
		} // switch(mode)

		// publish motor setpoints
		m0_pub.publish(mL_speed_setpoint_msg);
		m2_pub.publish(mL_speed_setpoint_msg);
		m4_pub.publish(mL_speed_setpoint_msg);

		m1_pub.publish(mR_speed_setpoint_msg);
		m3_pub.publish(mR_speed_setpoint_msg);
		m5_pub.publish(mR_speed_setpoint_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
