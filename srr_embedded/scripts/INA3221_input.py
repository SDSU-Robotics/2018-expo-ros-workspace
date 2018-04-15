#!/usr/bin/env python
#
# Test SDL_Pi_INA3221
# John C. Shovic, SwitchDoc Labs
# 03/05/2015
#
#

# imports

import sys
import random 
import SDL_Pi_INA3221
from std_msgs.msg import Float64
import rospy

# Main Program

iSense0 = SDL_Pi_INA3221.SDL_Pi_INA3221(addr=0x40)
iSense1 = SDL_Pi_INA3221.SDL_Pi_INA3221(addr=0x41)

def Listener():

	m0_pub  = rospy.Publisher('m0_current', Float64, queue_size=10)
        m1_pub  = rospy.Publisher('m1_current', Float64, queue_size=10)
        m2_pub  = rospy.Publisher('m2_current', Float64, queue_size=10)
        m3_pub  = rospy.Publisher('m3_current', Float64, queue_size=10)
        m4_pub  = rospy.Publisher('m4_current', Float64, queue_size=10)
        m5_pub  = rospy.Publisher('m5_current', Float64, queue_size=10)

	rospy.init_node('Listener', anonymous=True) 
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():

				

  			current_mA0 = 0
  			current_mA0 = iSense0.getCurrent_mA(1)  
			
  			current_mA1 = 0
  			current_mA1 = iSense0.getCurrent_mA(2)
  			  		
  			current_mA2 = 0
			current_mA2 = iSense0.getCurrent_mA(3)
  			
			current_mA3 = 0
                        current_mA3 = iSense1.getCurrent_mA(1)

                        current_mA4 = 0
                        current_mA4 = iSense1.getCurrent_mA(2)
                                        
                        current_mA5 = 0
                        current_mA5 = iSense1.getCurrent_mA(3)


  			print
			
			rospy.loginfo(current_mA0)
			m0_pub.publish(current_mA0)
		
			rospy.loginfo(current_mA1)
			m1_pub.publish(current_mA1)

			rospy.loginfo(current_mA2)
			m2_pub.publish(current_mA2)
			
			rospy.loginfo(current_mA3)
                        m3_pub.publish(current_mA3)
			
			rospy.loginfo(current_mA4)
                        m4_pub.publish(current_mA4)

			rospy.loginfo(current_mA5)
                        m5_pub.publish(current_mA5)



		
	
	rate.sleep()

if __name__ == '__main__':
	try:
		Listener()
	except rospy.ROSInterruptException:
		pass
