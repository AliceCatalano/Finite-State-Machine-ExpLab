#!/usr/bin/env python

"""
.. module:: battery_signal
   :platform: Unix
   :synopsis: Python code to randomly change the battery level
.. moduleauthor:: Alice Maria Catalano <s5157341@studenti.unige.it>

ROS Node to pblish the battery level, keeping it charget for a random period of time and keeping it in charge for 10 seconds

Publishes to:
    - /battery_signal a boolean flag to communicate when battery is low and when is totally charged

"""

import rospy
import random
from armor_api.armor_client import ArmorClient
from std_msgs.msg import Bool

timeTo_recharge = 10 #sec

def battery_signal():
    """
    Function to initialize the battery node and to publish the boolean value of the battery to the state ``Load_map``, advertised by :mod:`load_std_map`
    
    """
    
    pub = rospy.Publisher('battery_signal', Bool, queue_size=10)
    rospy.init_node('battery_signal_node', anonymous=True)
    
    while not rospy.is_shutdown():
        
        battery_status = 1
        pub.publish(battery_status)
        time_charged =  random.uniform( 20, 50 )
        rospy.sleep(time_charged)
        battery_status = 0
        pub.publish(battery_status)
        rospy.sleep(timeTo_recharge)
        battery_status = 1
        pub.publish(battery_status)

if __name__ == '__main__':
    """
    Entry point of the code
    """ 
    try:
        battery_signal()
    except rospy.ROSInterruptException:
        pass

