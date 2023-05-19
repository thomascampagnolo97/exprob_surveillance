#!/usr/bin/env python

"""
.. module:: battery_signal
   :platform: Unix
   :synopsis: Python code to randomly change the battery level
.. moduleauthor:: Thomas Campagnolo <s5343274@studenti.unige.it>

ROS Node to pblish the battery level, keeping it charget for a random period of time and keeping it in charge for 10 seconds

Publishes to:
    - /battery_signal a boolean flag to communicate when battery is low and when is totally charged

"""

import rospy
from armor_api.armor_client import ArmorClient
from std_msgs.msg import Bool

# Import constant name defined to structure the architecture
from exprob_surveillance import architecture_name_mapper as anm

BATTERY_CAPACITY = 40

world_done = 0


# time_to_recharge = 10 #sec

battery_level = BATTERY_CAPACITY
threshold = 4

def world_callback(data):
    """ 
    Callback function for the 
    
    """
    global world_done
    if data.data == 0:
        world_done = 0

    elif data.data == 1:
        world_done = 1

def battery_discharge(battery_level):
    for battery_level in range(BATTERY_CAPACITY, -2, -2):
        print("Battery level: ", battery_level)

        rospy.sleep(1.5)
    
        if battery_level <= threshold:
            battery_status = 0
            return battery_status
            break

def battery_recharge(battery_level):
    for battery_level in range(threshold, BATTERY_CAPACITY+1, 2):
        print("Battery level recharged: ", battery_level)
        rospy.sleep(0.5)

        if battery_level == BATTERY_CAPACITY:
            battery_status = 1
            return battery_status



def battery_behaviour():
    """
    Function to initialize the battery node and to publish the boolean value of the battery to the state ``...``, advertised by :mod:`...`
    
    """

    rospy.init_node(anm.NODE_ROBOT_BATTERY, log_level=rospy.INFO)

    pub = rospy.Publisher(anm.TOPIC_BATTERY_SIGNAL, Bool, queue_size=10)
    rospy.Subscriber(anm.TOPIC_SYNC_WORLD_BATTERY, Bool, world_callback) # world flag for sync

    
    
    while not rospy.is_shutdown():

        if world_done == 1:

            battery_status = battery_discharge(battery_level)
            pub.publish(battery_status)

            if battery_status == 0:
                battery_status = battery_recharge(battery_level)
                pub.publish(battery_status)

        else:
            print("Waiting map ... ")
            rospy.sleep(5)


if __name__ == '__main__':
    """
    Entry point of the code
    """ 
    try:
        battery_behaviour()
    except rospy.ROSInterruptException:
        pass
