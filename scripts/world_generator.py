#!/usr/bin/env python

"""
.. module:: world_generator
   :platform: Unix
   :synopsis: Python code to create the .owl ontology 
.. moduleauthor:: Thomas Campagnolo <s5343274@studenti.unige.it>

ROS Node to create the Ontology map adn initialzie the timestamps for the Rooms, which represents a 2D environment of 4 rooms and 3 corridors

Publishes to:
    - /loader a boolean flag to communicate when the map is totally created.

"""


import rospy
import rospkg
from armor_api.armor_client import ArmorClient
from std_msgs.msg import Bool, String
import time
import math
import os
import sys


rp = rospkg.RosPack()
assignment_path = rp.get_path('exprob_surveillance')
ONTOLOGY_FILE_PATH = os.path.join(assignment_path, "topological_map", "topological_map.owl")
WORLD_ONTOLOGY_FILE_PATH = os.path.join(assignment_path, "topological_map", "world_surveillance.owl")
WEB_PATH = 'http://bnc/exp-rob-lab/2022-23'



def timestamp_computation(list):
    timestamp = ''
    """
    Function to clean the queried time stamp for both Rooms and Robot's data property.

    Args:
        - *list* the list of queried objects section of the Armor service message.
    Returns:
        all the element between the double quotes
    
    """
    for i in list:
        for element in range(1, 11):
            timestamp=timestamp+i[element]
     
    return timestamp

def load_std_map():
    """
    Function to initialize the node, to initialize the publisher and to use the `Armor commands <https://github.com/EmaroLab/armor/blob/master/commands.md>`_ to create the ontology.
    It will publish a boolean that will be passed to the state ``Build_world``, advertised by :mod:`load_std_map`

    """

    pub = rospy.Publisher('world_loading', Bool, queue_size=10)
    rospy.init_node('world_generator_node', anonymous=True)
    pub.publish(False)

    print("Building the environment...")

    client = ArmorClient("example", "ontoRef")

    client.call('LOAD','FILE','',[ONTOLOGY_FILE_PATH, WEB_PATH, 'true', 'PELLET', 'false'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'E', 'D6'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'E', 'D7'])
    print('Doors D6 and D7 associated to room E (RECHARGING ROOM)')
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R1', 'D1'])
    print('Door D1 associated to room R1')
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R2', 'D2'])
    print('Door D2 associated to room R2')
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R3', 'D3'])
    print('Door D3 associated to room R3')
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R4', 'D4'])
    print('Door D4 associated to room R4')
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D1'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D2'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D5'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D6'])
    print('Doors D1, D2, D5, D6 associated to corridor C1')
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D3'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D4'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D5'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D7'])
    print('Doors D3, D4, D5, D7 associated to corridor C2')

   
    client.call('DISJOINT','IND','',['R1','R2','R3','R4','E','C1','C2','D1','D2','D3','D4','D5','D6','D7'])
    client.call('ADD','OBJECTPROP','IND',['isIn', 'Robot1', 'E'])
    print('Initialization: Robot1 is in room E')
    client.call('REASON','','',[''])
    print('Reasoning...')

    rospy.sleep(2)
    
    # The robot visits each room for the first time
    # Visit and timestamp creation for R1
    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'R1', 'E'])
    client.call('REASON','','',[''])
    
    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = timestamp_computation(rob_time.queried_objects)
    print('R1 old time: ', old_rob_time)
    current_time=str(math.floor(time.time()))
    print('R1 current time: ', current_time)

    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
    client.call('REASON','','',[''])
    
    client.call('ADD','DATAPROP','IND',['visitedAt','R1', 'Long', current_time])
    client.call('REASON','','',[''])
    rospy.sleep(12)

    # Visit and timestamp creation for R2

    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'R2', 'R1'])
    client.call('REASON','','',[''])
    
    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = timestamp_computation(rob_time.queried_objects)
    print('R2 old time: ', old_rob_time)
    current_time=str(math.floor(time.time()))
    print('R2 current time: ', current_time)
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
    client.call('REASON','','',[''])

    client.call('ADD','DATAPROP','IND',['visitedAt','R2', 'Long', current_time])
    client.call('REASON','','',[''])
    rospy.sleep(12)

    # Visit and timestamp creation for R3

    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'R3', 'R2']) 
    client.call('REASON','','',[''])
    
    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = timestamp_computation(rob_time.queried_objects)
    print('R3 old time: ', old_rob_time)
    current_time=str(math.floor(time.time()))
    print('R3 current time: ', current_time)    
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
    client.call('REASON','','',[''])

    client.call('ADD','DATAPROP','IND',['visitedAt','R3', 'Long', current_time])
    client.call('REASON','','',[''])
    rospy.sleep(12)

    # Visit and timestamp creation for R4

    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'R4', 'R3'])  
    client.call('REASON','','',[''])
    
    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = timestamp_computation(rob_time.queried_objects)
    print('R4 old time: ', old_rob_time)
    current_time=str(math.floor(time.time()))
    print('R4 current time: ', current_time)
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
    client.call('REASON','','',[''])

    client.call('ADD','DATAPROP','IND',['visitedAt','R4', 'Long', current_time])
    client.call('REASON','','',[''])
    rospy.sleep(12)
    
    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'C1', 'R4'])
    client.call('REASON','','',[''])

    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = timestamp_computation(rob_time.queried_objects)
    current_time=str(math.floor(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
    client.call('REASON','','',[''])


    client.call('SAVE','','',[WORLD_ONTOLOGY_FILE_PATH])

    while not rospy.is_shutdown():
        pub.publish(True)
    

if __name__ == '__main__':

    """
    Entrance point of the code
    """ 

    try:
    	load_std_map()
    except rospy.ROSInterruptException:
    	pass
   