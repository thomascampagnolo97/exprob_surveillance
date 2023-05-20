#!/usr/bin/env python

"""
.. module:: fsm_behaviour
   :platform: Unix
   :synopsis: Python code of the finite state machine for the robot
.. moduleauthor:: Thomas Campagnolo <s5343274@studenti.unige.it>

Overview:
The scenario involves a robot deployed in a indoor environment for surveillance purposes.
This is finite state machine that uses the `Smach tool <https://wiki.ros.org/smach>`_ to implement it in ROS. 
The FSM menages the surveillance behavior of the robot. It moves among locations with this policy:
    - it should mainly stay on corridors,
    - if a reachable room has not been visited for 20 seconds, the room become **URGENT** and the robot must visit it. 
When the robot's battery is low, it goes in the E location, and wait for some times (recharging action in `battery_node`) before to
start again with the above behaviour.

Publishes to:
    - /world_battery_sync a Boolean flag for synchronization reasons with the :mod:`battery` node. When the world is correctly loaded
      the battery's functionality start its execution

Subscribes  to:
    - /world_loading a Boolean flag to communicate when the map is created correctly.
    - /battery_signal a Boolean flag to communicate when battery is low and when is fully charged

"""

import rospkg
import rospy
import os
import time
import math
import random
import smach
import smach_ros

from armor_api.armor_client import ArmorClient
from std_msgs.msg import Bool

# Import constant name defined to structure the architecture
from exprob_surveillance import architecture_name_mapper as anm


# States of the Finite State Machine
STATE_BUILD_WORLD = 'BUILD_WORLD'       # State where the environment is build using the ontology
STATE_NO_EMERGENCY = 'NO_EMERGENCY'     # State of non-urgency room, the robot only moves between the corridors
STATE_SURVEILLANCE = 'SURVEILLANCE'     # State of urgency room, the robot must visit the urgency locations for surveillance purposes
STATE_RECHARGING = 'RECHARGING'         # State where the robot recharges its battery

# Transitions of the Finite State Machine.
TRANS_WORLD_DONE = 'world_done'             # Transition that identifies the correct loading of the map
TRANS_WAITING_MAP = 'waiting_map'           # Transition to check if the map is uploaded correctly
TRANS_BATTERY_LOW = 'battery_low'           # Transition for the low battery charge. The robot needs to recharge
TRANS_BATTERY_CHARGED = 'battery_charged'   # Transition notifying full battery charge
TRANS_URGENT_ROOM = 'urgent_room'           # Transition for room surveillance
TRANS_NO_URGENT_ROOM = 'no_urgent_room'     # Transition to notify that there are not urgent rooms

# Global variables and flags
wait_time = 1.5         # Global variable for the sleeping time
battery_status = 1      # Battery flag, 1 means that it is fully charged
urgency_status = 0      # Flag to identify the urgent rooms
world_loaded = False    # World flag, False when the map is not loaded, True when map is loaded correctly (pub in world_generator_node)
wb_sync = 0             # World-battery synchronization flag for the battery_node
time_threshold = 20     # Time threshold to signal the urgency of the rooms

shared_location = ''    # List of the shared locations given by the connectedTo data property of the OWL
urgent_rooms = ''       # List from the query of the individuals in the URGENT class
robot_position = ''     # List that contains always one element, which is the robot position in that moment
timestamp = ''          # List of the queried timestamp

# Arguments for loading the ontology
rp = rospkg.RosPack()
assignment_path = rp.get_path('exprob_surveillance')
WORLD_ONTOLOGY_FILE_PATH = os.path.join(assignment_path, "topological_map", "world_surveillance.owl") # also used for debugging the ontology
WEB_PATH = 'http://bnc/exp-rob-lab/2022-23'

# Colors for messages
class bcolors:
    BATTERY = '\033[96m'
    STATUS = '\033[92m'
    MOVING = '\033[93m'
    URGENT = '\033[91m'
    ENDC = '\033[0m'



def world_callback(data):
    """ 
    Callback function for the map publisher */world_loading*, that modifies the value of the global variable **world_loaded** and will let the code start.
    
    """
    global world_loaded
    if data.data == False:
        world_loaded = False

    elif data.data == True:
        world_loaded = True

def battery_callback(data):
    """ 
    Callback function for the map publisher */battery_signal*, that modifies the value of the **global variable battery_status** and will let the code start.
    
    """
    global battery_status
    if data.data == 0:
        battery_status = 0

    elif data.data == 1:
        battery_status =1

def path_planning(listNow, listDes, robPos):
    """
    This function is needed to plan the path from one position to another, recursively scanning the list of reachable point for the robot 
    and the connected area to the desired position.

    Args
        - **l1** the list of string from querying the *connectedTo* data property of the actual robot position
        - **l2** the list of string from querying the *connectedTo* data property of the desired robot location
        - **robPos** the actual robot position in the moment the function is called for checking operation
    
    Returns
        - **shared_location** the shared location between the two lists
    """

    global shared_location

    for i in listNow:
        for j in listDes:
            if i == j :
                shared_location = i
            elif robPos == j:
                shared_location = robPos
            else:
                shared_location = ''

    return shared_location

def time_set(timeList):
    """
    Function to rewrite the queried timestamp for both Rooms and Robot's data property.

    Args
        - **time_list** of queried objects section of the Armor service message
    
    Returns
        the integer value between the double quotes of the OWL, saved as list of string
    
    """
    timestamp = ''
    for i in timeList:
        for element in range(1, 11):
            timestamp=timestamp+i[element]
    return timestamp

def rooms_search(roomList):
    """
    Function to rewrite the queried data property list of the room, saving in the returned list as separate strings

    Args
        - **roomList** of queried objects section of the Armor service message

    Returns
        - **location_list** the list of strings of locations
    
    """
    location_list = []
    for i in roomList:
        
        if "R1" in i:
            location_list.append('R1') # room 1
        elif "R2" in i:
            location_list.append('R2') # room 2
        elif "R3" in i:
            location_list.append('R3') # room 3
        elif "R4" in i:
            location_list.append('R4') # room 4
        elif "C1" in i:
            location_list.append('C1') # corridor 1
        elif "C2" in i:
            location_list.append('C2') # corridor 2
        elif "E" in i:
            location_list.append('E') # special room for recharging
    return location_list

def robot_location(isInList):
    """
    Function to extract the element of the queried *isIn* robot's object property.

    Args
        - **isInList** of queried objects section of the Armor service message
        
    Returns
        - **location** the string containing the location name in which the Robot is actually in
    
    """
    for i in isInList:
        if "R1" in i:
            return 'R1'
        elif "R2" in i:
            return'R2'
        elif "R3" in i:
            return'R3'
        elif "R4" in i:
            return'R4'
        elif "C1" in i:
            return'C1'
        elif "C2" in i:
            return'C2'
        elif "E" in i:
            return'E'

def urgency():
    """
    Function that recursively checks on the *urgent_rooms* list if there are some URGENT rooms.
    If the list is empty the flag *urgency_status* is set to 0 (False), otherwise if there are some urgent rooms the flag will be set to 1.

    Returns
        - **urgent_rooms** list of the urgent rooms, modified in the :mod:`Surveillance` status.
    """

    global urgency_status
    global urgent_rooms

    client = ArmorClient("example", "ontoRef")
    client.call('REASON','','',[''])
    
    # query the ontology to check if there are some URGENT instances
    query_urgent = client.call('QUERY','IND','CLASS',['URGENT'])
    urgent_rooms = rooms_search(query_urgent.queried_objects) 

    if urgent_rooms == []:
        urgency_status = 0
    else:
        urgency_status = 1
        
    return urgent_rooms

def motion_control(robPos, desPos):
    """
    Function used when the Robot has to change its position in the environment.
    
    This is the main function that define the motion of the robot in the map and takes the help of three other functions 
        - :mod:`rooms_search`; 
        - :mod:`path_planning`; 
        - :mod:`time_set`.

    Args
        - **robPos** is the actual robot position when this function is called
        - **desPos** is the desired robot position
    
    Returns
        - **robot_position** the updated robot position at the end of the movement, when the location is changed

    """

    global shared_location
    global urgent_rooms
    global robot_position
    
    # client commands of the armor_api
    client = ArmorClient("example", "ontoRef")
    client.call('REASON','','',[''])
    
    # query the ontology about the room's object property `connectedTo` (spatial information), for the actual robot position and the desided one
    robot_connections = client.call('QUERY','OBJECTPROP','IND',['connectedTo', robPos])
    arrival_connections = client.call('QUERY','OBJECTPROP','IND',['connectedTo', desPos])
    
    # extract from the queries the robot possible moves (spatial information)
    robot_possible_moves = rooms_search(robot_connections.queried_objects)
    arrival_moves = rooms_search(arrival_connections.queried_objects)

    # For debugging uncomment the two lines below
    #print('Actual robot position, possible moves: ', robot_possible_moves)
    #print('Desired robot position, possible moves: ', arrival_moves)

    # Find a possible path in the shared location between the possible moves of the actual and desired positions
    shared_location = path_planning(robot_possible_moves, arrival_moves, robPos)

    # For debugging uncomment the line below
    #print('The shared location is: ', shared_location)
    
    # query the subclasses of the locations (ROOM, URGENT, CORRIDOR) from the ontology
    query_locations = client.call('QUERY','CLASS','IND',[desPos, 'true'])
    subclass_location = query_locations.queried_objects

    # For debugging uncomment the line below
    #print('The subclass of the location is: ', subclass_location)
    
    if subclass_location == ['URGENT'] or subclass_location == ['ROOM']:
        if shared_location == '':
            # if there are not shared location between the actual robot position and the desired one in the map, 
            # e.g. the robot is in R2 and R3 becomes urgent
            if 'C1' in robot_possible_moves:
                # replace the robot in the corridor C1 if C1 is in the robot possible moves list,
                # e.g. if the robot is in R2, C1 is a element of the robot possible moves list
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', robPos, 'C1'])
                client.call('REASON','','',[''])
            elif 'C2' in robot_possible_moves:
                # replace the robot in the corridor C2 if C2 is in the robot possible moves list,
                # e.g. if the robot is in R3, C2 is a element of the robot possible moves list
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', robPos, 'C2'])
                client.call('REASON','','',[''])
            
            # query the ontology about the position of the robot in the environment and extract the information with the function robot_location
            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = robot_location(query_position.queried_objects)

            #print('Robot position updated: ', robot_position) # For debugging uncomment this line
            
            # query the ontology about the room's object property `connectedTo` (spatial information), update the information about the possible moves
            robot_connections = client.call('QUERY','OBJECTPROP','IND',['connectedTo', robot_position])
            robot_possible_moves = rooms_search(robot_connections.queried_objects)
            # update the path of the shared location between the possible moves of the actual and desired positions
            shared_location= path_planning(robot_possible_moves, arrival_moves, robot_position)
            
            # print('Shared locations updated: ', shared_location) # For debugging uncomment this line

            # replace the robot position with the shared location
            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', shared_location, robPos])
            client.call('REASON','','',[''])
            # replace the shared position with the desired one
            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', desPos, shared_location])
            client.call('REASON','','',[''])
            
            # query the ontology about the position of the robot in the environment and extract the information with the function robot_location
            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = robot_location(query_position.queried_objects)
            
            #print('New robot position updated: ', robot_position) # For debugging uncomment this line
            
            rospy.sleep(wait_time)

            # updating all the timing information of the robot's and room's data properties `now` and `visitedAt` respectively
            # `now` data property
            rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
            old_rob_time = time_set(rob_time.queried_objects)
            # compute the current time
            current_time=str(math.floor(time.time()))
            client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
            client.call('REASON','','',[''])
            # `visitedAt` data property
            room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', desPos])
            old_room_time = time_set(room_time.queried_objects)
            client.call('REPLACE','DATAPROP','IND',['visitedAt', desPos, 'Long', current_time, old_room_time])
            client.call('REASON','','',[''])
        
        elif shared_location == robPos:
            # if a shared location is the actual robot position
            # replace the shared position with the desired one
            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', desPos, shared_location])
            client.call('REASON','','',[''])
            # update the robot position in the ontology and extract the information
            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = robot_location(query_position.queried_objects)

            rospy.sleep(wait_time)

            # updating all the timing information of the robot's and room's data properties `now` and `visitedAt` respectively
            # `now` data property
            rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
            old_rob_time = time_set(rob_time.queried_objects)
            current_time=str(math.floor(time.time()))
            client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
            client.call('REASON','','',[''])
            # `visitedAt` data property
            room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', desPos])
            old_room_time = time_set(room_time.queried_objects)
            client.call('REPLACE','DATAPROP','IND',['visitedAt', desPos, 'Long', current_time, old_room_time])
            client.call('REASON','','',[''])

        else:
            # if there is a location that is shared between the two list (see `path_planning` function) 
            # replace the robot position with the shared location
            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', shared_location, robPos])
            client.call('REASON','','',[''])
            # replace the shared position with the desired one
            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', desPos, shared_location])
            client.call('REASON','','',[''])
            # update the robot position in the ontology and extract the information
            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = robot_location(query_position.queried_objects)

            rospy.sleep(wait_time)

            # updating all the timing information of the robot's and room's data properties `now` and `visitedAt` respectively
            # `now` data property
            rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
            old_rob_time = time_set(rob_time.queried_objects)
            # compute the current time
            current_time=str(math.floor(time.time()))
            client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
            client.call('REASON','','',[''])
            # `visitedAt` data property
            room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', desPos])
            old_room_time = time_set(room_time.queried_objects)
            client.call('REPLACE','DATAPROP','IND',['visitedAt', desPos, 'Long', current_time, old_room_time])
            client.call('REASON','','',[''])
        
        return robot_position
    
    else:
        # if the desired position is not a URGENT subclass but is a CORRIDOR or the special room E
        if shared_location == '':
            # if there are not shared location between the actual robot position and the desired one in the map 
            # e.g. the robot is in R2 and the desired location is C2
            if 'C1' in robot_possible_moves:
                # replace the robot in the corridor C1 if C1 is in the robot possible moves list,
                # e.g. if the robot is in R2, C1 is a element of the robot possible moves list
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', robPos, 'C1'])
                client.call('REASON','','',[''])
            elif 'C2' in robot_possible_moves:
                # replace the robot in the corridor C2 if C2 is in the robot possible moves list,
                # e.g. if the robot is in R3, C2 is a element of the robot possible moves list
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', robPos, 'C2'])
                client.call('REASON','','',[''])
            
            # update the robot position in the ontology and extract the information
            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = robot_location(query_position.queried_objects)
            # replace the desired position with the actual updated
            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', desPos, robot_position])
            client.call('REASON','','',[''])
            # the final query is needed again to return the updated result
            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = robot_location(query_position.queried_objects)
        else:
            # if there is a shared location between the actual robot position and the desired one in the map 
            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', shared_location, robPos])
            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', desPos, shared_location])
            client.call('REASON','','',[''])
            # final query to return the updated robot's position
            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = robot_location(query_position.queried_objects)

        return robot_position    
    

class Build_world(smach.State):

    """
    Class that defines the *BUILD_WORLD* state, in which the code waits for the enviroment to be created.
    When the map is received and loaded the **world_loaded** boolean variable will be set to True, the robot will enter in 
    the Corridor and the outcome *TRANS_WORLD_DONE* will make the state end to switch to the next state *NO_EMERGENCY*.

    Args
        - **smachState** State base interface
    
    Returns
        - **TRANS_WAITING_MAP** transition that will keep the FSM active in this state
        - **TRANS_WORLD_DONE** transition condition that will make this state end and go to the new next state *NO_EMERGENCY*

    """

    def __init__(self):
        smach.State.__init__(self, outcomes=[TRANS_WORLD_DONE, 
                                             TRANS_WAITING_MAP,
                                             TRANS_BATTERY_LOW,
                                             TRANS_BATTERY_CHARGED,
                                             TRANS_URGENT_ROOM,
                                             TRANS_NO_URGENT_ROOM])
    
    def execute(self, userdata):

        global world_loaded
        global robot_position
        global pub_wb_sync

        # Subscriber of the world flag given by the world_generator_node
        rospy.Subscriber(anm.TOPIC_WORLD_LOAD, Bool, world_callback)
        # Publisher for the sync between world and battery
        pub_wb_sync = rospy.Publisher(anm.TOPIC_SYNC_WORLD_BATTERY, Bool, queue_size=10)

        rospy.sleep(wait_time)

        print("\n\n-------- Executing state BUILD WORLD --------\n")

        # Main Behaviuor of the state
        # If the world is not created yet, the battery node doesn't start its execution and the FSM remains in this state
        if world_loaded == False:
            print("Creating world ... \n")
            wb_sync = 0
            pub_wb_sync.publish(wb_sync)
            return TRANS_WAITING_MAP
        # if the world is created, the WORLD ONTOLOGY is loaded and start the sync with the battery node. The FSM goes to the next state.
        elif world_loaded == True:
            print("World created!")
            # client commands of the armor_api
            client = ArmorClient("example", "ontoRef") 
            client.call('LOAD','FILE','',[WORLD_ONTOLOGY_FILE_PATH, WEB_PATH, 'true', 'PELLET', 'false'])
            # manipulation of the Robot1, moved into corridor C1 from the intial room E
            client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', 'C1', anm.INIT_LOCATION)
            # Reasoning OWL
            client.call('REASON','','',[''])
            # Query from the ontology and find the actual robot position
            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = robot_location(query_position.queried_objects)
            # Output to visualize in the terminal the actual position of the robot
            print(f"{bcolors.STATUS}Actually the robot is in: {bcolors.ENDC}", robot_position)

            # Since the world is created and loaded correctly, set and publish the sync flag
            wb_sync = 1
            pub_wb_sync.publish(wb_sync)
            
            return TRANS_WORLD_DONE

class No_emergency(smach.State):
    """
    Class that defines the *NO_EMERGENCY* state.
    The robot will stay in the corridors until any room becomes *URGENT*. The logic is:
        - the robot will start in one corridor;
        - if no room is URGENT, the robot stays in the corridor 3 seconds and change to the other corridor with the function ``motion_control(robPos, desPos)`` (no transition to the other states);
        - if one or more rooms becomes URGENT, the FSM goes to the state *SURVEILLANCE*.

    **Important**: the state gets notified by the :mod:`battery_callback` which is the callback of the topic `/battery_signal` with higher priority. 
    This interrupt any other action performed in the other states.
    
    Args
        - **smachState** State base interface
    
    Returns
        - **TRANS_BATTERY_LOW** transition that change status going to the *Recharging*
        - **TRANS_URGENT_ROOM** transition that will make this state end and go to the *Surveillance*
        - **TRANS_NO_URGENT_ROOM** loop transition that will keep the FSM active in this state

    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=[TRANS_WORLD_DONE, 
                                             TRANS_WAITING_MAP,
                                             TRANS_BATTERY_LOW,
                                             TRANS_BATTERY_CHARGED,
                                             TRANS_URGENT_ROOM,
                                             TRANS_NO_URGENT_ROOM])

    def execute(self, userdata):

        global urgent_rooms
        global robot_position
        global time_threshold

        rospy.sleep(wait_time)

        print("\n\n-------- Executing state NO EMERGENCY --------\n")

        # client commands of the armor_api
        client = ArmorClient("example", "ontoRef")
        # Reasoning OWL
        client.call('REASON','','',[''])
        # Query from the ontology and find the actual robot position
        query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        robot_position = robot_location(query_position.queried_objects)
        # Output to visualize in the terminal the actual position of the robot
        print(f"{bcolors.STATUS}Actually the robot is in: {bcolors.ENDC}", robot_position)


        # Procedure for checking if there are some urgent rooms as long as the robot is in the corridors
        locations_list = ['R1', 'R2', 'R3', 'R4']
                
        for room in locations_list:
            # compute the current time
            current_time=str(math.floor(time.time()))
            # query the ontology about the room's object property `visitedAt` (time information), extract in old_room_rime
            room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', room])
            old_room_time = time_set(room_time.queried_objects)
            
            # cast the time values (string) to integer values and check if the threshold has been exceeded
            # if yes, the transition to the urgent rooms is returned
            if (int(current_time)-int(old_room_time)) > time_threshold:
                print(f"\n{bcolors.URGENT}Warning: There are some urgent rooms!{bcolors.ENDC}")
                # query the ontology about the robot's object property `now` (time information), extract in old_rob_time
                rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
                old_rob_time = time_set(rob_time.queried_objects)
                # compute the current time
                current_time=str(math.floor(time.time()))
                # replace the old_rob_time with the new value (current_time) of the robot's object property `now`
                client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
                # Reasoning OWL
                client.call('REASON','','',[''])
                # query the ontology about the room's object property `visitedAt` (time information), extract in old_room_rime
                room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', room])
                old_room_time = time_set(room_time.queried_objects)
                # Reasoning OWL
                client.call('REASON','','',[''])

                # list of the urgent rooms
                urgent_rooms = urgency()
                random.shuffle(urgent_rooms)
                # Output to visualize in the terminal the urgent rooms
                print(f"{bcolors.URGENT}The urgent rooms are: {bcolors.ENDC}", urgent_rooms)

                return TRANS_URGENT_ROOM

        # Main Behaviuor of the state
        if battery_status == 1: # battery conditions check, that task has the higher priority
            if urgency_status == 0 : # second condition, check if there are some urgent rooms
                if robot_position == 'C1':
                    print(f"{bcolors.MOVING}The Robot is in C1, should go in C2{bcolors.ENDC}\n")

                    rospy.sleep(3)

                    # call function to change the position of the robot (from actual position to the desired one)
                    motion_control(robot_position, 'C2')
                    client.call('REASON','','',[''])
                    
                    return TRANS_NO_URGENT_ROOM
                
                elif robot_position == "C2":
                    print(f"{bcolors.MOVING}The Robot is in C2, should go in C1{bcolors.ENDC}\n")

                    rospy.sleep(3)
                    
                    # call function to change the position of the robot (from actual position to the desired one)
                    motion_control(robot_position, 'C1')
                    client.call('REASON','','',[''])
                    
                    return TRANS_NO_URGENT_ROOM
                else:
                    motion_control(robot_position, 'C1')
                    return TRANS_NO_URGENT_ROOM                
            else:
                # there are some urgent rooms, go to the SURVEILLANCE state
                return TRANS_URGENT_ROOM
        else :
            # the battery charge is low, go to the RECHARGING state
            print(f"{bcolors.BATTERY}Warning: the battery charge is low!{bcolors.ENDC}\n")
            return TRANS_BATTERY_LOW

class Recharging(smach.State):
    """
    Class that defines the *RECHARGING* state.
    The battery charge is low, advertised by :mod:`battery_callback`, which will modify the variable flag *battery_status*
    
    Args
        - **smachState** State base interface
    
    Returns
        - **TRANS_BATTERY_CHARGED** transition that changes status, going back to the *NO_EMERGENCY* state
        - **TRANS_BATTERY_LOW** loop transition that will keep the FSM active in this state as long as the battery becomes fully recharged

    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=[TRANS_WORLD_DONE, 
                                             TRANS_WAITING_MAP,
                                             TRANS_BATTERY_LOW,
                                             TRANS_BATTERY_CHARGED,
                                             TRANS_URGENT_ROOM,
                                             TRANS_NO_URGENT_ROOM])

    def execute(self, userdata):

        global robot_position
        
        # client commands of the armor_api
        client = ArmorClient("example", "ontoRef")
        # Reasoning OWL
        client.call('REASON','','',[''])

        print("\n\n-------- Executing state RECHARGING --------\n")
        
        query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        robot_position = robot_location(query_position.queried_objects)

        rospy.sleep(wait_time)
        
        # Main Behaviuor of the state
        if battery_status == 0:
            print(f"{bcolors.BATTERY}Battery recharging ... {bcolors.ENDC}\n")
            # call function to change the position of the robot (from actual position to the recharging room, E)
            motion_control(robot_position, anm.CHARGE_LOCATION)
            client.call('REASON','','',[''])
            
            return TRANS_BATTERY_LOW
        else:
            print(f"{bcolors.BATTERY}Battery charging complete! {bcolors.ENDC}\n")
            # the battery charge is fully recharged, go to the NO_EMERGENCY state
            return TRANS_BATTERY_CHARGED

class Surveillance(smach.State):
    """
    Class that defines the *SURVEILLANCE* state. Only when the *urgency_status* variable is 1 the robot must visit the urgency rooms.
    It calls the :mod:`motion_control(robPos, desPos)` function to move the robot and gets advertised by the :mod:`battery_callback` in case of battery low.

    Args
        - **smachState** State base interface
    
    Returns

        - **TRANS_BATTERY_LOW** transition that change status to the *RECHARGING* state
        - **TRANS_NO_URGENT_ROOM** transition that specifies that no other room are urgent anymore and changes status to *NO_EMERGENCY* state
        - **TRANS_URGENT_ROOM** loop transition that will keep the FSM active in this state
    """
    
    
    def __init__(self):
        smach.State.__init__(self, outcomes=[TRANS_WORLD_DONE, 
                                             TRANS_WAITING_MAP,
                                             TRANS_BATTERY_LOW,
                                             TRANS_BATTERY_CHARGED,
                                             TRANS_URGENT_ROOM,
                                             TRANS_NO_URGENT_ROOM])
        
    def execute(self, userdata):

        global urgency_status
        global urgent_rooms
        global robot_position

        # client commands of the armor_api
        client = ArmorClient("example", "ontoRef")
        # reasoning OWL
        client.call('REASON','','',[''])

        print("\n\n-------- Executing state SURVEILLANCE --------\n")

        # list of the urgent rooms
        urgent_rooms = urgency()
        random.shuffle(urgent_rooms)
        # Output to visualize in the terminal the urgent rooms
        print(f"{bcolors.URGENT}The urgent rooms are: {bcolors.ENDC}", urgent_rooms)

        rospy.sleep(wait_time)
        
        # Main Behaviuor of the state
        if battery_status == 0:  
            # the battery charge is low
            print(f"{bcolors.BATTERY}Warning: the battery charge is low!{bcolors.ENDC}")
            return TRANS_BATTERY_LOW
        
        elif urgency_status == 0:
            # there are not urgent rooms 
            return TRANS_NO_URGENT_ROOM
        
        elif urgency_status == 1:
            # there are some urgent rooms
            for i in urgent_rooms :
                # check which are the urgent rooms in the list that must be surveillanced by the robot
                if "R1" in i:
                    query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                    robot_position = robot_location(query_position.queried_objects)
                    client.call('REASON','','',[''])
                    new_pose = motion_control(robot_position,'R1')
                    print(f"{bcolors.STATUS}The room surveillanced by the robot is: {bcolors.ENDC}", new_pose)
                    break
                
                elif "R2" in i:
                    query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                    robot_position = robot_location(query_position.queried_objects)
                    client.call('REASON','','',[''])
                    new_pose = motion_control(robot_position,'R2')
                    print(f"{bcolors.STATUS}The room surveillanced by the robot is: {bcolors.ENDC}", new_pose)
                    break
                
                elif "R3" in i:
                    query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                    robot_position = robot_location(query_position.queried_objects)
                    client.call('REASON','','',[''])
                    new_pose = motion_control(robot_position,'R3')
                    print(f"{bcolors.STATUS}The room surveillanced by the robot is: {bcolors.ENDC}", new_pose)
                    break
                
                elif "R4" in i:
                    query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                    robot_position = robot_location(query_position.queried_objects)
                    client.call('REASON','','',[''])
                    new_pose = motion_control(robot_position,'R4')
                    print(f"{bcolors.STATUS}The room surveillanced by the robot is: {bcolors.ENDC}", new_pose)
                    break
        
        return TRANS_URGENT_ROOM

def main():
    """
    Main function of the Finite State Machine that initializes the node *fsm_behaviour* using SMACH modules.
    This method create the FSM and specifies the states with the relative transitions.
    Since the behaviour of the battery has higher priority with respect to all the other states, here it's initialized
    the subscriber of the battery topic.

    """
    # Before to initialize the ROS node waits some time to allow the `world_generator` node to 
    # create the environment of the world from the ontology (OWL)
    rospy.sleep(wait_time)
    # ROS Node initialization
    #rospy.init_node('fsm_behaviour')
    rospy.init_node(anm.NODE_STATE_MACHINE, log_level=rospy.INFO)
    
    # Create the Finite State Machine
    sm = smach.StateMachine(outcomes=['interface'])
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(STATE_BUILD_WORLD, Build_world(), 
                                transitions={TRANS_WORLD_DONE : STATE_NO_EMERGENCY, 
                                             TRANS_WAITING_MAP : STATE_BUILD_WORLD,
                                             TRANS_BATTERY_LOW : STATE_BUILD_WORLD,
                                             TRANS_BATTERY_CHARGED : STATE_BUILD_WORLD,
                                             TRANS_URGENT_ROOM : STATE_BUILD_WORLD,
                                             TRANS_NO_URGENT_ROOM : STATE_BUILD_WORLD})
        
        smach.StateMachine.add(STATE_NO_EMERGENCY, No_emergency(), 
                                transitions={TRANS_WORLD_DONE : STATE_NO_EMERGENCY, 
                                             TRANS_WAITING_MAP : STATE_NO_EMERGENCY,
                                             TRANS_BATTERY_LOW : STATE_RECHARGING,
                                             TRANS_BATTERY_CHARGED : STATE_NO_EMERGENCY,
                                             TRANS_URGENT_ROOM : STATE_SURVEILLANCE,
                                             TRANS_NO_URGENT_ROOM : STATE_NO_EMERGENCY})
        
        smach.StateMachine.add(STATE_RECHARGING, Recharging(), 
                                transitions={TRANS_WORLD_DONE : STATE_RECHARGING, 
                                             TRANS_WAITING_MAP : STATE_RECHARGING,
                                             TRANS_BATTERY_LOW : STATE_RECHARGING,
                                             TRANS_BATTERY_CHARGED : STATE_NO_EMERGENCY,
                                             TRANS_URGENT_ROOM : STATE_RECHARGING,
                                             TRANS_NO_URGENT_ROOM : STATE_RECHARGING})
        
        smach.StateMachine.add(STATE_SURVEILLANCE, Surveillance(), 
                                transitions={TRANS_WORLD_DONE : STATE_SURVEILLANCE, 
                                             TRANS_WAITING_MAP : STATE_SURVEILLANCE,
                                             TRANS_BATTERY_LOW : STATE_RECHARGING,
                                             TRANS_BATTERY_CHARGED : STATE_SURVEILLANCE,
                                             TRANS_URGENT_ROOM : STATE_SURVEILLANCE,
                                             TRANS_NO_URGENT_ROOM : STATE_NO_EMERGENCY})
    
    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Subscribers to the topic `battery_signal`
    rospy.Subscriber(anm.TOPIC_BATTERY_SIGNAL, Bool, battery_callback) # battery flag (battery_status)

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__': 
    main()
