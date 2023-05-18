#!/usr/bin/env python

"""
.. module:: fsm_behaviour
   :platform: Unix
   :synopsis: Python code that defines the finite state machine for the robot
.. moduleauthor:: Thomas Campagnolo <s5343274@studenti.unige.it>

This is finite state machine that uses the `Smach tool <https://wiki.ros.org/smach>`_ to implement it in ROS. 
It menages the behavior  of a surveillance  robot, that stays mostly in corridors and goes to the rooms just when they turn **urgent** after 7 seconds of not visiting them 

Subscribes  to:
    - /loader a Boolean flag to communicate when the map is totally created.
    - /battery_signal a Boolean flag to communicate when battery is low and when is totally charged

"""
import roslib
import rospkg
import time
import math
import random
import os
import rospkg
import rospy
import smach
import smach_ros
from armor_api.armor_client import ArmorClient
from std_msgs.msg import Bool

# States of the Finite State Machine.
STATE_BUILD_WORLD = 'BUILD_WORLD'       # State where the environment is build using the ontology.
STATE_NO_EMERGENCY = 'NO_EMERGENCY'     # Status of non-urgency room, the robot only moves between the corridors.
STATE_SURVEILLANCE = 'SURVEILLANCE'     # State of urgency room, the robot must visit the urgency locations.
STATE_RECHARGING = 'RECHARGING'         # State where the robot recharges its battery.

# Transitions of the Finite State Machine.
TRANS_WORLD_DONE = 'world_done'             # Transition that identifies the correct loading of the map
TRANS_WAITING_MAP = 'waiting_map'           # Transition to check if the map is uploaded correctly.
TRANS_BATTERY_LOW = 'battery_low'           # Transition for the low battery charge. The robot needs to recharge.
TRANS_BATTERY_CHARGED = 'battery_charged'   # Transition notifying full battery charge.
TRANS_URGENT_ROOM = 'urgent_room'           # Transition for room surveillance. 
TRANS_NO_URGENT_ROOM = 'no_urgent_room'     # Transition to notify that there are not urgent rooms.


pause_time= 1.5         # Global variable for the sleeping time
battery_status = 1      # Battery flag, 1 means that it is charged
urgency_status = 0      # Flag for the urgent room
world_loaded = False    # World flag, False when the map is not loaded, True when map is loaded correctly

world_done = 0

shared_connection = ''  # String resulting from the connectedTo data property
are_urgent=''           # String resulting from the query of the individuals in the URGENT class
robot_position=''       # String that contains always one element, which is the robot position in that moment
timestamp = ''          # String that represent the queried timestamp


rp = rospkg.RosPack()
assignment_path = rp.get_path('exprob_surveillance')
ONTOLOGY_FILE_PATH = os.path.join(assignment_path, "topological_map", "topological_map.owl")
WORLD_ONTOLOGY_FILE_PATH = os.path.join(assignment_path, "topological_map", "world_surveillance.owl")
WEB_PATH = 'http://bnc/exp-rob-lab/2022-23'

class bcolors:
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

def callback_batt(data):
    """ 
    Callback function for the map publisher */battery_signal*, that modifies the value of the **global variable battery_status** and will let the code start.
    
    """
    global battery_status
    if data.data == 0:
        battery_status = 0

    elif data.data == 1:
        battery_status =1

def find_path(l1, l2, robPos):
    """
    This function is needed to plan the path from one position to another, recursively  scanning the list of reachable point for the robot and the connected area to the desired position

    Args
        - **l1** the list resulting from querying the *connectedTo* data property of the robot position, returning a list of strings

        - **l2** the list resulting from querying the *connectedTo* data property of the desired location, returning a list of strings

        - **robPos** the robot position in the moment the function is called to check if the robot position appears in the *connectedTo* list of the desired location
    
    Returns
        - **shared_connection** the shared variable that will be checked in the calling function :mod:`change_position`
    """
    global shared_connection

    for i in l1:
        for j in l2:
            if i == j :
                shared_connection = i
            elif robPos == j:
                shared_connection=robPos
            else:
                shared_connection=''
    return shared_connection

def find_time(list):
    """
    Function to rewrite the queried time stamp, deleting the not integer part of the string, for both Rooms and Robot's data property

    Args
        - **list** of queried objects section of the Armor service message
    
    Returns
        all the element between the double quotes
    
    """
    timestamp = ''
    for i in list:
        for element in range(1, 11):
            timestamp=timestamp+i[element]
    return timestamp

def find_list(list):
    """
    Function to rewrite the queried *connectedTo* data property list, extracting each element and saving it in the returned list as separate strings

    Args
        - **list** of queried objects section of the Armor service message

    Returns
        - **position_list** the list of strings of locations
    
    """
    position_list = []
    for i in list:
        
        if "R1" in i:
            position_list.append('R1')
        elif "R2" in i:
            position_list.append( 'R2')
        elif "R3" in i:
            position_list.append('R3')
        elif "R4" in i:
            position_list.append( 'R4')
        elif "C1" in i:
            position_list.append('C1')  
        elif "C2" in i:
            position_list.append('C2')
        elif "E" in i:
            position_list.append( 'E')
    return position_list

def find_individual(list):
    """
    Function to rewrite the queried *isIn* Robot's object property, extracting the element and saving it in the returned string

    Args
        - **list** of queried objects section of the Armor service message
        
    Returns
        - **location** the string containing the location name in which the Robot is in
    
    """
    for i in list:
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

def urgent_rooms():
    """
    Function that recursively checks on the *are_urgent* list and if the list is empty changes the flag *urgency_status* value to false

    Returns
        - **are_urgent** list of the urgent rooms, to be modified in the :mod:`Room_visiting` status.
    """

    global urgency_status
    global are_urgent

    client = ArmorClient("example", "ontoRef")
    client.call('REASON','','',[''])
    
    urgent_list=client.call('QUERY','IND','CLASS',['URGENT'])
    are_urgent = find_list(urgent_list.queried_objects) 
    #print('the urgent rooms in urgency: ', are_urgent)
    if are_urgent == []:
        urgency_status = 0
    else:
        urgency_status = 1
        
    return are_urgent

def change_position(robPos, desPos):
    """
    Function used everytime the Robot has to change position, taking into consideration different behaviors based on the different LOCATION's subclasses

        - **ROOM** if the robot moves to a Room the time stamp update is taken into cosideration 
        - **URGENT** the behavior is the same as in the above case 
        - **CORRIDOR** the robot will calculate the path to go into the corridor and advertise the State :mod:`No_emergency`
    
    This is the main fuction for motion and takes the help of three other functions :mod:`find_list`, :mod:`find_path`, :mod:`find_time`; all of them used to clean the queried string.
    
    Args

        - **robPos** is the robot position in the moment this function is called

        - **desPos** is the desired position that calls this function
    
    Returns

        - **robot_position** the updated robot position at the end of the location change
    """
    global shared_connection
    global are_urgent
    global robot_position
    
    client = ArmorClient("example", "ontoRef")
    client.call('REASON','','',[''])
    
    robot_connections = client.call('QUERY','OBJECTPROP','IND',['connectedTo', robPos])
    arrival_connections = client.call('QUERY','OBJECTPROP','IND',['connectedTo', desPos])
    
    robot_possible_moves = find_list(robot_connections.queried_objects)
    arrival_moves = find_list(arrival_connections.queried_objects)
    print('robot_possible_moves list: ', robot_possible_moves)
    print('arrival_moves list: ', arrival_moves)

    
    
    shared_connection= find_path(robot_possible_moves, arrival_moves, robPos)
    print('shared_connection: ', shared_connection)
    
    isRoom = client.call('QUERY','CLASS','IND',[desPos, 'true'])
    is_Room= isRoom.queried_objects
    print('is_Room: ', is_Room)
    
    if is_Room == ['URGENT'] or is_Room == ['ROOM']:
        
        
        if shared_connection == '':
            # Case in which the robot is in the other side of the map, i.e. is in R4 and R1 becomes urgent
            
            
            if 'C1' in robot_possible_moves:
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', robPos, 'C1'])
                client.call('REASON','','',[''])
            elif 'C2' in robot_possible_moves:
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', robPos, 'C2'])
                client.call('REASON','','',[''])
            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = find_individual(query_position.queried_objects)
            print('#####Robot position: ', robot_position)
            
            robot_connections = client.call('QUERY','OBJECTPROP','IND',['connectedTo', robot_position])
            robot_possible_moves = find_list(robot_connections.queried_objects)

            shared_connection= find_path(robot_possible_moves, arrival_moves, robot_position)
            print('shared_connection: ', shared_connection)

            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', shared_connection, robPos])
            client.call('REASON','','',[''])

            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', desPos, shared_connection])
            client.call('REASON','','',[''])

            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = find_individual(query_position.queried_objects)
            print('------Robot position: ', robot_position)

            rospy.sleep(pause_time)
            rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
            old_rob_time = find_time(rob_time.queried_objects)
            current_time=str(math.floor(time.time()))
            client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
            client.call('REASON','','',[''])
            
            room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', desPos])
            old_room_time = find_time(room_time.queried_objects)
            client.call('REPLACE','DATAPROP','IND',['visitedAt', desPos, 'Long', current_time, old_room_time])
            
            client.call('REASON','','',[''])
        
        elif shared_connection == robPos:
            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', desPos, shared_connection])
            client.call('REASON','','',[''])

            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = find_individual(query_position.queried_objects)

            rospy.sleep(pause_time)
            rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
            old_rob_time = find_time(rob_time.queried_objects)
            current_time=str(math.floor(time.time()))
            client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
            client.call('REASON','','',[''])
            
            room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', desPos])
            old_room_time = find_time(room_time.queried_objects)
            
            client.call('REPLACE','DATAPROP','IND',['visitedAt', desPos, 'Long', current_time, old_room_time])
            client.call('REASON','','',[''])


        else:
            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', shared_connection, robPos])
            client.call('REASON','','',[''])


            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', desPos, shared_connection])
            client.call('REASON','','',[''])


            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = find_individual(query_position.queried_objects)

            rospy.sleep(pause_time)
            rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
            old_rob_time = find_time(rob_time.queried_objects)
            current_time=str(math.floor(time.time()))
            client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
            client.call('REASON','','',[''])
            
            room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', desPos])
            old_room_time = find_time(room_time.queried_objects)
            
            client.call('REPLACE','DATAPROP','IND',['visitedAt', desPos, 'Long', current_time, old_room_time])
            client.call('REASON','','',[''])
        
        return robot_position
    
    else:
        #if the desired position is a corridor or room E
        if shared_connection == '':
            if 'C1' in robot_possible_moves:
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', robPos, 'C1'])
                client.call('REASON','','',[''])
            elif 'C2' in robot_possible_moves:
                client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', robPos, 'C2'])
                client.call('REASON','','',[''])
            
            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = find_individual(query_position.queried_objects)

            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', desPos, robot_position])
            client.call('REASON','','',[''])
            #the query is needed again to return the updated result
            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = find_individual(query_position.queried_objects)
        else:
            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', shared_connection, robPos])
            client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', desPos, shared_connection])
            client.call('REASON','','',[''])
            
            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = find_individual(query_position.queried_objects)
        return robot_position    
    
    return robot_position

class Build_world(smach.State):

    """
    Class that defines  the *Load_map* state, in which the code waits for the map,
    when the map is received the **loading** boolean variable will be False, the robot will enter in the Corridor and
    the outcome *uploaded_map* will make the state end to switch to the other state *Corridor_cruise*

    Args
        - **smachState** State base interface
    
    Returns
        - **waiting_map** transition condition that will keep this state active
        - **uploaded_map** transition condition that will make this state end and go to the new state

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
        global pub

        rospy.Subscriber("world_loading", Bool, world_callback) # world flag (world_loaded)

        pub = rospy.Publisher('world_done', Bool, queue_size=10)

        rospy.sleep(pause_time)
        
        if world_loaded == False:
            world_done = 0
            pub.publish(world_done)
            return TRANS_WAITING_MAP
        
        elif world_loaded == True:

            
            world_done = 1
            pub.publish(world_done)

            client = ArmorClient("example", "ontoRef") 
            client.call('LOAD','FILE','',[WORLD_ONTOLOGY_FILE_PATH, WEB_PATH, 'true', 'PELLET', 'false'])
            
            client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', 'C1', 'E')
            client.call('REASON','','',[''])

            query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robot_position = find_individual(query_position.queried_objects)
            print(f"{bcolors.MOVING}Robot moved into: {bcolors.ENDC}", robot_position)
            print(f"{bcolors.STATUS}Actually the robot is in: {bcolors.ENDC}", robot_position)
            
            return TRANS_WORLD_DONE

class No_emergency(smach.State):
    """
    Class that defines  the *Corridor_cruise* state, in which the main robot behavior is described.
    The surveillance  robot will cruise in the corridors until  any room gets *urgent*. The logic of the cruising is basic:
    the robot will start from one corridor, stay in it 2 seconds and change to the other corridor with the function ``change_position(robPos, desPos)``
    Moreover this state gets notified by the :mod:`callback_batt` and the :mod:`callback` which are the callback function of the publishers */loader* and */battery_status*
    
    Args
        - **smachState** State base interface
    
    Returns
        - **battery_low** transition condition that change status going to the *Recharging*
        - **urgent_room** transition condition that will make this state end and go to the new state
        - **stay_in_corridor** loop transition that will keep this state active

    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=[TRANS_WORLD_DONE, 
                                             TRANS_WAITING_MAP,
                                             TRANS_BATTERY_LOW,
                                             TRANS_BATTERY_CHARGED,
                                             TRANS_URGENT_ROOM,
                                             TRANS_NO_URGENT_ROOM])

    def execute(self, userdata):

        global are_urgent
        global robot_position
        rospy.sleep(pause_time)
        
        client = ArmorClient("example", "ontoRef")
        client.call('REASON','','',[''])
        
        query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        robot_position = find_individual(query_position.queried_objects)
        print(f"{bcolors.STATUS}Actually the robot is in: {bcolors.ENDC}", robot_position)


        
        loc_list = ['R1', 'R2', 'R3', 'R4']
                

        for room in loc_list:
            current_time=str(math.floor(time.time()))
            room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', room])
            old_room_time = find_time(room_time.queried_objects)
            
            if (int(current_time)-int(old_room_time))>20:
                print('THERE ARE SOME URGENT ROOMS!!!')
                rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
                old_rob_time = find_time(rob_time.queried_objects)
                current_time=str(math.floor(time.time()))
                client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
                client.call('REASON','','',[''])
                
                room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', room])
                old_room_time = find_time(room_time.queried_objects)
                
                #client.call('REPLACE','DATAPROP','IND',['visitedAt', 'R4', 'Long', old_room_time, old_room_time])
                client.call('REASON','','',[''])

                are_urgent = urgent_rooms()
                random.shuffle(are_urgent)
                print(f"{bcolors.URGENT}The urgent rooms are: {bcolors.ENDC}", are_urgent)

                return TRANS_URGENT_ROOM

        
        #are_urgent = urgent_rooms()
        #random.shuffle(are_urgent)
        

        #print('the urgent rooms are : ', are_urgent )

        if battery_status == 1:
            # The outher condition checks on battery, because that task has the highr priority
            if urgency_status == 0 :
             
                # The second condition to check on is the urgent rooms
                if robot_position == 'C1' :
                    print('the robot is in C1 should go in C2')
                    rospy.sleep(3.5)

                    change_position(robot_position, 'C2')
                    client.call('REASON','','',[''])
                    
                    return TRANS_NO_URGENT_ROOM
                
                elif robot_position == "C2"  :
                    print('the robot is in C2 should go in C1')
                    rospy.sleep(3.5)
                    
                    change_position(robot_position, 'C1')
                    client.call('REASON','','',[''])
                    
                    return TRANS_NO_URGENT_ROOM
                else:
                    change_position(robot_position, 'C1')
                    return TRANS_NO_URGENT_ROOM                
            else:
                return TRANS_URGENT_ROOM
        else :
            return TRANS_BATTERY_LOW

class Recharging(smach.State):
    """
    Class that defines  the *Recharging* state, which manages the behavior when the battery is low, getting advertised by :mod:`callback_batt`, which will modify the shared variable  *battery_status*
    
    Args
        - **smachState** State base interface
    
    Returns
        - **full_battery** transition condition that changes status, going back to the *Corridor_cruise*
        - **on_charge** transition condition that will keep this state active
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
        
        client = ArmorClient("example", "ontoRef")
        client.call('REASON','','',[''])
        
        query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        robot_position = find_individual(query_position.queried_objects)

        rospy.sleep(pause_time)
        
        if battery_status == 0:
            print("@@@@@@@@@@ BATTERY LOW! @@@@@@@")
            change_position(robot_position, 'E')
            client.call('REASON','','',[''])
            
            return TRANS_BATTERY_LOW
        else:
            return TRANS_BATTERY_CHARGED

class Surveillance(smach.State):
    """
    Class that defines  the *Room_visiting* state, which manages the behavior of the robot if it has to go to the room, so only when the *urgency_status* variable is True.
    It calls the :mod:`change_position(robPos, desPos)` function to move the robot and gets advertised by the :mod:`callback_batt in case of battery low.

    Args
        - **smachState** State base interface
    
    Returns

        - **battery_low** transition condition that change status going to the *Recharging* status

        - **no_urgent_room** transition condition that specifies that no other room are urgent anymore and changes status

        - **stay_in_room** transition condition that will keep this state active
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
        global are_urgent
        global robot_position

        client = ArmorClient("example", "ontoRef")
        client.call('REASON','','',[''])

        are_urgent = urgent_rooms()
        random.shuffle(are_urgent)
        
        print(f"{bcolors.MOVING}The urgent rooms are: {bcolors.ENDC}", are_urgent)

        
        # print('the urgent rooms in room visiting are: ', are_urgent)
        rospy.sleep(pause_time)
        
        if urgency_status == 0:
            return TRANS_NO_URGENT_ROOM
        
        elif battery_status == 0:
            return TRANS_BATTERY_LOW
        
        elif urgency_status == 1:
            
            for i in are_urgent :
                
                if "R1" in i:
                    query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                    robot_position = find_individual(query_position.queried_objects)
                    client.call('REASON','','',[''])
                    print('the robot was in ', robot_position)
                    new_pose = change_position(robot_position,'R1')
                    print('now the robot is in ', new_pose)
                    break
                
                elif "R2" in i:
                    query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                    robot_position = find_individual(query_position.queried_objects)
                    client.call('REASON','','',[''])
                    print('the robot was in ', robot_position)
                    new_pose = change_position(robot_position,'R2')
                    print('now the robot is in ', new_pose)
                    break
                
                elif "R3" in i:
                    query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                    robot_position = find_individual(query_position.queried_objects)
                    client.call('REASON','','',[''])
                    print('the robot was in ', robot_position)
                    new_pose = change_position(robot_position,'R3')
                    print('now the robot is in ', new_pose)
                    break
                
                elif "R4" in i:
                    query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
                    robot_position = find_individual(query_position.queried_objects)
                    client.call('REASON','','',[''])
                    print('the robot was in ', robot_position)
                    new_pose = change_position(robot_position,'R4')
                    print('now the robot is in ', new_pose)
                    break
        
        return TRANS_URGENT_ROOM

def main():
    """
    Main function of the Finite State Machine that initializes the node *fsm_behaviour* using SMACH modules.
    This method create the FSM and specifies the states with the relative transitions.
    Also initializes the subscription to the publishers.

    """
    # Before to initialize the ROS node waits some time to allow the `world_generator` node to create the environment of the world.
    rospy.sleep(pause_time)
    # ROS Node initialization
    rospy.init_node('fsm_behaviour')
    
    # Create the Finite State Machine
    sm = smach.StateMachine(outcomes=['Interface'])
    
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

    # Subscribers to the respective topics
    rospy.Subscriber("battery_signal", Bool, callback_batt) # battery flag (battery_status)

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__': 
    main()
