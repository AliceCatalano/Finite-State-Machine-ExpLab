#!/usr/bin/env python

"""
.. module:: finite_state_machine
   :platform: Unix
   :synopsis: Python code that defines the finite state machine for the robot
.. moduleauthor:: Alice Maria Catalano <s5157341@studenti.unige.it>

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
import rospy
import smach
import smach_ros
from armor_api.armor_client import ArmorClient
from std_msgs.msg import Bool

pause_time= 1.5         #global variable for the sleeping time
battery_status = 1      # Battery is charged
urgency_status = 0      # Battery flag for the urgent room
loading = True          # True map is not loaded, False map is loaded
shared_connection = ''  # String resulting from the connectedTo data property
are_urgent=''           # String resulting from the query of the individuals in the URGENT class
robot_position=''       # String that contains always one element, which is the robot position in that moment
timestamp = ''          # String that represent the queried timestamp

def callback(data):
    """ 
    Callback function for the map publisher */loader*, that modifies the value of the **global variable loading** and will let the code start.
    
    """
    global loading
    if data.data == True:
        loading = True

    elif data.data == False:
        loading = False

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
    print('the urgent rooms in urgency: ', are_urgent)
    if are_urgent == []:
        urgency_status =0
    else:
        urgency_status =1
        
    return are_urgent

def change_position(robPos, desPos):
    """
    Function used everytime the Robot has to change position, taking into consideration different behaviors based on the different LOCATION's subclasses

        - **ROOM** if the robot moves to a Room the time stap update is taken into cosideration 
        - **URGENT** the behavior is the same as in the above case 
        - **CORRIDOR** the robot will calculate the path to go into the corridor and advertise the State :mod:`Corridor_cruise`
    
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
    
    
    shared_connection= find_path(robot_possible_moves, arrival_moves, robPos)
    
    isRoom = client.call('QUERY','CLASS','IND',[desPos, 'true'])
    is_Room= isRoom.queried_objects
    
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
            
            robot_connections = client.call('QUERY','OBJECTPROP','IND',['connectedTo', robot_position])
            robot_possible_moves = find_list(robot_connections.queried_objects)

            shared_connection= find_path(robot_possible_moves, arrival_moves, robot_position)

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

class Load_map(smach.State):

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
        smach.State.__init__(self, outcomes=['waiting_map','uploaded_map'])
    
    def execute(self, userdata):
        global loading
        rospy.sleep(pause_time)
        
        if loading == True:
            return 'waiting_map'
        else:
            client = ArmorClient("example", "ontoRef") 
            client.call('LOAD','FILE','',['/root/ros_ws/src/assignment_1/topological_map/my_map2.owl', 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
            
            client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', 'C1', 'E')
            client.call('REASON','','',[''])
            
            return 'uploaded_map'

class Corridor_cruise(smach.State):
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
        smach.State.__init__(self, outcomes=['battery_low','urgent_room', 'stay_in_corridor'])

    def execute(self, userdata):
        global are_urgent
        global robot_position
        rospy.sleep(pause_time)
        
        client = ArmorClient("example", "ontoRef")
        client.call('REASON','','',[''])
        
        query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        robot_position = find_individual(query_position.queried_objects)
        
        
        are_urgent = urgent_rooms()
        print('the urgent rooms are : ', are_urgent )

        if battery_status == 1:
            # The outher condition checks on battery, because that task has the highr priority
            if urgency_status == 0 :
                # The second condition to check on is the urgent rooms
                if robot_position == 'C1' :
                    print('the robot is in C1 should go in C2')
                    rospy.sleep(pause_time)

                    change_position(robot_position, 'C2')
                    client.call('REASON','','',[''])
                    
                    return 'stay_in_corridor'
                
                elif robot_position == "C2"  :
                    print('the robot is in C2 should go in C1')
                    rospy.sleep(pause_time)
                    
                    change_position(robot_position, 'C1')
                    client.call('REASON','','',[''])
                    
                    return 'stay_in_corridor'
                else:
                    change_position(robot_position, 'C1')
                    return 'stay_in_corridor'
            else:
                return 'urgent_room'
        else :
            return 'battery_low'

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
        smach.State.__init__(self, outcomes=['on_charge', 'full_battery'])

    def execute(self, userdata):
        global robot_position
        
        client = ArmorClient("example", "ontoRef")
        client.call('REASON','','',[''])
        
        query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        robot_position = find_individual(query_position.queried_objects)

        rospy.sleep(pause_time)
        
        if battery_status == 0:
            change_position(robot_position, 'E')
            client.call('REASON','','',[''])
            
            return 'on_charge'
        else:
            return 'full_battery'

class Room_visiting(smach.State):
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
        smach.State.__init__(self, outcomes=['no_urgent_room', 'stay_in_room','battery_low'])
        
    def execute(self, userdata):
        global urgency_status
        global are_urgent
        global robot_position

        client = ArmorClient("example", "ontoRef")
        client.call('REASON','','',[''])

        are_urgent = urgent_rooms()
        
        print('the urgent rooms in room visiting are: ', are_urgent)
        rospy.sleep(pause_time) 
        
        if urgency_status == 0:
            return 'no_urgent_room'
        
        elif battery_status == 0:
            return 'battery_low'
        
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
        return 'stay_in_room'

def main():
    """
    This is the main function that initializes the node *finite_state_machine*, create the SMACH state machine and specifies the states with the transitions.
    Also initializes the subscription to the publishers.

    """
    rospy.sleep(pause_time)
    rospy.init_node('finite_state_machine')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Interface'])
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('LOAD_MAP', Load_map(), 
                               transitions={'uploaded_map':'CORRIDOR_CRUISE', 'waiting_map':'LOAD_MAP'})
        smach.StateMachine.add('CORRIDOR_CRUISE', Corridor_cruise(), 
                                transitions={'battery_low' : 'RECHARGING', 'urgent_room': 'ROOM_VISITING', 'stay_in_corridor' : 'CORRIDOR_CRUISE' })
        smach.StateMachine.add('RECHARGING', Recharging(), 
                                transitions={'on_charge':'RECHARGING', 'full_battery':'CORRIDOR_CRUISE'})
        smach.StateMachine.add('ROOM_VISITING', Room_visiting(), 
                                transitions={'no_urgent_room':'CORRIDOR_CRUISE', 'stay_in_room' : 'ROOM_VISITING', 'battery_low': 'RECHARGING' })
    
        # Execute SMACH plan

    # Execute SMACH plan
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    rospy.Subscriber("loader", Bool, callback)
    rospy.Subscriber("battery_signal", Bool, callback_batt)
    outcome = sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__': 
    main()
