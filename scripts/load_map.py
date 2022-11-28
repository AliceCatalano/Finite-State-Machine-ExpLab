#!/usr/bin/env python

"""
.. module:: load_map
   :platform: Unix
   :synopsis: Python code to create the .owl ontology 
.. moduleauthor:: Alice Maria Catalano <s5157341@studenti.unige.it>

ROS Node to create the Ontology map adn initialzie the timestamps for the Rooms, which represents a 2D environment of 4 rooms and 3 corridors

Publishes to:
    - /loader a boolean flag to communicate when the map is totally created.

"""


import rospy
from armor_api.armor_client import ArmorClient
from std_msgs.msg import Bool, String
import time
import math

def find_time(list):
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
    It will publish a boolean that will be passed to the state ``Load_map``, advertised by :mod:`load_std_map`

    """
    pub = rospy.Publisher('loader', Bool, queue_size=10)
    rospy.init_node('loader_node', anonymous=True)
    pub.publish(True)

    print( " Load the standard map" )
    client = ArmorClient("example", "ontoRef")

    client.call('LOAD','FILE','',['/root/ros_ws/src/Finite-state-machine-Lab/assignment_1/topological_map/topological_map.owl', 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'E', 'D6'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'E', 'D7'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R1', 'D1'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R2', 'D2'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R3', 'D3'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'R4', 'D4'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D1'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D2'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D5'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C1', 'D6'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D3'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D4'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D5'])
    client.call('ADD','OBJECTPROP','IND',['hasDoor', 'C2', 'D7'])

   
    client.call('DISJOINT','IND','',['R1','R2','R3','R4','E','C1','C2','D1','D2','D3','D4','D5','D6','D7'])
    client.call('ADD','OBJECTPROP','IND',['isIn', 'Robot1', 'E'])
    client.call('REASON','','',[''])
    
    # The robot visits each room for the first time
    # Visit and timestamp creation for R1

    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'R1', 'E'])
    client.call('REASON','','',[''])
    
    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = find_time(rob_time.queried_objects)
    current_time=str(math.floor(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
   
    client.call('REASON','','',[''])

    client.call('ADD','DATAPROP','IND',['visitedAt','R1', 'Long', current_time])
    client.call('REASON','','',[''])
    rospy.sleep(2)

    # Visit and timestamp creation for R2

    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'R2', 'R1'])
    client.call('REASON','','',[''])
    
    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = find_time(rob_time.queried_objects)
    current_time=str(math.floor(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
    client.call('REASON','','',[''])

    client.call('ADD','DATAPROP','IND',['visitedAt','R2', 'Long', current_time])
    client.call('REASON','','',[''])
    rospy.sleep(2)

    # Visit and timestamp creation for R3

    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'R3', 'R2']) 
    client.call('REASON','','',[''])
    
    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = find_time(rob_time.queried_objects)
    current_time=str(math.floor(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
    client.call('REASON','','',[''])

    client.call('ADD','DATAPROP','IND',['visitedAt','R3', 'Long', current_time])
    client.call('REASON','','',[''])
    rospy.sleep(2)

    # Visit and timestamp creation for R4

    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'R4', 'R3'])  
    client.call('REASON','','',[''])
    
    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = find_time(rob_time.queried_objects)
    rospy.sleep(2)
    current_time=str(math.floor(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
    client.call('REASON','','',[''])

    client.call('ADD','DATAPROP','IND',['visitedAt','R4', 'Long', current_time])
    client.call('REASON','','',[''])
    rospy.sleep(2)
    
    client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'C1', 'R4'])
    client.call('SAVE','','',['/root/ros_ws/src/Finite-state-machine-Lab/assignment_1/topological_map/my_map2.owl'])
    pub.publish(False)

if __name__ == '__main__':

    """
    Entrance point of the code
    """ 

    try:
    	load_std_map()
    except rospy.ROSInterruptException:
    	pass
   