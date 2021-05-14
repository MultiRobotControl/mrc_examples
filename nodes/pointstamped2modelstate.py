#!/usr/bin/env python3
'''
Subscribes to PointStamped messages, converts to and publishes model state.
'''

import copy
import sys
import numpy as np

import rospy

from geometry_msgs.msg import PointStamped
from gazebo_msgs.msg import ModelState

class Node():
    def __init__(self, model_name, reference_frame):
        self.modelmsg = ModelState()
        self.modelmsg.model_name = model_name
        self.modelmsg.reference_frame = reference_frame
        self.outpub = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size=1)


    def point_callback(self,msg):
        self.modelmsg.pose.position.x = msg.point.x
        self.modelmsg.pose.position.y = msg.point.y
        self.modelmsg.pose.position.z = msg.point.z

        self.outpub.publish(self.modelmsg)

        
if __name__ == '__main__':
    
    rospy.init_node('pointstamped2modelstate', anonymous=True)
    
    # Parameters
    model_name = rospy.get_param('~model_name','rabbit')
    reference_frame = rospy.get_param('~reference_frame','world')

    # Initiate node object and subscribe
    node=Node(model_name, reference_frame)
    pointsub = rospy.Subscriber("rabbit", PointStamped, node.point_callback)

    # Spin
    rospy.spin()
    
