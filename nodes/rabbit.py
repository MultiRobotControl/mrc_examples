#!/usr/bin/env python3
'''
Rabbit node.
'''
import itertools
import sys
from math import *

import rospy
from geometry_msgs.msg import PointStamped

def dist(p1, p2):
    return sqrt( (float(p1[0]) - float(p2[0]) )**2
                 + (float(p1[1]) - float(p2[1]) )**2 )

class Rabbit():
    def __init__(self, wpcycle, velocity):        
        self.velocity = velocity
        self.wpcycle = wpcycle
        self.t0 = rospy.Time.now()

        self.curr_wp = next(wpcycle)
        self.next_wp = next(wpcycle)

        self.pointpub = rospy.Publisher("rabbit", PointStamped, queue_size=1)
        self.point = PointStamped()
        self.point.point.x = self.curr_wp[0]
        self.point.point.y = self.curr_wp[1]
        if len(self.curr_wp) > 2:
            self.point.point.z = self.curr_wp[2]
        if len(self.curr_wp) > 3:
            self.velocity = self.curr_wp[3]
        self.point.header.stamp = rospy.Time.now()
        self.point.header.frame_id = "gazebo"
        
    def execute(self):
        # Find elapsed time
        now = rospy.Time.now()
        dtdur = now - self.t0
        dt = dtdur.to_sec()
        self.t0 = now

        # Distance to go this timestep
        ddist = self.velocity * dt
        # To next wp
        dist2next = dist(self.next_wp,
                         [self.point.point.x, self.point.point.y])
        # Don't go past, instead increment waypoint
        if ddist > dist2next:
            # Set to next
            self.point.point.x = self.next_wp[0]
            self.point.point.y = self.next_wp[1]
            if len(self.next_wp) > 2:
                self.point.point.z = self.next_wp[2]
            # Increment cycle
            self.curr_wp = self.next_wp
            self.next_wp = next(wpcycle)
            if len(self.curr_wp) > 3:
                self.velocity = self.curr_wp[3]
        else:
            # Find next point
            angle = atan2(float(self.next_wp[1]) - self.point.point.y,
                          float(self.next_wp[0]) - self.point.point.x)
            self.point.point.x += ddist*cos(angle)
            self.point.point.y += ddist*sin(angle)

        # Publish
        self.point.header.stamp = rospy.Time.now()
        self.pointpub.publish(self.point)
        
if __name__ == '__main__':
    
    rospy.init_node('rabbit', anonymous=True)
    
    # Parameters
    update_rate = rospy.get_param('~update_rate',10.0)
    vel = rospy.get_param('~velocity',1)
    try:
        wps = rospy.get_param('~waypoints')
    except KeyError:
        rospy.logfatal("There is not <waypoints> parameter for rabbit path")
        sys.exit(1)

    # Waypoints - hardcoded for now
    wpcycle = itertools.cycle(wps)
                   
    # Initiate node object
    rabbit = Rabbit(wpcycle, vel)

    # Spin
    r = rospy.Rate(update_rate)

    try:
        while not rospy.is_shutdown():
            rabbit.execute()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
