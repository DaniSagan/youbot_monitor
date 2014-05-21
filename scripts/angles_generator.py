#! /usr/bin/python
# -*- coding=utf-8 -*-

from __future__ import division
import rospy
import roslib; roslib.load_manifest("youbot_monitor")
import numpy
import math
from geometry_msgs.msg import Point
from youbot_monitor.msg import InvKinPose
from std_msgs.msg import Float32MultiArray

def main():
    rospy.init_node("angles_generator")
    pub = rospy.Publisher("/youbot_monitor/controller/arm_joint_angles", Float32MultiArray, latch=True)
    msg = Float32MultiArray()
    msg.data = [0.0] * 5
    """msg.target_position = Point()
    msg.target_position.x = 0.3
    msg.target_position.y = 0.2
    msg.target_position.z = 0.1
    msg.effector_angle = 0.0
    msg.solution = 0"""
    pub.publish(msg)
    #while not rospy.is_shutdown():
    #    pub.publish(msg)
    
    #aa = [45*math.pi/180, 90*math.pi/180]
    angs1 = [45*math.pi/180, 90*math.pi/180]
    angs2 = [-45*math.pi/180, 0, 45*math.pi/180, 90*math.pi/180]
    angs3 = [-45*math.pi/180, 0, 45*math.pi/180, 90*math.pi/180]
    angs_v = [[0.0, t1, t2, t3, 0.0] for t1 in angs1 for t2 in angs2 for t3 in angs3]
    
    for angs in angs_v:
        msg.data = angs
        rospy.loginfo("Joint angles: %s" % angs)
        pub.publish(msg) 
        rospy.sleep(2.0)
    
    
if __name__ == "__main__": main()
