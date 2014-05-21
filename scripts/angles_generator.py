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
    msg.data = [0.0, 45*math.pi/180, 0.0, 0.0, 0.0]
    pub.publish(msg)    
    rospy.sleep(5.0)

    angs1 = [45*math.pi/180, 90*math.pi/180]
    angs2 = [-45*math.pi/180, 0, 45*math.pi/180, 90*math.pi/180]
    angs3 = [-45*math.pi/180, 0, 45*math.pi/180, 90*math.pi/180]
    angs_v = [[0.0, t1, t2, t3, 0.0] for t1 in angs1 for t2 in angs2 for t3 in angs3]
    
    rospy.set_param("angle_generator_count", 0)
    rospy.set_param("running", True)
    for k in range(2):
        rospy.set_param("angle_generator_count", k)
        for angs in angs_v:
            msg.data = angs
            rospy.loginfo("Joint angles: %s" % angs)
            pub.publish(msg) 
            rospy.sleep(2.0) 
            
    rospy.set_param("running", False)
    
    
if __name__ == "__main__": main()
