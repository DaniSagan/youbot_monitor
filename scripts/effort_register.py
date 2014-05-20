#! /usr/bin/python
# -*- coding=utf-8 -*-

from __future__ import division
import rospy
import roslib; roslib.load_manifest("youbot_monitor")
import numpy
import math
import youbot

def main():
    yb = youbot.Youbot()
    
    """aa = [45*youbot.DEG_TO_RAD, 90*youbot.DEG_TO_RAD]
    angs_v = [[0.0, t1, t2, t3, 0.0] for t1 in aa for t2 in aa for t3 in aa]
    print angs_v
    rospy.sleep(5.0)
    
    for angs in angs_v:
        yb.set_joint_deltas(angs)
        print [ang*youbot.RAD_TO_DEG for ang in angs]
        rospy.sleep(10.0)"""
        
    while True:
        yb.publish_state()
        yb.set_joint_deltas([0.0, 45.0*youbot.DEG_TO_RAD, 0.0, 0.0, 0.0])
        yb.publish_state()
        rospy.sleep(0.01)
    
    
if __name__ == "__main__": main()
