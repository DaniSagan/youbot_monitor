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
from sensor_msgs.msg import JointState
import json
import urllib2
import urllib
import datetime

curr_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
target_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
curr_joint_effort = [0.0, 0.0, 0.0, 0.0, 0.0]

def joint_angles_callback(data):
    global curr_joint_angles
    curr_joint_angles = [d*180/math.pi for d in data.data]#list(data.data)
    
def joint_states_callback(data):
    global curr_joint_effort
    curr_joint_effort = list(data.effort)
    
def joint_angles_cmd_callback(data):
    global target_joint_angles
    target_joint_angles = [d*180/math.pi for d in data.data]#list(data.data)

def main():
    global curr_joint_angles
    global target_joint_angles
    global curr_joint_effort
    
    rospy.init_node("effort_register")
    arm_joint_angles_sub = rospy.Subscriber("/youbot_monitor/state/arm_joint_angles", Float32MultiArray, joint_angles_callback)
    joint_states_sub= rospy.Subscriber("/joint_states", JointState, joint_states_callback)
    target_joint_angles_sub = rospy.Subscriber("/youbot_monitor/controller/arm_joint_angles", Float32MultiArray, joint_angles_cmd_callback)
    
    rospy.loginfo("Waiting for node angles_generator...")
    rospy.delete_param("running")
    while not rospy.has_param("running"):
        rospy.sleep(0.01)
    next_file = False
    while rospy.get_param("running"):
        curr_count = rospy.get_param("angle_generator_count")
        with open("log" + str(curr_count) + ".txt", "w") as f:
            rospy.loginfo("Writing file %s", f.name)
            next_file = False
            while not rospy.is_shutdown() and not next_file:
                curr_time = rospy.get_rostime()
                json_str = json.dumps({"time": str(curr_time),
                                       "curr_joint_angles": curr_joint_angles,
                                       "target_joint_angles": target_joint_angles,
                                       "curr_joint_effort": curr_joint_effort})
                f.write(json_str + "\n")
                rospy.sleep(0.01)
                read_count = rospy.get_param("angle_generator_count")
                if read_count != curr_count or not rospy.get_param("running"): 
                    next_file = True
    
    info = urllib.urlencode({"info": "\nFinished %s\n" % str(datetime.datetime.now())})                
    urllib2.urlopen("http://danisagan-cc.netau.net/loginfo.php?" + info)
           
    
if __name__ == "__main__": main()
