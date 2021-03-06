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
import datetime
import json
import urllib2
import urllib

def post_message(message):
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    msg = {"SENDER": "Youbot", "TITLE": "Test has finished", "DATE": now, "MSG": message}
    str_msg = json.dumps(msg) + "\n"
    info = urllib.urlencode({"info": str_msg})
    urllib2.urlopen("http://danisagan-cc.netau.net/loginfo.php?" + info)

def main():
    rospy.init_node("angles_generator")
    pub = rospy.Publisher("/youbot_monitor/controller/arm_joint_angles", Float32MultiArray, latch=True)
    msg = Float32MultiArray()
    msg.data = [0.0, 45*math.pi/180, 0.0, 0.0, 0.0]
    pub.publish(msg)
    rospy.sleep(5.0)

    angs1 = [45*math.pi/180, 90*math.pi/180]
    angs2 = [-45*math.pi/180, 0, 45*math.pi/180, 90*math.pi/180]
    #angs3 = [-90*math.pi, -60*math.pi/180, -45*math.pi/180, -30*math.pi/180, 0, 45*math.pi/180, 90*math.pi/180]
    #angs3 = numpy.concatenate((numpy.linspace(-90, 90, 13)*math.pi/180, numpy.linspace(75, -90, 12)*math.pi/180))
    a = numpy.linspace(-45, 45, 91)
    angs3 = numpy.concatenate((a, a[:-1][::-1]))
    #angs_v = [[0.0, 90*math.pi/180, 0.0, t3, 0.0] for t3 in angs3]
    angs_v = [[0.0, 90*math.pi/180 + t3*math.pi/180, 0.0, 0.0, 0.0] for t3 in angs3]

    rospy.set_param("angle_generator_count", 0)
    rospy.set_param("running", True)
    for k in range(1):
        try:
            post_message("Cycle %d" % k)
        except:
            pass
        rospy.set_param("angle_generator_count", k)
        for angs in angs_v:
            msg.data = angs
            rospy.loginfo("Joint angles: %s" % angs)
            pub.publish(msg)
            rospy.sleep(10.0)

    rospy.set_param("running", False)
    try:
        post_message("Angles generator has finished succesfully")
    except:
        pass


if __name__ == "__main__": main()
