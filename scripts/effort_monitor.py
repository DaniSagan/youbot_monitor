#! /usr/bin/python

from __future__ import division
import roslib; roslib.load_manifest("youbot_monitor")
import rospy
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
from sensor_msgs.msg import JointState
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import random
import time 
import numpy

effs = [10.0 for k in range(5)]
names = ["" for k in range(5)]

def main():
    global effs
    global names
    fig = plt.figure()
    ax = fig.add_subplot(111)
    
    #ax.set_ybound(-10, 10)
    
    #effs = [10, 15, 12]
    #effs = [10.0 for k in range(15)]
    #effs = [10, 10, 0]
    
    plt.xticks(range(len(effs)))
    plt.grid()
    plt.show(block=False)
    rospy.init_node("effort_monitor")
    rospy.Subscriber("/joint_states", JointState, sub_callback)
    
    bars = ax.bar(numpy.arange(len(effs)), effs, width=0.8)
    #rospy.spin()
    
    while not rospy.is_shutdown():
        #for k in range(len(effs)):
        #    effs[k] = float(random.uniform(1,10))
        #print effs
        #[bar.set_height(effs[k]) for k, bar in enumerate(bars)]
        try:
            for k, bar in enumerate(bars):
                bar.set_height(effs[k])
            xtick_names = ax.set_xticklabels(names)
            plt.setp(xtick_names, rotation=45, fontsize=10, ha="center")
            plt.ylim([-10,10])
            fig.canvas.draw()
            plt.pause(1/30)
        except:
            pass
        #time.sleep(1/10)
        
        #rospy.spin
        
    return 0
    
    
def sub_callback(data):
    global effs
    global names
    effs = data.effort[:]
    names = data.name[-7:-2]
    #print effs
    
    
if __name__ == "__main__": main()
