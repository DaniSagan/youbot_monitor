#!/usr/bin/python
# coding=utf-8

from __future__ import division
import roslib; roslib.load_manifest("youbot_monitor")
from geometry_msgs.msg import Twist
import sys
import rospy
import curses

def main():
    rospy.init_node("base_controller")
    publisher = rospy.Publisher("/cmd_vel", Twist)
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.angular.z = 0.0
    
    win = curses.initscr()
    curses.noecho()
    curses.cbreak()
    running = True
    win.addstr(
"""=====================================
YOUBOT BASE CONTROLLER
=====================================
Move forward [w]    Move backward [s]
Move left    [a]    Move right    [d]
Turn left    [q]    Turn right    [e]
Stop         [x]
Quit         [z]  
""")
    win.refresh()
    while running:
        c = win.getch() 
        if chr(c) == "z": 
            running = False
        elif chr(c) == "x":
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
        elif chr(c) == "w":
            msg.linear.x = 0.1
            msg.linear.y = 0.0
            msg.angular.z = 0.0
        elif chr(c) == "s":
            msg.linear.x = -0.1
            msg.linear.y = 0.0
            msg.angular.z = 0.0
        elif chr(c) == "a":
            msg.linear.x = 0.0
            msg.linear.y = -0.1
            msg.angular.z = 0.0
        elif chr(c) == "d":
            msg.linear.x = 0.0
            msg.linear.y = 0.1
            msg.angular.z = 0.0
        elif chr(c) == "q":
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.1
        elif chr(c) == "e":
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = -0.1
        publisher.publish(msg)
    curses.nocbreak()
    win.keypad(0)
    curses.echo()
    curses.endwin()
    
if __name__ == "__main__": main()
