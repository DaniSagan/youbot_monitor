# coding=utf-8

from __future__ import division
import roslib; roslib.load_manifest("youbot_monitor")
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointValue
import wx
import rospy
import math

class Youbot:
    joints = [{"min": -169, "max": 169},
              {"min":  -65, "max":  90},
              {"min": -151, "max": 146},
              {"min": -102, "max": 102},
              {"min": -167, "max": 167}]

class MainWindow(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, style=wx.SYSTEM_MENU | wx.CAPTION | wx.CLOSE_BOX | wx.CLIP_CHILDREN)
        rospy.init_node("youbot_monitor")
        self.init_ui()
        self.publisher = rospy.Publisher("/arm_1/arm_controller/position_command", JointPositions)
        self.joint_pos_msg = JointPositions()
        self.joint_pos_msg.positions = []
        for k in range(5):
            self.joint_pos_msg.positions.append(JointValue())
            self.joint_pos_msg.positions[-1].joint_uri = "arm_joint_" + str(k+1)
            self.joint_pos_msg.positions[-1].unit = "rad"
            self.joint_pos_msg.positions[-1].value = 0.0
        
        
        self.publish()
        
    def init_ui(self):
        self.SetSize((800, 400))
        self.panel = wx.Panel(self)
        
        wx.StaticText(self.panel, label="Youbot Monitor", pos=(10, 10))
        self.joint_pos_sliders = []
        self.joint_pos_sliders.append(wx.Slider(self.panel, -1,
                                                0, -169, 169,
                                                pos=(10, 30),
                                                size=(500,-1),
                                                style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS))
        self.joint_pos_sliders[-1].Bind(wx.EVT_SLIDER, self.publish)
        
        self.joint_pos_sliders.append(wx.Slider(self.panel, -1,
                                                0, -65, 90,
                                                pos=(10, 80),
                                                size=(500,-1),
                                                style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS))
        self.joint_pos_sliders[-1].Bind(wx.EVT_SLIDER, self.publish)
                                                
        self.joint_pos_sliders.append(wx.Slider(self.panel, -1,
                                                0, -151, 146,
                                                pos=(10, 130),
                                                size=(500,-1),
                                                style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS))
        self.joint_pos_sliders[-1].Bind(wx.EVT_SLIDER, self.publish)
        
        self.joint_pos_sliders.append(wx.Slider(self.panel, -1,
                                                0, -102, 102,
                                                pos=(10, 180),
                                                size=(500,-1),
                                                style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS))
        self.joint_pos_sliders[-1].Bind(wx.EVT_SLIDER, self.publish)
                                                
        self.joint_pos_sliders.append(wx.Slider(self.panel, -1,
                                                0, -167, 167,
                                                pos=(10, 230),
                                                size=(500,-1),
                                                style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS))
        self.joint_pos_sliders[-1].Bind(wx.EVT_SLIDER, self.publish)
        
        self.Show()
        
        self.panel.Bind(wx.EVT_KEY_DOWN, self.say_hello)
        
    def publish(self, event=None):
        for k in range(5):
            value = self.joint_pos_sliders[k].GetValue()
            print "joint", k, ":", value, "º"            
            if k != 2:
                self.joint_pos_msg.positions[k].value = (value - Youbot.joints[k]["min"])*math.pi/180.0
            else:
                self.joint_pos_msg.positions[k].value = (value - Youbot.joints[k]["max"])*math.pi/180.0
        print "-------------------------"
        self.publisher.publish(self.joint_pos_msg) 
        
    def say_hello(self, event=None):
        print "hello"   
            
            
