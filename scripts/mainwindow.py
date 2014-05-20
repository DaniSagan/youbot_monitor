# coding=utf-8

from __future__ import division
import roslib; roslib.load_manifest("youbot_monitor")
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointVelocities
from brics_actuator.msg import JointValue
from sensor_msgs.msg import JointState
import wx
import rospy
import math

SIMULATOR = True

class Youbot:
    joints = [{"min": -169, "max": 169},
              #{"min":  -65, "max":  90},
              {"min":    0, "max": 155},
              #{"min": -151, "max": 146},
              {"min": -146, "max": 151},
              {"min": -102, "max": 102},
              {"min": -167, "max": 167}]        
        

class MainWindow(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, style=wx.SYSTEM_MENU | wx.CAPTION | wx.CLOSE_BOX | wx.CLIP_CHILDREN)
        rospy.init_node("arm_controller")
        self.init_ui()
        self.controlling = "pos"
        
        self.joint_current_pos = [0.0 for _ in range(5)]
        
        self.pos_publisher = rospy.Publisher("/arm_1/arm_controller/position_command", JointPositions)
        self.joint_pos_msg = JointPositions()
        self.joint_pos_msg.positions = [JointValue() for _ in range(5)]        
        for k, position in enumerate(self.joint_pos_msg.positions):
            position.joint_uri = "arm_joint_" + str(k+1)
            position.unit = "rad"
            position.value = 0.0
        
        self.vel_publisher = rospy.Publisher("/arm_1/arm_controller/velocity_command", JointVelocities)
        self.joint_vel_msg = JointVelocities()
        self.joint_vel_msg.velocities = [JointValue() for _ in range(5)]         
        for k, velocity in enumerate(self.joint_vel_msg.velocities):
            velocity.joint_uri = "arm_joint_" + str(k+1)
            velocity.unit = "s^-1 rad"
            velocity.value = 0.0 
            
        self.pos_subscriber = rospy.Subscriber("/joint_states", JointState, self.pos_callback)       
        
        self.publish()
        
    def init_ui(self):
        self.SetSize((800, 400))
        
        self.menu_bar = wx.MenuBar()
        self.options_menu = wx.Menu()
        self.toggle_control_item = self.options_menu.Append(wx.ID_ANY, "Toggle pos/vel control", "Toggle pos/vel control")
        self.menu_bar.Append(self.options_menu, "Options")
        self.Bind(wx.EVT_MENU, self.toggle_control, self.toggle_control_item)
        self.SetMenuBar(self.menu_bar)
        
        # Positions panel
        
        self.pos_panel = wx.Panel(self, wx.ID_ANY, pos=(0,0), size=self.GetSize())        
        wx.StaticText(self.pos_panel, label="Youbot Position Controller", pos=(10, 10))
        self.joint_pos_sliders = []
        self.joint_pos_sliders.append(wx.Slider(self.pos_panel, -1,
                                                0, -169, 169,
                                                pos=(10, 30),
                                                size=(500,-1),
                                                style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS))
        self.joint_pos_sliders[-1].Bind(wx.EVT_SLIDER, self.publish)
        
        self.joint_pos_sliders.append(wx.Slider(self.pos_panel, -1,
                                                90, 0, 155,
                                                pos=(10, 80),
                                                size=(500,-1),
                                                style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS))
        self.joint_pos_sliders[-1].Bind(wx.EVT_SLIDER, self.publish)
                                                
        self.joint_pos_sliders.append(wx.Slider(self.pos_panel, -1,
                                                0, -146, 151,
                                                pos=(10, 130),
                                                size=(500,-1),
                                                style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS))
        self.joint_pos_sliders[-1].Bind(wx.EVT_SLIDER, self.publish)
        
        self.joint_pos_sliders.append(wx.Slider(self.pos_panel, -1,
                                                0, -102, 102,
                                                pos=(10, 180),
                                                size=(500,-1),
                                                style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS))
        self.joint_pos_sliders[-1].Bind(wx.EVT_SLIDER, self.publish)
                                                
        self.joint_pos_sliders.append(wx.Slider(self.pos_panel, -1,
                                                0, -167, 167,
                                                pos=(10, 230),
                                                size=(500,-1),
                                                style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS))
        self.joint_pos_sliders[-1].Bind(wx.EVT_SLIDER, self.publish)
        
        # Velocities panel
        
        self.vel_panel = wx.Panel(self, wx.ID_ANY, pos=(0,0), size=self.GetSize())          
        wx.StaticText(self.vel_panel, label="Youbot Velocity Controller", pos=(10, 10))
        self.joint_vel_sliders = []
        self.joint_vel_sliders.append(wx.Slider(self.vel_panel, -1,
                                                0, -200, 200,
                                                pos=(10, 30),
                                                size=(500,-1),
                                                style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS))
        self.joint_vel_sliders[-1].Bind(wx.EVT_SLIDER, self.publish)
        
        self.joint_vel_sliders.append(wx.Slider(self.vel_panel, -1,
                                                0, -200, 200,
                                                pos=(10, 80),
                                                size=(500,-1),
                                                style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS))
        self.joint_vel_sliders[-1].Bind(wx.EVT_SLIDER, self.publish)
                                                
        self.joint_vel_sliders.append(wx.Slider(self.vel_panel, -1,
                                                0, -200, 200,
                                                pos=(10, 130),
                                                size=(500,-1),
                                                style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS))
        self.joint_vel_sliders[-1].Bind(wx.EVT_SLIDER, self.publish)
        
        self.joint_vel_sliders.append(wx.Slider(self.vel_panel, -1,
                                                0, -200, 200,
                                                pos=(10, 180),
                                                size=(500,-1),
                                                style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS))
        self.joint_vel_sliders[-1].Bind(wx.EVT_SLIDER, self.publish)
                                                
        self.joint_vel_sliders.append(wx.Slider(self.vel_panel, -1,
                                                0, -200, 200,
                                                pos=(10, 230),
                                                size=(500,-1),
                                                style=wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS))
        self.joint_vel_sliders[-1].Bind(wx.EVT_SLIDER, self.publish)
        
        self.vel_panel.Hide()
        self.pos_panel.Show()
        
        self.Show()
        
        #self.pos_panel.Bind(wx.EVT_KEY_DOWN, self.say_hello)
        
    def publish(self, event=None):
        if self.controlling == "pos":
            for k in range(5):
                value = self.joint_pos_sliders[k].GetValue()
                print "joint", k, ":", value, "º"            
                if k != 2:
                    self.joint_pos_msg.positions[k].value = (Youbot.joints[k]["max"] - value)*math.pi/180.0
                else:
                    self.joint_pos_msg.positions[k].value = (Youbot.joints[k]["min"] - value)*math.pi/180.0
            print "-------------------------"
            self.pos_publisher.publish(self.joint_pos_msg) 
        elif self.controlling == "vel":
            for k in range(5):
                value = self.joint_vel_sliders[k].GetValue()
                print "joint", k, ":", value, "º/s"            
                self.joint_vel_msg.velocities[k].value = -value*math.pi/180.0
                if self.joint_current_pos[k] > Youbot.joints[k]["max"] - 5: 
                    self.joint_vel_msg.velocities[k].value = 0.0
                if self.joint_current_pos[k] < Youbot.joints[k]["min"] + 5: 
                    self.joint_vel_msg.velocities[k].value = 0.0
                    
            print "-------------------------"
            self.vel_publisher.publish(self.joint_vel_msg)
        
    def say_hello(self, event=None):
        print "hello" 
        
    def toggle_control(self, event=None):
        if self.controlling == "pos": 
            self.controlling = "vel"
            self.pos_panel.Hide()
            self.vel_panel.Show()
        else: 
            self.controlling = "pos"
            self.vel_panel.Hide()
            self.pos_panel.Show()
            
    def pos_callback(self, data):
        if len(data.position) == 7:
            for k in range(5):
                if k != 2: 
                    self.joint_current_pos[k] = Youbot.joints[k]["max"] - data.position[k]*180/math.pi 
                else:      
                    self.joint_current_pos[k] = Youbot.joints[k]["min"] - data.position[k]*180/math.pi
            
        if SIMULATOR:
            for k in range(5):
                if k != 2: 
                    self.joint_current_pos[k] = Youbot.joints[k]["max"] - data.position[8+k]*180/math.pi 
                else:      
                    self.joint_current_pos[k] = Youbot.joints[k]["min"] - data.position[8+k]*180/math.pi
            
            """    if self.joint_current_pos[k] > Youbot.joints[k]["max"] - 5:
                    #self.joint_vel_msg.velocities[k].value = 0.0
                    #self.vel_publisher.publish(self.joint_vel_msg)
                    self.joint_pos_msg.positions[k].value = Youbot.joints[k]["max"] - 10
                    self.pos_publisher.publish(self.joint_pos_msg)
                    print "El mecanismo de seguridad ha actuado en el motor", k
                if self.joint_current_pos[k] < Youbot.joints[k]["min"] + 5:
                    #self.joint_vel_msg.velocities[k].value = 0.0
                    #self.vel_publisher.publish(self.joint_vel_msg)
                    self.joint_pos_msg.positions[k].value = Youbot.joints[k]["min"] + 10
                    self.pos_publisher.publish(self.joint_pos_msg)
                    print "El mecanismo de seguridad ha actuado en el motor", k""" 
                    
               
            
            
