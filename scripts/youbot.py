#! /usr/bin/python
# -*- coding=utf-8 -*-

from __future__ import division
import rospy
import roslib; roslib.load_manifest("youbot_monitor")
from brics_actuator.msg import JointPositions
from brics_actuator.msg import JointVelocities
from brics_actuator.msg import JointValue
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from youbot_monitor.msg import InvKinPose
import numpy
import math

RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0
SIMULATOR = True

def extended_vector(value_list):
    return numpy.matrix(value_list + [1.0]).transpose()
    
def normal_list(ext_vector):
    return [ext_vector[k,0] for k in range(ext_vector.shape[0] - 1)]

class Youbot:
    joint_limits = [{"min": -169.0*DEG_TO_RAD, "max": 169.0*DEG_TO_RAD},
                    {"min":    0.0*DEG_TO_RAD, "max": 155.0*DEG_TO_RAD},
                    {"min": -146.0*DEG_TO_RAD, "max": 151.0*DEG_TO_RAD},
                    {"min": -102.5*DEG_TO_RAD, "max": 102.5*DEG_TO_RAD},
                    {"min": -167.5*DEG_TO_RAD, "max": 167.5*DEG_TO_RAD}]
    
    joint_offsets = [169.0 * DEG_TO_RAD,
                       0.0 * DEG_TO_RAD,
                     146.0 * DEG_TO_RAD,
                     102.5 * DEG_TO_RAD,
                     167.5 * DEG_TO_RAD]
                     
    _c_d_g =   numpy.matrix([[-1.0,  0.0,  0.0,  0.0,  0.0,  joint_limits[0]["max"]],
                             [ 0.0, -1.0,  0.0,  0.0,  0.0,  joint_limits[1]["max"]],
                             [ 0.0,  0.0, -1.0,  0.0,  0.0,  joint_limits[2]["min"]],
                             [ 0.0,  0.0,  0.0, -1.0,  0.0,  joint_limits[3]["max"]],
                             [ 0.0,  0.0,  0.0,  0.0, -1.0,  joint_limits[4]["max"]],                             
                             [ 0.0,  0.0,  0.0,  0.0,  0.0,  1.0                   ]])
                             
    _c_g_d = _c_d_g.getI()
                           
    """_c_dd_dg = numpy.matrix([[-1.0,  0.0,  0.0,  0.0,  0.0],
                             [ 0.0, -1.0,  0.0,  0.0,  0.0],
                             [ 0.0,  0.0, -1.0,  0.0,  0.0],
                             [ 0.0,  0.0,  0.0, -1.0,  0.0],
                             [ 0.0,  0.0,  0.0,  0.0,  1.0]])"""
                             
    _c_d_t =   numpy.matrix([[ 1.0,  0.0,  0.0,  0.0,  0.0,  0.0],
                             [ 0.0,  1.0,  0.0,  0.0,  0.0,  0.0],
                             [ 0.0,  1.0,  1.0,  0.0,  0.0,  0.0],
                             [ 0.0,  1.0,  1.0,  1.0,  0.0,  0.0],
                             [ 0.0,  0.0,  0.0,  0.0,  1.0,  0.0],
                             [ 0.0,  0.0,  0.0,  0.0,  0.0,  1.0]])
                             
    _c_t_d = _c_d_t.getI()
                             
    """_c_dt_dd = numpy.matrix([[ 1.0,  0.0,  0.0,  0.0,  0.0],
                             [ 0.0,  1.0,  0.0,  0.0,  0.0],
                             [ 0.0,  1.0,  1.0,  0.0,  0.0],
                             [ 0.0,  1.0,  1.0,  1.0,  0.0],
                             [ 0.0,  0.0,  0.0,  0.0,  1.0]])"""
                             
    _L = [0.033, 0.155, 0.133, 0.218]
              
    def __init__(self):
        rospy.init_node("youbot_monitor")
        self.joint_current_pos = [0.0] * 5
        self.joint_current_vel = [0.0] * 5
        self.joint_current_eff = [0.0] * 5
        self.joint_target_pos = [0.0] * 5
        self.joint_target_vel = [0.0] * 5
        self.controlling = "pos"
        
        self.joint_pos_publisher = rospy.Publisher("/arm_1/arm_controller/position_command", JointPositions, latch=True)
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
            
        self.pos_subscriber = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        
        self._curr_gammas = None
        self._curr_deltas = None
        self._curr_thetas = None
        
        self.pos_publisher = rospy.Publisher("/youbot_monitor/state/gripper_position", Point, latch=True)
        self.angs_publisher = rospy.Publisher("/youbot_monitor/state/arm_joint_angles", Float32MultiArray, latch=True)
        self.pos_cmd_subscriber = rospy.Subscriber("/youbot_monitor/controller/gripper_position", InvKinPose, self.pos_cmd_callback)
        self.angs_cmd_subscriber = rospy.Subscriber("/youbot_monitor/controller/arm_joint_angles", Float32MultiArray, self.angs_cmd_callback)
        
    def joint_state_callback(self, data):
        """if len(data.position) == 7:
            for k in range(5):
                if k != 2: 
                    self.joint_current_pos[k] = Youbot.joints[k]["max"] - data.position[k] 
                else:      
                    self.joint_current_pos[k] = Youbot.joints[k]["min"] - data.position[k]"""
            
        if SIMULATOR:
            gammas = extended_vector(list(data.position)[8:13])
            self._curr_gammas = gammas
            self._curr_deltas = Youbot._c_d_g.getI()*gammas
            self._curr_thetas = Youbot._c_d_t*Youbot._c_d_g.getI()*gammas
            self.joint_curr_pos = normal_list(self._curr_deltas)
            self.joint_curr_eff = data.effort[8:13]
            
        else:
            if len(data.position) == 7:
                gammas = extended_vector(list(data.position)[0:6])
                self._curr_gammas = gammas
                self._curr_deltas = Youbot._c_d_g.getI()*gammas
                self._curr_thetas = Youbot._c_d_t*Youbot._c_d_g.getI()*gammas
                self.joint_curr_pos = normal_list(self._curr_deltas)
                self.joint_curr_eff = data.effort[0:6]
            
    def get_x_cil(self):
        if self._curr_thetas != None:
            thetas = [self._curr_thetas[k] for k in range(4)]
            return Youbot._L[0] + Youbot._L[1]*math.cos(thetas[1]) + Youbot._L[2]*math.cos(thetas[2]) + Youbot._L[3]*math.cos(thetas[3])
        else:
            return 0.0 
        
    def get_y_cil(self):
        if self._curr_thetas != None:
            thetas = [self._curr_thetas[k] for k in range(4)]
            return Youbot._L[1]*math.sin(thetas[1]) + Youbot._L[2]*math.sin(thetas[2]) + Youbot._L[3]*math.sin(thetas[3])
        else:
            return 0.0
        
    def get_pos_cil(self):
        return self.get_x_cil(), self.get_y_cil() 
        
    def get_pos(self):
        if self._curr_thetas != None:
            X, Y = self.get_pos_cil()
            return X*math.cos(self._curr_thetas[0]), X*math.sin(self._curr_thetas[0]), Y
        else:
            return 0.0, 0.0, 0.0
            
    def get_thetas_for_pos(self, pos, theta3=0.0, solution=0):
        assert solution in [0, 1, 2, 3]
        x, y, z = pos
        if solution in [0, 1]:
            theta0 = math.atan2(y, x)
        else:
            theta0 = math.atan2(-y, -x)
        X = x / math.cos(theta0)
        Y = z
        if solution in [0, 1]:
            Xarm = X - Youbot._L[0] - Youbot._L[3]*math.cos(theta3)
        else:
            Xarm = X + Youbot._L[0] - Youbot._L[3]*math.cos(theta3)
        Yarm = Y - Youbot._L[3]*math.sin(theta3)
        if solution in [0, 2]: 
            s = solve_simple_arm((Xarm, Yarm), Youbot._L[1], Youbot._L[2], solution=0)
        elif solution in [1, 3]: 
            s = solve_simple_arm((Xarm, Yarm), Youbot._L[1], Youbot._L[2], solution=1)
        if not s: return None
        else: return [theta0, s[0], s[1], theta3]
        
    def move_arm_to_pos(self, pos, theta3=0.0, solution=0):
        tt = self.get_thetas_for_pos(pos, theta3, solution=0) + [0.0]
        if not tt: return 
        thetas = extended_vector(tt)
        deltas = Youbot._c_d_t.getI()*thetas
        gammas = Youbot._c_d_g*Youbot._c_d_t.getI()*thetas
        for k, d in enumerate(deltas):
            if k < deltas.shape[0] - 1 and (d < Youbot.joint_limits[k]["min"] or d > Youbot.joint_limits[k]["max"]):
                rospy.logerr("Position not reachable")
                return
        for k, position in enumerate(self.joint_pos_msg.positions):
            position.value = gammas[k, 0]
        self.joint_pos_publisher.publish(self.joint_pos_msg) 
        
    def set_joint_deltas(self, deltas):
        assert len(deltas) == 5
        dd = extended_vector(list(deltas))
        gg = Youbot._c_d_g*dd
        for k, position in enumerate(self.joint_pos_msg.positions):
            position.value = gg[k, 0]
        self.joint_pos_publisher.publish(self.joint_pos_msg)
        
    def publish_state(self):
        x, y, z = self.get_pos()
        pos = Point()
        pos.x, pos.y, pos.z = x, y, z
        self.pos_publisher.publish(pos)
        
        if self._curr_deltas != None:  
            angs = Float32MultiArray()
            angs.data = list(self._curr_deltas) 
            self.angs_publisher.publish(angs) 
            
    def pos_cmd_callback(self, data):
        rospy.loginfo("Received gripper position command: [%f, %f, %f]" % (data.target_position.x, data.target_position.y, data.target_position.z))
        if data.solution in [0, 1, 2, 3]:
            pos = data.target_position.x, data.target_position.y, data.target_position.z
            self.move_arm_to_pos(pos, data.effector_angle, data.solution)        
        
    def angs_cmd_callback(self, data):
        print data.data
        rospy.loginfo("Received joint angles command: %s" % [d for d in data.data])
        if len(data.data) == 5:
            self.set_joint_deltas(data.data)
            
    def spin(self):
        while not rospy.is_shutdown():
            self.publish_state()
            rospy.sleep(0.01)
            
            
            
def solve_simple_arm(pos, L0, L1, solution=0):
    assert L0 >= 0.0 
    assert L1 >= 0.0
    assert solution in [0, 1] 
    x, y = pos
    D = (x**2 + y**2)**0.5
    if D > L0 + L1: 
        print "Position not reachable"
        return None
    phi = math.atan2(y, x)
    try:
        alpha = math.acos((L0**2 - L1**2 + D**2)/(2*L0*D))
        beta = math.acos((L1**2 - L0**2 + D**2)/(2*L1*D))
    except:
        return None
    if solution == 0:
        return [phi + alpha, phi - beta]
    elif solution == 1:
        return [phi - alpha, phi + beta]

def main():
    
    yb = Youbot()
    yb.spin()
    
            
if __name__ == "__main__": main()

