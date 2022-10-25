#!/usr/bin/env python3
# 4.5 cm from box
# 21 cm camera from edge of box
# camera 56.5 from platform

# obj name

LINK_1 = 10 
LINK_2 =  16.1 

import rospy
from time import sleep
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from ax12.msg import MotorCmd
import math
import numpy as np
from geometry_msgs.msg import PointStamped

class controller:
    def __init__(self):
        self.angle_range= {0:(0,299),1:(110,180),2:(150,150),3:(80,150),4:(150,240)}
        self.destination = {
                            0:np.array([-7,12,0.5]),
                            1:np.array([ 0,12,0.5]),
                            2:np.array([ 8,12,0.5])
                            } # 0,1,2 is id of object
        self.hand_in_range = False
        self.pub = None
        self.started = False
        self.standby = False
        self.coordinate = None
        self.reset = False
    
    def check_angle(self,position0,position1,position3):
        """
        Check whether motor command within limit
        """
        if position0 < self.angle_range[0][0] or position0 > self.angle_range[0][1]:
            print("motor0 out of range")
            return False
        
        if position1 < self.angle_range[1][0] or position1 > self.angle_range[1][1]:
            print("motor1 out of range")
            return False

        if position3 < self.angle_range[3][0] or position3 > self.angle_range[3][1]:
            print("motor3 out of range")
            return False

        return True
    
    def hand_coord_callback(self,coordinate):
        if coordinate.point.x > 450 and coordinate.point.x < 850 and coordinate.point.y > 300 and coordinate.point.y < 700:
            hand_coord = (coordinate.point.x,coordinate.point.y)
            print(hand_coord)
            self.hand_in_range = True
        else:
            self.hand_in_range = False

    def coordinate_callback(self,coordinate):
        self.coordinate = np.array([coordinate.x,coordinate.y,coordinate.z])
    
    def reset_callback(self,reset):
        if self.started == True and self.standby == True:
            self.reset = True
        
    def voice_cmd_callback(self,voice_cmd):
        """
            List of commands:
            1.) on
            2.) off (disconnect only)
            3.) sticky tape
            4.) box
            5.) screwdriver
            6.) exit/bye (kill all nodes)
        """

        if (voice_cmd.data).lower().find("on") != -1:
            if self.started == False:
                self.started = True
                motorcmd = MotorCmd()
                motorcmd.cmd = "on"
                self.pub.publish(motorcmd)
        
        elif (voice_cmd.data).lower().find("off") != -1:
            if self.started == True:
                self.started = False
                self.standby = False
                motorcmd = MotorCmd()
                motorcmd.cmd = "off"
                self.pub.publish(motorcmd)
        
        elif voice_cmd.data.lower().find("exit") != -1 or voice_cmd.data.lower().find("bye") != -1:
            if self.started == True:
                motorcmd = MotorCmd()
                motorcmd.cmd = "exit"
                self.pub.publish(motorcmd)
            rospy.signal_shutdown("exit")
            
        
        elif voice_cmd.data.lower().find("david") != -1:
            if self.started == True and self.standby == False:
                motorcmd = MotorCmd()
                motorcmd.cmd = "gripper_open"
                self.pub.publish(motorcmd)
                self.standby = True

        else:
            if self.started == True and self.standby == True:
                object = None
                if voice_cmd.data.lower().find("screwdriver") != -1: #-7
                    object = 0
                elif voice_cmd.data.lower().find("box") != -1: #0
                    object = 1
                elif voice_cmd.data.lower().find("sticky") != -1 or voice_cmd.data.lower().find("tape") != -1: #8
                    object = 2
                if object != None or self.coordinate is not None:
                    if object != None:
                        position0,position1,position3 = self.inverse_kinematics(self.destination[object])
                    else:
                        position0,position1,position3 = self.inverse_kinematics(self.coordinate)
                    if self.check_angle(position0,position1,position3) == True:
                        motorcmd = MotorCmd()
                        motorcmd.cmd = "kinematic"
                        motorcmd.position0 = position0
                        motorcmd.position1 = position1
                        motorcmd.position3 = position3
                        motorcmd.gripper_cmd = "close"
                        self.pub.publish(motorcmd)
                        sleep(0.1)
                        motorcmd = MotorCmd()
                        motorcmd.cmd = "home"
                        self.pub.publish(motorcmd)
                        sleep(0.1)
                        while not (self.hand_in_range or self.reset): # wait for hand obj
                            sleep(0.01)                # while hand is not detected, do nothing
                        if self.reset == False:
                            motorcmd = MotorCmd()
                            motorcmd.cmd = "hand"
                            self.pub.publish(motorcmd)
                        else:
                            print("reset")
                        self.hand_in_range = False
                        self.standby = False
                        self.reset = False
                
    def inverse_kinematics(self,coordinate):
        # coordinate is numpy 3 x 1 array
        x = coordinate[0]
        y = coordinate[1]
        z = coordinate[2]

        yaw = math.atan(y/(x-1.5))
        if yaw < 0:
            yaw = yaw + math.pi
        H01 = np.array([[math.cos(yaw),-math.sin(yaw),0,1.5],
                        [math.sin(yaw),math.cos(yaw) ,0,0  ],
                        [0            ,0             ,1,7.5],
                        [0            ,0             ,0,1  ]
                        ])

        motor1_joint = H01 @ np.transpose(np.array([[1.5,0,2,1]]))

        relative_distance = math.sqrt((x-motor1_joint[0][0])**2+(y-motor1_joint[1][0])**2) - 3 #obj radius

        relative_height = z - motor1_joint[2] 

        theta2 = -math.acos((relative_distance**2 + relative_height**2 - LINK_1**2 - LINK_2**2)/(2*LINK_1*LINK_2))
        theta1 = math.atan(relative_height/relative_distance) - math.atan((LINK_2*math.sin(theta2)/(LINK_1+LINK_2*math.cos(theta2))))
        if theta1 < 0:
            theta1 = math.pi + theta1
        position0 = 150 + math.degrees(yaw)
        position1 = 120 + (90 -math.degrees(theta1))
        position3 = 225 + math.degrees(theta2) 

        return position0,position1,position3

        
    def run(self):
        rospy.init_node("ax12",anonymous=True,disable_signals=True)
        self.pub = rospy.Publisher("/motor_cmd",MotorCmd,queue_size=1)
        rospy.Subscriber("/coordinate",Vector3,self.coordinate_callback)
        rospy.Subscriber("/hand_coord",PointStamped,self.hand_coord_callback)
        rospy.Subscriber("/voice_cmd",String,self.voice_cmd_callback)
        rospy.Subscriber("/reset",String,self.reset_callback)
        rospy.spin()
        
        
if __name__ == "__main__":
    try:
        robot_controller = controller()
        robot_controller.run()
    except rospy.ROSInterruptException:
        pass
