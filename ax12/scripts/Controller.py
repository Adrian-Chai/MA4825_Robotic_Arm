#!/usr/bin/env python3

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
                            0:np.array([[-7,12,0.5]]),
                            1:np.array([[ 0,12,0.5]]),
                            2:np.array([[ 8,12,0.5]])
                            } # 0,1,2 is id of object
        self.coordinate = None
        self.hand_in_range = False
        self.pub = None
        self.started = False
    
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
        if coordinate.point.x !=0 and coordinate.point.y !=0:
            hand_coord = (coordinate.point.x,coordinate.point.y)
            print(hand_coord)
            self.hand_in_range = True
        else:
            self.hand_in_range = False

    def coordinate_callback(self,coordinate):
        self.coordinate = np.transpose(np.array([[coordinate.x,coordinate.y,coordinate.z]]))
        
    def voice_cmd_callback(self,voice_cmd):
        """
            List of commands:
            1.) on
            2.) off (disconnect only)
            3.) home
            4.) hand
            4.) cube
            5.) exit (kill all nodes)
        """

        if (voice_cmd.data).lower().find("on") != -1:
            if self.started == False:
                self.started = True
                motorcmd = MotorCmd()
                motorcmd.cmd = "on"
                self.pub.publish(motorcmd)
        
        if (voice_cmd.data).lower().find("off") != -1:
            if self.started == True:
                self.started = False
                motorcmd = MotorCmd()
                motorcmd.cmd = "off"
                self.pub.publish(motorcmd)
        
        if voice_cmd.data.lower().find("exit") != -1:
            if self.started == True:
                motorcmd = MotorCmd()
                motorcmd.cmd = "exit"
                self.pub.publish(motorcmd)
            rospy.signal_shutdown("exit")
        
        if voice_cmd.data.lower().find("home") != -1:
            if self.started == True:
                motorcmd = MotorCmd()
                motorcmd.cmd = "home"
                self.pub.publish(motorcmd)

        if voice_cmd.data.lower().find("cube") != -1:
            if self.started == True:
                if self.coordinate is not None: # check for obj
                    position0,position1,position3 = self.inverse_kinematics(self.coordinate)
                    print("position0: ",position0)
                    print("position1: ",position1)
                    print("position3: ",position3)
                    if self.check_angle(position0,position1,position3) == True:
                        confirm = input("y/n")
                        if confirm == "y":
                            motorcmd = MotorCmd()
                            motorcmd.cmd = "kinematics"
                            motorcmd.position0 = position0
                            motorcmd.position1 = position1
                            motorcmd.position3 = position3
                            motorcmd.gripper_cmd = "close"
                            self.pub.publish(motorcmd)
                            sleep(2)
                            motorcmd = MotorCmd()
                            motorcmd.cmd = "hand"
                            self.pub.publish(motorcmd)
                            sleep(2)
                            while not self.hand_in_range:
                                sleep(0.1)
                            motorcmd = MotorCmd()
                            motorcmd.cmd = "gripper_open"
                            self.pub.publish(motorcmd)
                            # wait for hand obj
                            # while hand is not detected, do nothing
                            # while self.coordinate in range
                            sleep(2)
                            motorcmd = MotorCmd() # reset 
                            motorcmd.cmd = "home"
                            self.pub.publish(motorcmd)
                            
                            
                        elif confirm == "n":
                            print("fail")
                        self.coordinate = None

                
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
        rospy.spin()
        
        
if __name__ == "__main__":
    try:
        robot_controller = controller()
        robot_controller.run()
    except rospy.ROSInterruptException:
        pass
