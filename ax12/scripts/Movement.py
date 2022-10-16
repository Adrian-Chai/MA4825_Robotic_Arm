#!/usr/bin/env python3

from Ax12 import Ax12
import rospy
from time import sleep,time
from std_msgs.msg import String
from ax12.msg import MotorCmd
from dynamixel_sdk_examples.msg import SetPosition


class robot:
    def __init__(self):
        self.started = False

        Ax12.connect()
        
        # create AX12 instance with ID 
        self.motor0 = Ax12(0)  
        self.motor1 = Ax12(1)  
        self.motor2 = Ax12(2)  
        self.motor3 = Ax12(3)  
        self.gripper = Ax12(4) 
        self.motor_list = {0:self.motor0,1:self.motor1,2:self.motor2,3:self.motor3,4:self.gripper}

    def home(self):
        self.move_motor(self.motor0,(150))    
        self.move_motor(self.motor1,(150))
        self.move_motor(self.motor2,(60))
        self.move_motor(self.motor3,(150))
        self.wait_motor_stop(self.motor0)
        self.wait_motor_stop(self.motor1)
        self.wait_motor_stop(self.motor2)
        self.wait_motor_stop(self.motor3)

    
    def disconnect(self):
        self.motor0.set_torque_enable(0)
        self.motor1.set_torque_enable(0)
        self.motor2.set_torque_enable(0)
        self.motor3.set_torque_enable(0)
        self.gripper.set_torque_enable(0)
        Ax12.disconnect()
    
    def wait_motor_stop(self,motor):
        while motor.is_moving() == 1:
            sleep(0.1)
        
    def gripperopen(self):
        self.move_motor(self.gripper,240)

    def gripperclose(self):
        self.move_motor(self.gripper,155)
    
    def move_motor(self,motor, angle):
        input_pos = int(angle/(300/1024)) 
        motor.set_goal_position(input_pos)

    def cmd_callback(self,cmd_msg):
        self.wait_motor_stop(self.motor0)
        self.wait_motor_stop(self.motor1)
        self.wait_motor_stop(self.motor3)
        self.wait_motor_stop(self.gripper)
        print(cmd_msg)
        if cmd_msg.cmd == "on":
            print("Activate Robot")
            self.started = True
            self.motor0.set_moving_speed(80)
            self.motor1.set_moving_speed(80)
            self.motor2.set_moving_speed(80)
            self.motor3.set_moving_speed(80) 
            self.gripper.set_moving_speed(50)
            self.gripper.set_max_torque(250)
            self.home()
            self.gripperopen()
        
        if cmd_msg.cmd == "off":
            print("Deactivate Robot")
            self.started = False
            self.home()
            self.gripperopen()
            self.wait_motor_stop(self.gripper)
            self.gripperclose()
            self.wait_motor_stop(self.gripper)
            self.motor0.set_torque_enable(0)
            self.motor1.set_torque_enable(0)
            self.motor2.set_torque_enable(0)
            self.motor3.set_torque_enable(0)
            self.gripper.set_torque_enable(0)

        elif cmd_msg.cmd == "exit":
            print("Exit Program")
            rospy.signal_shutdown("exit program")
        
        elif cmd_msg.cmd == "home":
            self.home()
            self.gripperopen()

        elif cmd_msg.cmd == "hand": 
            self.move_motor(self.motor0,60) 
            self.move_motor(self.motor1,(150))
            self.move_motor(self.motor2,(60))
            self.move_motor(self.motor3,(150))
            self.move_motor(self.motor0,60)
        
        elif cmd_msg.cmd == "gripper_open":
            self.gripperopen()
        
        elif cmd_msg.cmd == "kinematics":
            """
            Rotate motor1 and 3 then 0
            """
            self.move_motor(self.motor0,cmd_msg.position0) 
            self.wait_motor_stop(self.motor0)
            self.move_motor(self.motor3,cmd_msg.position3)
            self.wait_motor_stop(self.motor3)
            self.move_motor(self.motor1,cmd_msg.position1)
            self.wait_motor_stop(self.motor1)
            if cmd_msg.gripper_cmd == "open":
                self.gripperopen()
            elif cmd_msg.gripper_cmd == "close":
                self.gripperclose()
            self.wait_motor_stop(self.gripper)
        

    def set_position_callback(self,position_msg):
        self.move_motor(self.motor_list[position_msg.id],position_msg.position)

    def exit_func(self):
        if self.started == False:
            pass
        else:
            self.home()
            self.gripperopen()
            self.wait_motor_stop(self.gripper)
            self.gripperclose()
            self.wait_motor_stop(self.gripper)
            self.disconnect()
            
    def run(self):
        rospy.init_node("ax12",anonymous=True,disable_signals=True)
        rospy.Subscriber("/motor_cmd",MotorCmd,self.cmd_callback)
        rospy.Subscriber("/set_position",SetPosition,self.set_position_callback)
        rospy.on_shutdown(self.exit_func)
        rospy.spin()
        
if __name__ == "__main__":
    try:
        robot_arm = robot()
        robot_arm.run()
    except rospy.ROSInterruptException:
        pass
