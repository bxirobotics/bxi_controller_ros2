import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
import communication.msg as bxiMsg
import communication.srv as bxiSrv
import nav_msgs.msg 
import sensor_msgs.msg
from threading import Lock
import numpy as np
# import torch
import time
import sys
import os
import math
from collections import deque
from std_msgs.msg import Header
# from geometry_msgs.msg import Pose
# from sensor_msgs.msg import JointState
# import signal

class BxiExample(Node):

    def __init__(self):

        super().__init__('bxi_example_py')
    
        qos = QoSProfile(depth=1, durability=qos_profile_sensor_data.durability, reliability=qos_profile_sensor_data.reliability)
        self.joy_pub = self.create_publisher(bxiMsg.MotionCommands, 'motion_commands', qos)

        self.dir = -1
        self.vel_forward= 0.2
        self.vel_back= -0.2

        # Send the first command after 10 ms, then alternate every two seconds.
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        # signal.signal(signal.SIGINT, self.usr_exit)

    def timer_callback(self):
        if self.timer.timer_period_ns != 2_000_000_000:
            self.timer.timer_period_ns = 2_000_000_000
            self.timer.reset()
        
        msg = bxiMsg.MotionCommands()
        
        msg.vel_des.y = 0.
        msg.yawdot_des = 0.
        
        msg.vel_des.x = self.vel_forward if self.dir == 1 else self.vel_back
        self.dir *= -1
        self.joy_pub.publish(msg)
        print("vel: ", msg.vel_des)
    
    def usr_exit(self):
        msg = bxiMsg.MotionCommands()
        
        msg.vel_des.x = 0.
        msg.vel_des.y = 0.
        msg.yawdot_des = 0.
        
        self.joy_pub.publish(msg)
        
        print("usr exit")
        
            
def main(args=None):
   
    time.sleep(5)
    
    rclpy.init(args=args)
    node = BxiExample()
    
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        print("finally exit------------------")
        node.usr_exit()
        executor.shutdown()
        node.destroy_node()
        
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    
