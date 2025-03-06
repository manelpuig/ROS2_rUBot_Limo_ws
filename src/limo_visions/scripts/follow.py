#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from math import *


objPose = Pose()
objPose.position.x = 0.0
objPose.position.y = 0.0
objPose.position.z = 0.0

vel = Twist()
vel.linear.x = 0.0
vel.linear.y = 0.0
vel.linear.z = 0.0
vel.angular.x = 0.0
vel.angular.y = 0.0
vel.angular.z = 0.0


class follow_object(Node):
    def __init__(self):   
        super().__init__('follow_object')

        #订阅位姿信息
        self.Pose_sub = self.create_subscription(Pose,"object_detect_pose", self.poseCallback,10)
        #发布速度指令
        self.vel_pub = self.create_publisher(Twist,'cmd_vel', 5)
    def poseCallback(self,Pose):

        X = Pose.position.x
        Y = Pose.position.y
        Z = Pose.position.z


        if Z >=14500 and Z <= 15500 :                            
            vel = Twist()
        elif Z < 14500 :
            vel = Twist()
            vel.linear.x = (1.0 - Z/14000) * 0.8
            vel = Twist()
            vel.linear.x = (1.0 - Z/15000) * 0.8
        else:
            print("No Z,cannot control!")
        self.vel_pub.publish(vel)
        self.get_logger().info( "Publsh velocity command[{} m/s, {} rad/s]".format(
                        vel.linear.x, vel.angular.z))

        if X > 310 and X < 330 :
            vel = Twist()
        elif X < 310 :
            vel = Twist()
            vel.angular.z = (1.0 - X/320) 
        elif X > 330 :
            vel = Twist()
            vel.angular.z = (1.0 - X/320) 
        else:
            print("No X,cannot control!")
        self.vel_pub.publish(vel)

        self.get_logger().info( "Publsh velocity command[{} m/s, {} rad/s]".format(
                        vel.linear.x, vel.angular.z))


if __name__ == '__main__':
    print( "Starting follow")
    rclpy.init()
    node = follow_object()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()