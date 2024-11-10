#!/usr/bin/env python3

import sys
import cv2
import numpy 
import rclpy
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
import time

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')    
        self.bridge = CvBridge() 
        self.subscriber = self.create_subscription(Image, '/camera1/image_raw', self.process_data, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        self.velocity = Twist()
        self.empty = False
        self.error = 0 
        self.action = ""
        self.get_logger().info("Node Started!")

        # Set a timer for publishing velocity commands at 10 Hz
        self.timer = self.create_timer(0.1, self.send_cmd_vel)
        self.start_time = time.time()  # Track the start time
        self.straight_duration = 1.0  # Duration for each straight movement phase (1 second)
        self.turn_duration = 0.5  # Duration for each turning phase (0.5 seconds)
        self.state = "move_forward"  # Initial state (either move forward or turn)
        self.last_switch_time = time.time()

    def send_cmd_vel(self):
        current_time = time.time()
        elapsed_time = current_time - self.last_switch_time

        # Switch between moving straight and turning
        if self.state == "move_forward":
            self.velocity.linear.x = 0.1  # Forward speed
            self.velocity.angular.z = 0.0  # No turning
            if elapsed_time >= self.straight_duration:
                # Time to switch to turning
                self.state = "turn"
                self.last_switch_time = current_time  # Reset the timer for the turn phase
                self.get_logger().info("Switching to turn")

        elif self.state == "turn":
            self.velocity.linear.x = 0.0  # No forward motion
            self.velocity.angular.z = 0.8  # Angular speed to turn
            if elapsed_time >= self.turn_duration:
                # Time to switch to moving forward
                self.state = "move_forward"
                self.last_switch_time = current_time  # Reset the timer for the move phase
                self.get_logger().info("Switching to move forward")

        # Publish the current velocity command
        self.publisher.publish(self.velocity)

    ## Subscriber Call Back

    def process_data(self, data): 
        self.get_logger().info("Image Received!")
        frame = self.bridge.imgmsg_to_cv2(data) 
        light_line = numpy.array([100, 0, 0])
        dark_line = numpy.array([160, 20, 20])
        mask = cv2.inRange(frame, light_line, dark_line)
        cv2.imshow('mask', mask)

        canny = cv2.Canny(mask, 30, 5)
        cv2.imshow('edge', canny)

        r1 = 200
        c1 = 0
        img = canny[r1:r1 + 200, c1:c1 + 512]
        cv2.imshow('crop', img)

        edge = []
        row = 150

        for i in range(512):
            if img[row, i] == 255:
                edge.append(i)
        print(edge)

        if len(edge) == 0:
            left_edge = 512 // 2
            right_edge = 512 // 2
            self.empty = True
            
        if len(edge) == 1:
            if edge[0] > 512 // 2:
                left_edge = 0
                right_edge = edge[0] 
                self.empty = False
            else:
                left_edge = edge[0]
                right_edge = 512
                self.empty = False               

        if len(edge) == 2:
            left_edge = edge[0]
            right_edge = edge[1] 
            self.empty = False       
            
        if len(edge) == 3:
            if edge[1] - edge[0] > 5: 
                left_edge = edge[0]
                right_edge = edge[1]
                self.empty = False
            else:
                left_edge = edge[0]
                right_edge = edge[2]
                self.empty = False
                
        if len(edge) == 4:
            left_edge = edge[0]
            right_edge = edge[2]
            self.empty = False
                
        if len(edge) >= 5:
            left_edge = edge[0]
            right_edge = edge[len(edge) - 1]
            self.empty = False           
    
        road_width = (right_edge - left_edge)
        frame_mid = left_edge + (road_width / 2)
        mid_point = 512 / 2
        img[row, int(mid_point)] = 255
        print(mid_point)
        self.error = mid_point - frame_mid 
        img[row, int(frame_mid)] = 255
        print(self.action)
        f_image = cv2.putText(img, self.action, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.imshow('frame', f_image)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
