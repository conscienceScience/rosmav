#!/usr/bin/env python3


import cv2
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Int16
from mavros_msgs.msg import ManualControl


class LaneFollowingNode(Node):
    def __init__(self):
        super().__init__("lane_following_node")

        self.lane_sub = self.create_subscription(
            Image,
            "bluerov2/camera",
            self.lane_callback,
            10
        )

        self.control_pub = self.create_publisher(
            ManualControl,
            "bluerov2/manual_control",
            10
        )

    def lane_callback(self, msg):
        """
        This callback will take in a image massage and return how the 
        rov should move in order to align and follow the lane
        Then it will publish the desired heading and 
        the desired movement forward
        """

        img: Image = msg
        lines = detect_lines(img)
        if lines is not None:
            #find index of steepest/closest line
            slopes, intercepts = get_slopes_intercepts(lines)
            max_slope = slopes[0]
            index = i
            for i in slopes:
                if(math.abs(slopes[i]>math.abs(max_slope))):
                    max_slope = slopes[i]
                    index = i
            
            #desired angle will be same no matter where the line is
            #calculation is different is slope is positive v. negative
            desired_angle = math.atan(1/max_slope)
            if(max_slope>0):
                desired_angle = math.atan(max_slope)
            
            
            msg = ManualControl()
            msg.header = desired_angle


            if(intercepts[index] >310 and intercepts[index]<330):
                #if intercept of steepest line is near the center, go forward
                msg.x = 100
                self.control_pub.publish(msg)

            else:
                #if steepest line not near center, go sideways
                msg = ManualControl()
                msg.y = 20
                self.control_pub.publish(msg)



        else:
            #if no lines detected, turn 90 degrees
            msg = ManualControl()
            msg.header = 90
            self.control_pub.publish(msg)
            


    
def detect_lines(img, threshold1 = 50, threshold2 = 150, apertureSize = 3, minLineLength  = 100, maxLineGap = 10):
    """
    this method takes an image and detects lines
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # convert to grayscale
    edges = cv2.Canny(gray, threshold1, threshold2, apertureSize) # detect edges
    lines = cv2.HoughLinesP(
                    edges,
                    1,
                    np.pi/180,
                    100,
                    minLineLength,
                    maxLineGap,
            ) # detect lines

    return lines


def draw_lines(img, lines, color = (0, 255, 0)):
    """
    this method draws lines on a given image
    method is not used, but I don't want to delete it
    """
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), color, 2)

    plt.imshow(img)
    return img


def get_slopes_intercepts(lines):
    """
    this method takes in an array of lines 
    return an array of slopes and array of intercepts
    """
    slopes = []
    intercepts = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        s = (y2 - y1)/(x2 - x1)
        slopes.append(s)
        intercepts.append(x1-y1/s)
    return slopes, intercepts



def main(args=None):
    rclpy.init(args=args)

    node = LaneFollowingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
