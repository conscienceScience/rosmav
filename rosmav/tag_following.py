#!/usr/bin/env python


import cv2
import matplotlib.pyplot as plt
from dt_apriltags import Detector
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Int16
from mavros_msgs.msg import ManualControl
from cv_bridge import CvBridge



class TagFollowingNode(Node):
    def __init__(self):
        super().__init__("tag_following_node")

        self.tag_sub = self.create_subscription(

            Image, 
            "bluerov2/camera",
            self.tag_callback,
            10
        )

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



    def tag_callback(self, msg):
        """
        This callback method takes an image and detects for april tags
        if there are april tags, the method will choose one of them and calculate its distance (m) from the rov
        using this information, we publish information to control the rov to move towards the tag
        """
        img:Image = msg
        tags = detect_tags(img)
        if tags is not None:
            position = get_tag_pos(tags[0])
            angle = (math.atan(position[0]/position[1]))*(180/math.pi)
            #angle in degrees
            distance = math.sqrt(position[0]**2+position[1]**2+position[2]**2)
            self.get_logger().info(f"Distance from April Tag: {distance}")
            self.get_logger().info(f"Desired heading: {angle}")

            power_heading = math.sin(angle)
            power_foward = math.sin(distance)

            manual_control_msg = ManualControl()
            manual_control_msg.header = power_heading
            manual_control_msg.y = power_foward

            self.control_pub.publish(manual_control_msg)

        else:
            self.get_logger().info("No tags detected.")
        
        
        

    



def detect_tags(img):
    """
    this method takes an image file and returns a list of tags that it detected
    """
    img = cv2.cvtColor(cv2.imread(img), cv2.COLOR_BGR2GRAY)
    at_detector = Detector(families='tag36h11',
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)
    tags = at_detector.detect(img, estimate_tag_pose=True, camera_params=[2218, 625, 1875, 1125], tag_size=0.1)

    return tags

def get_tag_coordinate(tag):
    """
    this method takes a detected tag
    and returns the coordinates of its top left corner
    """
    
    corners = tag.corners
    return corners[0]
    
def recommend_direction(center):
    """
    this method takes in the center coordinate of the closest detected tag
    and returns a direction (left, right, or forward)
    based on where the tag is in the image from the rov
    """
    
    if(center[0]<225):
        return 'left'
    elif(center[0]<400):
        return "forward"
    else:
        return "right"
    


def get_tag_pos(tag):
    """
    return the translational vector of how far away the tag is
    in an array
    """
    return (tag.pose_t)

