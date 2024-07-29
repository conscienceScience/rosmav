#!/usr/bin/env python


import cv2
import matplotlib.pyplot as plt
from dt_apriltags import Detector

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16
from mavros_msgs.msg import ManualControl






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





tags = detect_tags('tag3.png')
tag = tags[1]
pos = get_tag_pos(tag)
print(pos)