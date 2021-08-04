#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")

    args = parser.parse_args()
    # bag_file = "/home/rishabh/quad_ws/sara_sim.bag"
    # image_topic = "/drone1/front_cam/camera/image"
    # output_dir = "drone1_cam"

    print("Extract images from {} on topic {} into {}".format(args.bag_file,
                                                              args.image_topic, args.output_dir))

    bag = rosbag.Bag(args.bag_file, "r")
    print("loaded bag")
    bridge = CvBridge()
    count = 0
    print("starting loop")
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        print("loaded image")
        # print("Time: {}".format(t))
        # line_thickness = 1
        # cv2.line(cv_img, (cv_img.shape[1]/2, 0), (cv_img.shape[1]/2, cv_img.shape[0]), (128, 0, 0),
        #          thickness=line_thickness)
        # cv2.line(cv_img, (0, cv_img.shape[0] / 2), (cv_img.shape[1], cv_img.shape[0]/2), (128, 0, 0),
        #          thickness=line_thickness)
        cv2.imwrite(os.path.join(args.output_dir, "{}.png".format(count)), cv_img)
        print("Wrote image {}".format(count))

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()