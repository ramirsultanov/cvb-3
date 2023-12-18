#!/usr/bin/env python3
# encoding: utf-8

import os
import cv2

import rospy
import rospkg

# http://wiki.ros.org/cv_bridge
from cv_bridge import CvBridge

# http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
from sensor_msgs.msg import Image

from typing import Final

# constants
ROS_NODE_NAME: Final[str] = "subscriber_py"
ROS_PACKAGE_PATH: Final[os.PathLike] = rospkg.RosPack().get_path("template")

ROS_IMAGE_TOPIC_SUB: Final[str] = "image"


def image_callback(msg: Image, cv_bridge: CvBridge) -> None:
  image = cv_bridge.imgmsg_to_cv2(msg)
  img_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
  img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
  h, s, v = cv2.split(img_hsv)
  cv2.imshow('hue', h)
  cv2.imshow('saturation', s)
  cv2.imshow('value', v)
  cv2.waitKey()


def main() -> None:
  rospy.init_node(ROS_NODE_NAME)

  rospy.loginfo(f"ROS package: {ROS_PACKAGE_PATH}")
  rospy.loginfo(f"OpenCV version: {cv2.__version__}")

  # wait for the images to arrive or throw an exception
  sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC_SUB, Image, timeout=3.0)

  if sample is not None:
    rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")

  cv_bridge: CvBridge = CvBridge()

  rospy.Subscriber(ROS_IMAGE_TOPIC_SUB, Image, lambda msg: image_callback(msg, cv_bridge), queue_size=None)

  rospy.spin()


if __name__ == '__main__':
  main()
