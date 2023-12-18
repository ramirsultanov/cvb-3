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

ROS_NODE_NAME: Final[str] = "subscriber_py"
ROS_PACKAGE_PATH: Final[os.PathLike] = rospkg.RosPack().get_path("template")

ROS_IMAGE_TOPIC_SUB: Final[str] = "image"
ROS_IMAGE_TOPIC_PUB: Final[str] = "image_resized"

RESIZE_SIZE = (128, 128)


def image_callback(msg: Image, cv_bridge: CvBridge, pub: rospy.Publisher) -> None:
  image = cv_bridge.imgmsg_to_cv2(msg)
  cv2.imshow('image', image)
  cv2.waitKey()
  imageResized = cv2.resize(image, RESIZE_SIZE)
  cv2.imshow('imageResized', imageResized)
  cv2.waitKey()
  pub.publish(cv_bridge.cv2_to_imgmsg(image, encoding="mono8"))


def main() -> None:
  rospy.init_node(ROS_NODE_NAME)

  rospy.loginfo(f"ROS package: {ROS_PACKAGE_PATH}")
  rospy.loginfo(f"OpenCV version: {cv2.__version__}")

  # wait for the images to arrive or throw an exception
  sample: Image = rospy.wait_for_message(ROS_IMAGE_TOPIC_SUB, Image, timeout=3.0)

  if sample is not None:
    rospy.loginfo(f"Encoding: {sample.encoding}, Resolution: {sample.width, sample.height}")

  cv_bridge: CvBridge = CvBridge()

  pub: rospy.Publisher = rospy.Publisher(ROS_IMAGE_TOPIC_PUB, Image, queue_size=10)

  rospy.Subscriber(ROS_IMAGE_TOPIC_SUB, Image, lambda msg: image_callback(msg, cv_bridge, pub), queue_size=None)

  rospy.spin()


if __name__ == '__main__':
  main()
