#!/usr/bin/env python3
# encoding: utf-8

import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

from typing import Final

# constants
ROS_NODE_NAME: Final[str] = "publisher"

ROS_PARAM_PUB_RATE: Final[int] = 30
ROS_IMAGE_TOPIC: Final[str] = "image"


def generate_image(width = 320, height = 240):
  image: cv2.Mat = np.zeros((height, width), dtype = np.uint8)
  for x in range(height):
    for y in range(width):
      image[x][y] = np.random.randint(256)
  bridge: CvBridge = CvBridge()
  return bridge.cv2_to_imgmsg(image, encoding="mono8")


def main() -> None:
  rospy.init_node(ROS_NODE_NAME)

  pub_frequency: int = rospy.get_param("~rate", ROS_PARAM_PUB_RATE)

  # Q: Почему здесь не нужно писать rospy.resolve_name(ROS_IMAGE_TOPIC)?
  publisher = rospy.Publisher(ROS_IMAGE_TOPIC, Image, queue_size=10)

  # Обратите внимание: топик "image" может переименоваться при запуске ROS-узла.
  # rosrun project_template publisher.py image:=image_raw
  # Более подробно об этом можно узнать по ссылке: http://wiki.ros.org/Names
  rospy.loginfo(f"Publishing to '{rospy.resolve_name(ROS_IMAGE_TOPIC)}' at {pub_frequency} Hz ...")

  rate = rospy.Rate(pub_frequency)

  while not rospy.is_shutdown():
    # Задание 1: сгенерируйте случайное изображение.
    # Разрешение: 320 x 240 (ширина x высота).
    # Формат пикселей: монохром, 8-бит.
    # Создайте функцию для генерации изображения "generate_image(width = 320, height = 240)".
    publisher.publish(generate_image(width = 320, height = 240))

    rate.sleep()


if __name__ == '__main__':
    main()
