#!/usr/bin/env python3
# encoding: utf-8

import os
import cv2
import numpy as np

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
  img = cv_bridge.imgmsg_to_cv2(msg)

  cv2.imshow("Original", img)

  # do dft saving as complex output
  dft = np.fft.fft2(img, axes=(0,1))

  # apply shift of origin to center of image
  dft_shift = np.fft.fftshift(dft)

  # generate spectrum from magnitude image (for viewing only)
  mag = np.abs(dft_shift)
  spec = np.log(mag) / 20

  # create circle mask
  radius = 32
  mask = np.zeros_like(img)
  cy = mask.shape[0] // 2
  cx = mask.shape[1] // 2
  cv2.circle(mask, (cx,cy), radius, (255,255,255), -1)[0]

  # blur the mask
  mask2 = cv2.GaussianBlur(mask, (19,19), 0)

  # apply mask to dft_shift
  dft_shift_masked = np.multiply(dft_shift,mask) / 255
  dft_shift_masked2 = np.multiply(dft_shift,mask2) / 255


  # shift origin from center to upper left corner
  back_ishift = np.fft.ifftshift(dft_shift)
  back_ishift_masked = np.fft.ifftshift(dft_shift_masked)
  back_ishift_masked2 = np.fft.ifftshift(dft_shift_masked2)


  # do idft saving as complex output
  img_back = np.fft.ifft2(back_ishift, axes=(0,1))
  img_filtered = np.fft.ifft2(back_ishift_masked, axes=(0,1))
  img_filtered2 = np.fft.ifft2(back_ishift_masked2, axes=(0,1))

  # combine complex real and imaginary components to form (the magnitude for) the original image again
  img_back = np.abs(img_back).clip(0,255).astype(np.uint8)
  img_filtered = np.abs(img_filtered).clip(0,255).astype(np.uint8)
  img_filtered2 = np.abs(img_filtered2).clip(0,255).astype(np.uint8)


  # cv2.imshow("ORIGINAL", img)
  # cv2.imshow("SPECTRUM", spec)
  # cv2.imshow("MASK", mask)
  # cv2.imshow("MASK2", mask2)
  # cv2.imshow("ORIGINAL DFT/IFT ROUND TRIP", img_back)
  # cv2.imshow("FILTERED DFT/IFT ROUND TRIP", img_filtered)
  # cv2.imshow("Low Frequency Filter", img_filtered2)

  cv2.imshow("Low Frequency Filter", img_filtered2)
  # img = img_filtered2.copy()

  # create white circle mask on black background and invert so black circle on white background
  radius = 32
  mask = np.zeros_like(img)
  cy = mask.shape[0] // 2
  cx = mask.shape[1] // 2
  cv2.circle(mask, (cx,cy), radius, (255,255,255), -1)[0]
  mask = 255 - mask

  # blur the mask
  mask2 = cv2.GaussianBlur(mask, (19,19), 0)

  # apply mask to dft_shift
  dft_shift_masked = np.multiply(dft_shift,mask) / 255
  dft_shift_masked2 = np.multiply(dft_shift,mask2) / 255


  # shift origin from center to upper left corner
  back_ishift = np.fft.ifftshift(dft_shift)
  back_ishift_masked = np.fft.ifftshift(dft_shift_masked)
  back_ishift_masked2 = np.fft.ifftshift(dft_shift_masked2)


  # do idft saving as complex output
  img_back = np.fft.ifft2(back_ishift, axes=(0,1))
  img_filtered = np.fft.ifft2(back_ishift_masked, axes=(0,1))
  img_filtered2 = np.fft.ifft2(back_ishift_masked2, axes=(0,1))

  # combine complex real and imaginary components to form (the magnitude for) the original image again
  # multiply by 3 to increase brightness
  img_back = np.abs(img_back).clip(0,255).astype(np.uint8)
  img_filtered = np.abs(3*img_filtered).clip(0,255).astype(np.uint8)
  img_filtered2 = np.abs(3*img_filtered2).clip(0,255).astype(np.uint8)
  cv2.imshow("High Frequency Filter", img_filtered2)

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
