#!/usr/bin/env python

# -*- encoding: UTF-8 -*-

##  @file speech.py
#   @brief File storing information about the speech services.

##  @package speech
#   @brief Package storing information about the speech services.

import sys
import os
import re
import subprocess
import time
import math
import numpy

# OpenCV stuff
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from guessing_game_new.srv import *


import rospy
from threading import Lock

# path_to_darknet = '/Users/rootmac/Documents/workspace/darknet/'

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("kinect2/qhd/image_color", Image, self.image_cb)
    self.image_service = rospy.Service("image_request", image_request, self.fetch_image)
    self.image = None
    self.lock = Lock()

  def predict_label(self,path):
    # change working dir and run darknet
    # in current version, darknet has to be installed in home directory (~/darknet/)
    # call the bash script install_darknet.sh to install darknet
    os.chdir(os.path.join(os.path.expanduser("~"), "darknet/"))
    command = './darknet classifier predict cfg/imagenet22k.dataset cfg/extraction.cfg extraction.weights ' + path
    val = subprocess.check_output(command.split(' '))

    # postprocess the results and return
    val = val.split('\n')
    ret = [d for d in val if re.search('[a-z]+: [\d.]+', d)]

    return {d.split(': ')[0] : float(d.split(': ')[1]) for d in ret}

  def fetch_image(self,req):
    while self.image is None:
      rospy.loginfo("Self.image does not exist yet. Waiting for 5 secs")
      rospy.sleep(5)

    self.lock.acquire()
    cv2.imwrite("in.png",self.image)
    ret = str(self.predict_label("in.png"))
    rospy.loginfo(ret)
    self.lock.release()
    return image_requestResponse(ret)

  def image_cb(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    if not self.lock.locked():
      self.image = cv_image
      cv2.imshow("Image window", self.image)
      cv2.waitKey(1)


if __name__ == "__main__":
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    rospy.loginfo("Vision node running...")
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()