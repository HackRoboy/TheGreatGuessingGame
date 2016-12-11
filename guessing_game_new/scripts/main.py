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
import rospy

# OpenCV stuff
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from imitation_game.srv import * #IMPORT YOUR SERVICES
from imitation_game.msg import * #IMPORT YOUR MESSAGES

# path_to_darknet = '/Users/rootmac/Documents/workspace/darknet/'

class main_node:

  def __init__(self):

    self.bridge = CvBridge()
    self.vision_sub = rospy.Subscriber("image_classification", String, self.classification_cb)
    self.speech_sub = rospy.Subscriber("gg_speech", Image, self.speech_cb)

  def classification_cb(self,data):
    # do something

  def speech_cb(self,data):
    # do something    

if __name__ == "__main__":
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()