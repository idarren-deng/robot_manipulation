#!/usr/bin/env python
from __future__ import print_function
import ipdb
import sys
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class calibrationAssistant():

    def __init__(self):
      self.bridge = CvBridge()
      self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
      self.image_pub = rospy.Publisher("pa_image/calibration",Image)

    def callback(self,data):
      try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        cv2.line(cv_image,(16,0),(16,480),(0,255,0),1)
        cv2.line(cv_image,(39,0),(39,480),(0,255,0),1)
        cv2.line(cv_image,(0,116),(640,116),(0,255,0),1)
        cv2.line(cv_image,(0,139),(640,139),(0,255,0),1)
      except CvBridgeError as e:
        print(e)

      try:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      except CvBridgeError as e:
        print(e)

def main(args):
  ca=calibrationAssistant()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node("calibration_assistant")
    main(sys.argv)