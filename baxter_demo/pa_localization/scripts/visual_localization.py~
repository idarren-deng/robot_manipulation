#!/usr/bin/env python
from __future__ import print_function
import ipdb
import numpy as np
import math
import sys
import rospy
import cv2
from distutils.version import LooseVersion

from std_msgs.msg import String
from sensor_msgs.msg import Image
from pa_localization.msg import pa_location
from cv_bridge import CvBridge, CvBridgeError

class visual_localization:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
    self.image_pub = rospy.Publisher("pa_image/pick_localization",Image)
    self.location_pub = rospy.Publisher("pick_location",pa_location)

    self.pick_location=pa_location()

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      img_gray=cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
      img_roi=img_gray[116:346,16:361]
      ret,thresh=cv2.threshold(img_roi,80,255,cv2.THRESH_BINARY)
      if (LooseVersion(cv2.__version__).version[0]==2):
          contours,_=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
      else:
          _,contours,_=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
      c_max=max(contours,key=cv2.contourArea)
      img_roi=cv2.cvtColor(img_roi,cv2.COLOR_GRAY2BGR)

      rect=cv2.minAreaRect(c_max)
      if (LooseVersion(cv2.__version__).version[0]==2):
          box=cv2.cv.BoxPoints(rect)
      else:
          box=cv2.boxPoints(rect)
      box=np.int0(box)

      self.pick_location.x=rect[0][0]/23.0*25.0
      self.pick_location.y=rect[0][1]/23.0*25.0
      self.pick_location.angle=rect[2]

      cv2.drawContours(img_roi,c_max,-1,(255,0,0))
      cv2.drawContours(img_roi,[box],0,(0,0,255),1)

    except CvBridgeError as e:
      print(e)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_roi, "bgr8"))
      self.location_pub.publish(self.pick_location)
    except CvBridgeError as e:
      print(e)

def main(args):
  vl=visual_localization()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node("pa_localization")
    main(sys.argv)