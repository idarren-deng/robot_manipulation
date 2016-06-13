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
from copy import copy


class visual_localization:
  def __init__(self):

    self.wait=0

    self.max_area=70
    self.min_area=10
    self.max_rate=2
    self.min_rate=0.5

    self.bridge=CvBridge()
    rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
    self.image_pub=rospy.Publisher("pa_image/pick_localization",Image,queue_size=1)
    self.thread_pub=rospy.Publisher("pa_image/thresh",Image,queue_size=1)
    self.location_pub=rospy.Publisher("pick_location",pa_location,queue_size=10)

    self.pick_location=pa_location()

  def callback(self,data):

    if self.wait==0:
      rospy.sleep(3.0)
      self.wait=1

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      img_gray=cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
      img_roi=img_gray[116:346,16:361]
      thresh=cv2.adaptiveThreshold(img_roi,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,71,80)
      thresh=255-thresh
      if (LooseVersion(cv2.__version__).version[0]==2):
        contours,_=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
      else:
        _,contours,_=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

      img_roi=cv2.cvtColor(img_roi,cv2.COLOR_GRAY2BGR)

      cntrs=[]
      cnt=np.concatenate((contours[0]))
      for i in contours:
          area=cv2.contourArea(i)
          if (area>self.min_area) and (area<self.max_area):
              rect=cv2.minAreaRect(i)
              rate=rect[1][0]/rect[1][1]
              if rate>self.min_rate and rate<self.max_rate:
                  cntrs.append(i)
      try:
          cnt=np.concatenate((cntrs[0],cntrs[1],cntrs[2]))
      except:
          print("warning!")

      M=cv2.moments(cntrs[0])
      cx_0=M['m10']/M['m00']
      cy_0=M['m01']/M['m00']
      M=cv2.moments(cntrs[1])
      cx_1=M['m10']/M['m00']
      cy_1=M['m01']/M['m00']
      M=cv2.moments(cntrs[2])
      cx_2=M['m10']/M['m00']
      cy_2=M['m01']/M['m00']

      center=[(cx_0,cy_0),(cx_1,cy_1),(cx_2,cy_2)]

      cx=(cx_0+cx_1+cx_2)/3.0
      cy=(cy_0+cy_1+cy_2)/3.0
      dist=[(cx_0-cx)*(cx_0-cx)+(cy_0-cy)*(cy_0-cy),(cx_1-cx)*(cx_1-cx)+(cy_1-cy)*(cy_1-cy),(cx_2-cx)*(cx_2-cx)+(cy_2-cy)*(cy_2-cy)]
      cx=center[dist.index(min(dist))][0]
      cy=center[dist.index(min(dist))][1]

      if (LooseVersion(cv2.__version__).version[0]==2):
          [vx,vy,x,y]=cv2.fitLine(cnt,cv2.cv.CV_DIST_L2,0,0.01,0.01)
      else:
          [vx,vy,x,y]=cv2.fitLine(cnt,cv2.DIST_L2,0,0.01,0.01)

      cv2.circle(img_roi,(int(cx),int(cy)),7,(0,255,255),1)

      self.pick_location.x=cx/0.945/1000.0
      self.pick_location.y=cy/0.945/1000.0
      self.pick_location.angle=math.atan(vy/vx)

      # angle=180*math.atan(vy/vx)/math.pi
      # print(angle)

      cv2.drawContours(img_roi,cntrs[0],-1,(0,255,0))
      cv2.drawContours(img_roi,cntrs[1],-1,(0,0,255))
      cv2.drawContours(img_roi,cntrs[2],-1,(255,0,0))

    except CvBridgeError as e:
      print(e)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_roi, "bgr8"))
      self.thread_pub.publish(self.bridge.cv2_to_imgmsg(thresh, "mono8"))
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
