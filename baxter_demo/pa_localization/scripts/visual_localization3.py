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

    self.bridge = CvBridge()
    rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
    self.image_pub=rospy.Publisher("pa_image/pick_localization",Image,queue_size=1)
    self.thread_pub=rospy.Publisher("pa_image/thresh",Image,queue_size=1)
    self.location_pub_1=rospy.Publisher("pick_location_1",pa_location,queue_size=5)
    self.location_pub_2=rospy.Publisher("pick_location_2",pa_location,queue_size=5)

    self.pick_location0=pa_location()
    self.pick_location1=pa_location()

  def callback(self,data):

    if self.wait==0:
      rospy.sleep(1.0)
      self.wait=1

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      img_gray=cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
      img_roi=img_gray[116:350,16:400]
      img_0=copy(img_roi)
      # _,thresh=cv2.threshold(img_roi,100,255,cv2.THRESH_BINARY)
      thresh=cv2.adaptiveThreshold(img_roi,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,111,60)
      thresh=255-thresh
      th_temp=copy(thresh)
      if (LooseVersion(cv2.__version__).version[0]==2):
        contours,_=cv2.findContours(th_temp,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
      else:
        _,contours,_=cv2.findContours(th_temp,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

      ##### leap motion detection
      cnt_leap=max(contours,key = cv2.contourArea)
      M=cv2.moments(cnt_leap)
      cx_leap=M['m10']/M['m00']
      cy_leap=M['m01']/M['m00']
      if (LooseVersion(cv2.__version__).version[0]==2):
        [vx_leap,vy_leap,x_leap,y_leap]=cv2.fitLine(cnt_leap,cv2.cv.CV_DIST_L2,0,0.01,0.01)
      else:
        [vx_leap,vy_leap,x_leap,y_leap]=cv2.fitLine(cnt_leap,cv2.DIST_L2,0,0.01,0.01)
      self.pick_location0.x=(cx_leap-14.0)/0.933/1000.0
      self.pick_location0.y=(cy_leap-6.0)/0.933/1000.0
      self.pick_location0.angle=math.atan(vy_leap/vx_leap)
      angle0=180*math.atan(vy_leap/vx_leap)/math.pi

      rows,cols = img_0.shape[:2]
      lefty0 = int((-x_leap*vy_leap/vx_leap) + y_leap)
      righty0 = int(((cols-x_leap)*vy_leap/vx_leap)+y_leap)

      img_0=cv2.cvtColor(img_0,cv2.COLOR_GRAY2BGR)
      #cv2.drawContours(img_0,cnt_leap,-1,(0,255,0))
      cv2.circle(img_0,(int(cx_leap),int(cy_leap)),5,(0,255,0),1)
      if(abs(abs(angle0)-90)>0.5):
        cv2.line(img_0,(cols-1,righty0),(0,lefty0),(0,255,0),1)
      else:
        cv2.line(img_0,(int(cx_leap),0),(int(cx_leap),rows),(0,255,0),1)

      ## box detection
      cntrs=[]
      if(len(contours)>0):
      ##  cnt=np.concatenate(contours[0])
        for i in contours:
            area=cv2.contourArea(i)
            if (area>self.min_area) and (area<self.max_area):
                rect=cv2.minAreaRect(i)
                rate=rect[1][0]/rect[1][1]
                if rate>self.min_rate and rate<self.max_rate:
                    cntrs.append(i)
      if(len(cntrs)>=3):
        cnt_box=np.concatenate((cntrs[0],cntrs[1],cntrs[2]))
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
        cx_box=(cx_0+cx_1+cx_2)/3.0
        cy_box=(cy_0+cy_1+cy_2)/3.0
        dist=[(cx_0-cx_box)*(cx_0-cx_box)+(cy_0-cy_box)*(cy_0-cy_box),(cx_1-cx_box)*(cx_1-cx_box)+(cy_1-cy_box)*(cy_1-cy_box),(cx_2-cx_box)*(cx_2-cx_box)+(cy_2-cy_box)*(cy_2-cy_box)]
        cx_box=center[dist.index(min(dist))][0]
        cy_box=center[dist.index(min(dist))][1]

        if (LooseVersion(cv2.__version__).version[0]==2):
          [vx_box,vy_box,x_box,y_box]=cv2.fitLine(cnt_box,cv2.cv.CV_DIST_L2,0,0.01,0.01)
        else:
          [vx_box,vy_box,x_box,y_box]=cv2.fitLine(cnt_box,cv2.DIST_L2,0,0.01,0.01)

        self.pick_location1.x=cx_box/0.985/1000.0
        self.pick_location1.y=cy_box/0.985/1000.0
        self.pick_location1.angle=math.atan(vy_box/vx_box)
        angle1=180*math.atan(vy_box/vx_box)/math.pi

        rows,cols = img_0.shape[:2]
        lefty1 = int((-x_box*vy_box/vx_box) + y_box)
        righty1 = int(((cols-x_box)*vy_box/vx_box)+y_box)

        #cv2.drawContours(img_0,cnt_box,-1,(0,255,0))
        cv2.circle(img_0,(int(cx_box),int(cy_box)),5,(0,0,255),1)
        if(abs(abs(angle1)-90)>0.5):
          cv2.line(img_0,(cols-1,righty1),(0,lefty1),(0,0,255),1)
        else:
          cv2.line(img_0,(int(cx_box),0),(int(cx_box),rows),(0,0,255),1)

    except CvBridgeError as e:
      print(e)

    img_gray=cv2.cvtColor(img_gray,cv2.COLOR_GRAY2BGR)
    img_gray[116:350,16:400]=img_0
    cv2.imshow("Preview",thresh)
    cv2.imshow("Preview2",img_gray)
    # cv2.imwrite("/home/darren/baxter_ws/src/baxter_demo/pa_localization/scripts/2.bmp",img_gray)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_0, "bgr8"))
      self.thread_pub.publish(self.bridge.cv2_to_imgmsg(thresh, "mono8"))
      self.location_pub_1.publish(self.pick_location0)
      self.location_pub_2.publish(self.pick_location1)
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
