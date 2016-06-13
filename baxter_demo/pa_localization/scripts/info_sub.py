#!/usr/bin/env python

import rospy
from pa_localization.msg import pa_location

class infoSub():
    def __init__(self):
        self._info_sub=rospy.Subscriber("pick_location",pa_location,self.callback)
        self._pose=pa_location()

    def callback(self,pose):
        self._pose=pose

    def getPose(self):
        return self._pose

if __name__ == '__main__':
    rospy.init_node("Info_sub")
    info_sub=infoSub()
    rospy.spin()