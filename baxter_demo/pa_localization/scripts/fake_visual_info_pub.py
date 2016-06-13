#!/usr/bin/env python

import rospy
from pa_localization.msg import pa_location

def main():
    rospy.init_node("fake_camera")
    fake_pub=rospy.Publisher("pick_location",pa_location,queue_size=2)

    pick_location=pa_location()
    pick_location.x=0.02
    pick_location.y=0.05
    pick_location.angle=0.01

    rate=rospy.Rate(50)

    rospy.loginfo("Fake visual info is publishing...")

    while not rospy.is_shutdown():
        fake_pub.publish(pick_location)
        rate.sleep()

if __name__ == '__main__':
    main()

