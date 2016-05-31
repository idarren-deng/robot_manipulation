#!/usr/bin/env python

# Makes both the right and the left arm go home from which ever position they are at.
#import pdb 
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from birl_recorded_motions import GoHome as gh

def shutdown():
    rospy.loginfo("Node has been terminated. Closing gracefully.")
    rospy.sleep(5)

if __name__ == '__main__':
    try: 
        # If you want to debug, uncomment next line and import pdb
        #pdb.set_trace()

        # Init the ROS node
        rospy.init_node("go_home")        

        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(shutdown)

        # Enable the robot's arms
        print("Getting robot state...")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        rs.state().enabled
        
        print("Enabling robot...")
        rs.enable()

        if not rospy.is_shutdown():
            gh.GoHome()        

    except:
        rospy.loginfo("Exception thrown...")
    

