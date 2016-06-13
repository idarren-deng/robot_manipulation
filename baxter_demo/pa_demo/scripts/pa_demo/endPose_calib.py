#!/usr/bin/env python
import sys
import argparse
import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION
from arm_action import (computerIK, computerApproachPose)

from birl_recorded_motions import paHome_rightArm as rh
from copy import copy
import PyKDL
import tf

def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter # create ArgumentParser object
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments') # set required strings
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb')
    args = parser.parse_args(rospy.myargv()[1:]) # return objects

    # Set limb and gripper side
    limb = args.limb

    # get robot State
    print("Getting robot state...")
    rs=baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot...")
    rs.enable()
    print("Running. Ctrl-c to quit")

    # Create Joint name list
    jNamesList=['right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']

    arm=baxter_interface.limb.Limb(limb)
    current_js=[arm.joint_angle(joint) for joint in arm.joint_names()]

    rospy.loginfo('Starting endPose calib...')

    # Step 1, go to home position
    rospy.loginfo('I will first move to home posotion')
    rh.paHome_rightArm()
    rospy.sleep(3.0)

    # Create kinematics method
    kin=computerIK()
    cal_pose=computerApproachPose(limb)

    ############################
    # Origin
    x=  0.5697421207443628
    y= -0.3757602427005593
    z= -0.16661352985672628
    qx=-0.02201441500561172
    qy= 0.9997420543324004
    qz=-0.005537153050450577
    qw= 0.0007281489151126663
    ############################
    for i in range(0,5):
        for j in range(0,5):
            x=x+0.025*i
            y=y+0.025*j
            ref_p=baxter_interface.limb.Limb.Point(x,y,z)
            ref_q=baxter_interface.limb.Limb.Quaternion(qx,qy,qz,qw)
            ref_pose={'position':ref_p, 'orientation':ref_q}

            approach_J_1,approach_J_2,refJoints=cal_pose.get_approach_joints_2(ref_pose)
            arm.move_to_joint_positions(approach_J_1)
            rospy.sleep(2.0)
            arm.move_to_joint_positions(approach_J_2)
            rospy.sleep(2.0)
            arm.move_to_joint_positions(refJoints)
            log=[i,j]
            print(log)
            print(ref_pose)
            print(arm.endpoint_pose())
            rospy.sleep(2.0)
            arm.move_to_joint_positions(approach_J_2)
            rospy.sleep(2.0)
            arm.move_to_joint_positions(approach_J_1)
            rospy.sleep(2.0)

if __name__ == "__main__":
    rospy.init_node("endPose_calib")
    main()





