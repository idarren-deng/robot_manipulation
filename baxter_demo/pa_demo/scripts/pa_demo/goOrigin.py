#!/usr/bin/env python
import sys
import argparse
import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION
from hand_action import GripperClient
from arm_action import (computerIK, computerApproachPose)

from birl_recorded_motions import paHome_rightArm as rh
from copy import copy
import PyKDL

#------------------------------------ Design Parameters ------------------------------
## Flags
reference_origin_pose_flag = 1 		# Used to determine whether to used the saved joint angle data
                                    # for the reference origin position (true) or not (false).
reference_placing_pose_flag = 1 	# Used to determine whether to used the saved joint angle data

def main():
    global reference_origin_pose_flag, reference_placing_pose_flag

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
    ha=GripperClient(limb)

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

    rospy.loginfo('Starting pa demo...')

    # Step 1, go to home position
    rospy.loginfo('Step 1: I will first move to home position, and open the gripper...')
    rh.paHome_rightArm()
    ha.open()
    rospy.sleep(1.0)

    # Create kinematics method
    kin=computerIK()
    cal_pose=computerApproachPose(limb)

    # Get Starting Position
    startPose=arm.endpoint_pose()
    startJoints=kin.calIK_PY_KDL(limb,startPose)

    #--------------- Get starting and Set the reference origin positions -----------------------
    if reference_origin_pose_flag:    # Used saved reference origin position

        ############################
        x=  0.5697421207443628
        y= -0.3757602427005593
        z= -0.16661352985672628
        qx=-0.02201441500561172
        qy= 0.9997420543324004
        qz=-0.005537153050450577
        qw= 0.0007281489151126663
        ############################

        origin_p=baxter_interface.limb.Limb.Point(x,y,z)
        origin_q=baxter_interface.limb.Limb.Quaternion(qx,qy,qz,qw)
        origin_pose={'position':origin_p, 'orientation':origin_q}

    else:  # Use reference origin position from manual teleoperation
        rospy.logwarn('Be careful! Using new reference origin position.\n Usage: Open a new terminal and use keyboard teleoperation: '
                      'roslaunch baxter_end_effector_control end_effector_control.launch keyboard:=true \n')
        key=raw_input('When finished, pres any key...')
        rospy.loginfo('New reference origin position recorded')
        origin_pose=arm.endpoint_pose()
        reference_origin_pose_flag=1

        # Back to home position
        rospy.loginfo('I will move back to home position, and open the gripper...')
        rh.paHome_rightArm()
        ha.open()
        rospy.sleep(1.0)

    originJoints=kin.calIK_PY_KDL(limb,origin_pose)

    arm.move_to_joint_positions(originJoints)
    rospy.sleep(2.0)

if __name__ == "__main__":
    rospy.init_node("go_Origin")
    main()
