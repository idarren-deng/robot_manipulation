#!/usr/bin/env python

#------------------------------------ Imports ------------------------------
import ipdb

import argparse
import sys

import tf
import rospy


# Pose Stamped and Transformation
from geometry_msgs.msg import (
    PoseStamped,
    Quaternion,
)

from rbx1_nav.transform_utils import quat_to_angle

# Kinematics
from baxter_pykdl import baxter_kinematics
import PyKDL
from math import pi

# Baxter Messages
# Baxter Stuff
import baxter_interface
from baxter_interface import CHECK_VERSION


def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter # create ArgumentParser object
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments') # set required strings
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='send joint trajectory to which limb'
    )
    args = parser.parse_args(rospy.myargv()[1:]) # return objects

    # Set limb and gripper side
    limb = args.limb

    # Get Current Joint Positions first and command them
    arm = baxter_interface.limb.Limb(limb)
    current_angles = [arm.joint_angle(joint) for joint in arm.joint_names()]

    referencePose=arm.endpoint_pose()
    print(referencePose['position'])
    print(referencePose['orientation'])

    qref=referencePose['orientation']
    rot_mat=PyKDL.Rotation.Quaternion(qref.x,qref.y,qref.z,qref.w)
    rot_goal=rot_mat.GetRPY()
    rot_goal=list(rot_goal)
    print(rot_goal)

    q_goal=tf.transformations.quaternion_from_euler(0,0,0,axes='sxyz').tolist()
    q_goal=baxter_interface.limb.Limb.Quaternion(q_goal[0],q_goal[1],q_goal[2],q_goal[3])
    print(q_goal)


if __name__ == "__main__":
    rospy.init_node("get_pose")
    main()
