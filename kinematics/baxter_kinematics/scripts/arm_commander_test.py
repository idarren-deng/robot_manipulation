#!/usr/bin/env python

import ipdb

import argparse
import sys
import rospy

import baxter_interface
import arm_commander


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

    arm_cmd=arm_commander.ArmCommander(limb)

    print arm_cmd.get_fk()


if __name__ == "__main__":
    rospy.init_node("arm_commander_test")
    main()