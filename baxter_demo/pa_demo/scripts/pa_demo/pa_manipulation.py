#!/usr/bin/env python
import sys
import argparse
import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION
from hand_action import GripperClient
from arm_action import (computerIK, computerApproachPose)

from pa_localization.msg import pa_location
from birl_recorded_motions import paHome_rightArm as rh
from copy import copy
import PyKDL
import tf

import threading

#------------------------------------ Design Parameters ------------------------------
## Flags
reference_origin_pose_flag = 1 		# Used to determine whether to used the saved joint angle data
                                    # for the reference origin position (true) or not (false).
reference_placing_pose_flag = 1 	# Used to determine whether to used the saved joint angle data
                                    # for the reference placing position (true) or not (false).

class Thread(threading.Thread):
    def __init__(self):
        # Subscribe the pickingPose from pa_localization
        self._pickingPose_sub=rospy.Subscriber("pick_location",pa_location,self.callback)
        threading.Thread.__init__(self)
        self.start()
        self._lock=threading.Lock()

    def callback(self,pose):
        self._pose=pose

    def getPose(self):
        with self._lock:
            rospy.sleep(1.0)
            return self._pose

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
    rospy.loginfo('Step 1: I will first move to home posotion, and open the gripper...')
    rh.paHome_rightArm()
    ha.open()
    rospy.sleep(1.0)

    # Create kinematics method
    kin=computerIK()
    cal_pose=computerApproachPose(limb)

    # Get Starting Position
    startPose=arm.endpoint_pose()
    startJoints=kin.calIK_PY_KDL(limb,startPose)

    # Put the male part on the table
    rospy.loginfo('Please put the male part on the table, and I will capture the location for picking')
    key=raw_input('When finished, press any key...')

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

    else:    # Use reference origin position from manual teleoperation
        rospy.logwarn('Be careful! Using new reference origin postion.\n Usage: Open a new terminal and use keyboard teleoperation: '
                      'roslaunch baxter_end_effector_control end_effector_control.launch keyboard:=true \n')
        key=raw_input('When finished, pres any key...')
        rospy.loginfo('New reference origin position recorded')
        origin_pose=arm.endpoint_pose()
        reference_origin_pose_flag=1

        # Back to home position
        rospy.loginfo('I will move back to home posotion, and open the gripper...')
        rh.paHome_rightArm()
        ha.open()
        rospy.sleep(1.0)

    ########################################################
    #    Do: pa_localization
    #    require: reference_origin_pose
    #    return:  picking_pose---{'position': picking_p, 'orientation': picking_q}
    ########################################################
    pp_thread=Thread()
    picking_pose_visual=pp_thread.getPose()
    print(picking_pose_visual)

    picking_pose=copy(origin_pose) # refering picking_pose from pa_localization to origin

    ref_x=copy(picking_pose['position'][0])
    ref_y=copy(picking_pose['position'][1])
    ref_z=copy(picking_pose['position'][2])

    ref_q=picking_pose['orientation']
    rot_mat=PyKDL.Rotation.Quaternion(ref_q.x,ref_q.y,ref_q.z,ref_q.w)
    rot_rpy=rot_mat.GetRPY()
    rot_rpy=list(rot_rpy)

    p_x=ref_x+picking_pose_visual.x
    p_y=ref_y+picking_pose_visual.y
    p_z=ref_z
    q=tf.transformations.quaternion_from_euler(rot_rpy[0],rot_rpy[1],rot_rpy[2]+picking_pose_visual.angle,axes='sxyz').tolist()
    picking_p=baxter_interface.limb.Limb.Point(p_x,p_y,p_z)
    picking_q=baxter_interface.limb.Limb.Quaternion(q[0],q[1],q[2],q[3])
    picking_pose={'position': picking_p, 'orientation': picking_q}

    #--------------- Picking Process -----------------------
    # Calculate ApproachPose
    approach_J_1,approach_J_2,pickingJoints=cal_pose.get_approach_joints_2(picking_pose)

    print(arm.endpoint_pose())

    rospy.loginfo('Start picking...')
    arm.move_to_joint_positions(approach_J_1)
    rospy.sleep(2.0)
    print(arm.endpoint_pose())
    arm.move_to_joint_positions(approach_J_2)
    rospy.sleep(2.0)
    print(arm.endpoint_pose())
    arm.move_to_joint_positions(pickingJoints)
    rospy.sleep(2.0)
    print(arm.endpoint_pose())

    # close gripper
    ha.close()
    rospy.sleep(2.0)

    rospy.loginfo('Lift up...')
    arm.move_to_joint_positions(approach_J_2)
    rospy.sleep(2.0)
    arm.move_to_joint_positions(approach_J_1)
    rospy.sleep(2.0)

    #--------------- Set the reference placing positions -----------------------
    if reference_placing_pose_flag:    # Used saved reference origin position

        ############################
        x=  0.5697421207443628
        y= -0.3757602427005593
        z= -0.16661352985672628
        qx=-0.02201441500561172
        qy= 0.9997420543324004
        qz=-0.005537153050450577
        qw= 0.0007281489151126663
        ############################

        placing_p=baxter_interface.limb.Limb.Point(x,y,z)
        placing_q=baxter_interface.limb.Limb.Quaternion(qx,qy,qz,qw)
        placing_pose={'position':placing_p, 'orientation':placing_q}

    else:    # Use reference placing position from manual teleoperation
        rospy.logwarn('Be careful! Using new reference placing postion.\n Usage: Open a new terminal and use keyboard teleoperation: '
                      'roslaunch baxter_end_effector_control end_effector_control.launch keyboard:=true \n')
        key=raw_input('When finished, pres any key...')
        rospy.loginfo('New reference placing position recorded')
        placing_pose=arm.endpoint_pose()
        reference_placing_pose_flag=1

        # Back to home position
        rospy.loginfo('I will move back to home posotion, and open the gripper...')
        rh.paHome_rightArm()
        ha.open()
        rospy.sleep(1.0)

    #--------------- Placing Process -----------------------
    # Calculate ApproachPose
    approach_J_1,approach_J_2,placingJoints=cal_pose.get_approach_joints_2(placing_pose)

    print(arm.endpoint_pose())

    rospy.loginfo('Start placing...')
    arm.move_to_joint_positions(approach_J_1)
    rospy.sleep(2.0)
    print(arm.endpoint_pose())
    arm.move_to_joint_positions(approach_J_2)
    rospy.sleep(2.0)
    print(arm.endpoint_pose())
    arm.move_to_joint_positions(placingJoints)
    rospy.sleep(2.0)
    print(arm.endpoint_pose())

    # open gripper
    ha.open()
    rospy.sleep(2.0)

    rospy.loginfo('Release and leave...')
    arm.move_to_joint_positions(approach_J_2)
    rospy.sleep(2.0)
    arm.move_to_joint_positions(approach_J_1)
    rospy.sleep(2.0)

if __name__ == "__main__":
    rospy.init_node("pa_manipulation")
    main()
