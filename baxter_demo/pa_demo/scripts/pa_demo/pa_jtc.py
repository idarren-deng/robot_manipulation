#!/usr/bin/env python
#------------------------------------ Imports ------------------------------
import ipdb

import argparse
import sys

import tf
import rospy
import actionlib

from copy import copy

# Action modules: gripper and trajectory
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

# Open and Close Grippers 
import pa_closeHand
import pa_openHand

# Moving Arms to Home position
from birl_recorded_motions import paHome_rightArm as gh

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

from baxter_core_msgs.msg import (
    EndpointState,
    JointCommand,
)

# Joint Trajectory Action Cient/Server Messages
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
) 

#------------------------------------ Design Parameters ------------------------------
## Flags
reference_pose_flag = 0 		# Used to determine whether to used saved joint angle data for the reference position (true) or not (false).

#Inverse Kinematics Computation
PY_KDL=0
TRAC_IK=1
kinematics_flag=PY_KDL 			# Used to determine wheter to use kdl or trac_ik for kinematics

# Movement Algorithm
MOVE_JNT_PSTN=0
JOINT_ACT_CLN=1
approach_flag=MOVE_JNT_PSTN         	# Used to determine whether to use move_to_joint_positions or joint_trajectory_action_client approach

# Globals 
_joints = JointCommand()
#------------------------------------ Class ___________ ------------------------------
class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient( # create simple client w/ topic and msg type
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal() 		# trajectory(header/points);
								# path_tolerance
                                                                # goal_tolerance 
                                                                # goal_time_tolerance
        self._goal_time_tolerance = rospy.Time(0.1) 		# Reach goal within this tolerance of final time.
        self._goal.goal_time_tolerance = self._goal_time_tolerance # Assign to goal object. 
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0)) # Connect to server within this time. 
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

#------------------------------------ Global Methods ------------------------------
def callback(joints):
    joints=JointCommand()
    _joints.mode=joints.mode
    _joints.names=copy(joints.names)
    _joints.command=copy(joints.command)
    

def calcInvKin(posePub,currPose,limb):
    """ This function converts an EndpointState to a PoseStamped and publishes the data, which is used by the track_IK node to produce a 7 dof joint angle solution of type JointState's."""

    # Convert EndpointState to PoseStamped
    tempPose = PoseStamped()
    tempPose.header.stamp=rospy.Time.now()
    tempPose.header.frame_id=limb

    # Copy the position from a dictionary to a xyz
    tempPose.pose.position.x=currPose['position'][0]
    tempPose.pose.position.y=currPose['position'][1]
    tempPose.pose.position.z=currPose['position'][2]

    # Copy the orientation from the dictionary to xyzw
    tempPose.pose.orientation.x=currPose['orientation'][0]
    tempPose.pose.orientation.y=currPose['orientation'][1]
    tempPose.pose.orientation.z=currPose['orientation'][2]
    tempPose.pose.orientation.w=currPose['orientation'][3]

    # Publish
    posePub.publish(tempPose)
    posePub.publish(tempPose)

    # Get a single message of type baxter_core_msgs/JointCommand
    _joints = rospy.wait_for_message("joints", JointCommand)#timeout, default=none

    # return JointCommand
    return _joints

def main():
    """RSDK Joint Trajectory Example: Simple Action Client

    Creates a client of the Joint Trajectory Action Server
    to send commands of standard action type,
    control_msgs/FollowJointTrajectoryAction.

    Make sure to start the joint_trajectory_action_server.py
    first. Then run this example on a specified limb to
    command a short series of trajectory points for the arm
    to follow.
    """
    global reference_pose_flag # Will use this global variable
    reference_pose_flag=1  # we use the saved pose
    global _joints
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

    #gripper = args.limb

    print("Initializing node pa_jtc_tracIK... ")
    rospy.init_node("pa_jtc_tracIK")

    # Create publisher and Subscriber only if we use TRAC_IK
    if kinematics_flag==TRAC_IK:
        posePub=rospy.Publisher("pose",PoseStamped,queue_size=2)
        rospy.Subscriber("joints",JointCommand,callback)

    # Create Kinematic Objects
    kin = baxter_kinematics(limb)

    # Get robot State
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    
    # Create Joint Names List
    jNamesl=['right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
      
    # Get Current Joint Positions first and command them
    arm = baxter_interface.limb.Limb(limb)    
    current_angles = [arm.joint_angle(joint) for joint in arm.joint_names()]

    # Create open and close gripper object
    pc=pa_closeHand.GripperClient(limb)
    po=pa_openHand.GripperClient(limb)

    ################################# State Machine############################
    #--------------- Get Starting and Reference positions -----------------------
    rospy.loginfo('Starting pivot approach demo....')

    # Go to the Home Position
    rospy.loginfo('I will first move to the home position, as a standard starting point. There you will be able to let the robot grip the male part.')
    gh.paHome_rightArm()

    rospy.loginfo('I will now open the gripper and prepare to pick up the male camera mold....')
    po.open()

    # Capture Starting Position
    rospy.loginfo('Lets capture the starting pose  of the robot') 
    startPose=arm.endpoint_pose()
    startJoints=kin.inverse_kinematics(startPose['position'],startPose['orientation']).tolist()
    startJoints_=dict(zip(jNamesl,startJoints))

    # Record reference position
    if not reference_pose_flag:
        rospy.loginfo('Now please move to the reference locations.\n Open a new terminal and use keyboard teleoperation: roslaunch baxter_end_effector_control end_effector_control.launch keyboard:=true')
        key=raw_input('When you have finished, pres any key. Then I will record this as the reference location #1: \n')
        referencePose1=arm.endpoint_pose()
        reference_pose_flag=1

    # Used saved pose from manual teleoperation
    else:
        x=  0.6809201353879665
        y= -0.33405871642505175
        z= -0.17701920976005074
        qx=-0.018981384751769574
        qy= 0.9995535901740978
        qz= 0.005352975725038132
        qw= 0.02244266147016172
        ref_p=baxter_interface.limb.Limb.Point(x,y,z)
        ref_q=baxter_interface.limb.Limb.Quaternion(qx,qy,qz,qw)
        referencePose1={'position': ref_p, 'orientation': ref_q}

    # Compute the inverse kinematics and place the solution in a dictionary
    if kinematics_flag==PY_KDL:
        referenceJoints=kin.inverse_kinematics(referencePose1['position'],referencePose1['orientation']).tolist()
        referenceJoints_=dict(zip(jNamesl,referenceJoints))
        
    else:
        referenceJoints=calcInvKin(posePub,referencePose1,limb)
        referenceJoints_=dict( zip( list(referenceJoints.names),list(referenceJoints.command) ) )

    #----------------------------------- Approach to location #1, step 1
    # Add 5 cm to reference pose to set a point higher in the z+ direction
    approach1Pose=copy(referencePose1);
    _x=copy(referencePose1['position'][0])
    _y=copy(referencePose1['position'][1])
    _z=copy(referencePose1['position'][2])
    _z=_z+0.05
    del_z=baxter_interface.limb.Limb.Point(_x,_y,_z)
    
    approach1Pose['position']=copy(del_z)
    if kinematics_flag==PY_KDL:
        approach1Joints=kin.inverse_kinematics(approach1Pose['position'],approach1Pose['orientation']).tolist()
        approach1Joints_=dict(zip(jNamesl,approach1Joints))
    else:
        approach1Joints=calcInvKin(posePub,approach1Pose,limb)    
        approach1Joints_=dict( zip( list(approach1Joints.names),list(approach1Joints.command) ) )

    #----------------------------------- Approach to location #1, step 2
    # Add 1 cm to referene pose
    approach2Pose=copy(referencePose1);
    _z=copy(referencePose1['position'][2])
    _z=_z+0.01
    del_z=baxter_interface.limb.Limb.Point(_x,_y,_z)
    
    approach2Pose['position']=copy(del_z)

    if kinematics_flag==PY_KDL:
        approach2Joints=kin.inverse_kinematics(approach2Pose['position'],approach2Pose['orientation']).tolist()
        approach2Joints_=dict(zip(jNamesl,approach2Joints))

    else:
        approach2Joints=calcInvKin(posePub,approach2Pose,limb)    
        approach2Joints_=dict( zip( list(approach2Joints.names),list(approach2Joints.command) ) )

    #----------------------------------- Approach to location #2, step 3
    # Add 1 cm to referene pose
    approach3Pose=copy(referencePose1);
    _z=copy(referencePose1['position'][2])
    _z=_z+0.10
    del_z=baxter_interface.limb.Limb.Point(_x,_y,_z)

    approach3Pose['position']=copy(del_z)

    if kinematics_flag==PY_KDL:
        approach3Joints=kin.inverse_kinematics(approach3Pose['position'],approach3Pose['orientation']).tolist()
        approach3Joints_=dict(zip(jNamesl,approach3Joints))

    else:
        approach3Joints=calcInvKin(posePub,approach3Pose,limb)
        approach3Joints_=dict( zip( list(approach3Joints.names),list(approach3Joints.command) ) )


    ############################## Moving to Points ####################################
    #1. Approach 0
    rospy.loginfo('001 Moving to starting position 1/7.')
    arm.move_to_joint_positions(startJoints_)
    rospy.sleep(2)

    #2. Appraoch 1
    rospy.loginfo('002 Moving to approach 1 2/7.')
    arm.move_to_joint_positions(approach1Joints_)
    rospy.sleep(2)

    #3. Appraoch 2
    rospy.loginfo('003 Moving to approach 2 3/7.')
    arm.move_to_joint_positions(approach2Joints_)
    rospy.sleep(2)

    #4. Reference location 1
    rospy.loginfo('004 Moving to reference location 1 position 4/7.')
    arm.move_to_joint_positions(referenceJoints_)
    rospy.sleep(2)

    #5. Gripper CLosed
    pc.close()
    rospy.sleep(1.0)

    #6. Appraoch 2
    rospy.loginfo('005 Moving to approach 2 3/7.')
    arm.move_to_joint_positions(approach2Joints_)
    rospy.sleep(2)

    #8. Appraoch 0
    rospy.loginfo('007 Moving to approach 0 0/7.')
    arm.move_to_joint_positions(approach3Joints_)
    rospy.sleep(2)




if __name__ == "__main__":
    try:
        main()
    except:
        rospy.loginfo('pa_jtc_tracIK terminated.')
