#!/usr/bin/env python
import sys
import argparse
import rospy

import actionlib
from copy import copy

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import (EndpointState,JointCommand)

# Action modules: trajectory
from control_msgs.msg import (FollowJointTrajectoryAction,FollowJointTrajectoryGoal)
from geometry_msgs.msg import (PoseStamped,Quaternion)
from trajectory_msgs.msg import JointTrajectoryPoint


# Kinematics
from baxter_pykdl import baxter_kinematics
import PyKDL
from math import pi

class arm_trajectory(object):
    def __init__(self,limb):
        ns='robot/limb/' + limb + '/'
        self._client=actionlib.SimpleActionClient(ns + "follow_joint_trajectory",FollowJointTrajectoryAction)
        self._goal=FollowJointTrajectoryGoal()
        self._goal_time_tolerance=rospy.Time(0.1)
        self._goal.goal_time_tolerance=self._goal_time_tolerance

        traj_server=self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not traj_server:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running trajectory client.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_traj_point(self,positions,time):
        point=JointTrajectoryPoint()
        point.positions=copy(positions)
        point.time_from_start=rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp=rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self,timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self,limb):
        self._goal=FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance=self._goal_time_tolerance
        self._goal.trajectory.joint_names=[limb + '_' + joint for joint in ['s0','s1','e0','e1','w0','w1','w2']]

class computerIK(object):
    def __init__(self,limb):
        # Create Joint name list
        self._limb=limb
        self.jNamesList=[self._limb+'_s0',self._limb+'_s1',self._limb+'_e0',self._limb+'_e1',self._limb+'_w0',self._limb+'_w1',self._limb+'_w2']

    def calIK_PY_KDL(self,goal):
        kin=baxter_kinematics(self._limb)
        ikJoints=kin.inverse_kinematics(goal['position'],goal['orientation']).tolist()
        ikJoints=dict(zip(self.jNamesList,ikJoints))
        return ikJoints

class computerApproachPose(object):
    def __init__(self,limb):
        self._limb=limb

    def get_approach_joints_2(self,target_pose,offset_1,offset_2):  # with two approachJoints return
        # approach #1
        approach_pose_1=self.get_cartesian_offset_pose(target_pose,offset_1)
        # approach #2
        approach_pose_2=self.get_cartesian_offset_pose(target_pose,offset_2)

        ik_solver=computerIK(self._limb)
        approach_joints_1=ik_solver.calIK_PY_KDL(approach_pose_1)
        approach_joints_2=ik_solver.calIK_PY_KDL(approach_pose_2)
        target_joints=ik_solver.calIK_PY_KDL(target_pose)

        return approach_joints_1,approach_joints_2,target_joints

    def get_cartesian_offset_pose(self,current_pose,offset):
        offset_pose=copy(current_pose)
        # set position
        _x=copy(offset_pose['position'][0])+offset[0]
        _y=copy(offset_pose['position'][1])+offset[1]
        _z=copy(offset_pose['position'][2])+offset[2]
        offset_position=baxter_interface.limb.Limb.Point(_x,_y,_z)
        offset_pose['position']=copy(offset_position)
        # To Do: set orientation

        return offset_pose















