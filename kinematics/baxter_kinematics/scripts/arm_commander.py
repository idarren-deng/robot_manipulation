#!/usr/bin/env python

import argparse
import sys
import rospy
import xmltodict

from threading import Lock

import baxter_interface

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from baxter_core_msgs.msg import CollisionDetectionState

from baxter_pykdl import baxter_kinematics
import PyKDL
from trac_ik_baxter.srv import (GetConstrainedPositionIK, GetConstrainedPositionIKRequest)

class ArmCommander(object):

    def __init__(self, name):

        self._arm=name+'_arm'
        self._limb=baxter_interface.limb.Limb(name)

        self._robot=moveit_commander.RobotCommander()
        self._group=moveit_commander.MoveGroupCommander(self._arm)
        self._baxter_fk=baxter_kinematics(name)

        rospy.Subscriber('/robot/limb/{}/collision_detection_state'.format(name), CollisionDetectionState, self._cb_collision, queue_size=1)
        self._stop_reason = ''  # 'collision' could cause a trajectory to be stopped
        self._stop_lock = Lock()

    def __del__(self):
        print "Arm Commander exit..."

    ######################################### CALLBACKS #########################################
    def _cb_collision(self, msg):
        if msg.collision_state:
            with self._stop_lock:
                self._stop_reason = 'collision'
    #############################################################################################

    def ee_name(self):
        return self._group.get_end_effector_link()

    def joint_names(self):
        return self._limb.joint_names()

    def joint_limits(self):
        xml_urdf=rospy.get_param('robot_description')
        dict_urdf=xmltodict.parse(xml_urdf)
        joints_urdf = []
        joints_urdf.append([j['@name'] for j in dict_urdf['robot']['joint'] if j['@name'] in self.joint_names()])
        joints_urdf.append([[float(j['limit']['@lower']), float(j['limit']['@upper'])] for j in dict_urdf['robot']['joint'] if j['@name'] in self.joint_names()])
        # reorder the joints limits
        return dict(zip(self.joint_names(),
                        [joints_urdf[1][joints_urdf[0].index(name)] for name in self.joint_names()]))

    def get_ee_pose(self):
        pose=self._limb.endpoint_pose()
        return [[pose['position'].x, pose['position'].y, pose['position'].z],
                [pose['orientation'].x, pose['orientation'].y, pose['orientation'].z, pose['orientation'].w]]

    def get_current_joints(self):
        current_angles=[self._limb.joint_angle(joint) for joint in self.joint_names()]
        return current_angles

    def get_fk(self):
        fk=self._baxter_fk.forward_position_kinematics(dict(zip(self.joint_names(), self.current_joints())))
        return [fk[:3], fk[-4:]]

    def get_ik(self, ee_poses, seeds=(), params=None):
        ik_req=GetConstrainedPositionIKRequest
        if params is None:
            ik_req.num_steps=1
        else:
            ik_req.end_tolerance=params['end_tolerance']
            ik_req.num_steps=params['num_attempts']

        for ee_pose in ee_poses:
            ik_req.pose_stamp.append(ee_pose)

        if len(seeds)==0:
            seeds=[self.get_current_joints()]












