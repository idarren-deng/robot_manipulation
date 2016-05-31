#!/usr/bin/env python
"""
Uses keyboard keys to update the cartesian pose goal for the end effector. The updates are published to the end_effector_command_pose_stamped topic, which can then be used by inverseKinematics routine, which in turns moves the arm. 
"""
import copy

import rospy

import baxter_interface
import baxter_external_devices
import tf

from baxter_interface import CHECK_VERSION

from std_msgs.msg import (
	String,
)

from geometry_msgs.msg import (
    Point,
    Quaternion,
    Pose,
    PoseStamped,
)

commandPoseStampedPublisher = rospy.Publisher("end_effector_command_pose_stamped", PoseStamped, queue_size=1)

# Default variables
current_limb = 'right'
current_rotation = 'x'
limbPoseStamped = PoseStamped()
global_move_speed = 0.001
global_rotate_speed = 0.001
global_speed_factor = 0.001

def map_keyboard():
	printHelper()
	rospy.loginfo('press ? to print help')
	while not rospy.is_shutdown():
		c = baxter_external_devices.getch()
		if c:
			#catch Esc or ctrl-c
			if c in ['\x1b', '\x03']:
				rospy.signal_shutdown("Finished.Exiting...")
				return
			checkCommand(c)


def checkCommand(command):
	global limbPoseStamped
	global global_move_speed
	global global_rotate_speed
	global global_speed_factor
	global current_limb
	global current_rotation
	
	if (command == '?'):
		printHelper()
	elif (command == 'w'):
		limbPoseStamped.pose.position.z += global_move_speed
	elif (command == 's'):
		limbPoseStamped.pose.position.z -= global_move_speed
	elif (command == 'a'):
		limbPoseStamped.pose.position.y += global_move_speed
	elif (command == 'd'):
		limbPoseStamped.pose.position.y -= global_move_speed
	elif (command == 'q'):
		limbPoseStamped.pose.position.x -= global_move_speed
	elif (command == 'e'):
		limbPoseStamped.pose.position.x += global_move_speed
	elif (command == 'k'):
		global_move_speed += global_speed_factor
		printControlState()
	elif (command == 'l'):
		global_move_speed -= global_speed_factor
		if (global_move_speed < 0):
			global_move_speed = 0
		printControlState()
	elif (command == 'r'):
		rotateLimb(True)
	elif (command == 't'):
		rotateLimb(False)
	elif (command == 'i'):
		global_rotate_speed += global_speed_factor
		printControlState()
	elif (command == 'o'):
		global_rotate_speed -= global_speed_factor
		if (global_rotate_speed < 0):
			global_rotate_speed = 0
		printControlState()
	elif (command == 'f'):
		if (current_limb == 'right'):
			current_limb = 'left'
		else:
			current_limb = 'right'
		initLimbPose()
		printControlState()
	elif (command == 'x'):
		current_rotation = 'x'
		printControlState()
	elif (command == 'y'):
		current_rotation = 'y'
		printControlState()
	elif (command == 'z'):
		current_rotation = 'z'
		printControlState()
		
	## Caution!
	commandPoseStampedPublisher.publish(limbPoseStamped)

def rotateLimb(clockwise):
	global current_rotation
	global global_rotate_speed
	global limbPoseStamped
	
	quaternion = (
		limbPoseStamped.pose.orientation.x,
		limbPoseStamped.pose.orientation.y,
		limbPoseStamped.pose.orientation.z,
		limbPoseStamped.pose.orientation.w,
	)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	
	target_rotate_speed = copy.copy(global_rotate_speed)
	if not clockwise:
		target_rotate_speed = -target_rotate_speed
		
	r = 0
	p = 0
	y = 0
	if (current_rotation == 'x'):
		r = target_rotate_speed
	elif (current_rotation == 'y'):
		p = target_rotate_speed
	elif (current_rotation == 'z'):
		y = target_rotate_speed
		
	quaternion = tf.transformations.quaternion_from_euler(euler[0] + r, euler[1] + p, euler[2] + y)
	limbPoseStamped.pose.orientation.x = quaternion[0]
	limbPoseStamped.pose.orientation.y = quaternion[1]
	limbPoseStamped.pose.orientation.z = quaternion[2]
	limbPoseStamped.pose.orientation.w = quaternion[3]
	print 'rotated...'

def printHelper():
	print '\nControl key maping:\n'
	print 'w ---> move up'
	print 's ---> move down'
	print 'a ---> move left'
	print 'd ---> move right'
	print 'q ---> move backward'
	print 'e ---> move forward'
	print 'k ---> move speed increase'
	print 'l ---> move speed decrease\n'
	
	print 'r ---> rotation angle +'
	print 't ---> rotation angle -'
	print 'i ---> rotation speed increase'
	print 'o ---> rotation speed decrease\n'
	
	print 'f ---> switch control limb'
	print 'x ---> switch rotation control to x'
	print 'y ---> switch rotation control to y'
	print 'z ---> switch rotation control to z\n'
	
	print 'Current control State:'
	printControlState()

def printControlState():
	print("\nControling %s limb ..." % current_limb)
	print("Move speed is: %.3lf meter per press" % global_move_speed)
	print("Controling rotation %s ..." % current_rotation)
	print("Rotate speed is: %.3lf radius per press" % global_rotate_speed)

def initLimbPose():
	global limbPoseStamped
	global current_limb
	
	limb = baxter_interface.Limb(current_limb)
	currentPose = dict()
	while not ("position" in currentPose):
		currentPose = limb.endpoint_pose()
	
	limbPoseStamped.pose.position = Point(
				currentPose["position"].x,
				currentPose["position"].y,
				currentPose["position"].z,
			)
	limbPoseStamped.pose.orientation = Quaternion(
				currentPose["orientation"].x,
				currentPose["orientation"].y,
				currentPose["orientation"].z,
				currentPose["orientation"].w,
			)
	
	limbPoseStamped.header.frame_id = current_limb
	limbPoseStamped.header.stamp = rospy.Time().now()

def main():
	rospy.init_node("keyboard_control")
	
	try:
		initLimbPose()
		map_keyboard()
	except():
		pass

if __name__ == '__main__':
	main()
