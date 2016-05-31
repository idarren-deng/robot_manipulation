#!/usr/bin/env python2

# Makes both the right and the left arm go home from which ever position they are at.
#import pdb 
import baxter_interface
from baxter_interface import RobotEnable

class GoHome():
    def __init__(self):
        
        print("Starting Homing Motion")

        # Home Angles for the right and left arms
        r_home_angles = {'right_s0': -0.978679742670894,
                         'right_s1': -1.0089758632316308,
                         'right_e0': 0.07094661143970038,
                         'right_e1': 1.650179832567734,
                         'right_w0': -0.12156797743991904,
                         'right_w1': 0.8095583608065271,
                         'right_w2': 0.055223308363874894}
        l_home_angles = {'left_s0': 0.6615292147755847,
                         'left_s1': -0.8828059434280556,
                         'left_e0': 0.26691265709206197,
                         'left_e1': 1.3449176557785365,
                         'left_w0': -0.15838351634916897,
                         'left_w1': 1.0032234352770606,
                         'left_w2': -0.028378644575880154}

    	#Create an instance for the left and left arms
        print("Creating limbs")
        r_limb = baxter_interface.Limb('right')
        l_limb = baxter_interface.Limb('left')
    
        # Get the current angles
        print("Getting Current Angles")
        r_angles = r_limb.joint_angles()
        l_angles = l_limb.joint_angles()

        # Move home
        print 'Moving home...'
        r_limb.move_to_joint_positions(r_home_angles)
        l_limb.move_to_joint_positions(l_home_angles)
        print 'Finished moving home...'
