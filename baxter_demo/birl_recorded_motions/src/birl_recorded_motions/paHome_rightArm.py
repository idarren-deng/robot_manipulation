#!/usr/bin/env python2

# Makes both the right and the left arm go home from which ever position they are at.
#import pdb 
import baxter_interface
from baxter_interface import RobotEnable

class paHome_rightArm():
    def __init__(self):
        
        print("Starting Homing Motion")

        # Home Angles for the right and left arms
   
        r_home_angles = {'right_s0': -0.2286,
                         'right_s1': -1.0044,
                         'right_w0': -0.6535,
                         'right_w1':  1.0028,
                         'right_w2':  0.5196,
                         'right_e0':  1.2598,
                         'right_e1':  2.0003,
                     }

    	#Create an instance for the left and left arms
        print("Creating limbs")
        r_limb = baxter_interface.Limb('right')
    
        # Get the current angles
        print("Getting Current Angles")
        r_angles = r_limb.joint_angles()

        # Move home
        print 'Moving home...'
        r_limb.move_to_joint_positions(r_home_angles)
        print 'Finished moving home...'
