#!/usr/bin/env python2

# Makes both the right and the left arm go home from which ever position they are at.
#import pdb 
import baxter_interface
from baxter_interface import RobotEnable

class GoHomeLeft():
    def __init__(self):
        
        print("Starting Homing Motion")

        # Home Angles for the left arm
        l_home_angles = {'left_w0': 0.6477233869445801, 
                         'left_w1': 1.007825376489258, 
                         'left_w2': -0.48282045243530275, 
                         'left_e0': -1.1504855895996096, 
                         'left_e1': 1.9232284106140138, 
                         'left_s0': -0.07823302009277344, 
                         'left_s1': -0.9675583808532715}

    	#Create an instance for the left and left arms
        print("Creating limb")
        l_limb = baxter_interface.Limb('left')
    
        # Get the current angles
        print("Getting Current Angles")
        l_angles = l_limb.joint_angles()

        # Move home
        print 'Moving home...'
        l_limb.move_to_joint_positions(l_home_angles)
        print 'Finished moving home...'
