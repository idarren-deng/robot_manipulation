#!/usr/bin/env python2

# Makes both the right and the left arm go home from which ever position they are at.
#import pdb 
import baxter_interface
from baxter_interface import RobotEnable

class paHomePosition():
    def __init__(self):
        
        print("Starting Homing Motion")

        # Home Angles for the right and left arms
        r_home_angles = {'right_s0': -0.44255345679931646,
                         'right_s1':  0.02070874061279297,
                         'right_w0': -0.06366020262451172,
                         'right_w1':  0.9786797415527344,
                         'right_w2':  1.5623594306762696,
                         'right_e0':  1.6133642918151856,
                         'right_e1':  1.7867041206481935}
        l_home_angles = {'left_w0':   0.06634466900024415,
                         'left_w1':   1.085291406188965,
                         'left_w2':   1.6302380804626466,
                         'left_e0':  -1.598791474346924,
                         'left_e1':   1.710388576538086,
                         'left_s0':   0.4229952017761231,
                         'left_s1':   0.06557767860717774}

    	#Create an instance for the left and left arms
        print("Creating limbs")
        r_limb = baxter_interface.Limb('right')
        l_limb = baxter_interface.Limb('left')
    
        # Get the current angles
        print("Getting Current Angles")
        r_angles = r_limb.joint_angles()
        l_angles = l_limb.joint_angles()

        # Move home with block function and smooth filter
        print 'Moving home...'
        r_limb.move_to_joint_positions(r_home_angles)
        l_limb.move_to_joint_positions(l_home_angles)
        print 'Finished moving home...'
