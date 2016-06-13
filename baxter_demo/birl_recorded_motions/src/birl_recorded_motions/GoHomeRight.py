#!/usr/bin/env python2

# Makes both the right and the left arm go home from which ever position they are at.
#import pdb 
import baxter_interface
from baxter_interface import RobotEnable

class GoHomeRight():
    def __init__(self):
        
        print("Starting Homing Motion")

        # Home Angles for the right and left arms
        r_home_angles = {'right_s0': 0.07669903930664063, 
                         'right_s1': -0.9660244000671387, 
                         'right_w0': -0.6438884349792481, 
                         'right_w1': 1.0074418812927246, 
                         'right_w2': 0.483203947631836, 
                         'right_e0': 1.1481846184204103, 
                         'right_e1': 1.9232284106140138}

    	#Create an instance for the left and left arms
        print("Creating right limbs")
        r_limb = baxter_interface.Limb('right')
    
        # Get the current angles
        print("Getting Current Angles")
        r_angles = r_limb.joint_angles()

        # Move home
        print 'Moving home...'
        r_limb.move_to_joint_positions(r_home_angles)
        print 'Finished moving home...'
        
        # Move arms in a series of steps from current position to homing position
        # wayPointNo=10
        # for i in range(1, 11):
        #     print("waypoint %d",i)
        #     r_temp = (r_home_angles - r_angles)/wayPointNo
        #     r_wayP = r_angles - r_temp*i
            
        #     l_temp = (l_home_angles - l_angles)/wayPointNo
        #     l_wayP = l_angles - l_temp*i

        #     r_limb.move_to_joint_positions(r_wayP)
        #     r_angles = r_limb.joint_angles()

        #     l_limb.move_to_joint_positions(l_wayP)
        #     l_angles = l_limb.joint_angles()
