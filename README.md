# robot_manipulation
This is the customized package for manipulation task using Baxter robot.

# baxter_common
Based on the baxter_common package from baxter[https://github.com/RethinkRobotics/baxter_common]

Updates:

-new robots_description package, with URDF and meshes for Table, Asus Kinect
  folders: launch, robots, urdf, meshes
  
-modifed rethink_ee_description package, with customized pa gripper.

  urdf/electric_gripper/rethink_electric_pa_gripper.xacro
  
  urdf/finger/pa_finger.xacro
  
  urdf/finger/pa_tip.xacro
  
# baxter_simulator
Based on the baxter_simulator package from baxter[https://github.com/RethinkRobotics/baxter_simulator]
Updates:

-new launch file for pa_demo world, 

 `roslaunch baxter_gazebo baxter_pa_world.launch`
 
# baxter_demo
Updates:

-new pa_localization package

 using visual_localization.py to localize a random pos box, and return the center and angle for picking
 
 using table_pos_calibration.py, with a image guider to set the camera and table in right position.
 
# hand_design

3D model for customized gripper for pa demo.
