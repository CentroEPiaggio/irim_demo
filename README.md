# IRIM DEMO

This is the package for the demo with the Franka Robot at I-RIM event - Maker Faire Rome 2019. Tested on Ubuntu 16.04 with ROS Kinetic.

## What this demo does
Basically, the Panda robot mounted with a PISA/IIT SoftHand will be commanded by a state machine which takes an input from an RGB-D camera. Each time there is at least an object, with an aruco tag on it, in the FOV of the camera, the robot will grasp the object and place it. There are different strategies of grasping, placing and handover according to the tag id. 

## How to run the demo

In order to run a complete demo of the state machine run the follwing commands in seperate terminal windows:
- Launch the Panda robot with the SoftHand `roslaunch irim_demo launchPandaSoftHand.launch` 
- Launch the vision stuff `roslaunch irim_demo vision_irim.launch` (if using opennni2)
- Launch the server which provides the basic robot controls (uses joint_trajectory controller) `roslaunch panda_softhand_control launchControlServer.launch`
- Launch the server that provides the basic tasks planning and enables their execution (grasp, place, home, ...) `roslaunch panda_softhand_control launchTaskServer.launch `
- Launch the finite state machine that automatizes everything `rosrun irim_utils planning_sm.py`

### Want to view the states of the FSM?
Run `rosrun smach_viewer smach_viewer.py` after installing it using the following command: `sudo apt install ros-kinetic-smach-viewer`