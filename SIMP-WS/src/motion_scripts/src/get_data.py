#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("l_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

# Get the reference frame for a certain group 
print "Reference frame: %s" % group.get_planning_frame()
# Get the end-effector link for a certain group 
print "End effector: %s" % group.get_end_effector_link()
# Get a list with all of the groups of the robot 
print "Robot Groups:"
print robot.get_group_names()
# Get the current values of the joints
print "Current Joint Values:"
print group.get_current_joint_values()
# Get the current values of the joints
print "Current Pose:"
print group.get_current_pose()
# Check the general status of the robot
print "Robot State:"
print robot.get_current_state()