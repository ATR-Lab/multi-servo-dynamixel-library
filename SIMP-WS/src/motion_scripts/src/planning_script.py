#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_py', anonymous=True)

# Creat a RobotCommander object, which is, basically, an interface to our robot.
robot = moveit_commander.RobotCommander()

# Create a PlanningSceneInterface object, which is, basically, an interface to the world that surrounds the robot
scene = moveit_commander.PlanningSceneInterface()

# Create a MoveGroupCommander object, which is an interface to the manipulator group of joints that we defined when we created the MoveIt package
group = moveit_commander.MoveGroupCommander("l_arm")

# By publishing into this topic, we will be able to visualize the planned motion through the MoveIt RViz interface.
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

# Ceate a Pose object, which is the type of message that we will send as a goal. 
# Then, we just give values to the variables that will define the goal Pose.
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.96
pose_target.position.y = 0
pose_target.position.z = 1.18
group.set_pose_target(pose_target)

# Calculate the plan
plan1 = group.plan()

rospy.sleep(5)

moveit_commander.roscpp_shutdown()