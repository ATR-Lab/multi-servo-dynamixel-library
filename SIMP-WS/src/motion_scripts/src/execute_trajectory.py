#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("l_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

# pose_target = geometry_msgs.msg.Pose()
# pose_target.orientation.w = 1.0
# pose_target.position.x = 0.96
# pose_target.position.y = 0
# pose_target.position.z = 1.18
# group.set_pose_target(pose_target)

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.511
pose_target.position.y = 0.355
pose_target.position.z = 0.890
group.set_pose_target(pose_target)

# - Translation: [0.511, 0.355, 0.590]
# - Rotation: in Quaternion [0.704, -0.063, -0.704, -0.063]
#             in RPY (radian) [-2.401, 1.571, 0.562]
#             in RPY (degree) [-137.567, 89.994, 32.181]
# At time 1561709799.259
# - Translation: [0.511, 0.355, 0.590]
# - Rotation: in Quaternion [0.704, -0.063, -0.704, -0.063]
#             in RPY (radian) [-2.401, 1.571, 0.562]
#             in RPY (degree) [-137.567, 89.994, 32.181]


plan1 = group.plan()

rospy.sleep(5)


# Executes Plan
group.go(wait=True)

moveit_commander.roscpp_shutdown()