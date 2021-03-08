#!/usr/bin/env python
__author__ = "Irvin Steve Cardenas"
__email__ = "irvin@irvincardenas.com"

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose

class OCUBroker():
  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    try:
      rospy.init_node('ocu_moveit_broker', anonymous=True)
    except rospy.ROSInterruptException: pass

  def initialize(self):
    moveit_commander.roscpp_initialize(sys.argv)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.l_group = moveit_commander.MoveGroupCommander("l_arm")
    self.r_group = moveit_commander.MoveGroupCommander("r_arm")

    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
    
    self.ocu_pose_plan_subscriber = rospy.Subscriber('/ocu_moveit/left_arm/plan', Pose, self.l_process_plan)
    self.ocu_pose_execution_subscriber = rospy.Subscriber('/ocu_moveit/left_arm/execute', Pose, self.l_process_command)

    self.ocu_pose_plan_subscriber = rospy.Subscriber('/ocu_moveit/right_arm/plan', Pose, self.r_process_plan)
    self.ocu_pose_execution_subscriber = rospy.Subscriber('/ocu_moveit/right_arm/execute', Pose, self.r_process_command)


  def l_process_plan(self, pose_msg):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = pose_msg.orientation.w
    pose_target.position.x = pose_msg.position.x
    pose_target.position.y = pose_msg.position.y
    pose_target.position.z = pose_msg.position.y
    self.l_group.set_pose_target(pose_target)
    self.l_plan = self.l_group.plan()
    rospy.sleep(5)

  def l_process_command(self, msg):
    self.l_group.go(wait=True)


  def r_process_plan(self, pose_msg):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = pose_msg.orientation.w
    pose_target.position.x = pose_msg.position.x
    pose_target.position.y = pose_msg.position.y
    pose_target.position.z = pose_msg.position.y
    self.r_group.set_pose_target(pose_target)
    self.r_plan = self.r_group.plan()
    rospy.sleep(5)

  def r_process_command(self, msg):
    self.r_group.go(wait=True)
  
if __name__ == '__main__':
  try:
    ocu_moveit_broker = OCUBroker()
    ocu_moveit_broker.initialize()
    rospy.spin()
  except rospy.ROSInterruptException: pass