#! /usr/bin/env python

import rospy
import time
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
from math import radians, sin, cos
import numpy as np


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

robot_center_length = 0.6
robot_rsh_length = 0.2
robot_rarm_length = 0.6
robot_lsh_length = -0.2
robot_larm_length = 0.6

body_center_length = 1.5
body_nect_length = 0.17
body_goggle_length = 0.12

body_rsh_length = 0.17
body_rarm_length = 0.7
body_rjoy_length = 0.1

body_lsh_length = -0.17
body_larm_length = 0.7
body_ljoy_length = 0.1


unity_head_x = 0.3
unity_head_y = 1.78
unity_head_z = 0.2

unity_lhand_x = 0.1
unity_lhand_y = 1.61
unity_lhand_z = 0.8

unity_rhand_x = 0.5
unity_rhand_y = 1.61
unity_rhand_z = 0.8

unity_angle_head_x = 0.3
unity_angle_head_y = 1.78
unity_angle_head_z = 0.2

unity_angle_lhand_x = 0.1
unity_angle_lhand_y = 1.61
unity_angle_lhand_z = 0.8

unity_angle_rhand_x = 0.5
unity_angle_rhand_y = 1.61
unity_angle_rhand_z = 0.8

class HomogeneousMatrix(object):
    # Creates a homogeneous matrix.

    def __init__(self):
        # [0,0] - [2,2] rotation
        # [0,3] - [2,3] position
        # [3,0] - [3,2] perspective parameters
        # [3,3] scale factor

        self.matrix = np.identity(4)

    def __getitem__(self, key):
        return self.matrix[key]

    def get(self):
        return self.matrix

    def roll(self, angle_X):
        # Rotates the homogeneous matrix by angle_X by the X axis

        rolled_by = np.identity(4)
        rolled_by[1, 1] = cos(radians(angle_X))
        rolled_by[1, 2] = -(sin(radians(angle_X)))
        rolled_by[2, 1] = sin(radians(angle_X))
        rolled_by[2, 2] = cos(radians(angle_X))

        self.matrix = np.dot(self.matrix, rolled_by)

    def pitch(self, angle_Y):
        # Rotates the homogeneous matrix by angle_Y by the Y axis

        pitched_by = np.identity(4)
        pitched_by[0, 0] = cos(radians(angle_Y))
        pitched_by[0, 2] = sin(radians(angle_Y))
        pitched_by[2, 0] = -(sin(radians(angle_Y)))
        pitched_by[2, 2] = cos(radians(angle_Y))

        self.matrix = np.dot(self.matrix, pitched_by)

    def yaw(self, angle_Z):
        # Rotates the homogeneous matrix by angle_Z by the Z axis

        yawed_by = np.identity(4)
        yawed_by[0, 0] = cos(radians(angle_Z))
        yawed_by[0, 1] = -(sin(radians(angle_Z)))
        yawed_by[1, 0] = sin(radians(angle_Z))
        yawed_by[1, 1] = cos(radians(angle_Z))

        self.matrix = np.dot(self.matrix, yawed_by)

    def set_pos(self, posX, posY, posZ):
        # Sets the position of the homogeneous matrix

        self.matrix[0, 3] = posX
        self.matrix[1, 3] = posY
        self.matrix[2, 3] = posZ

    def set_perspective(self, X, Y, Z):
        # Sets the perspective parameter of the homogeneous matrix

        self.matrix[3, 0] = X
        self.matrix[3, 1] = Y
        self.matrix[3, 2] = Z

    def set_scale(self, scale):
        # Sets the scale parameter of the homogeneous matrix0.1

        self.matrix[3, 3] = scale

    def set_a(self, a):
        # Sets the 'a' parameter of the DH convention

        self.matrix[0, 3] = a

    def set_alpha(self, alpha):
        # Sets the 'alpha = angle_X' parameter of the DH convention

        self.roll(alpha)

    def set_d(self, d):
        # Sets the 'd' parameter of the DH convention

        self.matrix[2, 3] += d

    def set_beta(self, beta):
        # Sets the 'beta = angle_Y' parameter of the DH convention

        self.pitch(beta)

    def set_theta(self, theta):
        # Sets the 'theta = angle_Z' parameter of the DH convention

        self.yaw(theta)


    def set_parent(self, parent):
        self.matrix = np.dot(parent, self.matrix)


# offset from a jostick head to a wrist
def getHandOffset(argLength, argAangle_X, argAngle_Y, argAngle_Z):
    handbase = HomogeneousMatrix()
    joy = HomogeneousMatrix()
    handbase.set_pos(0,0,0)
    joy.set_pos(0,0,argLength)
    #    ---------------------
    handbase.set_alpha(argAangle_X)
    handbase.set_beta(argAngle_Y)
    handbase.set_theta(argAngle_Z)
    # ---------------------------------
    joy.set_parent(handbase.get())
    return joy

# offset from a goggle to a center of a head
def getHeadOffset(argLength,  argAangle_X, argAngle_Y, argAngle_Z):
    headbase = HomogeneousMatrix()
    head = HomogeneousMatrix()
    headbase.set_pos(0,0,0)
    head.set_pos(argLength,0,0)
    #    ---------------------
    headbase.set_alpha(argAangle_X)
    headbase.set_beta(argAngle_Y)
    headbase.set_theta(argAngle_Z)
    # ---------------------------------
    head.set_parent(headbase.get())
    return head

# offset from a center of a body head to a center of global map 
def getBodyOffset(argHeadOffset, argX, argY, argZ):
    goggle = HomogeneousMatrix() 
    goggle.set_pos((argX - argHeadOffset[0,3]),(-argY -argHeadOffset[1,3]),(argZ -argHeadOffset[2,3])) 
    print ("2) %5.5f %5.5f %5.5f " % (goggle[0,3], goggle[1,3],goggle[2,3]))
    # --- calculate an offset info for a center of a body position from the global location with the robot height
    goggle.set_pos(goggle[0,3], goggle[1,3], (goggle[2,3] - (body_center_length - robot_center_length )))
    print ("3) %5.5f %5.5f %5.5f " % (goggle[0,3], goggle[1,3],goggle[2,3]))

    return goggle



def getHandLocation(argBodyOffset, argHandOffset, argX, argY, argZ):
    handLocation = HomogeneousMatrix()
    handLocation.set_pos( argX - argHandOffset[0,3], -argY - argHandOffset[1,3], argZ - argHandOffset[2,3])   
    handLocation.set_pos((handLocation[0,3] - argBodyOffset[0,3]), (handLocation[1,3] - argBodyOffset[1,3]), (handLocation[2,3] - argBodyOffset[2,3]))
    return handLocation


class OCUConnector:
  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)

    moveit_commander.roscpp_initialize(sys.argv)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path'
      , moveit_msgs.msg.DisplayTrajectory, queue_size=1)

    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.l_group = moveit_commander.MoveGroupCommander("l_arm")
    self.r_group = moveit_commander.MoveGroupCommander("r_arm")

    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path'
      , moveit_msgs.msg.DisplayTrajectory, queue_size=1)
    
    # self.ocu_pose_plan_subscriber = rospy.Subscriber('/ocu_moveit/left_arm/plan'
    #   , Pose, self.l_process_plan)
    # self.ocu_pose_execution_subscriber = rospy.Subscriber('/ocu_moveit/left_arm/execute'
    #   , Pose, self.l_process_command)

    # self.ocu_pose_plan_subscriber = rospy.Subscriber('/ocu_moveit/right_arm/plan'
    #   , Pose, self.r_process_plan)
    # self.ocu_pose_execution_subscriber = rospy.Subscriber('/ocu_moveit/right_arm/execute'
    #   , Pose, self.r_process_command)


    # self.pub_left = rospy.Publisher('/ocu_moveit/left_arm/plan', Pose, queue_size=10)
    # sefl.pub_right = rospy.Publisher('/ocu_moveit/right_arm/plan', Pose, queue_size=10)

  def initialize(self):
    rospy.loginfo("Initializing the OCU Connects, creating subscribers...")
    rospy.Subscriber("/tb/rhs/raw", Twist, self.rhand_callback)
    rospy.Subscriber("/tb/lhs/raw", Twist, self.lhand_callback)
    rospy.Subscriber("/tb/head/raw", Twist, self.head_callback)


  # subscriber callback
  def head_callback(self, data):
    global unity_head_x
    global unity_head_y 
    global unity_head_z 
    global unity_angle_head_x
    global unity_angle_head_y
    global unity_angle_head_z
    unity_head_x = data.linear.x
    unity_head_y = data.linear.y
    unity_head_z = data.linear.z
    unity_angle_head_x = data.angular.x 
    unity_angle_head_y = data.angular.y 
    unity_angle_head_z = data.angular.z 
    #print "data in callback is: "
    #print unity_head_x
    

  # subscriber callback
  def rhand_callback(self, data):
    global unity_lhand_x 
    global unity_lhand_y 
    global unity_lhand_z 
    global unity_angle_lhand_x
    global unity_angle_lhand_y
    global unity_angle_lhand_z
    unity_lhand_x = data.linear.x
    unity_lhand_y = data.linear.y
    unity_lhand_z = data.linear.z
    unity_angle_lhand_x = data.angular.x 
    unity_angle_lhand_y = data.angular.y 
    unity_angle_lhand_z = data.angular.z 

  # subscriber callback
  def lhand_callback(self, data):
    global unity_rhand_x 
    global unity_rhand_y 
    global unity_rhand_z 
    global unity_angle_rhand_x
    global unity_angle_rhand_y
    global unity_angle_rhand_z
    unity_rhand_x = data.linear.x
    unity_rhand_y = data.linear.y
    unity_rhand_z = data.linear.z
    unity_angle_rhand_x = data.angular.x 
    unity_angle_rhand_y = data.angular.y 
    unity_angle_rhand_z = data.angular.z 

    headOffset = getHeadOffset(body_goggle_length, unity_angle_head_z, unity_angle_head_x, unity_angle_head_y)
    leftHandOffset = getHandOffset(body_ljoy_length, unity_angle_lhand_z, unity_angle_lhand_x, unity_angle_lhand_y)
    rightHandOffset = getHandOffset(body_rjoy_length, unity_angle_rhand_z, unity_angle_rhand_x, unity_angle_rhand_y)

    print "data in loop is: "
    print unity_head_x

    # goggle global location in VR (x = 0.3, y = 1.78, z= 0.2) 
    bodyOffset = getBodyOffset ( headOffset, unity_head_z, unity_head_x,  unity_head_y )
    # lefthand global location in VR (x = 0.1, y = 1.61, z= 0.8) 
    leftHandLocation = getHandLocation ( bodyOffset , leftHandOffset,  unity_lhand_z,  unity_lhand_x, unity_lhand_y)
    # righthand global location in VR (x = 0.5, y = 1.61, z= 0.8) 
    rightHandLocation = getHandLocation ( bodyOffset , rightHandOffset,  unity_rhand_z,  unity_rhand_x, unity_rhand_y)

    print ("head angle	%5.5f %5.5f %5.5f : position  	%5.5f %5.5f %5.5f " % (unity_angle_head_z, unity_angle_head_z, unity_angle_head_z)
    print ("lhand	%5.5f %5.5f %5.5f " % (headOffset[0,3], headOffset[1,3],headOffset[2,3]))
    print ("rhand	%5.5f %5.5f %5.5f " % (headOffset[0,3], headOffset[1,3],headOffset[2,3]))
    print ("headOffset	%5.5f %5.5f %5.5f " % (headOffset[0,3], headOffset[1,3],headOffset[2,3]))
    print ("bodyOffset	%5.5f %5.5f %5.5f " % (bodyOffset[0,3], bodyOffset[1,3],bodyOffset[2,3]))
    print ("leftHandOffset	%5.5f %5.5f %5.5f " % (leftHandOffset[0,3], leftHandOffset[1,3],leftHandOffset[2,3]))
    print ("rightHandOffset	%5.5f %5.5f %5.5f " % (rightHandOffset[0,3], rightHandOffset[1,3],rightHandOffset[2,3]))
    print ("leftHandLocation	%5.5f %5.5f %5.5f " % (leftHandLocation[0,3], leftHandLocation[1,3],rightHandOffset[2,3]))
    print ("rightHandLocation	%5.5f %5.5f %5.5f " % (rightHandLocation[0,3], rightHandLocation[1,3],rightHandLocation[2,3]))    


    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = leftHandLocation[0,3]
    pose_target.position.y = leftHandLocation[1,3]
    pose_target.position.z = leftHandLocation[2,3]
    rospy.loginfo("POSE  x: %5.5f, y: %5.5f, z: %5.5f " % (leftHandLocation[0,3], leftHandLocation[1,3], leftHandLocation[2,3]) )
    self.l_group.set_pose_target(pose_target)
    self.l_plan = self.l_group.plan()

  def l_process_command(self, msg):
    self.l_group.go(wait=True)



if __name__ == '__main__': 
  try:
    rospy.init_node('ocu_moveit_broker', anonymous=True)
  except rospy.ROSInterruptException: pass

  rospy.loginfo("Starting end effector node...")
  # 1) calculate each offset  ( argAangle_X = 0.0 degree,  argAangle_Y = 0.0 degree ,  argAangle_Z = 0.0 degree  ) 
  
  ocu_connector = OCUConnector()
  ocu_connector.initialize()

  rospy.spin()

  # while not rospy.is_shutdown():
    # headOffset = getHeadOffset(body_goggle_length,0,0,0)
    # leftHandOffset = getHandOffset(body_ljoy_length,0,0,0)
    # rightHandOffset = getHandOffset(body_rjoy_length,0,0,0)

    # print "data in loop is: "
    # print unity_head_x

    # # goggle global location in VR (x = 0.3, y = 1.78, z= 0.2) 
    # bodyOffset = getBodyOffset ( headOffset, unity_head_x, unity_head_y,  unity_head_z )
    # # lefthand global location in VR (x = 0.1, y = 1.61, z= 0.8) 
    # leftHandLocation = getHandLocation ( bodyOffset , leftHandOffset,  unity_lhand_x,  unity_lhand_y, unity_lhand_z)
    # # righthand global location in VR (x = 0.5, y = 1.61, z= 0.8) 
    # rightHandLocation = getHandLocation ( bodyOffset , rightHandOffset,  unity_rhand_x,  unity_rhand_y, unity_rhand_z)

    # print ("headOffset	%5.5f %5.5f %5.5f " % (headOffset[0,3], headOffset[1,3],headOffset[2,3]))
    # print ("bodyOffset	%5.5f %5.5f %5.5f " % (bodyOffset[0,3], bodyOffset[1,3],bodyOffset[2,3]))
    # print ("leftHandOffset	%5.5f %5.5f %5.5f " % (leftHandOffset[0,3], leftHandOffset[1,3],leftHandOffset[2,3]))
    # print ("rightHandOffset	%5.5f %5.5f %5.5f " % (rightHandOffset[0,3], rightHandOffset[1,3],rightHandOffset[2,3]))
    # print ("leftHandLocation	%5.5f %5.5f %5.5f " % (leftHandLocation[0,3], leftHandLocation[1,3, ],rightHandOffset[2,3],))
    # print ("rightHandLocation	%5.5f %5.5f %5.5f " % (rightHandLocation[0,3], rightHandLocation[1,3],rightHandLocation[2,3]))
    # time.sleep(1)

  # rospy.spin()
