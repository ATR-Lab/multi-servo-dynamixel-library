#! /usr/bin/python3

from threading import Thread

import rospy
import actionlib

from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from dynomix_tools.dynamixel_control_table import MODEL_NUMBER_2_MOTOR_NAME
from dynomix_tools.dynamixel_control_table import MOTOR_CONTROL_TABLE
import dynamixel_sdk.port_handler as port_h
import dynamixel_sdk.packet_handler as packet_h
from pprint import pprint

from dynomix_driver.dyn_const import *

from dynamixel_sdk.robotis_def import * # Uses Dynamixel SDK library
import dynamixel_sdk.group_sync_write as gsw
import dynamixel_sdk.group_sync_read as gsr
import dynamixel_sdk.group_bulk_write as gbw
import dynamixel_sdk.group_bulk_read as gbr
import struct


class Segment():
  def __init__(self, num_joints):
    self.start_time = 0.0  # trajectory segment start time
    self.duration = 0.0  # trajectory segment duration
    self.positions = [0.0] * num_joints
    self.velocities = [0.0] * num_joints

class JointTrajectoryActionController():
  def __init__(self, controller_namespace, controllers):
    print("HELLO FROM JOINT_TRAJECTORY_CONTROLLER")
    rospy.loginfo("HELLO_FROM_JOINT_TRAJECTORY_CONTROLLER")
    for controller in controllers:
      print(controller)
    self.update_rate = 1000
    self.state_update_rate = 50
    self.trajectory = []
    self.port_namespace = "l_arm_port" #TODO Irvin added
    
    self.controller_namespace = controller_namespace
    self.joint_names = [c.joint_name for c in controllers] # IRVIN UPDATED 

    self.joint_to_controller = {}
    for c in controllers:
      self.joint_to_controller[c.joint_name] = c

    self.port_to_joints = {}
    for c in controllers:
      if c.port_namespace not in self.port_to_joints: self.port_to_joints[c.port_namespace] = []
      self.port_to_joints[c.port_namespace].append(c.joint_name)

    # self.port_to_io = {}
    # for c in controllers:
    #   if c.port_namespace in self.port_to_io: continue
    #   self.port_to_io[c.port_namespace] = c.dxl_io

    # NOTE: Note that if we are doing mutliple port management 
    # we also need to reference the different port_handler(s) and packet_handler(s) 
    # for each respective servo
    self.port_handler = {}
    self.packet_handler = {}
    for c in controllers:
      rospy.logwarn("C.PORTNAMESPACE" + c.port_namespace)
      if c.port_namespace in self.port_handler: continue
      self.port_handler[c.port_namespace] = c.port_handler
      self.packet_handler[c.port_namespace] = c.packet_handler
      rospy.logwarn(type(c.port_namespace))

    # NOTE: Note that if we are doing mutliple port management 
    # we also need to reference the different port_handler(s) and packet_handler(s) 
    # for each respective servo
    # self.packet_handler = {}
    # for c in controllers:
    #   if c.port_namespace in self.port_namespace: continue
    #   self.packet_handler[c.port_namespace] = c.packet_handler
        
    self.joint_states = dict(zip(self.joint_names, [c.joint_state for c in controllers]))
    self.num_joints = len(self.joint_names)
    self.joint_to_idx = dict(zip(self.joint_names, range(self.num_joints)))

    self.port_handler_irvin   = port_h.PortHandler("/dev/ttyUSB0")      
    self.packet_handler_irvin = packet_h.PacketHandler(2.0)

  def initialize(self):
    ns = self.controller_namespace + '/joint_trajectory_action_node/constraints'
    self.goal_time_constraint = rospy.get_param(ns + '/goal_time', 0.0)
    self.stopped_velocity_tolerance = rospy.get_param(ns + '/stopped_velocity_tolerance', 0.01)
    self.goal_constraints = []
    self.trajectory_constraints = []
    self.min_velocity = rospy.get_param(self.controller_namespace + '/joint_trajectory_action_node/min_velocity', 0.1)
    
    for joint in self.joint_names:
      self.goal_constraints.append(rospy.get_param(ns + '/' + joint + '/goal', -1.0))
      self.trajectory_constraints.append(rospy.get_param(ns + '/' + joint + '/trajectory', -1.0))
      
    # Message containing current state for all controlled joints
    self.msg = FollowJointTrajectoryFeedback()
    self.msg.joint_names = self.joint_names
    self.msg.desired.positions = [0.0] * self.num_joints
    self.msg.desired.velocities = [0.0] * self.num_joints
    self.msg.desired.accelerations = [0.0] * self.num_joints
    self.msg.actual.positions = [0.0] * self.num_joints
    self.msg.actual.velocities = [0.0] * self.num_joints
    self.msg.error.positions = [0.0] * self.num_joints
    self.msg.error.velocities = [0.0] * self.num_joints
    
    return True

  def start(self):
    self.running = True
    
    self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', JointTrajectory, self.process_command)
    self.state_pub = rospy.Publisher(self.controller_namespace + '/state', FollowJointTrajectoryFeedback, queue_size=1)
    self.action_server = actionlib.SimpleActionServer(self.controller_namespace + '/follow_joint_trajectory',
                                                      FollowJointTrajectoryAction,
                                                      execute_cb=self.process_follow_trajectory,
                                                      auto_start=False)
    self.action_server.start()
    # Thread(target=self.update_state).start()  # TODO: Irvin should

  def stop(self):
    self.running = False

  def process_command(self, msg):
    if self.action_server.is_active(): self.action_server.set_preempted()
    
    while self.action_server.is_active():
      rospy.sleep(0.01)
        
    self.process_trajectory(msg)

  def process_follow_trajectory(self, goal):
    self.process_trajectory(goal.trajectory)

  def process_trajectory(self, traj):
    num_points = len(traj.points)
    
    # make sure the joints in the goal match the joints of the controller
    if set(self.joint_names) != set(traj.joint_names):
      res = FollowJointTrajectoryResult()
      res.error_code=FollowJointTrajectoryResult.INVALID_JOINTS
      msg = 'Incoming trajectory joints do not match the joints of the controller'
      rospy.logerr(msg)
      rospy.logerr(' self.joint_names={%s}' % (set(self.joint_names)))
      rospy.logerr(' traj.joint_names={%s}' % (set(traj.joint_names)))
      self.action_server.set_aborted(result=res, text=msg)
      return
      
    # make sure trajectory is not empty
    if num_points == 0:
      msg = 'Incoming trajectory is empty'
      rospy.logerr(msg)
      self.action_server.set_aborted(text=msg)
      return
        
    # correlate the joints we're commanding to the joints in the message
    # map from an index of joint in the controller to an index in the trajectory
    lookup = [traj.joint_names.index(joint) for joint in self.joint_names]
    durations = [0.0] * num_points
    
    # find out the duration of each segment in the trajectory
    durations[0] = traj.points[0].time_from_start.to_sec()
    
    for i in range(1, num_points):
      durations[i] = (traj.points[i].time_from_start - traj.points[i - 1].time_from_start).to_sec()
        
    if not traj.points[0].positions:
      res = FollowJointTrajectoryResult()
      res.error_code=FollowJointTrajectoryResult.INVALID_GOAL
      msg = 'First point of trajectory has no positions'
      rospy.logerr(msg)
      self.action_server.set_aborted(result=res, text=msg)
      return
        
    trajectory = []
    time = rospy.Time.now() + rospy.Duration(0.01)
    
    for i in range(num_points):
      seg = Segment(self.num_joints)
      
      if traj.header.stamp == rospy.Time(0.0):
        seg.start_time = (time + traj.points[i].time_from_start).to_sec() - durations[i]
      else:
        seg.start_time = (traj.header.stamp + traj.points[i].time_from_start).to_sec() - durations[i]
          
      seg.duration = durations[i]
      
      # Checks that the incoming segment has the right number of elements.
      if traj.points[i].velocities and len(traj.points[i].velocities) != self.num_joints:
        res = FollowJointTrajectoryResult()
        res.error_code=FollowJointTrajectoryResult.INVALID_GOAL
        msg = 'Command point %d has %d elements for the velocities' % (i, len(traj.points[i].velocities))
        rospy.logerr(msg)
        self.action_server.set_aborted(result=res, text=msg)
        return
          
      if len(traj.points[i].positions) != self.num_joints:
        res = FollowJointTrajectoryResult()
        res.error_code=FollowJointTrajectoryResult.INVALID_GOAL
        msg = 'Command point %d has %d elements for the positions' % (i, len(traj.points[i].positions))
        rospy.logerr(msg)
        self.action_server.set_aborted(result=res, text=msg)
        return
          
      for j in range(self.num_joints):
        if traj.points[i].velocities:
          seg.velocities[j] = traj.points[i].velocities[lookup[j]]
        if traj.points[i].positions:
          seg.positions[j] = traj.points[i].positions[lookup[j]]
            
      trajectory.append(seg)
        
    rospy.loginfo('Trajectory start requested at %.3lf, waiting...', traj.header.stamp.to_sec())
    rate = rospy.Rate(self.update_rate)
    
    while traj.header.stamp > time:
      time = rospy.Time.now()
      rate.sleep()
        
    end_time = traj.header.stamp + rospy.Duration(sum(durations))
    seg_end_times = [rospy.Time.from_sec(trajectory[seg].start_time + durations[seg]) for seg in range(len(trajectory))]
    
    rospy.loginfo('Trajectory start time is %.3lf, end time is %.3lf, total duration is %.3lf', time.to_sec(), end_time.to_sec(), sum(durations))
    
    self.trajectory = trajectory
    traj_start_time = rospy.Time.now()
    
    for seg in range(len(trajectory)):
      rospy.logdebug('current segment is %d time left %f cur time %f' % (seg, durations[seg] - (time.to_sec() - trajectory[seg].start_time), time.to_sec()))
      rospy.logdebug('goal positions are: %s' % str(trajectory[seg].positions))
      
      # first point in trajectories calculated by OMPL is current position with duration of 0 seconds, skip it
      if durations[seg] == 0:
        rospy.logdebug('skipping segment %d with duration of 0 seconds' % seg)
        continue
          
      multi_packet = {}
      
      for port, joints in self.port_to_joints.items():
        vals = []
        
        for joint in joints:
          j = self.joint_names.index(joint)
          
          start_position = self.joint_states[joint].current_pos
          if seg != 0: start_position = trajectory[seg - 1].positions[j]
              
          desired_position = trajectory[seg].positions[j]
          rospy.logwarn('DESIRED_POSITIONS: ' + str(trajectory[seg].positions[j]))
          desired_velocity = max(self.min_velocity, abs(desired_position - start_position) / durations[seg])
          
          self.msg.desired.positions[j] = desired_position
          self.msg.desired.velocities[j] = desired_velocity
          
          # probably need a more elegant way of figuring out if we are dealing with a dual controller
          if hasattr(self.joint_to_controller[joint], "master_id"):
            master_id = self.joint_to_controller[joint].master_id
            slave_id = self.joint_to_controller[joint].slave_id
            master_pos, slave_pos = self.joint_to_controller[joint].pos_rad_to_raw(desired_position)
            spd = self.joint_to_controller[joint].spd_rad_to_raw(desired_velocity)
            vals.append((master_id, master_pos, spd))
            vals.append((slave_id, slave_pos, spd))
          else: # Only one controller
            motor_id = self.joint_to_controller[joint].motor_id
            pos = self.joint_to_controller[joint].pos_rad_to_raw(desired_position)
            spd = self.joint_to_controller[joint].spd_rad_to_raw(desired_velocity)
            vals.append((motor_id, pos, spd))
          
        multi_packet[port] = vals
          
      for port, vals in multi_packet.items():
      #   self.port_to_io[port].set_multi_position_and_speed(vals)  # TODO: IRVIN Commented
        rospy.logwarn("SENDING MULTI_POSITION_AND_SPEED...")
        rospy.logwarn("VALS:::: %s" % (vals,))
        self.__irvin_set_multi_position_and_speed(vals, port) # TODO: IRVIN's new function
        # rospy.loginfo("MULTI_PACKET>ITEMS::: " + port)
      
      # for port in self.port_handler:
      #   rospy.loginfo("PORT_HANDLE:::::" + port)

      while time < seg_end_times[seg]:
        # check if new trajectory was received, if so abort current trajectory execution
        # by setting the goal to the current position
        if self.action_server.is_preempt_requested():
          msg = 'New trajectory received. Aborting old trajectory.'
          multi_packet = {}
            
          for port, joints in self.port_to_joints.items():
            vals = []
              
            for joint in joints:
              cur_pos = self.joint_states[joint].current_pos
              
              motor_id = self.joint_to_controller[joint].motor_id
              pos = self.joint_to_controller[joint].pos_rad_to_raw(cur_pos)
              
              vals.append((motor_id, pos))
                
            multi_packet[port] = vals
              
            # for port, vals in multi_packet.items():
            #   self.port_to_io[port].set_multi_position(vals)

            for port, vals in multi_packet.items():
              rospy.logwarn("SENDING MULTI_POSITION...")
              self.__irvin_set_multi_position(vals, port)
              # rospy.loginfo("MULTI_PACKET>ITEMS>>>" + port)

            # for port in self.port_handler:
            #   rospy.loginfo("PORT_HANDLER>>>>" + port)
              
            self.action_server.set_preempted(text=msg)
            rospy.logwarn(msg)
            return
            
        rate.sleep()
        time = rospy.Time.now()
          
      # Verifies trajectory constraints
      for j, joint in enumerate(self.joint_names):
        if self.trajectory_constraints[j] > 0 and self.msg.error.positions[j] > self.trajectory_constraints[j]:
          res = FollowJointTrajectoryResult()
          res.error_code=FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
          msg = 'Unsatisfied position constraint for %s, trajectory point %d, %f is larger than %f' % \
              (joint, seg, self.msg.error.positions[j], self.trajectory_constraints[j])
          rospy.logwarn(msg)
          self.action_server.set_aborted(result=res, text=msg)
          return
              
    # let motors roll for specified amount of time
    rospy.sleep(self.goal_time_constraint)
    
    for i, j in enumerate(self.joint_names):
      rospy.logdebug('desired pos was %f, actual pos is %f, error is %f' % (trajectory[-1].positions[i], self.joint_states[j].current_pos, self.joint_states[j].current_pos - trajectory[-1].positions[i]))
        
    # Checks that we have ended inside the goal constraints
    for (joint, pos_error, pos_constraint) in zip(self.joint_names, self.msg.error.positions, self.goal_constraints):
      if pos_constraint > 0 and abs(pos_error) > pos_constraint:
        res = FollowJointTrajectoryResult()
        res.error_code=FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
        msg = 'Aborting because %s joint wound up outside the goal constraints, %f is larger than %f' % \
              (joint, pos_error, pos_constraint)
        rospy.logwarn(msg)
        self.action_server.set_aborted(result=res, text=msg)
        break
    else:
      msg = 'Trajectory execution successfully completed'
      rospy.loginfo(msg)
      res = FollowJointTrajectoryResult()  
      res.error_code=FollowJointTrajectoryResult.SUCCESSFUL
      self.action_server.set_succeeded(result=res, text=msg)

  def __irvin_set_multi_position_and_speed(self, valueTuples, port):
    """
    Set different positions and speeds for multiple servos.
    Should be called as such:
    set_multi_position_and_speed( ( (id1, position1, speed1), (id2, position2, speed2), (id3, position3, speed3) ) )
    """

    # TODO: { START OF IRVIN'S implentation of JointTrajectoryActionController

    # prepare value tuples for call to syncwrite
    writeableVals = []

    groupBulkWrite = gbw.GroupBulkWrite(self.port_handler[port], self.packet_handler[port])
    for val in valueTuples:
      sid       = val[0]
      position  = val[1]
      speed     = val[2]
      rospy.logwarn("VAL POSITION::: " + str(position))
      # split speed into 2 bytes
      if speed >= 0:
        loSpeedVal = int(speed % 256)
        hiSpeedVal = int(speed >> 8)
      else:
        loSpeedVal = int((1023 - speed) % 256)
        hiSpeedVal = int((1023 - speed) >> 8)
      # split position into 2 bytes
      loPositionVal = int(position % 256)
      hiPositionVal = int(position >> 8)
      rospy.logwarn("VAL SPEED::: " + str(speed))

      # model_name /dynamixel/l_arm_port/motor_info/11/model_number

      model_number = rospy.get_param('/dynamixel/l_arm_port/motor_info/' + str(sid) + '/model_number')

      if model_number == MX_64_T_2_NUMBER:
        address_vel = MX_64_T_2_GOAL_VELOCITY
        address_vel_len = MX_64_T_2_GOAL_VELOCITY_LENGTH
        address_pos = MX_64_T_2_GOAL_POSITION
        address_pos_len = MX_64_T_2_GOAL_POSITION_LENGTH
      elif model_number == MX_106_T_2_NUMBER:
        address_vel = MX_106_T_2_GOAL_VELOCITY
        address_vel_len = MX_106_T_2_GOAL_VELOCITY_LENGTH
        address_pos = MX_106_T_2_GOAL_POSITION
        address_pos_len = MX_106_T_2_GOAL_POSITION_LENGTH

      rospy.loginfo("model_num:::::::: " + str(model_number))
      rospy.loginfo("KEYS::::::::::::::::::::::::::::::::::::::::::::::::::")
      model_name = MODEL_NUMBER_2_MOTOR_NAME[model_number]['name']
      ADDR_GOAL_POSITION = MOTOR_CONTROL_TABLE[model_name]['goal_position']['address']
      LEN_GOAL_POSITION = MOTOR_CONTROL_TABLE[model_name]['goal_position']['size']
      ADDR_GOAL_VELOCITY = MOTOR_CONTROL_TABLE[model_name]['profile_velocity']['address']
      LEN_GOAL_VELOCITY = MOTOR_CONTROL_TABLE[model_name]['profile_velocity']['size']

      param_goal_velocity = bytearray(struct.pack("f", speed))  
      dxl_addparam_result_vel = groupBulkWrite.addParam(sid, address_vel, address_vel_len, param_goal_velocity)
      if dxl_addparam_result_vel != True:
        rospy.logerr("[ID:%03d] groupSyncWrite addparam velocity failed" % sid)

        # quit()

        # rospy.loginfo("model_num: " + str(model_number))
        # rospy.loginfo(" model_name: "  + model_name)
        # rospy.loginfo(" address: " + str(address)) 
        # rospy.loginfo(" length: " + str(length))

        # #TODO Implement SYNC_WRITE
        # rospy.logwarn("PORT")
        # rospy.logwarn(port)
        # rospy.logwarn('self.packet_handler[port]')
        # rospy.logwarn(type(self.packet_handler[port]))
        # rospy.logwarn('self.port_handler[port]')
        # rospy.logwarn(type(self.port_handler[port]))
        # rospy.logwarn(self.port_handler[port].getPortName())
        # curr_packet_handler = self.packet_handler[port]
        # curr_port_handler = self.port_handler[port]
        # curr_packet_handler.write4ByteTxRx(curr_port_handler, sid, address, position)
    # Syncwrite goal position
    dxl_comm_result = groupBulkWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
      rospy.logwarn("%s" % self.packet_handler[port].getTxRxResult(dxl_comm_result))
    groupBulkWrite.clearParam()

    for val in valueTuples:
      sid       = val[0]
      position  = val[1]
      speed     = val[2]
      rospy.logwarn("VAL POSITION::: " + str(position))
      # split speed into 2 bytes
      if speed >= 0:
        loSpeedVal = int(speed % 256)
        hiSpeedVal = int(speed >> 8)
      else:
        loSpeedVal = int((1023 - speed) % 256)
        hiSpeedVal = int((1023 - speed) >> 8)
      # split position into 2 bytes
      loPositionVal = int(position % 256)
      hiPositionVal = int(position >> 8)
      rospy.logwarn("VAL SPEED::: " + str(speed))

      model_number = rospy.get_param('/dynamixel/l_arm_port/motor_info/' + str(sid) + '/model_number')

      if model_number == MX_64_T_2_NUMBER:
        address_vel = MX_64_T_2_GOAL_VELOCITY
        address_vel_len = MX_64_T_2_GOAL_VELOCITY_LENGTH
        address_pos = MX_64_T_2_GOAL_POSITION
        address_pos_len = MX_64_T_2_GOAL_POSITION_LENGTH
      elif model_number == MX_106_T_2_NUMBER:
        address_vel = MX_106_T_2_GOAL_VELOCITY
        address_vel_len = MX_106_T_2_GOAL_VELOCITY_LENGTH
        address_pos = MX_106_T_2_GOAL_POSITION
        address_pos_len = MX_106_T_2_GOAL_POSITION_LENGTH

      param_goal_position = bytearray(struct.pack("i", position))  
      dxl_addparam_result_pos = groupBulkWrite.addParam(sid, address_pos, address_pos_len, param_goal_position)
      if dxl_addparam_result_pos != True:
        rospy.logerr("[ID:%03d] groupSyncWrite addparam position failed" % sid)

    # Syncwrite goal position
    dxl_comm_result = groupBulkWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
      rospy.logwarn("%s" % self.packet_handler[port].getTxRxResult(dxl_comm_result))

    groupBulkWrite.clearParam()
    
  # TODO: } ^END OF IRVIN'S IMPLEMENTATION CODE above ends

    # use sync write to broadcast multi servo message
    # self.sync_write(DXL_GOAL_POSITION_L, tuple(writeableVals))

  def __irvin_set_multi_position(self, valueTuples , port):
    """
    Set different positions for multiple servos.
    Should be called as such:
    set_multi_position( ( (id1, position1), (id2, position2), (id3, position3) ) )
    """
    # prepare value tuples for call to syncwrite
    writeableVals = []

    for vals in valueTuples:
      sid = vals[0]
      position = vals[1]
      # split position into 2 bytes
      loVal = int(position % 256)
      hiVal = int(position >> 8)
      writeableVals.append( (sid, loVal, hiVal) )
      
      # model_name /dynamixel/l_arm_port/motor_info/11/model_number
      rospy.logwarn('ID++++++++++++++ ' + sid)
      model_number = rospy.get_param('/dynamixel/l_arm_port/motor_info/' + str(sid) + '/model_number')
      rospy.logwarn("KEYS++++++++++++++++++++++++++ ")
      rospy.logwarn(model_number.keys())
      model_name = MODEL_NUMBER_2_MOTOR_NAME[model_number]['name']
      address = MOTOR_CONTROL_TABLE[model_name]['goal_position']['address']
      length = MOTOR_CONTROL_TABLE[model_name]['goal_position']['size']

      rospy.logwarn("model_num: " + model_number + " model_name: " 
        + model_name + " address: " + address + " length: " + length)

      self.packet_handler[port].writeTxRx(self.port_handler[port], sid, address, length, position )

    # # use sync write to broadcast multi servo message
    # self.sync_write(DXL_GOAL_POSITION_L, writeableVals)

  def update_state(self):
    rate = rospy.Rate(self.state_update_rate)
    while self.running and not rospy.is_shutdown():
      self.msg.header.stamp = rospy.Time.now()
      
      # Publish current joint state
      for i, joint in enumerate(self.joint_names):
        state = self.joint_states[joint]
        self.msg.actual.positions[i] = state.current_pos
        self.msg.actual.velocities[i] = abs(state.velocity)
        self.msg.error.positions[i] = self.msg.actual.positions[i] - self.msg.desired.positions[i]
        self.msg.error.velocities[i] = self.msg.actual.velocities[i] - self.msg.desired.velocities[i]
        
      self.state_pub.publish(self.msg)
      rate.sleep()