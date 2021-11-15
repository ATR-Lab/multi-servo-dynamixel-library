#! /usr/bin/python

from __future__ import division

__author__ = 'Irvin Steve Cardenas'
__email__ = 'irvin@irvincardenas.com'

import rospy
from dynomix_joint_controller import DynamixelJointController
from dynomix_tools.dynamixel_tools import DynamixelTools
from dynomix_driver import sdk_serial_wrapper as sdk_io
# from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import MotorStateList
from dynamixel_msgs.msg import JointState
from dynomix_tools.dynamixel_control_table import MODEL_NUMBER_2_MOTOR_NAME
from dynomix_tools.dynamixel_control_table import MOTOR_CONTROL_TABLE

from dynamixel_sdk.robotis_def import * # Uses Dynamixel SDK library
import dynamixel_sdk.group_sync_write as gsw
import dynamixel_sdk.group_sync_read as gsr
import struct

class JointPositionController(DynamixelJointController):
  def __init__(self, port_handler, packet_handler, controller_namespace, port_namespace):
    DynamixelJointController.__init__(self, port_handler, packet_handler, controller_namespace, port_namespace)
    print("HELLO from JOINT_POSITION_CONTROLLER")

    rospy.loginfo("MOTOR ID: " + self.controller_namespace + '/motor/id')

    self.motor_id       = rospy.get_param(self.controller_namespace + '/motor/id')
    self.model_number   = rospy.get_param('/dynamixel/'+ self.port_namespace + '/motor_info/' + str(self.motor_id) + '/model_number')
    rospy.logwarn("MODEL_NUM::: " + str(self.model_number))
    self.model_name     = MODEL_NUMBER_2_MOTOR_NAME[self.model_number]
    self.initial_position_raw = rospy.get_param(self.controller_namespace + '/motor/init')
    self.min_angle_raw  = rospy.get_param(self.controller_namespace + '/motor/min')
    self.max_angle_raw  = rospy.get_param(self.controller_namespace + '/motor/max')
    self.torque_limit   = 1.0 # TODO: Irvin Fix
    if rospy.has_param(self.controller_namespace + '/motor/acceleration'):
        self.acceleration = rospy.get_param(self.controller_namespace + '/motor/acceleration')
    else:
        self.acceleration = None
    
    self.flipped = self.min_angle_raw > self.max_angle_raw

    self.joint_state = JointState(name=self.joint_name, motor_ids=[self.motor_id]) # IRVIN

  def initialize(self):
    # verify that the expected motor is connected and responding
    available_ids = rospy.get_param('dynamixel/%s/connected_ids' % self.port_namespace, [])
    if not self.motor_id in available_ids:
      rospy.logwarn('The specified motor id is not connected and responding.')
      rospy.logwarn('Available ids: %s' % str(available_ids))
      rospy.logwarn('Specified id: %d' % self.motor_id)
      return False
    
    # TODO: IRVIN implment this >>>>>>>
    #
    self.RADIANS_PER_ENCODER_TICK = rospy.get_param('dynamixel/%s/%d/radians_per_encoder_tick' % (self.port_namespace, self.motor_id))
    self.ENCODER_TICKS_PER_RADIAN = rospy.get_param('dynamixel/%s/%d/encoder_ticks_per_radian' % (self.port_namespace, self.motor_id))
    

    if self.flipped:
      self.min_angle = (self.initial_position_raw - self.min_angle_raw) * self.RADIANS_PER_ENCODER_TICK
      self.max_angle = (self.initial_position_raw - self.max_angle_raw) * self.RADIANS_PER_ENCODER_TICK
    else:
      self.min_angle = (self.min_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK
      self.max_angle = (self.max_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK
      
    self.ENCODER_RESOLUTION = rospy.get_param('dynamixel/%s/%d/encoder_resolution' % (self.port_namespace, self.motor_id))
    self.MAX_POSITION = self.ENCODER_RESOLUTION - 1
    self.VELOCITY_PER_TICK = rospy.get_param('dynamixel/%s/%d/radians_second_per_encoder_tick' % (self.port_namespace, self.motor_id))
    self.MAX_VELOCITY = rospy.get_param('dynamixel/%s/%d/max_velocity' % (self.port_namespace, self.motor_id))
    self.MIN_VELOCITY = self.VELOCITY_PER_TICK
    # self.VELOCITY_PER_TICK = 0.02
    # self.MAX_VELOCITY = 0.5
    # self.MIN_VELOCITY = 0.1
    
    # if self.compliance_slope is not None: self.set_compliance_slope(self.compliance_slope)
    # if self.compliance_margin is not None: self.set_compliance_margin(self.compliance_margin)
    # if self.compliance_punch is not None: self.set_compliance_punch(self.compliance_punch)
    if self.torque_limit is not None: self.set_torque_limit(self.torque_limit)  # TODO: IRVIN uncommment this first
    # if self.acceleration is not None:
    #   rospy.loginfo("Setting acceleration of %d to %d" % (self.motor_id, self.acceleration))
    #   self.dxl_io.set_acceleration(self.motor_id, self.acceleration)

    self.joint_max_speed = rospy.get_param(self.controller_namespace + '/joint_max_speed', self.MAX_VELOCITY)
    # self.joint_max_speed = 0.2
    
    if self.joint_max_speed < self.MIN_VELOCITY: self.joint_max_speed = self.MIN_VELOCITY
    elif self.joint_max_speed > self.MAX_VELOCITY: self.joint_max_speed = self.MAX_VELOCITY
    
    if self.joint_speed < self.MIN_VELOCITY: self.joint_speed = self.MIN_VELOCITY
    elif self.joint_speed > self.joint_max_speed: self.joint_speed = self.joint_max_speed
    
    rospy.logwarn("SETTTING THE SPEED:::::::::::" + str(self.joint_speed))
    self.set_speed(self.joint_speed)
    
    return True

  def pos_rad_to_raw(self, pos_rad):
    rospy.logwarn("POS_RAD_TO_RAD::::: " + str(pos_rad) + " MIN_ANGLE: " + str(self.min_angle) + " MAX_ANGLE: " + str(self.max_angle) + " INITIAL_POS_RAW: " + str(self.initial_position_raw) + " FLIPPED: " + str(self.flipped) + " ENCODER_TICKS_PER_RAD::: " + str(self.ENCODER_TICKS_PER_RADIAN))
    if pos_rad < self.min_angle: pos_rad = self.min_angle
    elif pos_rad > self.max_angle: pos_rad = self.max_angle
    return self.rad_to_raw(pos_rad, self.initial_position_raw, self.flipped, self.ENCODER_TICKS_PER_RADIAN)

  def spd_rad_to_raw(self, spd_rad):
    if spd_rad < self.MIN_VELOCITY: spd_rad = self.MIN_VELOCITY
    elif spd_rad > self.joint_max_speed: spd_rad = self.joint_max_speed
    # velocity of 0 means maximum, make sure that doesn't happen
    return max(1, int(round(spd_rad / self.VELOCITY_PER_TICK)))

  def set_torque_enable(self, torque_enable):
    mcv = (self.motor_id, torque_enable)
    self.dxl_io.set_multi_torque_enabled([mcv])

  def set_speed(self, speed):
    # mcv = (self.motor_id, self.spd_rad_to_raw(speed))
    # self.dxl_io.set_multi_speed([mcv])
    LEN_GOAL_VELOCITY = MOTOR_CONTROL_TABLE[str(self.model_name['name'])]['profile_velocity']['size']
    rospy.logwarn('SIZE::: %d', LEN_GOAL_VELOCITY)
    ADDR_GOAL_VELOCITY = MOTOR_CONTROL_TABLE[str(self.model_name['name'])]['profile_velocity']['address']
    groupSyncWrite = gsw.GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)
    # param_goal_velocity = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]))]
    param_goal_velocity = bytearray(struct.pack("f", speed))  
    dxl_addparam_result = groupSyncWrite.addParam(self.motor_id, param_goal_velocity)

    if dxl_addparam_result != True:
      rospy.logerr("[ID:%03d] groupSyncWrite addparam speed failed" % self.motor_id)
      # quit()
    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
      rospy.logwarn("%s" % packetHandler.getTxRxResult(dxl_comm_result))


  def set_compliance_slope(self, slope):
    if slope < DXL_MIN_COMPLIANCE_SLOPE: slope = DXL_MIN_COMPLIANCE_SLOPE
    elif slope > DXL_MAX_COMPLIANCE_SLOPE: slope = DXL_MAX_COMPLIANCE_SLOPE
    mcv = (self.motor_id, slope, slope)
    self.dxl_io.set_multi_compliance_slopes([mcv])

  def set_compliance_margin(self, margin):
    if margin < DXL_MIN_COMPLIANCE_MARGIN: margin = DXL_MIN_COMPLIANCE_MARGIN
    elif margin > DXL_MAX_COMPLIANCE_MARGIN: margin = DXL_MAX_COMPLIANCE_MARGIN
    else: margin = int(margin)
    mcv = (self.motor_id, margin, margin)
    self.dxl_io.set_multi_compliance_margins([mcv])

  def set_compliance_punch(self, punch):
    if punch < DXL_MIN_PUNCH: punch = DXL_MIN_PUNCH
    elif punch > DXL_MAX_PUNCH: punch = DXL_MAX_PUNCH
    else: punch = int(punch)
    mcv = (self.motor_id, punch)
    self.dxl_io.set_multi_punch([mcv])

  # def set_torque_limit(self, max_torque):
  #   if max_torque > 1: max_torque = 1.0         # use all torque motor can provide
  #   elif max_torque < 0: max_torque = 0.0       # turn off motor torque
  #   raw_torque_val = int(DXL_MAX_TORQUE_TICK * max_torque)
  #   mcv = (self.motor_id, raw_torque_val)
  #   self.dxl_io.set_multi_torque_limit([mcv])

  def set_acceleration_raw(self, acc):
    if acc < 0: acc = 0
    elif acc > 254: acc = 254
    self.dxl_io.set_acceleration(self.motor_id, acc)



  def set_torque_enable(self, torque_enable):
    mcv = (self.motor_id, torque_enable)
    # TODO: Use sdk_wrapper
    # self.sdk_io.set_multi_torque_enabled([mcv])
    # size      = DynamixelTools.getAddressSizeByModel(self.model_name['name'], 'torque_enable')
    # address   = DynamixelTools.getAddressSizeByModel(self.model_name['name'], 'address')
    size = MOTOR_CONTROL_TABLE[str(self.model_name['name'])]['torque_enable']['size']
    rospy.logwarn('SIZE::: %d', size)
    address = MOTOR_CONTROL_TABLE[str(self.model_name['name'])]['torque_enable']['address']
    rospy.logwarn('SIZE::: %d', address)
    if size == 1:
      result = self.packet_handler.write1ByteTxOnly(
        self.port_handler, 
        self.motor_id,
        address,
        max_torque)

    elif size == 2:
      result = self.packet_handler.write1ByteTxOnly(
        self.port_handler, 
        self.motor_id,
        address,
        int(max_torque))

  def set_torque_limit(self, max_torque):
    size = MOTOR_CONTROL_TABLE[str(self.model_name['name'])]['torque_enable']['size']
    rospy.logwarn('SIZE::: %d', size)
    address = MOTOR_CONTROL_TABLE[str(self.model_name['name'])]['torque_enable']['address']
    rospy.logwarn('SIZE::: %d', address)

    if size == 2:
      result = self.packet_handler.write2ByteTxOnly(
        self.port_handler, 
        self.motor_id,
        address,
        max_torque)

    elif size ==1:
      result = self.packet_handler.write1ByteTxOnly(
        self.port_handler, 
        self.motor_id,
        address,
        int(max_torque))

  def process_motor_states(self, state_list):
    if self.running:
      state = filter(lambda state: state.id == self.motor_id, state_list.motor_states)
      if state:
        state = state[0]
        self.joint_state.motor_temps = [state.temperature]
        self.joint_state.goal_pos = self.raw_to_rad(state.goal, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)
        self.joint_state.current_pos = self.raw_to_rad(state.position, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)
        self.joint_state.error = state.error * self.RADIANS_PER_ENCODER_TICK
        self.joint_state.velocity = state.speed * self.VELOCITY_PER_TICK
        self.joint_state.load = state.load
        self.joint_state.is_moving = state.moving
        self.joint_state.header.stamp = rospy.Time.from_sec(state.timestamp)
            
        self.joint_state_pub.publish(self.joint_state)


  def process_command(self, msg):
    rospy.logwarn("PROCESSING JOINT_POSITION_CONTROLLER COMMAND::::::::")
    data = msg.data
    angle = msg.data
    # goal_position = int(dynomix_tools.convertRadian2Position(radian_val, 1000, 2000, -1.5, 1.5))
    # self.packet_handler.write2ByteTxRx(self.port_handler, MOTOR_ID, MX_64_GOAL_POSITION, goal_position)

    size = MOTOR_CONTROL_TABLE[str(self.model_name['name'])]['goal_position']['size']
    rospy.logwarn('GOAL_POSITION_ADDRESS_SIZE::: %d', size)
    address = MOTOR_CONTROL_TABLE[str(self.model_name['name'])]['goal_position']['address']
    rospy.logwarn('GOAL_POSITION_ADDRESS::: %d', address)



    if size == 4:
      result = self.packet_handler.write4ByteTxOnly(
        self.port_handler, 
        self.motor_id,
        address,
        self.pos_rad_to_raw(int(data)))
    elif size == 2:
      result = self.packet_handler.write2ByteTxOnly(
        self.port_handler, 
        self.motor_id,
        address,
        self.pos_rad_to_raw(int(data)))
      
    #   rospy.loginfo(self.packet_handler.getTxRxResult(result))
    #   rospy.logwarn(self.packet_handler.getRxPacketError(result))
