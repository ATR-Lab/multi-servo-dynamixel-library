#! /usr/bin/python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from dynamixel_manipulation.srv import SetSpeed

class DynamixelJointController:
  """
  Classes extending DynamixelJointController will implement the empty functions
  """
  def __init__(self, port_handler, packet_handler, controller_namespace, port_namespace):
    self.running = False
    self.port_handler = port_handler
    self.packet_handler = packet_handler
    self.port_namespace = port_namespace
    self.controller_namespace = controller_namespace
    self.joint_name = rospy.get_param(self.controller_namespace + '/joint_name')
    self.joint_speed = rospy.get_param(self.controller_namespace + '/joint_speed', 1.0)
    self.compliance_slope = rospy.get_param(self.controller_namespace + '/joint_compliance_slope', None)
    self.compliance_margin = rospy.get_param(self.controller_namespace + '/joint_compliance_margin', None)
    self.compliance_punch = rospy.get_param(self.controller_namespace + '/joint_compliance_punch', None)
    self.torque_limit = rospy.get_param(self.controller_namespace + '/joint_torque_limit', None)
    print("HELLO FROM DYNAMIXELJOINTCONTROLLER")

    self.__ensure_limits()
    self.speed_service = rospy.Service(self.controller_namespace + '/set_speed', SetSpeed, self.process_set_speed)
    # self.torque_service = rospy.Service(self.controller_namespace + '/torque_enable', TorqueEnable, self.process_torque_enable)
    # self.compliance_slope_service = rospy.Service(self.controller_namespace + '/set_compliance_slope', SetComplianceSlope, self.process_set_compliance_slope)
    # self.compliance_marigin_service = rospy.Service(self.controller_namespace + '/set_compliance_margin', SetComplianceMargin, self.process_set_compliance_margin)
    # self.compliance_punch_service = rospy.Service(self.controller_namespace + '/set_compliance_punch', SetCompliancePunch, self.process_set_compliance_punch)
    # self.torque_limit_service = rospy.Service(self.controller_namespace + '/set_torque_limit', SetTorqueLimit, self.process_set_torque_limit)


  def __ensure_limits(self):
    # TODO: Implement
    # raise NotImplementedError
    if self.compliance_slope is not None:
      if self.compliance_slope < DXL_MIN_COMPLIANCE_SLOPE: self.compliance_slope = DXL_MIN_COMPLIANCE_SLOPE
      elif self.compliance_slope > DXL_MAX_COMPLIANCE_SLOPE: self.compliance_slope = DXL_MAX_COMPLIANCE_SLOPE
      else: self.compliance_slope = int(self.compliance_slope)
      
    if self.compliance_margin is not None:
      if self.compliance_margin < DXL_MIN_COMPLIANCE_MARGIN: self.compliance_margin = DXL_MIN_COMPLIANCE_MARGIN
      elif self.compliance_margin > DXL_MAX_COMPLIANCE_MARGIN: self.compliance_margin = DXL_MAX_COMPLIANCE_MARGIN
      else: self.compliance_margin = int(self.compliance_margin)
      
    if self.compliance_punch is not None:
      if self.compliance_punch < DXL_MIN_PUNCH: self.compliance_punch = DXL_MIN_PUNCH
      elif self.compliance_punch > DXL_MAX_PUNCH: self.compliance_punch = DXL_MAX_PUNCH
      else: self.compliance_punch = int(self.compliance_punch)
      
    if self.torque_limit is not None:
      if self.torque_limit < 0: self.torque_limit = 0.0
      elif self.torque_limit > 1: self.torque_limit = 1.0

  def initialize(self):
    raise NotImplementedError

  def start(self):
    self.running = True
    self.joint_state_pub = rospy.Publisher(self.controller_namespace + '/state', JointState, queue_size=1)
    self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', Float64, self.process_command)
    # self.motor_state_sub = rospy.Subscriber('motor_state/%s' % self.port_namespace, MotorStateList, self.process_motor_states)

  def stop(self):
    # TODO: Finish implementing
    self.running = False
    self.joint_state_pub.unregister()
    self.command_sub.unregister()
    
  def set_speed(self, speed):
    raise NotImplementedError

  def process_set_speed(self, req):
    self.set_speed(req.speed)
    return [] # success

  def rad_to_raw(self, angle, initial_position_raw, flipped, encoder_ticks_per_radian):
    """ angle is in radians """
    #print 'flipped = %s, angle_in = %f, init_raw = %d' % (str(flipped), angle, initial_position_raw)
    angle_raw = angle * encoder_ticks_per_radian
    #print 'angle = %f, val = %d' % (math.degrees(angle), int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw)))
    return int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw))

  def raw_to_rad(self, raw, initial_position_raw, flipped, radians_per_encoder_tick):
    return (initial_position_raw - raw if flipped else raw - initial_position_raw) * radians_per_encoder_tick

  def process_command(self):
    raise NotImplementedError
