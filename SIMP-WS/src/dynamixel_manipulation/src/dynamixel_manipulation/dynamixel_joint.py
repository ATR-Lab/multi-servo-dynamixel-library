import dynamixel_sdk.port_handler as port_handler
import dynamixel_sdk.packet_handler as packet_packet

class DynamixelJoint:
  """
  Representation of a Dynamixel joint / actuator
  Uses Dynamixel SDK to write position values and read data
  """
  def initialize(self):
    raise NotImplementedError

  def writeJointValues(actuator_id, joint_values):
    """Writes joint values in Radians"""

  def writeJointValue(actuator_id, joint_value_rad):
    """Writes joint values in Radians"""
    

  def writeGoalPosition(actuator_id, goal_position_rad):
    """Write goal position"""