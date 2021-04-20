# Import custom libraries and Port / Packet Handler
from .dyn_const import *
import dynamixel_sdk.port_handler as port_h
import dynamixel_sdk.packet_handler as packet_h
from dynamixel_sdk import *
from dynomix_tools import dynamixel_tools

# Import standard and threading libraries
import time
import serial
from array import array
from binascii import b2a_hex
from threading import Lock
import rospy
import math


class SDKSerialWrapper:
  """
  Replacement for Dynamixel-IO, slimed down to only have neccessary functions
  """
  def __init__(self, port, baudrate, feedback_echo=False, protocol_version=2.0):
    # Initialize Port and Packet Handler, also make needed variables / objects
    # TODO: Should try/catch in case of errors
    self.serial_mutex   = Lock()
    self.port_handler   = port_h.PortHandler(port)
    self.packet_handler = packet_h.PacketHandler(protocol_version)
    self.port_handler.openPort()
    self.port_handler.setBaudRate(baudrate)
    self.port = port
    self.baudrate = baudrate
    self.dynotools = dynamixel_tools.DynamixelTools()
    self.raw_to_deg_switch={
      '54024' : self.raw_to_deg_pulse,
      '311' : self.raw_to_deg_static
    }
    self.deg_to_raw_switch={
      '54024' : self.deg_to_raw_pulse,
      '311' : self.deg_to_raw_static
    }
    self.shift_register_values = {
      'H54_200_S500_R_2' : 0,
      'MX_64_T_2' : 0
    }

  def debugOutput(self, servo_id, register, value):
    print("DEBUG OUTPUT AT SERVO " + str(servo_id) + " OF " + str(register) + ": " + str(value))


  def read(self, servo_id, address, size):
    """Read "size" bytes of data from servo with a given "servo_id" at
    the register with "address". "address" is an integer between 0 and 57.
    It is recommended to use the constant in module for readability

    e.g: to read from servo with 1,
      read(1, MX_106_GOAL_POSITION, 2)
    """
    # Reads from Dynamixel SDK packet handler
    with self.serial_mutex:
      # If size is 4, use the 4 byte tx only function
      if size == 4:
        try:
          result = self.packet_handler.read4ByteTxRx(
            self.port_handler, 
            servo_id,
            address)
        except:
          result = self.packet_handler.read2ByteTxRx(
            self.port_handler, 
            servo_id,
            address)

      # If size is 2, use the 2 byte tx only function
      elif size == 2:
        result = self.packet_handler.read2ByteTxRx(
          self.port_handler, 
          servo_id,
          address)

      # Else just use whatever size needed
      else:
        result = self.packet_handler.readTxRx(self.port_handler, servo_id, address, size)

      # wait for response packet from the motor
      timestamp = time.time()
      time.sleep(0.00235) #0.00235 is a debug time value

    return result


  def write(self, servo_id, address, size, data):
    """ Write the values from the "data" list to the servo with "servo_id"
    starting with data[0] at "address", continuing through data[n-1] at
    "address" + (n-1), where n = len(data). "address" is an integer between
    0 and 49. It is recommended to use the constants in module dynamixel_const
    for readability. "data" is a list/tuple of integers.
    To set servo with id 1 to position 276, the method should be called
    like:
        write(1, DXL_GOAL_POSITION_L, (20, 1))
    """    
    with self.serial_mutex:
      
      # If size is 4, use the 4 byte tx only function
      if size == 4:
        result = self.packet_handler.write4ByteTxOnly(
          self.port_handler, 
          servo_id,
          address,
          data)

      # If size is 2, use the 2 byte tx only function
      elif size == 2:
        result = self.packet_handler.write2ByteTxOnly(
          self.port_handler, 
          servo_id,
          address,
          data)

      # Else just use whatever size needed
      else:
        result = self.packet_handler.writeTxRx(self.port_handler, servo_id, address, size, data)

      # wait for response packet from the motor
      timestamp = time.time()
      time.sleep(0.00235)

    return result


  # TODO: Look into this function and figure out how it is used.
  def sync_write(self, address, data):
    """ Use Broadcast message to send multiple servos instructions at the
    same time. No "status packet" will be returned from any servos.
    "address" is an integer between 0 and 49. It is recommended to use the
    constants in module dynamixel_const for readability. "data" is a tuple of
    tuples. Each tuple in "data" must contain the servo id followed by the
    data that should be written from the starting address. The amount of
    data can be as long as needed.
    To set servo with id 1 to position 276 and servo with id 2 to position
    550, the method should be called like:
        sync_write(DXL_GOAL_POSITION_L, ( (1, 20, 1), (2 ,38, 2) ))
    
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

    with self.serial_mutex:
      self.__write_serial(packetStr)
    """

  def ping(self, servo_id):
    """ Ping the servo with "servo_id". This causes the servo to return a
    "status packet". This can tell us if the servo is attached and powered,
    and if so, if there are any errors.
    """
    # Get packet handler to try and ping out for the motor ID
    with self.serial_mutex:
      result = self.packet_handler(self.port_handler, servo_id)

      # wait for response packet from the motor
      timestamp = time.time()
      time.sleep(0.0013)
    
    return result


  ##########################################
  # SINGLE SERVO FUNCTIONS
  ##########################################
  def set_torque_enabled(self, servo_id, motor_info, enabled):
    """
    Sets the value of the torque enabled register to 1 or 0. When the
    torque is disabled the servo can be moved manually while the motor is
    still powered.
    """
    # Get Model Name and Model Number (e.g., MX_64_T_2 - 311)
    model_number = motor_info[str(servo_id)]['model_number']
    model_name = self.dynotools.getModelNameByModelNumber(model_number)

    register_torque_enabled = self.dynotools.getRegisterAddressByModel(model_name, 
                                                                      "torque_enable")
    register_torque_enabled_length = self.dynotools.getAddressSizeByModel(model_name, 
                                                                        "torque_enable")

    response = self.write(servo_id, register_torque_enabled, register_torque_enabled_length, enabled)

    # TODO: Either implement or remove error handling
    # if response:
      # self.exception_on_error(response[4], servo_id, '%sabling torque' % 'en' if enabled else 'dis')
    
    return response


  def set_goal_position(self, servo_id, goal_position, motor_info):
    """
    Set the servo with servo_id to the specified goal position.
    Position value must be positive.
    """
    # Get Model Name and Model Number (e.g., MX_64_T_2 - 311)
    model_number = motor_info[str(servo_id)]['model_number']
    model_name = self.dynotools.getModelNameByModelNumber(model_number)

    register_goal_position = self.dynotools.getRegisterAddressByModel(model_name, 
                                                                      "goal_position")
    register_goal_position_length = self.dynotools.getAddressSizeByModel(model_name, 
                                                                        "goal_position")

    raw_pos = self.deg_to_raw_switch[str(model_number)](model_number, goal_position, motor_info[str(servo_id)]['max_angle'])

    response = self.write(servo_id, register_goal_position, register_goal_position_length, int(raw_pos))

    return response


  #################################
  # Servo status access functions #
  #################################
  # TODO: Implement these functions, maybe?
  """
  def get_model_number(self, servo_id):
    # Reads the servo's model number (e.g. 12 for AX-12+).
    response = self.read(servo_id, DXL_MODEL_NUMBER_L, 2)
    if response:
      self.exception_on_error(response[4], servo_id, 'fetching model number')
    return response[5] + (response[6] << 8)

  def get_firmware_version(self, servo_id):
    # Reads the servo's firmware version.
    response = self.read(servo_id, DXL_VERSION, 1)
    if response:
      self.exception_on_error(response[4], servo_id, 'fetching firmware version')
    return response[5]

  def get_return_delay_time(self, servo_id):
    # Reads the servo's return delay time.
    response = self.read(servo_id, DXL_RETURN_DELAY_TIME, 1)
    if response:
      self.exception_on_error(response[4], servo_id, 'fetching return delay time')
    return response[5]

  def get_drive_mode(self, servo_id):
    # Reads the servo's drive mode.
    response = self.read(servo_id, DXL_DRIVE_MODE, 1)
    if response:
      self.exception_on_error(response[4], servo_id, 'fetching drive mode')
    return response[5]
  """

  """
  def get_current(self, servo_id, model_number, model_name):
    # Reads the servo's current consumption (if supported by model)
    # Make sure model is supported
    if not model_number in DXL_MODEL_TO_PARAMS:
      raise UnsupportedFeatureError(model_number, DXL_CURRENT_L)

    # Case of regular present current
    if DXL_CURRENT_L in DXL_MODEL_TO_PARAMS[model_number]['features']:
      # Register Address and Length variables
      register_present_current = self.dynotools.getRegisterAddressByModel(model_name, "present_current")
      register_present_current_length = self.dynotools.getAddressSizeByModel(model_name, "present_current")

      # Read using present position
      raw_response = self.read(servo_id, register_present_current, register_present_current_length)
      response = raw_response[0]
      
      # TODO: Either implement or remove error handling
      # if response:
        # self.exception_on_error(response[4], servo_id, 'fetching sensed current')

      # Caculate by bit shift left first index by eight, then add result to index 0
      # Example with [66, 6, 0, 0]: 66 << 8 = 1536; 1536 + 66 = 1602
      current = response[0] + (response[1] << 8)
      return 0.0045 * (current - 2048)

    # Case of sensed current
    if DXL_SENSED_CURRENT_L in DXL_MODEL_TO_PARAMS[model_number]['features']:
      # Register Address and Length variables
      register_present_current = self.dynotools.getRegisterAddressByModel(model_name, "sensed_current")
      register_present_current_length = self.dynotools.getAddressSizeByModel(model_name, "sensed_current")

      # Read using present position
      raw_response = self.read(servo_id, register_sensed_current, register_sensed_current_length)
      response = raw_response[0]

      # TODO: Either implement or remove error handling
      # if response:
        # self.exception_on_error(response[4], servo_id, 'fetching sensed current')
     
      # Caculate by bit shift left first index by eight, then add result to index 0
      # Example with [66, 6, 0, 0]: 66 << 8 = 1536; 1536 + 66 = 1602      
      current = response[0] + (response[1] << 8)
      return 0.01 * (current - 512)

    # No known way of supporting current
    else:
      raise UnsupportedFeatureError(model_number, DXL_CURRENT_L)
    """


  def get_angle_limits(self, servo_id, model_name):
    """
    Returns the min and max angle limits from the specified servo.
    """
    # read in 4 consecutive bytes starting with low value of clockwise angle limit
    # Register Address and Length variables for Angle Min
    register_angle_min = self.dynotools.getRegisterAddressByModel(model_name, "angle_limit_min")
    register_angle_min_length = self.dynotools.getAddressSizeByModel(model_name, "angle_limit_min")
    
    # Register Address and Length variables for Angle Max
    register_angle_max = self.dynotools.getRegisterAddressByModel(model_name, "angle_limit_max")
    register_angle_max_length = self.dynotools.getAddressSizeByModel(model_name, "angle_limit_max")

    # Read using angle min
    raw_response_min = self.read(servo_id, register_angle_min, register_angle_min_length)
    angle_min = raw_response_min[0]

    # Read using angle max
    raw_response_max = self.read(servo_id, register_angle_max, register_angle_max_length)
    angle_max = raw_response_max[0]

    if angle_min > angle_max:
      result = self.packet_handler.readTxRx(self.port_handler, servo_id, register_angle_min, register_angle_min_length)
      response_min = result[0]
      if response_min[3] >= 1:
        angle_min_ref = [255, 255, 255, 255]
        response_min[0] = angle_min_ref[0] - response_min[0]
        response_min[1] = angle_min_ref[1] - response_min[1]
        response_min[2] = angle_min_ref[2] - response_min[2]
        response_min[3] = angle_min_ref[3] - response_min[3]
        angle_min = (response_min[0] + 1) + (response_min[1] << 8) + (response_min[2] << 16) + (response_min[3] << 32)
        angle_min *= -1
      else:
        angle_min = response_min[0] + (response_min[1] << 8) + (response_min[2] << 16) + (response_min[3] << 32)

    # return the data in a dictionary
    return {'min': angle_min, 'max': angle_max}
  

  def get_voltage_limits(self, servo_id, model_name):
    """
    Returns the min and max voltage limits from the specified servo.
    """
    # Register Address and Length variables for Voltage Max
    register_max_voltage = self.dynotools.getRegisterAddressByModel(model_name, "max_voltage")
    register_max_voltage_length = self.dynotools.getAddressSizeByModel(model_name, "max_voltage")

    # Register Address and Length variables for Voltage Min
    register_min_voltage = self.dynotools.getRegisterAddressByModel(model_name, "min_voltage")
    register_min_voltage_length = self.dynotools.getAddressSizeByModel(model_name, "min_voltage")

    # Read using Voltage Min
    raw_response_min = self.read(servo_id, register_min_voltage, register_min_voltage_length)
    min_voltage = raw_response_min[0]

    # Read using Voltage Max
    raw_response_max = self.read(servo_id, register_max_voltage, register_max_voltage_length)
    max_voltage = raw_response_max[0]

    # return the data in a dictionary
    return {'min' : min_voltage, 'max' : max_voltage}


  def get_goal(self, servo_id, model_name):
    """ Reads the servo's position value from its registers. """
    # Register Address and Length variables
    register_goal_position = self.dynotools.getRegisterAddressByModel(model_name, "goal_position")
    register_goal_position_length = self.dynotools.getAddressSizeByModel(model_name, "goal_position")

    result = self.packet_handler.readTxRx(self.port_handler, servo_id, register_goal_position, register_goal_position_length)
    response = result[0]

    try:
      if register_goal_position_length == 2:
        position = response[0] + (response[1] << 8)

      elif register_goal_position_length == 4:      
        if response[3] >= 1:
          goal_position_ref = [255, 255, 255, 255]
          response[0] = goal_position_ref[0] - response[0]
          response[1] = goal_position_ref[1] - response[1]
          response[2] = goal_position_ref[2] - response[2]
          response[3] = goal_position_ref[3] - response[3]
          position = (response[0] + 1) + (response[1] << 8) + (response[2] << 16) + (response[3] << 32)
          position *= -1
      
        else:
          position = response[0] + (response[1] << 8) + (response[2] << 16) + (response[3] << 32)
    
    except:
      # Read using present position
      raw_response = self.read(servo_id, register_goal_position, register_goal_position_length)
      position = raw_response[0]

    # TODO: Either implement or remove error handling
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching present position')

    return position


  def get_position(self, servo_id, model_name):
    """ Reads the servo's position value from its registers. """
    # Register Address and Length variables
    register_present_position = self.dynotools.getRegisterAddressByModel(model_name, "present_position")
    register_present_position_length = self.dynotools.getAddressSizeByModel(model_name, "present_position")

    result = self.packet_handler.readTxRx(self.port_handler, servo_id, register_present_position, register_present_position_length)
    response = result[0]

    try:     
      if response[3] >= 1:
        present_position_ref = [255, 255, 255, 255]
        response[0] = present_position_ref[0] - response[0]
        response[1] = present_position_ref[1] - response[1]
        response[2] = present_position_ref[2] - response[2]
        response[3] = present_position_ref[3] - response[3]
        position = (response[0] + 1) + (response[1] << 8) + (response[2] << 16) + (response[3] << 32)
        position *= -1
      
      else:
        position = response[0] + (response[1] << 8) + (response[2] << 16) + (response[3] << 32)
    
    except:
      raw_response = self.read(servo_id, register_present_position, register_present_position_length)
      position = raw_response[0]

    # Read using present position
    # raw_response = self.read(servo_id, register_present_position, register_present_position_length)
    # position = raw_response[0]

    # TODO: Either implement or remove error handling
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching present position')
    
    return position

  def get_speed(self, servo_id, model_name):
    """ Reads the servo's speed value from its registers. """
    # Register Address and Length variables
    register_present_speed = self.dynotools.getRegisterAddressByModel(model_name, "present_velocity")
    register_present_speed_length = self.dynotools.getAddressSizeByModel(model_name, "present_velocity")

    # Read using present position
    raw_response = self.packet_handler.readTxRx(self.port_handler, servo_id, register_present_speed, register_present_speed_length)
    response = raw_response[0]

    try:
      if register_present_speed_length == 2:
        speed = response[0] + (response[1] << 8)

      elif register_present_speed_length == 4:      
        if response[3] >= 1:
          present_speed_ref = [255, 255, 255, 255]
          response[0] = present_speed_ref[0] - response[0]
          response[1] = present_speed_ref[1] - response[1]
          response[2] = present_speed_ref[2] - response[2]
          response[3] = present_speed_ref[3] - response[3]
          speed = (response[0] + 1) + (response[1] << 8) + (response[2] << 16) + (response[3] << 32)
          speed *= -1
      
        else:
          speed = response[0] + (response[1] << 8) + (response[2] << 16) + (response[3] << 32)

    except:
      # Read using present position
      raw_response = self.read(servo_id, register_present_speed, register_present_speed_length)
      speed = raw_response[0]

    # TODO: Either implement or remove error handling
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching present speed')

    return speed

  def get_temperature(self, servo_id, model_name):
    """ Reads the servo's temperature value from its registers. """
    # Register Address and Length variables
    register_present_temperature = self.dynotools.getRegisterAddressByModel(model_name, "present_temperature")
    register_present_temperature_length = self.dynotools.getAddressSizeByModel(model_name, "present_temperature")

    # Read using present position
    raw_response = self.read(servo_id, register_present_temperature, register_present_temperature_length)
    temperature = raw_response[0]

    if temperature > 99:
      result = self.packet_handler.readTxRx(self.port_handler, servo_id, register_present_temperature, register_present_temperature_length)
      response = result[0]
      temperature = response[0]

    return temperature

  def get_voltage(self, servo_id, model_name):
    """ Reads the servo's voltage. """
    # Register Address and Length variables for Present Voltage
    register_present_voltage = self.dynotools.getRegisterAddressByModel(model_name, "present_voltage")
    register_present_voltage_length = self.dynotools.getAddressSizeByModel(model_name, "present_voltage")

    # Read using Present Voltage
    raw_response = self.read(servo_id, register_present_voltage, register_present_voltage_length)
    voltage = raw_response[0]

    # TODO: Either implement or remove error handling
    # if response:
      # self.exception_on_error(response[4], servo_id, 'fetching supplied voltage')

    # 1 unit = 100 mV
    return voltage / 10.0


  def get_moving(self, servo_id, model_name):
    """ Reads the servo's temperature value from its registers. """
    # Register Address and Length variables
    register_moving = self.dynotools.getRegisterAddressByModel(model_name, "moving")
    register_moving_length = self.dynotools.getAddressSizeByModel(model_name, "moving")

    try:
      result = self.packet_handler.readTxRx(self.port_handler, servo_id, register_moving, register_moving_length)
      response = result[0]
      moving = response[0]
    except:
      # Read using present position
      raw_response = self.read(servo_id, register_moving, register_moving_length)
      moving = raw_response[0]

    return moving


  def get_feedback(self, servo_id, motor_info):
    """
    Returns the id, goal, position, error, speed, load, voltage, temperature
    and moving values from the specified servo.
    """
    # Get Model Name and Model Number (e.g., MX_64_T_2 - 311)
    model_number = motor_info[str(servo_id)]['model_number']
    model_name = self.dynotools.getModelNameByModelNumber(model_number)

    # Get feedback information
    goal = self.get_goal(servo_id, model_name)
    position = self.get_position(servo_id, model_name)
    error = position - goal
    speed = self.get_speed(servo_id, model_name)
    voltage = self.get_voltage(servo_id, model_name)
    temperature = self.get_temperature(servo_id, model_name)
    moving = self.get_moving(servo_id, model_name)

    # Get max and min angles from motor_info
    max_angle = motor_info[str(servo_id)]['max_angle']
    
    degree_position = self.raw_to_deg_switch[str(model_number)](model_number, position, max_angle)
    degree_goal = self.raw_to_deg_switch[str(model_number)](model_number, goal, max_angle)
    rpm_speed = motor_info[str(servo_id)]['rpm_per_tick'] * speed

    # Return above in a container form
    return { 'timestamp': 0,
             'id': servo_id,
             'goal': int(degree_goal),
             'position': int(degree_position),
             'error': error,
             'speed': int(rpm_speed),
             'load': 0,
             'voltage': voltage,
             'temperature': temperature,
             'moving': bool(moving) }
  

  def raw_to_deg_pulse(self, model_number, raw_pos, max_angle):
    base_degree = DXL_MODEL_TO_PARAMS[model_number].get('pulse_const', .088) * raw_pos
    if base_degree < 0:
      base_degree += 360
    return math.floor(base_degree)

  
  def deg_to_raw_pulse(self, model_number, deg_pos, max_angle):
    if deg_pos >= 180:
      deg_pos -= 360
    degree_position = deg_pos / DXL_MODEL_TO_PARAMS[model_number].get('pulse_const', .088)
    return degree_position
  

  def raw_to_deg_static(self, model_number, raw_pos, max_angle):
    base_degree = DXL_MODEL_TO_PARAMS[model_number].get('pulse_const', .088) * raw_pos
    if base_degree <= 180:
      reverse_position = max_angle / 2
    else:
      reverse_position = -(max_angle / 2)
    reverse_position *= DXL_MODEL_TO_PARAMS[model_number].get('pulse_const', .088)
    return (math.ceil(base_degree) + math.floor(reverse_position))


  def deg_to_raw_static(self, model_number, deg_pos, max_angle):
    if deg_pos <= 180:
      reverse_position = max_angle / 2
    else:
      reverse_position = -(max_angle / 2)
    degree_position = deg_pos / DXL_MODEL_TO_PARAMS[model_number].get('pulse_const', .088)
    return (math.ceil(degree_position) + math.floor(reverse_position))
  
  # TODO: look into if we need this function and below classes for error handling,
  #       and if so, how to implement them properly without serial calls.
  """
  def exception_on_error(self, error_code, servo_id, command_failed):
        global exception
        exception = None
        ex_message = '[servo #%d on %s@%sbps]: %s failed' % (servo_id, self.port, self.baudrate, command_failed)

        if not isinstance(error_code, int):
            msg = 'Communcation Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, 0)
            return
        if not error_code & DXL_OVERHEATING_ERROR == 0:
            msg = 'Overheating Error ' + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if not error_code & DXL_OVERLOAD_ERROR == 0:
            msg = 'Overload Error ' + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if not error_code & DXL_INPUT_VOLTAGE_ERROR == 0:
            msg = 'Input Voltage Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_ANGLE_LIMIT_ERROR == 0:
            msg = 'Angle Limit Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_RANGE_ERROR == 0:
            msg = 'Range Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_CHECKSUM_ERROR == 0:
            msg = 'Checksum Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_INSTRUCTION_ERROR == 0:
            msg = 'Instruction Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
  """


class FatalErrorCodeError(Exception):
  def __init__(self, message, ec_const):
      Exception.__init__(self)
      self.message = message
      self.error_code = ec_const
  def __str__(self):
      return self.message


class UnsupportedFeatureError(Exception):
    def __init__(self, model_id, feature_id):
        Exception.__init__(self)
        if model_id in DXL_MODEL_TO_PARAMS:
            model = DXL_MODEL_TO_PARAMS[model_id]['name']
        else:
            model = 'Unknown'
        self.message = "Feature %d not supported by model %d (%s)" %(feature_id, model_id, model)
    def __str__(self):
        return self.message