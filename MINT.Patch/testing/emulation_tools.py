#Nathan Moder
import time

class EmulatedMotor:
    def __init__(self, _id, _stateID, _voltage, _temperature, _angle, _speed):
        self.id=_id
        self.stateID=_stateID
        self.voltage=_voltage
        self.temperature=_temperature
        self.angle=_angle
        self.speed=_speed
        self.goal_pos=0
        self.start_time=0
        self.moving=False
        self.last_check=0
    def changeName(self, new_name):
        self.name=new_name
    def set_goal_speed(self, new_angle):      
        
        if new_angle==self.angle:
            self.goal_pos=self.angle
            self.speed=0
            self.moving=False
        else:
            self.speed=1
            self.goal_pos=new_angle
            self.moving=True
            self.last_check=self.start_time=time.time()

        
    def check_while_running(self):
        moment=time.time()
        tchange=moment-self.last_check
        self.last_check=moment
        self.angle=self.angle+(tchange*18)
        if self.angle > self.goal_pos:
            self.angle=self.goal_pos
        if self.angle==self.goal_pos:
            self.moving=False
            self.speed=0


global tm1
tm1=EmulatedMotor(1,"IDLE",50,60,0,0)
global tm2
tm2=EmulatedMotor(5,"IDLE",50,61,25,0)
global tm3
tm3=EmulatedMotor(1,"IDLE",70,65,10,0)

class DynomixSerialProxy():
  def __init__(self, 
    port_name='/dev/ttyUSB0', 
    port_namespace='ttyUSB0', 
    baud_rate=1000000, 
    min_motor_id=1,
    max_motor_id=25,
    update_rate=5,
    diagnostics_rate=1,
    error_level_temp=75,
    warn_level_temp=70,
    readback_echo=False,
    protocol_version=2.0):
    self.motors=[]
    self.port_name=port_name
    if(port_namespace=='port_1'):
      #self.motors[0]=tm1
      self.motors.append(1)
      #self.motors[1]=tm2
      self.motors.append(15)
    if(port_namespace=='port_2'):
      self.motors.append(110)
    #print("proxy init")
  def connect(self):
    self.__find_motors()
  def __find_motors(self):
    return 1
  def set_goal_position(self, servo_id, goal):
        #print(self.port)
        if self.port_name=="/dev/port_1":
            #print(servo_id)
            if 1==int(servo_id):
                #print("into if")
                tm1.set_goal_speed(goal)
            elif int(servo_id)==15:
                tm2.set_goal_speed(goal)
        elif self.port_name=="/dev/port_2":
            if int(servo_id)==110:
                tm3.set_goal_speed(goal)

  def get_feedback(self,servo_id):
    if self.port_name=="/dev/port_1":
      if int(servo_id)==1:
        if tm1.moving:
          tm1.check_while_running()
        return {
              'id':tm1.id,
              'goal': 0,
              'position': tm1.angle,
              'error' : 0,
              'speed' : tm1.speed,
              'load' : 0,
              'voltage' : tm1.voltage,
              'temperature' : tm1.temperature,
              'moving' : tm1.moving
              }
      if int(servo_id)==15:
        if tm2.moving:
          tm2.check_while_running()
        return {
              'id':tm2.id,
              'goal': 0,
              'position': tm2.angle,
              'error' : 0,
              'speed' : tm2.speed,
              'load' : 0,
              'voltage' : tm2.voltage,
              'temperature' : tm2.temperature,
              'moving' : tm2.moving
              }
    if self.port_name=="/dev/port_2":
      if int(servo_id)==110:
        if tm3.moving:
          tm3.check_while_running()
        return {
          'id':tm3.id,
          'goal': 0,
          'position': tm3.angle,
          'error' : 0,
          'speed' : tm3.speed,
          'load' : 0,
          'voltage' : tm3.voltage,
          'temperature' : tm3.temperature,
          'moving' : tm3.moving
          }
    return 'no such servo'
  def set_torque_enabled(self, sid, thing):
    return 1

class SDKSerialWrapper:
    def __init__(self, port, baudrate, feedback_echo=False):
        self.port=port
        #print("wrapper init")

    def read(self, servo_id, address, size):
        print("wrapper read")
    def write(self, servo_id, address, data):
        print("wrapper write")
    def sync_write(self, address, data):
        print("wrapper syncwrite")
    
    def set_goal_velocity(self, servo_id, goal):
        
        #print(self.port)
        if self.port=="/dev/port_1":
            #print(servo_id)
            if 1==int(servo_id):
                #print("into if")
                tm1.set_goal_speed(goal)
            elif int(servo_id)==5:
                tm2.set_goal_speed(goal)
        elif self.port=="/dev/port_2":
            if int(servo_id)==1:
                tm3.set_goal_speed(goal)

    def get_feedback(self,servo_id):
        if self.port=="/dev/port_1":
            if int(servo_id)==1:
                if tm1.moving:
                    tm1.check_while_running()
                return {
                    'id':tm1.id,
                    'goal': 0,
                    'position': tm1.angle,
                    'error' : 0,
                    'speed' : tm1.speed,
                    'load' : 0,
                    'voltage' : tm1.voltage,
                    'temperature' : tm1.temperature,
                    'moving' : tm1.moving
                }
            if int(servo_id)==5:
                if tm2.moving:
                    tm2.check_while_running()
                return {
                    'id':tm2.id,
                    'goal': 0,
                    'position': tm2.angle,
                    'error' : 0,
                    'speed' : tm2.speed,
                    'load' : 0,
                    'voltage' : tm2.voltage,
                    'temperature' : tm2.temperature,
                    'moving' : tm2.moving
                }
        if self.port=="/dev/port_2":
            if int(servo_id)==1:
                if tm3.moving:
                    tm3.check_while_running()
                return {
                    'id':tm3.id,
                    'goal': 0,
                    'position': tm3.angle,
                    'error' : 0,
                    'speed' : tm3.speed,
                    'load' : 0,
                    'voltage' : tm3.voltage,
                    'temperature' : tm3.temperature,
                    'moving' : tm3.moving
                }
        return 'no such servo'