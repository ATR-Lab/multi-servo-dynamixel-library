
from modelTranslator import tm1
from modelTranslator import tm2
from modelTranslator import tm3

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
    self.motors={}
    if(port_namespace=="port_1"):
      #self.motors[0]=tm1
      self.motors[0]=1
      #self.motors[1]=tm2
      self.motors[1]=5
    if(port_namespace=="port_2"):
      self.motors[0]=1
    #print("proxy init")