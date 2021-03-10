#!/usr/bin/env python

from threading import Thread, Lock
import sys
import rospy

from dynamixel_manipulation.srv import StartController
from dynamixel_manipulation.srv import StartControllerResponse 
from dynamixel_manipulation.srv import StopController
from dynamixel_manipulation.srv import StopControllerResponse 
from dynamixel_manipulation.srv import RestartController
from dynamixel_manipulation.srv import RestartControllerResponse 

from joint_position_controller import JointPositionController
from joint_trajectory_action_controller import JointTrajectoryActionController
from dynomix_driver.dynomix_serial_proxy import DynomixSerialProxy

class DynomixControllerManager:
  """
  Takes care of managing the controllers 
  This includes serial connection establishment
  """
  def __init__(self):
    rospy.init_node('dynomix_controller_manager', anonymous=True)
    # rospy.on_shutdown(self.on_shutdown)

    self.waiting_meta_controllers = []
    # self.controllers = {
    #   "l_shoulder_pitch_controller":{
    #     "controller_name": "l_shoulder_pitch_controller",
    #     "port_name": "/dev/ttyUSB0",
    #     "joint_name": "l_shoulder_pitch_joint",
    #     "joint_speed": 1.17,
    #     "port_namespace": "l_arm_port"
    #   },
    #   "l_shoulder_pan_controller": {
    #     "controller_name": "l_shoulder_pan_controller",
    #     "port_name": "/dev/ttyUSB0",
    #     "joint_name": "l_shoulder_pan_joint",
    #     "joint_speed": 1.17,
    #     "port_namespace": "l_arm_port"
    #   }
    # }

    self.controllers = {}
    self.serial_proxies = {}
    self.diagnostics_rate = rospy.get_param('~diagnostics_rate', 1)

    self.start_controller_lock = Lock()
    self.stop_controller_lock = Lock()

    manager_namespace = rospy.get_param('~namespace')
    serial_ports = rospy.get_param('~serial_ports')

    for port_namespace, port_config in serial_ports.items():
      port_name = port_config['port_name']
      baud_rate = port_config['baud_rate']
      readback_echo = port_config['readback_echo'] if 'readback_echo' in port_config else False
      protocol_version = port_config['protocol_version'] if 'protocol_version' in port_config else 2.0
      min_motor_id = port_config['min_motor_id'] if 'min_motor_id' in port_config else 0
      max_motor_id = port_config['max_motor_id'] if 'max_motor_id' in port_config else 253
      update_rate = port_config['update_rate'] if 'update_rate' in port_config else 5
      error_level_temp = 75
      warn_level_temp = 70

      if 'diagnostics' in port_config:
        if 'error_level_temp' in port_config['diagnostics']:
          error_level_temp = port_config['diagnostics']['error_level_temp']
        if 'warn_level_temp' in port_config['diagnostics']:
          warn_level_temp = port_config['diagnostics']['warn_level_temp']


      serial_proxy = DynomixSerialProxy(port_name,
                          port_namespace,
                          baud_rate,
                          min_motor_id,
                          max_motor_id,
                          update_rate,
                          self.diagnostics_rate,
                          error_level_temp,
                          warn_level_temp,
                          readback_echo)

      serial_proxy.connect()

      # self.serial_proxies[port_namespace] = serial_proxy

      # serial_proxy = DynomixSerialProxy("/dev/ttyUSB0",
      #                     "l_arm_port",
      #                     "1000000",
      #                     1,
      #                     20,
      #                     5,
      #                     5,
      #                     75,
      #                     70,
      #                     0)
      # serial_proxy.connect()


      # will create a set of services for each serial port under common manager namesapce
      # e.g. /dynamixel_manager/robot_arm_port/start_controller
      #      /dynamixel_manager/robot_head_port/start_controller
      # where 'dynamixel_manager' is manager's namespace
      #       'robot_arm_port' and 'robot_head_port' are human readable names for serial ports
      rospy.Service('%s/%s/start_controller' % (manager_namespace, port_namespace), StartController, self.start_controller)
      rospy.Service('%s/%s/stop_controller' % (manager_namespace, port_namespace), StopController, self.stop_controller)
      rospy.Service('%s/%s/restart_controller' % (manager_namespace, port_namespace), RestartController, self.restart_controller)

      self.serial_proxies[port_namespace] = serial_proxy

    # services for 'meta' controllers, e.g. joint trajectory controller
    # these controllers don't have their own serial port, instead they rely
    # on regular controllers for serial connection. The advantage of meta
    # controller is that it can pack commands for multiple motors on multiple
    # serial ports.
    # NOTE: all serial ports that meta controller needs should be managed by
    # the same controler manager.
    rospy.Service('%s/meta/start_controller' % manager_namespace, StartController, self.start_controller)
    rospy.Service('%s/meta/stop_controller' % manager_namespace, StopController, self.stop_controller)
    rospy.Service('%s/meta/restart_controller' % manager_namespace, RestartController, self.restart_controller)


  def on_shutdown(self):
    for serial_proxy in self.serial_proxies.value():
      serial_proxy.disconnect()

  def check_deps(self):
    controllers_still_waiting = []

    for i,(controller_name,deps,kls) in enumerate(self.waiting_meta_controllers):
      if not set(deps).issubset(self.controllers.keys()):
        controllers_still_waiting.append(self.waiting_meta_controllers[i])
        rospy.logwarn('[%s] not all dependencies started, still waiting for %s...' % (controller_name, str(list(set(deps).difference(self.controllers.keys())))))
      else:
        dependencies = [self.controllers[dep_name] for dep_name in deps]
        controller = kls(controller_name, dependencies)
        rospy.loginfo("DEPENDENCIES SIZE::: %d", len(dependencies))
        
        if controller.initialize():
          controller.start()
          self.controllers[controller_name] = controller
                
    self.waiting_meta_controllers = controllers_still_waiting[:]

  def start_controller(self, req):
    """
    For now, this launches all controller manually.
    Future implementation uses services and launch files
    """
    port_name = req.port_name
    package_path = req.package_path
    module_name = req.module_name
    class_name = req.class_name
    controller_name = req.controller_name

    rospy.loginfo('Controller=====')
    self.start_controller_lock.acquire()

    if controller_name in self.controllers:
      rospy.loginfo('Controller===== %s in controllers dictionary', controller_name)
      self.start_controller_lock.release()
      return StartControllerResponse(False, 'Controller [%s] already started. If you want to restart it, call restart.' % controller_name)

    try:
      if module_name not in sys.modules:
        # import if module not previously imported
        package_module = __import__(package_path, globals(), locals(), [module_name], -1)
      else:
        # reload module if previously imported'
        # print(sys.modules)
        print("::: package_path::: " + package_path + " ::: module_name::: " + module_name)
        # package_module = reload(sys.modules[package_path]) # IRVIN DELETING
        #TODO: Fix this below
        # controller_module = getattr(package_module, module_name)
      controller_module = reload(sys.modules[module_name])
    except ImportError, ie:
      self.start_controller_lock.release()
      return StartControllerResponse(False, 'Cannot find controller module. Unable to start controller %s\n%s' % (module_name, str(ie)))
    except SyntaxError, se:
      self.start_controller_lock.release()
      return StartControllerResponse(False, 'Syntax error in controller module. Unable to start controller %s\n%s' % (module_name, str(se)))
    except Exception, e:
        self.start_controller_lock.release()
        return StartControllerResponse(False, 'Unknown error has occured. Unable to start controller %s\n%s' % (module_name, str(e)))
    
    kls = getattr(controller_module, class_name)

    # kls = None
    # if module_name == 'joint_position_controller':
    #   kls = JointPositionController

    # elif module_name == 'joint_trajectory_action_controller':
    #   kls = JointTrajectoryActionController
    print('PORT_NAME:::: %s', port_name)
    
    if port_name == 'meta':
      print("::::::::::::: Meta: %s", module_name)
      self.waiting_meta_controllers.append((controller_name,req.dependencies,kls))
      self.check_deps()
      self.start_controller_lock.release()
      return StartControllerResponse(True, '')
        
    if port_name != 'meta' and (port_name not in self.serial_proxies):
      self.start_controller_lock.release()
      print("::::::::::::::Not Meta: %s", port_name)
      return StartControllerResponse(False, 'Specified port [%s] not found, available ports are %s. Unable to start controller %s' % (port_name, str(self.serial_proxies.keys()), controller_name))


    # for controller_info in self.controllers:
    #   joint_controller = JointPositionController(
    #     serial_proxies[controller_info['port_name']].port_handler, 
    #     serial_proxies[controller_info['port_name']].packet_handler, 
    #     "irvins_namespace")
    #   joint_controller.start()

    # controller = None

    # if module_name == 'joint_position_controller':
    #   controller = JointPositionController(
    #     self.serial_proxies["l_arm_port"].port_handler, 
    #     self.serial_proxies["l_arm_port"].packet_handler, 
    #     self.controllers['l_shoulder_pitch_controller']['controller_name'], 
    #     "l_arm_port")

    # elif module_name == 'joint_trajectory_action_controller':
    #   controller = JointTrajectoryActionController(
    #      "l_arm_port", 
    #     self.controllers)
    
    # controller.start()

    # controller = kls(
    #   self.serial_proxies[port_name].port_handler,
    #   self.serial_proxies[port_name].packet_handler,
    #   controller_name, 
    #   port_name)

    # self.start_controller_lock.release()
           
    controller = kls(self.serial_proxies[port_name].port_handler, self.serial_proxies[port_name].packet_handler, controller_name, port_name)
    
    if controller.initialize():
      controller.start()
      self.controllers[controller_name] = controller
      
      self.check_deps()
      self.start_controller_lock.release()
        
      return StartControllerResponse(True, 'Controller %s successfully started.' % controller_name)
    else:
      self.start_controller_lock.release()
      return StartControllerResponse(False, 'Initialization failed. Unable to start controller %s' % controller_name)


    # return StartControllerResponse(True, 'Controller %s successfully started.' % controller_name)

    # if controller.initialize():
    #   controller.start()
    #   self.controllers[controller_name] = controller
      
    #   self.check_deps()
    #   self.start_controller_lock.release()
      
    #   return StartControllerResponse(True, 'Controller %s successfully started.' % controller_name)
    # else:
    #   self.start_controller_lock.release()
    #   return StartControllerResponse(False, 'Initialization failed. Unable to start controller %s' % controller_name)


  def stop_controller(self, req):
    controller_name = req.controller_name
    self.stop_controller_lock.acquire()
    
    if controller_name in self.controllers:
      self.controllers[controller_name].stop()
      del self.controllers[controller_name]
      self.stop_controller_lock.release()
      return StopControllerResponse(True, 'controller %s successfully stopped.' % controller_name)
    else:
      self.self.stop_controller_lock.release()
      return StopControllerResponse(False, 'controller %s was not running.' % controller_name)

  def restart_controller(self, req):
    response1 = self.stop_controller(StopController(req.controller_name))
    response2 = self.start_controller(req)
    return RestartControllerResponse(response1.success and response2.success, '%s\n%s' % (response1.reason, response2.reason))

if __name__ == '__main__':
  try:
    manager = DynomixControllerManager()
    rospy.spin()
  except rospy.ROSInterruptException: pass