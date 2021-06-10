#! /usr/bin/python3

__author__ = "Irvin Steve Cardenas"
__email__ = "irvin@irvincardenas.com"

import sys
import os
from optparse import OptionParser

import rospy
from dynamixel_manipulation.srv import StartController
from dynamixel_manipulation.srv import StartControllerResponse 
from dynamixel_manipulation.srv import StopController
from dynamixel_manipulation.srv import StopControllerResponse 
from dynamixel_manipulation.srv import RestartController
from dynamixel_manipulation.srv import RestartControllerResponse 


parser = OptionParser()

def manage_controller(controller_name, port_namespace, controller_type, command, deps, start, stop, restart):
  try:
    controller = rospy.get_param(controller_name + '/controller')
    package_path = controller['package']
    module_name = controller['module']
    class_name = controller['type']
    rospy.loginfo("Starting controllers - name: %s package_path: %s module_name: %s  class_name: %s", controller, package_path, module_name, class_name)
  except KeyError as ke:
    rospy.logerr('[%s] configuration error: could not find controller parameters on parameter server' % controller_name)
    sys.exit(1)
  except Exception as e:
    rospy.logerr('[%s]: %s' % (controller_name, e))
    sys.exit(1)
  
  # try:
  #   rospy.loginfo('Attempting to start the controller...')
  #   response = start(port_namespace, package_path, module_name, class_name, controller_name, deps)
  #   if response.success: rospy.loginfo("SUCCESS::: %s", response.reason)
  #   else: rospy.logerr(response.reason)
  # except rospy.ServiceException, e:
  #   rospy.logerr('Service call failed: %s' % e)

  if command.lower() == 'start':
    try:
        rospy.loginfo('Attempting to start the controller...')
        response = start(port_namespace, package_path, module_name, class_name, controller_name, deps)
        if response.success: rospy.loginfo(response.reason)
        else: rospy.logerr(response.reason)
    except rospy.ServiceException:
        rospy.logerr('Service call failed: ')
  elif command.lower() == 'stop':
    try:
        response = stop(controller_name)
        if response.success: rospy.loginfo(response.reason)
        else: rospy.logerr(response.reason)
    except rospy.ServiceException:
        rospy.logerr('Service call failed: ')
  elif command.lower() == 'restart':
    try:
        response = restart(port_namespace, package_path, module_name, class_name, controller_name, deps)
        if response.success: rospy.loginfo(response.reason)
        else: rospy.logerr(response.reason)
    except rospy.ServiceException:
        rospy.logerr('Service call failed: ')
  else:
      rospy.logerr('Invalid command.')
  parser.print_help()

if __name__ == '__main__':
  try:
    rospy.init_node('dynomix_spawner', anonymous=True)

    parser.add_option('-m', '--manager', metavar='MANAGER', 
      help='specified serial port is managed by MANAGER')
    parser.add_option('-p', '--port', metavar='PORT',
      help='motors of specified controllers are connected to PORT')
    parser.add_option('-t', '--type', metavar='TYPE', default='simple', choices=('simple', 'meta'),
      help='type of controller to be loaded (simple|meta) [default: %default]')
    parser.add_option('-c', '--command', metavar='COMMAND', 
      default='start', choices=('start', 'stop', 'restart'),
      help='command to perform on specified controllers: start, stop, restart [default: %default]')

    (options, args) = parser.parse_args(rospy.myargv()[1:])

    if len(args) < 1:
      parser.error('Specify at least one controller name')
    
    manager_namespace = options.manager
    port_namespace = options.port
    controller_type = options.type
    command = options.command 
    joint_controllers = args

    if controller_type == 'meta': port_namespace = 'meta'

    print("Declaring service.... controller_type: %s. port_namespace: %s", controller_type, port_namespace)
    start_service_name = '%s/%s/start_controller' % (manager_namespace, port_namespace)
    stop_service_name = '%s/%s/stop_controller' % (manager_namespace, port_namespace)
    restart_service_name = '%s/%s/restart_controller' % (manager_namespace, port_namespace)
        
    parent_namespace = 'global' if rospy.get_namespace() == '/' else rospy.get_namespace()
    
    print("Waiting for service..." + start_service_name)
    rospy.wait_for_service(start_service_name)
    rospy.wait_for_service(stop_service_name)
    rospy.wait_for_service(restart_service_name)
    
    start_controller = rospy.ServiceProxy(start_service_name, StartController)
    stop_controller = rospy.ServiceProxy(stop_service_name, StopController)
    restart_controller = rospy.ServiceProxy(restart_service_name, RestartController)

    rospy.loginfo('%s controller_spawner: All services are up, spawning controllers...' % port_namespace)

    if controller_type == 'simple':
      for controller_name in joint_controllers:
        manage_controller(controller_name, port_namespace, controller_type, command, [], start_controller, stop_controller, restart_controller)
    elif controller_type == 'meta':
      controller_name = joint_controllers[0]
      dependencies = joint_controllers[1:]
      rospy.loginfo("DEPENDENCIES of METS:  %d", len(dependencies))
      manage_controller(controller_name, port_namespace, controller_type, command, dependencies, start_controller, stop_controller, restart_controller)

  except rospy.ROSInterruptException: pass