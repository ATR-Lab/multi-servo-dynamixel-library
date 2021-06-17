"""
The startup code for MintPatch.
Nathan Moder
3/29/2021
"""

import os
from pynput import keyboard
import time

from dynomix_driver.sdk_serial_wrapper import SDKSerialWrapper
from dynomix_driver.dynomix_serial_proxy import DynomixSerialProxy

class PortManager:
    def __init__(self,_port_name, _setup_info):
        """
        Constructor. Takes a string of the port name as input, 
        and uses it to set up ROS connections.
        Also needs the settings that will be used to set up the proxies and wrapper.
        On completion, the PortManager, SerialProxy, and Wrapper will function.
        """
        # The port_name refers to the path to the USB port the motors are attached to
        self.port_name = _port_name

        # Initializes a ROS node for this port.
        # Necessary for the real Wrapper and Proxy to work.
        #rospy.init_node('portManager', anonymous=True)
        
        # Constructs a wrapper for the port using the settings provided
        self.wrapper = SDKSerialWrapper('/dev/{_port_name}'.format(_port_name=_port_name),
                                                                _setup_info["baudrate"])

        # Constructs a proxy for the port using the settings provided
        self.proxy = DynomixSerialProxy("/dev/{_port_name}".format(_port_name=_port_name),
        _port_name,  _setup_info["baudrate"], _setup_info["minID"], _setup_info["maxID"], 
        _setup_info["updateRate"], _setup_info["diagnosticsRate"])
        
        # Activates the proxy
        # Turns on the connection to the physical motors.
        # Gathers static information about them.
        # Stores the list of motor id's attached in proxy.motors
        self.proxy.connect()

        # Stores the list of motors in the PortManager
        # This may not be strictly necessary
        # TODO: Consider removing and calling down to the proxy for needs.
        self.servos = self.proxy.motors

    
    def is_empty(self):
        """
        Returns whether or not any servos are attached to the port.
        """
        return self.servos.__len__ == 0

class RobotManager:
    def __init__(self, _port_list, _setup_info):
        """
        Constructor. _port_list is an array of strings; 
        this constructor will set up every port manager.
        _setup_info is a dictionary of settings that are used for the ports; 
        it is indexed by port name.
        After execution, the manager and all port managers will be ready to function.

        TODO: Implement code to get rid of empty portManagers, 
        if we'll be checking every port of the appropriate type
        """
        # Defines empty arrays
        # Array of pairs of (port_name(string), servo_id(int))
        self.servo_list = []

        # A dictionary that connects the PortManager objects to strings of their names
        self.ports_by_name = {}

        # String array of the paths to the ports
        self.port_names = _port_list   

        # For every port listed in the arguments, it makes a
        # port manager. Then, it makes not of every servo attached to it.
        for portn in _port_list:
            # Enter the loop for every port name.
            temp_manager = PortManager(portn,_setup_info[portn])
            
            # We want to make sure we aren't storing empty ports.
            if not temp_manager.is_empty():

                # For this port, a Port Manager is set up.
                # Since _setup_info is indexed by port name, we simply index it
                # The same name will is used to index ports_by_name 
                self.ports_by_name[portn] = temp_manager
                                 
            else:
                # If it is empty, we have no need to go into the other loop.
                continue

            # Utilizes and looks through the list of servos in the port manager.
            for servo in self.ports_by_name[portn].servos:

                # Stores a pair of values in the servo_list.
                # We want it to seperately track the port and servo ID for later ease.
                self.servo_list.append((portn, servo))


    def check_included(self, pname, sid):
        """
        Simple seach through the servo list to see if it exists.
        Could be used for preventing errors with faulty indexes
        and tool calls.
        Returns a boolean.
        """
        # Uses itterate and keep track algorithm
        found = False

        for pair in self.servo_list:
            # As this is a pair of port name and servo ID, we check both to see
            # if we found our matching servo.
            if pair[0] == pname and int(pair[1]) == int(sid):
                found = True
       
        return found
    

    def list_servos(self):
        """
        Returns the string list of every servo's name
        """
        # Initialize names list
        names = []

        # Check our servo list
        for pair in self.servo_list:
            psid = pair[1]
            sid = ''

            # If ID is 0-9, concatonate two zeros
            if psid < 10:
                sid = '00'+str(psid)

            # If ID is 10-99, concatonate one zero
            elif psid < 100:
                sid = '0'+str(psid)

            # Just concatonate the ID if 100-256
            else:
                sid = str(psid)

            # Append the string ID to the names list
            # e.g., ttyUSB0_014
            names.append('{pname}_{sid}'.format(pname = pair[0], sid = sid))

        return names


    def list_all_servo_info(self):
        """
        goes through every servo and gets info
        uses PortManager and Wrapper
        returns an array of dictionaries
        """
        # Defines the empty array which will hold the dictionaries
        info_dict = []
        
        # Loops through every Port Manager using their name,
        # since that is how they are stored
        for portn in self.port_names:
            port = self.ports_by_name[portn]

            # This loop goes through the servos on the port. 
            # ids is an integer number that corresponds
            # to the value in a register on the motor.
            for ids in port.servos:
                # Accesses the wrapper held by the port manager.
                # From it, it uses get_feedback to obtain a comprehensive
                # list of dynamic information from the servo.
                temp_dict = port.proxy.get_feedback(ids)
                sid = ''

                if ids < 10:
                    sid = '00'+str(ids)

                elif ids < 100:
                    sid = '0'+str(ids)

                else:
                    sid = str(ids)

                # The ID returned only describes the motor, but we need to
                # provide the port name for it to be useful to outside systems.
                # We use the format function to create our new Servo Name:
                # <port_name>_<motor_id>
                # For example, for motor 5 on the port "USB0", we would have:
                # ttyUSB0_005
                # TODO: Must currently be adapted if we want double-digit motor ID's
                temp_dict['id'] = '{portn}_{id}'.format(portn = portn, id = sid)

                # We add the obtained and edited dictionary to the list we will return.
                info_dict.append(temp_dict)

        return info_dict


    def list_ports(self):
        """
        Returns an array of the names of every port.
        """
        return self.port_names

def main():
    #"""
    # Setttings for configuring our pinging to the motors.
    settings_set = {}

    # Get the port name through a system call
    raw_port_command = os.popen("ls /dev/ttyUSB*")
    raw_ports = raw_port_command.read()
    port_list_raw = raw_ports.split()
    port_list = []

    # Real motors, and matches the launch file simple_l_arm_controller_manager.launch
    # port_list = {port}
    settings = {
        'baudrate': 1000000,
        'minID': 1,
        'maxID': 40,
        'updateRate' : 20,
        'diagnosticsRate' : 1
    }

    # Get Port List without /dev/
    for raw_port in port_list_raw:
        port = raw_port.replace('/dev/', '')
        settings_set[port] = settings
        port_list.append(port)

    manager = RobotManager(port_list, settings_set)

    puppet_arm = {"1": 0, "2": 0, "3": 0, "4": 0, "5": 0, "11": 0, "12": 0, "13": 0, "14": 0, "15": 0}
    puppet_arm_temp = {"1": 0, "2": 0, "3": 0, "4": 0, "5": 0, "11": 0, "12": 0, "13": 0, "14": 0, "15": 0}
    puppet_arm_moving = {"1": 0, "2": 0, "3": 0, "4": 0, "5": 0, "11": 0, "12": 0, "13": 0, "14": 0, "15": 0}
    master_arm = {"21": 0, "22": 0, "23": 0, "24": 0, "25": 0}

    manager.ports_by_name["ttyUSB0"].proxy.set_torque_enabled(1, [1])
    manager.ports_by_name["ttyUSB0"].proxy.set_torque_enabled(2, [1])
    manager.ports_by_name["ttyUSB0"].proxy.set_torque_enabled(3, [1])
    manager.ports_by_name["ttyUSB0"].proxy.set_torque_enabled(4, [1])
    manager.ports_by_name["ttyUSB0"].proxy.set_torque_enabled(5, [1])
    manager.ports_by_name["ttyUSB0"].proxy.set_torque_enabled(11, [1])
    manager.ports_by_name["ttyUSB0"].proxy.set_torque_enabled(12, [1])
    manager.ports_by_name["ttyUSB0"].proxy.set_torque_enabled(13, [1])
    manager.ports_by_name["ttyUSB0"].proxy.set_torque_enabled(14, [1])
    manager.ports_by_name["ttyUSB0"].proxy.set_torque_enabled(15, [1])

    for key_arm in puppet_arm:
        idict_tmp = manager.ports_by_name["ttyUSB0"].proxy.get_feedback(int(key_arm))
        puppet_arm[key_arm] = idict_tmp["position"]
        puppet_arm_temp[key_arm] = idict_tmp["temperature"]
        puppet_arm_moving[key_arm] = idict_tmp["moving"]
        manager.ports_by_name["ttyUSB0"].proxy.set_speed(int(key_arm), 40)

    for key_arm in master_arm:
        idict_tmp = manager.ports_by_name["ttyUSB1"].proxy.get_feedback(int(key_arm))
        master_arm[key_arm] = idict_tmp["position"]

    timestamp = time.time()

    while True:
        
        if (time.time() - timestamp < 0.3):
            print("DEBUG TIMESTAMP: %s" % str(timestamp))
            timestamp = time.time()
            for idx_id in master_arm:
                manager.ports_by_name["ttyUSB0"].proxy.set_goal_position((int(idx_id) - 10), master_arm[idx_id])
                manager.ports_by_name["ttyUSB0"].proxy.set_goal_position((int(idx_id) - 20), (master_arm[idx_id]))

            for key_arm in puppet_arm:
                idict_tmp = manager.ports_by_name["ttyUSB0"].proxy.get_feedback(int(key_arm))
                puppet_arm[key_arm] = idict_tmp["position"]
                puppet_arm_temp[key_arm] = idict_tmp["temperature"]
                puppet_arm_moving[key_arm] = idict_tmp["moving"]

            for key_arm in master_arm:
                idict_tmp = manager.ports_by_name["ttyUSB1"].proxy.get_feedback(int(key_arm))
                master_arm[key_arm] = idict_tmp["position"]

            print("== DEBUG CONSOLE ==")
            print("== PUPPET ARMS ==")
            for key_arm in puppet_arm:
                print("= ARM ID %s: -- POSITION: %s -- TEMP: %s -- MOVING: %s" % (key_arm, str(puppet_arm[key_arm]), str(puppet_arm_temp[key_arm]), str(puppet_arm_moving[key_arm])))

            print("== MASTER ARM ==")
            for key_arm in master_arm:
                print("= ARM ID %s: %s" % (key_arm, str(master_arm[key_arm])))
        timestamp = time.time()


        
        


if __name__ == '__main__':
    main()