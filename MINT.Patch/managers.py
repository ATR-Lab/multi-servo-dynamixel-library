#Nathan Moder

# Imports for Emulated Motors
from testing.emulation_tools import SDKSerialWrapper
from testing.emulation_tools import DynomixSerialProxy


# Imports for Real Motors
#from dynomix_driver.sdk_serial_wrapper import SDKSerialWrapper
#from dynomix_driver.dynomix_serial_proxy import DynomixSerialProxy
#import rospy


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

class ServoLog:
    def __init__(self,servo_name):
        self.output_name=servo_name
        self.info_dictionary={}

    def log_servo(self, info_dictionary):
        self.info_dictionary=info_dictionary

    def print_servo(self):
        # Order to be printed:
        # id, stateID, voltage, temperature, angle, speed
        # The dictionary has a boolean value declaring whether the motor is moving.

        #stored to a temporary value for readability
        idict=self.info_dictionary
        if idict['moving']:
            state = 'RUNNING'
        
        else:
            state = 'IDLE'
        
        # Prints the info out in one line as a formatted string.
        # Every element is seperated by a space.
        return '\"id\": {id}, \"state\": {state}, \"voltage\": {voltage}, \"temp\": {temp}, \"position\": {pos}, \"speed\": {speed}'.format(id = self.output_name, 
        state = state, voltage = idict["voltage"], temp = idict["temperature"], 
        pos = idict["position"], speed = idict["speed"])

# Uses funcitons from RobotManager and the wrapper, but does not need to import them.
class ServerManager:
    def __init__(self,_manager):
        """
        Constructor. Just needs a RobotManager called _manager, 
        which the GUITranslator will access
        """
        # The RobotManager the translator will use.
        self.manager = _manager
        
        # An initially empty list of the servo names of all running motors
        # A servo name includes both the port and the ID, formatted:
        # <portname>_<motorID>
        self.running_motors = []

        # A list of ServoLogs, indexed by servo names(formatted as above).
        # It is used to speed up gathering information, by
        # keeping the data from unchanging motors.
        self.log_list={}


    def move_motor(self, console_input, input_length):
        """
        Function member will enable torque, and then change the goal position,
        which then appends the servo to a list of running motors.
        Returns nothing.
        """
        
        #If we don't have a motor and a position, this is an error
        if input_length != 3:
            return True

        #extract the needed name from the input
        servo_name=console_input[1]
        
        #These values hold the same role as in stop_motor.
        pname = servo_name[:-4]
        sid = int(servo_name[-3:])
        
        # Trying to move a motor not attached to the system would be 
        # just as bad as stopping one.
        if self.manager.check_included(pname,sid):

            # Enable torque for motor to have it set goal position.
            self.manager.ports_by_name[pname].proxy.set_torque_enabled(sid, [1])

            # Set goal position of the motor
            # self.manager.ports_by_name[pname].proxy.set_goal_position(sid,int(console_input[2]*(1000/88)))
            self.manager.ports_by_name[pname].proxy.set_goal_position(sid,(int(console_input[2])))

            # We add the motor to a list of currently running motors.
            self.running_motors.append(servo_name)
        return True


    def update_motor(self, servo_name):
        """
        Stores the full information set for one named servo
        into the appropriate Log.
        No output or returns.
        """
        
        # The port name and motor ID, extracted from the full servo name.
        # Used to navigate the architecture to access the proxy/wrapper.
        pname = servo_name[:-4]
        mid = int(servo_name[-3:])

        # We cannot try to access a motor which isn't attached.
        if self.manager.check_included(pname, mid):
        
            # Defines an empty set which will store the information dictionary.
            idict = {}
        
            # Uses the wrapper to fill that dictionary with the needed entries.
            idict = self.manager.ports_by_name[pname].proxy.get_feedback(mid)
        
            # Since the log list is indexed by servo name, we add the information here to the Log
            self.log_list[servo_name].log_servo(idict)

        else:
            print("No such servo")
        

    def scan(self,ignore1,ignore2):
        """
        Gathers information on every motor attached to the system,
        setting up the Logs. It *must* be run in order for
        future updates to work.
        """

        # Since the RobotManager has the data structures which are needed
        # to access every servo, the data collection is handled by that class.
        # This function returns an array of dictionaries.
        all_info = self.manager.list_all_servo_info()
        
        # We loop through all the individual dictionaries, and create a servo log for each.
        # This will give us a data structure of every servo attached to the computer, and
        # prepare them to be updated only as needed.
        motor_info=''
        for lmotor in all_info:

            # Creates the Log, giving it its name and index.
            # if the Log already exists, no errors nor bugs will occur.
            # It just overwrites what's already there.
            self.log_list[lmotor['id']]=ServoLog(lmotor['id'])
            
            #The information in the dictionary is passed to the Log.
            self.log_list[lmotor['id']].log_servo(lmotor)

            #The Log prints all of its information.
            motor_info=motor_info+self.log_list[lmotor['id']].print_servo()+'\n'
        
        return motor_info


    def running_update(self, console_input=1, input_length=1):
        
        # Uses the Drivers to update *only* the motors
        # which are running, and therefore changing.
        for rmotor in self.running_motors:
            self.update_motor(rmotor)

        #Returns a string with all motor info
        motor_info='['
        for servo_log in self.log_list:
            motor_info+='{'+str(self.log_list[servo_log].print_servo())+'}, '
        motor_info=motor_info[:-2]+']'
        return motor_info
