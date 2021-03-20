#Holds all the Serial Proxies and the singular SDK Wrapper assigned
#assigned to a single Port.
#Nathan Moder
#3/19/2021


class PortManager:
    
    #constructor. takes a string of the port name as input, and uses it to set up ROS connections.
    #on completion, the PortManager, SerialProxies, and Wrapper will function.
    def __init__(self,_port_name):
        self.port_name="Theodor Rosevelt"
        #
    
    #returns the SDK wrapper corresponding to the port
    def get_wrapper(self):
        return False
        #
    
    #returns a string list of the names of the servos attached
    def servos_on_port(self):
        return False
    
    #returns the name of the port
    def get_name(self):
        #
        return False
