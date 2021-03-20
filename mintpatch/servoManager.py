#Creates the Servo Manager, which will store information about
# which port each servo is attached to.
#It also handles the startup of MintPatch.
#Nathan Moder
#3/19/2021

from GUItranslator import GUITranslator
#might not actually need translator import

class ServoManager:

    #Constructor. PortManagers passed in _port_list must already be functional
    #TODO: implement
    def __init__(self, _port_list):
        self.servo_list={}
        self.port_names={}
        #defines empty arrays
        #for loop with the ports. matches servo names to port names

    #simple seach through the servo list to see if it exists.
    #could be used for map security
    #TODO:look into python indexing errors
    #TODO: implement
    def check_included(self, _query_servo):
        in_manager=False    
        
        return in_manager
    
    #returns the string list of every servo's name
    def list_servos(self):
        return self.servo_list

    #goes through every servo and gets info
    #uses PortManager and Wrapper
    #TODO: implement
    def list_all_servo_info(self):
        info_dict={}
        return info_dict
    
    def list_ports(self):
        return self.port_names
