#Creates the class responsible for communication with the GUI
#Nathan Moder
#3/18/2021


class GUITranslator:

    #constructor. mostly empty; translator should be created by the manager which sends itself
    #Code written!
    #TODO: Debug
    def __init__(self,_manager):
        self.manager=_manager
        self.running_motors={}

    #possible alternative to passing the manager in the constructor    
    #def connect(self,_manager):
    #    self.manager=_manager
    
    #a function for updating all running servo motors.
    #TODO: Implement
    def update_running(self):
        print(3)
        
        #necessary information is stored in the class
    
    #a function for updating the all the information on every servo motor.
    #TODO: Implement
    def update_all(self):
        print(3)
        
        #all the information needed for the function
        # is stored in the manager.
    
    def add_scan(self):
        print(3)
        #this is the first function called by the GUI.
        #it will get the full dictionary of starting info from the servers.
        #we still need to decide whether that will be a return
        #value or a consule output.
    
    #this is the function that MintPatch will be in while idle.
    #TODO: Learn about python input streams
    #TODO: Implement
    def listen(self):
        print(3)
        
        #we need to establish a communication protocol for
        #the python command line.