#A functional, standalone testing environment for use in intial GUI testing.
#Nathan Moder
#3/18/2021


class ModelGUITranslator:
    def __init__(self,_manager):
        #mostly empty; translator should be created by the manager which sends itself
        #self.manager=_manager
        self.running_motors={}

    #possible alternative to passing the manager in the constructor
    
    #def connect(self,_manager):
    #    self.manager=_manager
    
    def update_running(self):
        print(3)
        #a function for updating all running servo motors.
        #necessary information is stored in the class
    
    def update_all(self):
        print(3)
        #a function for updating the information on every servo motor.
        #all the information needed for the function
        # is stored in the manager.
    
    def add_scan(self):
        print(3)
        #this is the first function called by the GUI.
        #it will get the full dictionary of starting info from the servers.
        #we still need to decide whether that will be a return
        #value or a consule output.
    
    def listen(self):
        print(3)
        #this is the function that MintPatch will be in while idle.
        #we need to establish a communication protocol for
        #the python command line.