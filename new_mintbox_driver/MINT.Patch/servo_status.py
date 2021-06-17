#Prints all the motors
#Nathan Moder
#4/18/2021

import zerorpc

if __name__ == '__main__':
    manager=zerorpc.Client()
    manager.connect("tcp://127.0.0.1:4242")
    print(manager.running_update())
    manager.disconnect("tcp://127.0.0.1:4242")
