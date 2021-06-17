#Moves a specified servo to a specified angle
#Nathan Moder
#4/18/2021

import zerorpc
import json,sys


def read_from_json():
    """
    Boiler-plate code for interacting with the commands sent from Javascript
    Returns a string
    """
    # jin becomes the next line in the JSON file created by the Node.
    # It needs to be translated
    jin = sys.stdin.readline()
    print(json.loads(jin))

    # loads() does the translation
    return json.loads(jin)
    #return jin

if __name__ == '__main__':
    manager=zerorpc.Client()
    manager.connect("tcp://127.0.0.1:4242")
    jsin=read_from_json()
    cinar=('move '+jsin).split()
    print(manager.move_motor(cinar,cinar.__len__()))
    manager.disconnect("tcp://127.0.0.1:4242")
