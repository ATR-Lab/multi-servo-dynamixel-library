import zerorpc
import json
from io import StringIO

if __name__ == '__main__':
    manager=zerorpc.Client()
    manager.connect("tcp://127.0.0.1:4242")
    while(True):
        print(manager.running_update())
    manager.disconnect("tcp://127.0.0.1:4242")