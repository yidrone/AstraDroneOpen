import sys
import os
from time import sleep
  
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
  
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK

def test():
    
    cam = SIYISDK(server_ip="192.168.1.25", port=37260)

    if not cam.connect():
        print("No connection ")
        exit(1)

    val = cam.requestZoomIn()
    sleep(1)
    val = cam.requestZoomHold()
    sleep(1)
    print("Zoom level: ", cam.getZoomLevel())

    val = cam.requestZoomOut()
    sleep(1)
    val = cam.requestZoomHold()
    sleep(1)
    print("Zoom level: ", cam.getZoomLevel())

    cam.disconnect()

if __name__ == "__main__":
    test()
