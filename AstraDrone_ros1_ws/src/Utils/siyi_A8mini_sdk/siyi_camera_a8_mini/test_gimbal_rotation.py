from time import sleep
import sys
import os

current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK
from stream import SIYIRTSP

def test():

    rtsp = SIYIRTSP(rtsp_url="rtsp://192.168.144.25:8554/main.264",debug=True)
    
    cam = SIYISDK(server_ip="192.168.144.25", port=37260)
   
    if not cam.connect():
        exit(1)

    cam.setGimbalRotation(0,-90.0)

    if (cam.getRecordingState()<0):
        print("Toggle recording")
        cam.requestRecording()
        sleep(1)

    rtsp.setShowWindow(True)

    print("Recording state: ", cam.getRecordingState())

    if (cam.getRecordingState()==cam._record_msg.TF_EMPTY):
        print("TF card lsot is empty", cam._record_msg.TF_EMPTY)

    if (cam.getRecordingState()==cam._record_msg.ON):
        print("Recording is ON. Sending requesdt to stop recording")
        cam.requestRecording()
        sleep(2)
    
    cam.requestHardwareID()
    rpy = cam.getAttitude()  

    print("gimbal angles (yaw,pitch,roll) deg: ", rpy)
    print("Camera hardware ID: ", cam.getHardwareID())
    print("Recording state: ", cam.getRecordingState())
    print("Firmware version: ", cam.getFirmwareVersion())
    
    cam.disconnect()

if __name__ == "__main__":
    test()
    
