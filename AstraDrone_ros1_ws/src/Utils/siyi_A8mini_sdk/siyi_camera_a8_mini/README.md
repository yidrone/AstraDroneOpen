# SIYI A8 Mini Gimbal Kamera

Code for SIYI's A8 mini Gimbal Camera. Allows yaw/pitch control, as well as Auto Center, Zoom, etc.

Connect the camera and power as stated in the a8 mini documentation. 
Connect the camera to PC or onboard computer using the ethernet cable that comes with it. The current implementation uses UDP communication.

Do the PC wired network configuration. Make sure to assign a manual IP address to your computer

For example, IP 192.168.144.10

Gateway 192.168.144.25

Netmask 255.255.255.0

And then you should be able to ping the camera:

$ ping 192.168.144.25

And see something like;

64 bytes from 192.168.144.25: icmp_seq=1 ttl=64 time=0.308 ms

64 bytes from 192.168.144.25: icmp_seq=2 ttl=64 time=0.750 ms

64 bytes from 192.168.144.25: icmp_seq=3 ttl=64 time=0.255 ms


## Build

In order to run the program, you can build it on device for prototyping.

Install the gstreamer dependencies:

$ sudo apt install libgstreamer1.0-dev libgstrtspserver-1.0-dev build-essential cmake git

$ sudo apt install ffmpeg


Get the source code, which is part of this repository, either using scp, or via a git clone.

git clone https://github.com/enginksz/siyi_camera_a8_mini.git

## Usage

Examples

$ python3 test_video.py

$ python3 test_gimbal_rotation.py
