# USB_HID_ROS_Bridge
A simple rospy subsciber to turn a conencted USB relay on and off
Based on https://github.com/jaketeater/Very-Simple-USB-Relay

Tested with ROS Indigo and Kinetic 

uses libusb libudev-dev ... (see)

how to run relay node:
did the following--but not sure if this was necessary:

sudo chown root:root ros_relay_bridge.py
sudo chmod u+s ros_relay_bridge.py

open a terminal
type:  sudo -s
then: rosrun usb_relay ros_relay_bridge.py

should see rostopic: /relay_state

can then publish to relay, e.g.:
rostopic pub /relay_state std_msgs/Int32 01

rostopic pub /relay_state std_msgs/Int32 00

codes:
00  left num is relay choice: 0=all, 1=relay1, 2= relay2
    right num: 1 is enable, 0 is disable
    
rostopic pub /relay_state std_msgs/Int32 00    turns off both relays
