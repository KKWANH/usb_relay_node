#!/usr/bin/env python
from __future__ import print_function
from usb_relay import Relays
from time import sleep
import rospy
from std_msgs.msg import Int32

relays = Relays()
states = {"0":[0, False], "1":[0, True], "10":[1, False], "11":[1,True], "20":[2, False],
"21":[2, True]}
def callback(data):
    int_data = int(data.data)
    switch = states[str(int_data)][0]
    state = states[str(int_data)][1]
    relay.state(switch, on=state)
    print(relay.state(0))

def listener():
    rospy.init_node('relay', anonymous=True)
    for key, value in relays.devices.items():
        rospy.Service(key, Int32)
    rospy.Subscriber("relay_state", Int32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()





