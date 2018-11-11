#!/usr/local/bin/python2.7
from relay import Relay
from time import sleep
import rospy
from std_msgs.msg import Int32

relay = Relay(idVendor=0x16c0, idProduct=0x05df)
states = {"0":[0, False], "1":[0, True], "10":[1, False], "11":[1,True], "20":[2, False],
"21":[2, True]}
def callback(data):
    int_data = int(data.data)
    switch = states[str(int_data)][0]
    state = states[str(int_data)][1]
    relay.state(switch, on=state)
    print relay.state(0)

def listener():
    rospy.init_node('relay', anonymous=True)
    rospy.Subscriber("relay_state", Int32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()





