#!/usr/bin/env python
from __future__ import print_function

import re
import rospy
from usb_relay_wsn.msg import RelayState, RelayStates
from usb_relay_wsn.srv import SetRelay, SetRelayRequest, SetRelayResponse

from usb_relay import Relays

try:
    # noinspection PyUnresolvedReferences
    from typing import Dict
except ImportError:
    pass

relays = Relays()

pub = {}  # type: Dict[str, rospy.Publisher]


def send_states(name):  # type: (str) -> None
    """
    Sends the current relay states to the appropriate topic.

    :param name: The name of the relay to send.
    """
    # The module with the most amount of relays
    most_relays = max(relays.devices[name], key=lambda relay: relay.relay_count)
    out = map(lambda x: RelayState(id=x[0] + 1, state=x[1]), enumerate(most_relays.get_states()))
    pub[name].publish(RelayStates(states=out))


def callback(name, data):  # type: (str, SetRelayRequest) -> SetRelayResponse
    success = True
    message = ""
    for relay in relays.devices[name]:
        for req in data.states:  # type: RelayState
            print(req)
            try:
                print(req.id)
                relay.set(req.state, relay=req.id)
            except ValueError:
                success = False
                message = "Invalid relay ID: {}".format(req.id)
            except Exception:
                success = False
                message = "An error occurred"
    try:
        send_states(name)
    except Exception:
        success = False
        message = "An error occurred"
    return SetRelayResponse(success=success, message=message)


def listener():
    rospy.init_node('relay', anonymous=True)
    for key, value in relays.devices.items():
        # remove characters that aren't valid in a ROS path
        sanitized_name = re.sub(r'[^0-9a-zA-Z_]', '', key)
        if len(sanitized_name) == 0:
            print("Warning: relay with name {} doesn't have any valid characters!".format(key))
            continue
        rospy.Service(sanitized_name, SetRelay, lambda data: callback(key, data))
        pub[key] = rospy.Publisher(sanitized_name, RelayStates, queue_size=10, latch=True)
        send_states(key)
    rospy.spin()


if __name__ == '__main__':
    listener()
