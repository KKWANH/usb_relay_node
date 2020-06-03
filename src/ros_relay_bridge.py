#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, Ben Schattinger
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function

import re
import sys

import rospy
from usb_relay_wsn.msg import RelayState, RelayStates
from usb_relay_wsn.srv import SetRelay, SetRelayRequest, SetRelayResponse

from usb_relay import Relays
from std_msgs.msg import Int32

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
    print(RelayStates(states=most_relays.get_states()))
    pub[name].publish(RelayStates(states=most_relays.get_states()))


def callback(name, data):  # type: (str, SetRelayRequest) -> SetRelayResponse
    success = True
    message = ""
    for relay in relays.devices[name]:
        for req in data.states:  # type: RelayState
            try:
                relay.set(req.state, relay=req.id)
            except ValueError:
                success = False
                message = "Invalid relay ID: {}".format(req.id)
            except Exception as e:
                print(e, file=sys.stderr)
                success = False
                message = str(e)
    try:
        send_states(name)
    except Exception as e:
        print(e, file=sys.stderr)
        success = False
        message = str(e)
    return SetRelayResponse(success=success, message=message)


def callback_compat(name, data):  # type: (str, Int32) -> None
    """
    Interprets a legacy relay state call. It is taken from the format::

        {"0":[0, False], "1":[0, True], "10":[1, False], "11":[1,True], "20":[2, False], "21":[2, True]}

    So, when left-padded with a 0, the first number signifies the relay ID and the second number signifies
    whether the relay is on (1) or off (0).
    """
    code = str(data.data).zfill(2)
    if len(code) == 2:
        relay_id = int(code[0])
        relay_state = code[1] == '1'
    else:
        print("Invalid relay code: {}".format(code), file=sys.stderr)
        return

    for relay in relays.devices[name]:
        try:
            relay.set(relay_state, relay=relay_id)
        except ValueError:
            print("Invalid relay ID: {}".format(relay_id), file=sys.stderr)
        except Exception as e:
            print(e, file=sys.stderr)
    try:
        send_states(name)
    except Exception as e:
        print(e, file=sys.stderr)


def listener():
    rospy.init_node('relay', anonymous=True)
    for key, value in relays.devices.items():
        # remove characters that aren't valid in a ROS path
        sanitized_name = re.sub(r'[^0-9a-zA-Z_]', '', key)
        if len(sanitized_name) == 0:
            print("Warning: relay with name {} doesn't have any valid characters!".format(key))
            continue
        rospy.Service(sanitized_name, SetRelay, lambda data: callback(key, data))
        rospy.Subscriber(sanitized_name + '/compat', Int32, lambda data: callback_compat(key, data))
        pub[key] = rospy.Publisher(sanitized_name, RelayStates, queue_size=10, latch=True)
        send_states(key)

    if not bool(pub):
        print("Warning: no relays connected!", file=sys.stderr)
    rospy.spin()


if __name__ == '__main__':
    listener()
