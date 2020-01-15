# usb_relay_wsn
A ROS node to control the states of relays

## Setup

Follow the
[Getting Started guide](src/usb_relay/README.md#getting-started) of the
usb_relay module, as well as the next step, [Changing the name of a
module][change-name].

## Usage

To start the node from the command line, run `rosrun usb_relay_wsn
ros_relay_bridge.py`. This will listen for service calls for each relay,
using each relay's name as the topic. If multiple relays have the same
name, they will be controlled together.

It is recommended to namespace this node. To do this from the command
line, run `ROS_NAMESPACE=relays rosrun usb_relay_wsn
ros_relay_bridge.py`. This will prefix each relay with `relays`, so a
relay with a name of `serno` will be controlled from `/relays/serno`.

This node will listen for service calls using the
[SetRelay](srv/SetRelay.srv) service type. It will publish relay states
with the [RelayStates](msg/RelayStates.msg) message type. Both the
service and topic use the name of the relay. **To change the name of a
relay, [follow the instructions to change the name of a
module][change-name]**.

The following examples are shown without a namespace:

### `rostopic` from the command line
```bash
# Shows the state of the relays with a name of 'serno'
rostopic echo /serno
```

### `rosservice` from the command line
Used to change the state of relays.

```bash
# Turning relay 1 on with a name of 'serno'
rosservice call /serno '[{id: 1, state: true}]'
# Turning all relays on with a name of 'serno'
rosservice call /serno '[{id: 0, state: true}]'
# Turning relay 1 on and relay 2 off with a name of 'serno'
rosservice call /serno '[{id: 1, state: true}, {id: 2, state: false}]'
```

[change-name]: src/usb_relay/README.md#changing-the-name-of-a-module