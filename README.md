# USB Relay ROS Node

This ROS node controls a USB relay device, specifically handling left and right signals with blinking functionality. It uses the `usb_relay_device.so` library to control relay channels based on signals received from other ROS nodes.

## Features

1. **Left and Right Signal Control**:
   - Subscribes to `left` and `right` topics to control the respective relay channels.
   - If both signals are active, the "ALL" channel is activated.

2. **Blinking Functionality**:
   - Each signal blinks with a configurable duration, creating a visual indicator.

3. **Automatic Shutdown**:
   - When the node is terminated, all relay channels are turned off.

## Requirements

- ROS installed and set up.
- `usb_relay_device.so` library placed in an accessible directory. For detailed setup, refer to [USB Relay HID](https://github.com/pavel-a/usb-relay-hid/tree/master).

## Usage

1. Run the Node:
```bash
rosrun <your_package_name> usb_relay_node.py
```

2. Control Signals:
- Publish to the left and right topics with std_msgs/Bool messages to turn on/off the respective signals.
```bash
rostopic pub /left std_msgs/Bool "data: true"   # Turn left signal on
rostopic pub /right std_msgs/Bool "data: true"  # Turn right signal on
```

## Code Explanation
### Main Functionality
The `USBRelayNode` class handles device connection, listens for left and right signals, and manages relay control with blinking functionality.

### Core Methods
- `monitor_device_connection`: Continuously monitors the connection to the USB relay device.
- `toggle_relay_channel`: Turns a relay channel on or off using `usb_relay_device_open_one_relay_channel` and `usb_relay_device_close_one_relay_channel` from `usb_relay_device.so`.
- `handle_blinking`: Checks for active signals (left, right, or both) and blinks the relay channels with the set blink_duration.
- `shutdown`: Ensures all relay channels are turned off and cleans up the device handle before exit.

