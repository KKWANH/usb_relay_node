#!/usr/bin/env python

import rospy
import ctypes
import time
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Bool

# Load the shared library
libpath = "/home/ebike/workspaces/ebike2x_ws/src/usb_relay_node/bin-Linux-x64/usb_relay_device.so"
usb_relay = ctypes.CDLL(libpath)

# Initialize the library and set up ctypes function types
usb_relay.usb_relay_init()
usb_relay.usb_relay_device_enumerate.restype = ctypes.c_void_p
usb_relay.usb_relay_device_open.restype = ctypes.c_void_p
usb_relay.usb_relay_device_open_one_relay_channel.argtypes = [ctypes.c_void_p, ctypes.c_int]
usb_relay.usb_relay_device_close_one_relay_channel.argtypes = [ctypes.c_void_p, ctypes.c_int]
usb_relay.usb_relay_device_close.argtypes = [ctypes.c_void_p]
usb_relay.usb_relay_exit.argtypes = []

frequency = rospy.get_param('/frequency', 15)

class USBRelayNode:
    def __init__(self):
        rospy.init_node('usb_relay_node')
        rospy.loginfo("USB Relay Node initialized.")

        self.service = rospy.Service('toggle_relay', SetBool, self.handle_toggle_relay)
        self.left_subscriber = rospy.Subscriber('left', Bool, self.handle_left_signal)
        self.right_subscriber = rospy.Subscriber('right', Bool, self.handle_right_signal)
        
        self.device_handle = None
        self.relay_channel = {'left': 1, 'right': 2}
        self.relay_state = {'left': False, 'right': False}
        self.blink_duration = rospy.Duration(0.5)  # Blink every 0.5 seconds
        self.last_blink_time = rospy.Time.now()
        self.is_warning_printed = False

        # Start monitoring the connection
        self.monitor_device_connection()

    def monitor_device_connection(self):
        rate = rospy.Rate(frequency)
        while not rospy.is_shutdown():
            if self.device_handle is None:
                self.device_handle = self.open_device()
                if self.device_handle:
                    rospy.loginfo("USB relay device connected.")
            if not self.is_device_connected():
                if not self.is_warning_printed:
                    rospy.logwarn("USB relay device has been disconnected. Waiting for reconnect...")
                self.device_handle = None
                self.is_warning_printed = True
            else:
                try:
                    self.is_warning_printed = False
                    self.handle_blinking()
                except Exception as e:
                    rospy.logwarn(f"Device error detected: {e}. Disconnecting and waiting for reconnection...")
                    self.close_device()  # Close and reset handle for reconnection
            rate.sleep()

    def open_device(self):
        device_list = usb_relay.usb_relay_device_enumerate()
        if device_list == 0:
            return None

        device_handle = usb_relay.usb_relay_device_open(device_list)
        return device_handle if device_handle else None

    def is_device_connected(self):
        """
        Check if the device handle is still valid.
        :return: True if the device is connected, False otherwise.
        """
        # Check connection by attempting a simple operation (e.g., get status)
        if self.device_handle:
            return usb_relay.usb_relay_device_get_status_bitmap(self.device_handle) != -1
        return False


    def close_device(self):
        """Safely close the device and reset the handle."""
        if self.device_handle:
            usb_relay.usb_relay_device_close(self.device_handle)
            self.device_handle = None
            rospy.logwarn("USB relay device handle cleared. Waiting for reconnection...")

    def handle_blinking(self):
        current_time = rospy.Time.now()
        if current_time - self.last_blink_time >= self.blink_duration:
            if self.relay_state['left'] and self.relay_state['right']:
                # Blink both channels simultaneously
                current_status = self.get_relay_status(self.relay_channel['left'])
                self.toggle_relay_channel(self.relay_channel['left'], not current_status)
                self.toggle_relay_channel(self.relay_channel['right'], not current_status)
            else:
                # Turn off both channels if both are not active
                if self.relay_state['left']:
                    self.toggle_relay_channel(self.relay_channel['left'], False)
                    self.toggle_relay_channel(self.relay_channel['right'], not self.get_relay_status(self.relay_channel['right']))
                if self.relay_state['right']:
                    self.toggle_relay_channel(self.relay_channel['left'], not self.get_relay_status(self.relay_channel['left']))
                    self.toggle_relay_channel(self.relay_channel['right'], False)

            self.last_blink_time = current_time


    def handle_left_signal(self, msg):
        self.relay_state['left'] = msg.data
        if not msg.data:
            self.toggle_relay_channel(self.relay_channel['left'], False)

    def handle_right_signal(self, msg):
        self.relay_state['right'] = msg.data
        if not msg.data:
            self.toggle_relay_channel(self.relay_channel['right'], False)
    
    def handle_toggle_relay(self, req):
        if not self.device_handle:
            return SetBoolResponse(success=False, message="USB relay device not connected.")
        return self.toggle_relay_channel(self.relay_channel['left'], req.data)

    def toggle_relay_channel(self, channel, state):
        if not self.device_handle:
            return SetBoolResponse(success=False, message="Device not connected.")

        try:
            if state:
                result = usb_relay.usb_relay_device_open_one_relay_channel(self.device_handle, channel)
                success_message = "Relay turned ON" if result == 0 else "Failed to turn ON relay"
            else:
                result = usb_relay.usb_relay_device_close_one_relay_channel(self.device_handle, channel)
                success_message = "Relay turned OFF" if result == 0 else "Failed to turn OFF relay"
            return SetBoolResponse(success=result == 0, message=success_message)
        except Exception as e:
            rospy.logerr(f"Error toggling relay channel: {e}")
            self.close_device()  # Close and reset on error
            return SetBoolResponse(success=False, message="Error toggling relay channel.")

    def get_relay_status(self, channel):
        if not self.device_handle:
            rospy.logwarn("Device not connected. Cannot retrieve relay status.")
            return False
        try:
            status = usb_relay.usb_relay_device_get_status_bitmap(self.device_handle)
            return (status & (1 << (channel - 1))) != 0
        except Exception as e:
            rospy.logerr(f"Error retrieving relay status: {e}")
            self.close_device()  # Reset connection on error
            return False

    def shutdown(self):
        if self.device_handle:
            self.toggle_relay_channel(self.relay_channel['left'], False)
            self.toggle_relay_channel(self.relay_channel['right'], False)
            usb_relay.usb_relay_device_close(self.device_handle)
            rospy.loginfo("USB relay device closed.")
        usb_relay.usb_relay_exit()
        rospy.loginfo("USB Relay library cleanup complete.")

if __name__ == '__main__':
    node = USBRelayNode()
    rospy.on_shutdown(node.shutdown)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.shutdown()
