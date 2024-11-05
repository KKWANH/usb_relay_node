import re
import sys
import hid

# Define the Vendor ID and Product ID for your USB Relay device
VENDOR_ID = 0x16c0
PRODUCT_ID = 0x05df

class Relay:
    def __init__(self, vid=VENDOR_ID, pid=PRODUCT_ID):
        try:
            self.device = hid.Device(vid=vid, pid=pid)
            print(f"Device manufacturer: {self.device.manufacturer}")
            print(f"Product: {self.device.product}")
        except IOError as ex:
            print(f"Failed to open device: {ex}")
            sys.exit(1)

    def set(self, state, relay=1):
        """
        Set the relay state.
        :param state: Boolean - True for ON, False for OFF
        :param relay: Relay ID (1 or 2 for USBRelay2)
        """
        command = [0x00, relay, int(state)]
        command = bytearray(command + [0x00] * (64 - len(command)))  # Padding to 64 bytes

        try:
            self.device.write(command)
            print(f"Relay {relay} set to {'ON' if state else 'OFF'}")
        except Exception as e:
            print(f"Error setting relay {relay}: {e}")

    def get_state(self):
        """Returns the current state of the relays."""
        try:
            self.device.write(bytearray([0x01] + [0x00] * 63))  # Example command to request state
            state = self.device.read(64)  # Adjust bytes if needed based on the relay protocol
            print(f"Relay states: {state}")
            return state
        except Exception as e:
            print(f"Error reading state: {e}")
            return None

    def close(self):
        """Close the device connection."""
        self.device.close()


class RelayController:
    def __init__(self):
        self.relays = Relay()

    def send_states(self):
        """
        Prints the current relay states to the console.
        """
        states = self.relays.get_state()
        print(f"Current relay states: {states}")

    def set_relay(self, relay_id, state):
        """
        Sets the relay state.
        :param relay_id: The ID of the relay to set.
        :param state: True for ON, False for OFF.
        """
        self.relays.set(state, relay=relay_id)
        self.send_states()

    def set_relay_compat(self, code):
        """
        Interprets a legacy relay state call.
        :param code: Integer representing relay ID and state.
        """
        code_str = str(code).zfill(2)
        relay_id = int(code_str[0])
        relay_state = code_str[1] == '1'

        self.relays.set(relay_state, relay=relay_id)
        self.send_states()


def main():
    # Initialize relay controller
    controller = RelayController()

    # Example of setting relay states
    # Relay ID 1 to ON, Relay ID 2 to OFF
    controller.set_relay(1, True)
    controller.set_relay(2, False)

    # Example of setting relay with legacy format - turning relay 0 on
    controller.set_relay_compat(10)

    # Close device after operations
    controller.relays.close()


if __name__ == '__main__':
    main()
