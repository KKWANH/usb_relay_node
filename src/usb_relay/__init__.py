# coding=utf-8
from __future__ import print_function
import hid

try:
	# noinspection PyUnresolvedReferences
	from typing import Dict
except ImportError:
	pass


class Relays:
	"""
	Gets a list of relay modules by serial number. :code:`devices` is a dictionary of serial numbers to :code:`Relay`s.
	"""
	def __init__(self, vendor=0x16c0, product=0x05df):
		# type: (int, int) -> Relays
		"""
		:param vendor: the USB vendor ID of the relay modules.
		:param product: the USB product ID of the relay modules.
		"""
		self.devices = {}  # type: Dict[str, Relay]
		for device in hid.enumerate(vendor, product):
			dev = hid.device()
			dev.open_path(device["path"])
			dev.set_nonblocking(1)
			relay_count = int(device['product_string'][8])
			serial = "".join(map(chr, dev.get_feature_report(1, 5))).strip(chr(0))
			print(serial)
			self.devices[serial] = Relay(dev, relay_count)


class Relay:
	"""
	See https://github.com/jaketeater/Very-Simple-USB-Relay for the basic operations (turning on and off).
	https://github.com/darrylb123/usbrelay was the inspiration for serial numbers.
	"""

	def __init__(self, device, count):  # type: (hid.device, int) -> Relay
		"""
		Creates a relay. Note: this will likely not be called from user-facing code. Use Relays instead.
		:param device: the hid.device that this relay represents
		:param count: the number of relays in this module
		"""
		self.d = device
		self.relay_count = count

	def set_serial(self, serial):  # type: (str) -> None
		"""
		Sets the serial number of the relay. The serial number must be no more than 5 bytes. Note that it does not have
		to be a "number", it can be any string of bytes.
		:param serial: The serial number to set
		:raises ValueError: if the serial number is too long
		"""
		if len(serial) > 5:
			raise ValueError("Serial numbers must be at most 5 characters")
		message = [0xFA]
		message += [ord(c) for c in serial]
		message.append(0x00)
		self.d.send_feature_report(message)

	def set(self, state, relay=1):  # type: (bool, int) -> None
		"""
		Turns a relay on or off.
		:param state: the state of the relay, on and off being True and False respectively
		:param relay: the 1-indexed relay number to turn on or off
		:raises ValueError: if the relay ID is invalid
		"""
		if relay < 1:
			raise ValueError("Relay IDs must be at least 1.")
		if relay > self.relay_count:
			raise ValueError("Relay IDs must be no more than {}.".format(self.relay_count))
		if relay == 0:
			if state:
				message = [0xFE]
			else:
				message = [0xFC]
		else:
			if state:
				message = [0xFF, relay]
			else:
				message = [0xFD, relay]
		self.d.send_feature_report(message)

	def get_states(self):  # type: () -> [bool]
		"""
		Gets the current state of all relays.
		:return: A list of booleans, representing the state of each relay
		"""
		# https://github.com/jaketeater/Very-Simple-USB-Relay/blob/19c43388befcc0526a3baf30228cde95417c19b5/relay.py#L30
		states = self.d.get_feature_report(1, 8)[7]
		states = [bool(int(x)) for x in list('{0:08b}'.format(states))[0:self.relay_count]]
		states.reverse()
		return states

	def get(self, relay=1):  # type: (int) -> bool
		"""
		Gets the current state of a relay.
		:param relay: the 1-indexed relay number whose state will be returned
		:return: True or False being on or off respectively
		:raises ValueError: if the relay ID is invalid
		"""
		if relay < 1:
			raise ValueError("Relay IDs must be at least 1.")
		if relay > self.relay_count:
			raise ValueError("Relay IDs must be no more than {}.".format(self.relay_count))
		return self.get_states()[relay]


def main():
	relays = Relays()
	r = relays.devices["PAINT"]
	r.set(True)
	print(r.get_states())
	r.set_serial("PAINT")


if __name__ == '__main__':
	main()
