# coding=utf-8
from __future__ import print_function
import hid
from collections import defaultdict

try:
	# noinspection PyUnresolvedReferences
	from typing import Dict, List
except ImportError:
	pass


class Relays:
	"""
	Gets a list of relay modules by name. :code:`devices` is a dictionary of name to :code:`Relay`s.
	"""

	def __init__(self, vendor=0x16c0, product=0x05df):
		# type: (int, int) -> Relays
		"""
		:param vendor: the USB vendor ID of the relay modules.
		:param product: the USB product ID of the relay modules.
		"""
		self.devices = defaultdict(list)  # type: Dict[str, List[Relay]]
		for device in hid.enumerate(vendor, product):
			dev = hid.device()
			dev.open_path(device["path"])
			dev.set_nonblocking(1)
			relay_count = int(device['product_string'][8])
			name = bytes(bytearray(dev.get_feature_report(1, 5))).split(b'\x00')
			name = name[0] if len(name) > 0 else ''
			self.devices[name].append(Relay(dev, relay_count))

	def deduplicated(self):  # type: () -> Dict[str, Relay]
		devices = {}
		for key, value in self.devices.items():
			if len(value) > 0:
				devices[key] = value[0]
		return devices


class Relay:
	"""
	See https://github.com/jaketeater/Very-Simple-USB-Relay for the basic operations (turning on and off).
	https://github.com/darrylb123/usbrelay was the inspiration for names.
	"""

	def __init__(self, device, count):  # type: (hid.device, int) -> Relay
		"""
		Creates a relay. Note: this will likely not be called from user-facing code. Use Relays instead.

		:param device: the hid.device that this relay represents
		:param count: the number of relays in this module
		"""
		self.d = device
		self.relay_count = count

	def set_name(self, name):  # type: (str) -> None
		"""
		Sets the name of the relay. The name must be no more than 5 bytes. It may be any 5 bytes, including unicode.

		:param name: The name number to set
		:raises ValueError: if the name number is too long
		"""
		if not (isinstance(name, bytes) or isinstance(name, bytearray)):
			name = name.encode('utf-8')
		if len(name) > 5:
			raise ValueError("name numbers must be at most 5 characters")
		message = [0xFA]
		message += [c for c in name.ljust(5, b'\x00')]
		message.append(0x00)
		self.d.send_feature_report(message)

	def set(self, state, relay=1):  # type: (bool, int) -> None
		"""
		Turns a relay on or off. If :code:`relay` is 0, it controls all relays.

		:param state: the state of the relay, on and off being True and False respectively
		:param relay: the 1-indexed relay number to turn on or off
		:raises ValueError: if the relay ID is invalid
		"""
		if relay < 0:
			raise ValueError("Relay IDs must be at least 0.")
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

	def get_name(self):  # type: () -> bytes
		"""
		Gets the current name of the relay.

		:return: The name of the relay
		"""
		name = bytes(bytearray(self.d.get_feature_report(1, 5))).split(b'\x00')
		return name[0] if len(name) > 0 else ''

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
