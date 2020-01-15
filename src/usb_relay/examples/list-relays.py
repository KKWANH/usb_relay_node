#!/usr/bin/env python
"""
This will turn on each relay of every module plugged in and then turn it off again.
"""
from __future__ import print_function
from time import sleep

# You shouldn't need this next block

if __name__ == '__main__':
	if __package__ is None:
		import sys
		from os import path

		sys.path.append(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))

from usb_relay import Relays

relays = Relays()

for key, value in relays.devices.items():
	for relay in value:
		print("Relay with name \"{}\" has {} relays".format(key, relay.relay_count))
