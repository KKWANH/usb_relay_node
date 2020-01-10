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
		print("Relay with name \"{}\" has {} relays".format(key.decode('utf-8', 'replace'), relay.relay_count))
		for i in range(1, relay.relay_count+1):
			relay.set(True, i)
			sleep(0.5)
		for i in range(1, relay.relay_count+1):
			relay.set(False, i)
			sleep(0.5)
