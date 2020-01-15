#!/usr/bin/env python
"""
This will find the relay with the name "serno" and turn its first relay on for one second,
then turn it off.
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
try:
	r = relays.deduplicated()[b"serno"]
	r.set(True, 1)
	sleep(1)
	r.set(False, 1)
except KeyError:
	print("Relay with name serno not found!", file=sys.stderr)
