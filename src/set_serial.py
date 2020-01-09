#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division, print_function
from __future__ import unicode_literals

from sys import stderr
from threading import Thread
from sys import version_info
from usb_relay import Relay
import time
import hid
import atexit

try:
	from typing import Dict, List
except ImportError:
	pass
try:
	input = raw_input
except NameError:
	pass


class AnsiEscapes:
	HIDE_CURSOR = '\x1b[?25l'
	SHOW_CURSOR = '\x1b[?25h'
	CYAN = "\033[0;36m"
	GREEN = "\033[0;32m"
	RED = "\033[0;31m"
	END = "\033[0m"


class Infinite(object):
	file = stderr
	check_tty = True
	hide_cursor = True
	phases = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]

	def __init__(self, message=''):
		self.index = 0
		self.ended = False
		self.message = message

		if self.file and self.is_tty():
			if self.hide_cursor:
				print(AnsiEscapes.HIDE_CURSOR, end='', file=self.file)
			atexit.register(self.finish)
			print(self.message, end='', file=self.file)
			self.file.flush()
		else:
			print(self.message)

	def update(self):
		i = self.index % len(self.phases)
		self.write(AnsiEscapes.CYAN + self.phases[i] + AnsiEscapes.END)

	def clearln(self):
		if self.file and self.is_tty():
			print('\r\x1b[K', end='', file=self.file)

	def write(self, s):
		if self.file and self.is_tty():
			line = s + " " + self.message
			print('\r' + line, end='', file=self.file)
			self.file.flush()

	def writeln(self, line):
		if self.file and self.is_tty():
			self.clearln()
			print(line, end='', file=self.file)
			self.file.flush()

	def finish(self, message=None):
		if self.ended:
			return
		self.ended = True
		if self.file and self.is_tty():
			self.writeln(message if message else "  " + self.message)
			print(file=self.file)
			if self.hide_cursor:
				print(AnsiEscapes.SHOW_CURSOR, end='', file=self.file)
			try:
				atexit.unregister(self.finish)
			except AttributeError:
				pass
		elif message:
			print(message)

	def succeed(self, message=None):
		self.finish(message=AnsiEscapes.GREEN + "✔" + AnsiEscapes.END + " " + (message if message else self.message))

	def is_tty(self):
		try:
			return self.file.isatty() if self.check_tty else True
		except AttributeError:
			return False

	def next(self, n=1):
		self.index = self.index + n
		self.update()


class FindThread(Thread):
	device = None  # type: Relay or None

	def __init__(self, vendor=0x16c0, product=0x05df):
		Thread.__init__(self)
		self.device = None
		self.vendor = vendor
		self.product = product
		self.daemon = True

	def run(self):
		devices = []
		for device in hid.enumerate(self.vendor, self.product):
			devices.append(device["path"])
		while True:
			new_devices = []
			for new_device in hid.enumerate(self.vendor, self.product):
				if new_device["path"] not in devices:
					d = hid.device()
					d.open_path(new_device["path"])
					try:
						relay_count = int(new_device['product_string'][8])
					except IndexError:
						continue
					self.device = Relay(d, relay_count)
					return
				new_devices.append(new_device["path"])
			devices = new_devices
			time.sleep(1)


def prompt(message=''):  # type: (str or unicode) -> str or bytes
	try:
		while True:
			print(message, end=": ")
			if version_info[0] == 2:
				response = bytearray(input())
			else:
				response = input().encode('utf-8')
			if len(response) > 5:
				print(AnsiEscapes.RED + "✖" + AnsiEscapes.END + " Serial numbers must be at most 5 bytes")
			else:
				return response
	except EOFError:
		raise KeyboardInterrupt()


if __name__ == '__main__':
	print("Plug in your module. If it is currently plugged in, unplug it and plug it back in.")
	try:
		a = FindThread()
		a.start()
		p = Infinite('Looking for new devices...')
		while a.is_alive():
			p.next()
			time.sleep(1 / 15)
		serial = a.device.get_serial()
		relay_count = a.device.relay_count
		p.succeed("Found a module named \"{}\" with {} relay{}".format(
			serial.decode('utf-8', 'replace'),
			relay_count,
			'' if relay_count == 1 else 's'
		))
		new_serial = prompt(message="Enter the new serial number")
		a.device.set_serial(new_serial)
		print(AnsiEscapes.GREEN + "✔" + AnsiEscapes.END + " successfully set the serial number!")
	except KeyboardInterrupt:
		pass
