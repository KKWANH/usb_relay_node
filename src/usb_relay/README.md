# USB-Relay
This module provides utilities for controlling USB relay modules. Its
key focus is controlling multiple relay modules from the same computer.

## Features
- Compatible with both **Python 2 and 3**
- Control multiple relays from the same computer, even with the same USB
  ID
- Get & set names of relay modules
- Dynamically retrieve the number of relays in a module

## Getting started
You'll need the [hidapi] module, which may be installed with:

```bash
sudo apt-get install python-dev libusb-1.0-0-dev libudev-dev python-pip
sudo pip install hidapi
```

You should also allow all users to access the relays without `sudo`:

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="16c0", ATTR{idProduct}=="05df", MODE="777"' | sudo tee /etc/udev/rules.d/99-usbrelay.rules
sudo udevadm control --reload-rule
```

Now, either restart your computer or unplug and plug back in every
module.

### Changing the name of a module
Run the Python file [set_serial.py]. Then, plug in the module if it is
currently unplugged, or unplug it and plug it back in. Then, enter the
new name.

[set_serial.py]: set_serial.py

[hidapi]: https://github.com/trezor/cython-hidapi

## Documentation
See [examples] for usage.

Use the class [Relays] to list all relays connected to the system.
Either use its property `devices`, which is a dictionary with keys being
the name and values of arrays being [Relay]s, or use the method
`deduplicated`, which will return a dictionary with keys being the name
and values being a [Relay]. In the second case, an essentially random
relay will be picked if multiple relays with the same name are used.

A [Relay] has the following methods:

```python
def set_name(self, name):  # type: (str) -> None
    """
    Sets the name of the relay. The name must be no more than 5 bytes. It may be any 5 bytes, including unicode.

    :param name: The name number to set
    :raises ValueError: if the name number is too long
    """

def set(self, state, relay=1):  # type: (bool, int) -> None
    """
    Turns a relay on or off. If :code:`relay` is 0, it controls all relays.

    :param state: the state of the relay, on and off being True and False respectively
    :param relay: the 1-indexed relay number to turn on or off
    :raises ValueError: if the relay ID is invalid
    """

def get_name(self):  # type: () -> bytes
    """
    Gets the current name of the relay.

    :return: The name of the relay
    """

def get_states(self):  # type: () -> [bool]
    """
    Gets the current state of all relays.

    :return: A list of booleans, representing the state of each relay
    """

def get(self, relay=1):  # type: (int) -> bool
    """
    Gets the current state of a relay.

    :param relay: the 1-indexed relay number whose state will be returned
    :return: True or False being on or off respectively
    :raises ValueError: if the relay ID is invalid
    """
```

[examples]: examples
[Relays]: __init__.py#L13
[Relay]: __init__.py#L42

### Inspiration
This uses ideas from [Very-Simple-USB-Relay] and [usbrelay].

[Very-Simple-USB-Relay]: https://github.com/jaketeater/Very-Simple-USB-Relay
[usbrelay]: https://github.com/darrylb123/usbrelay