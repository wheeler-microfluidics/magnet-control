![https://ci.appveyor.com/api/projects/status/github/wheeler-microfluidics/magnet-control?branch=master&svg=true](https://ci.appveyor.com/api/projects/status/github/wheeler-microfluidics/magnet-control?branch=master&svg=true)
# magnet-control #

Control package for magnet stage with two states: engaged and disengaged.

## Overview ##

This package contains:

 - Firmware compatible with Arduino Uno or Mega2560.
 - Installable Python package for interfacing with Arduino firmware through
   serial port or i2c (through a serial-to-i2c proxy).

## Install ##

The Python package can be installed through `pip` using the following command:

    pip install wheeler.magnet-control

## Upload firmware ##

To upload the pre-compiled firmware included in the Python package, run the
following command:

    python -m magnet_control.bin.upload <board type>

replacing `<board type>` with either `uno` or `mega2560`, depending on the
model of the board.

This will attempt to upload the firmware by automatically discovering the
serial port.  On systems with multiple serial ports, use the `-p` command line
argument to specify the serial port to use.  For example:

    python -m magnet_control.bin.upload -p COM3 uno

--------------------------------------------------

## Usage ##

After uploading the firmware to the board, the `magnet_control.Proxy` class can be
used to interact with the Arduino device.

See the session log below for example usage.

### Example interactive session ###

    >>> from serial import Serial
    >>> from magnet_control import Proxy

Connect to serial device.

    >>> serial_device = Serial('/dev/ttyUSB0', baudrate=115200)

Initialize a device proxy using existing serial connection.

    >>> proxy = Proxy(serial_device)

Query the number of bytes free in device RAM.

    >>> proxy.ram_free()
    409

Query descriptive properties of device.

    >>> proxy.properties()
    base_node_software_version                               0.9.post8.dev141722557
    name                                                                  magnet_control
    manufacturer                                                        Wheeler Lab
    url                           http://github.com/wheeler-microfluidics/open-d...
    software_version                                                            0.1
    dtype: object

Use Arduino API methods interactively.

    >>> # Set pin 13 as output
    >>> proxy.pin_mode(13, 1)
    >>> # Turn led on
    >>> proxy.digital_write(13, 1)
    >>> # Turn led off
    >>> proxy.digital_write(13, 0)

Engage magnet.

    >>> proxy.magnet_engage()

Disengage magnet.
    >>> proxy.magnet_disengage()


### Configuration and state ###

The device stores a *configuration* and a *state*.  The configuration is
serialized and stored in EEPROM, allowing settings to persist across device
resets.  The state is stored in device memory and is reinitialized each time
the device starts up.

Print (non-default) configuration values.

    >>> print proxy.config
    serial_number: 2
    baud_rate: 115200
    i2c_address: 17


### Other methods ###

Below is a list of the attributes of the `magnet_control.Proxy` Python class.  Note
that many of the [Arduino API][1] functions (e.g., `pin_mode`, `digital_write`,
etc.) are exposed through the RPC API.

    >>> proxy.
    proxy.analog_read                      proxy.microseconds
    proxy.analog_write                     proxy.milliseconds
    proxy.array_length                     proxy.name
    proxy.base_node_software_version       proxy.on_config_baud_rate_changed
    proxy.begin                            proxy.on_config_i2c_address_changed
    proxy.buffer_size                      proxy.on_config_serial_number_changed
    proxy.config                           proxy.pin_mode
    proxy.delay_ms                         proxy.properties
    proxy.delay_us                         proxy.ram_free
    proxy.digital_read                     proxy.read_eeprom_block
    proxy.digital_write                    proxy.reset_config
    proxy.echo_array                       proxy.reset_state
    proxy.get_buffer                       proxy.save_config
    proxy.i2c_address                      proxy.serialize_config
    proxy.i2c_available                    proxy.serialize_state
    proxy.i2c_buffer_size                  proxy.set_i2c_address
    proxy.i2c_read                         proxy.software_version
    proxy.i2c_read_byte                    proxy.str_echo
    proxy.i2c_request                      proxy.update_config
    proxy.i2c_request_from                 proxy.update_eeprom_block
    proxy.i2c_scan                         proxy.url
    proxy.i2c_write
    proxy.load_config
    proxy.manufacturer
    proxy.max_i2c_payload_size
    proxy.max_serial_payload_size

--------------------------------------------------

## Firmware development ##

The Arduino firmware/sketch is located in the `magnet_control/Arduino/magnet_control`
directory.  The key functionality is defined in the `magnet_control::Node` class in
the file `Node.h`.

Running the following command will build the firmware using [SCons][2] for
Arduino Uno and Arduino Mega2560, and will package the resulting firmware in a
Python package, ready for distribution.

    paver sdist

### Adding new remote procedure call (RPC) methods ###

New methods may be added to the RPC API by adding new methods to the
`magnet_control::Node` class in the file `Node.h`.

# Author #

Copyright 2015 Christian Fobel <christian@fobel.net>


[1]: https://www.arduino.cc/en/Reference/HomePage
[2]: http://www.scons.org/
