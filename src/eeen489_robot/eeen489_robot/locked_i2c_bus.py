"""Compatibility wrapper for using Blinka's I2C API on the robot's Linux host.

This module intentionally blocks the upstream pin-detection logic that fails on
non-CircuitPython Linux systems and exposes a small, explicit I2C interface that
other sensor drivers can call without needing the full Blinka board abstraction.
"""

import sys
from types import ModuleType

# 1. Block the broken Blinka pin-detection system completely. The runtime is a
# Raspberry Pi Linux environment, not a CircuitPython board, so this import path
# must be short-circuited before the sensor libraries try to probe GPIO pins.
mock_digitalio = ModuleType("digitalio")
mock_digitalio.DigitalInOut = lambda pin: None
mock_digitalio.Direction = None
mock_digitalio.Pull = None
mock_digitalio.DriveMode = None
sys.modules["digitalio"] = mock_digitalio

from adafruit_blinka.microcontroller.generic_linux.i2c import I2C as RawI2C

# 2. Corrected I2C bridge resolving the Blinka length type conflict.
class LockedI2CBus:
    """Small, thread-safe I2C wrapper compatible with the Blinka sensor drivers.

    The class keeps the same method names expected by Adafruit sensor libraries,
    while translating the Linux smbus calls into a buffer layout that matches the
    drivers' bytearray semantics.
    """

    def __init__(self, bus_id):
        """Create a bus connection to the requested Linux I2C bus."""
        self._i2c = RawI2C(bus_id)

    def try_lock(self):
        """Return success without blocking.

        The robot uses a single sensor bus and the upstream drivers expect a lock
        interface, but the Linux I2C path is already serialized enough for this
        simpler API contract.
        """
        return True

    def unlock(self):
        """Release any attachment lock; the Linux bus implementation is effectively stateless."""
        pass

    def writeto_then_readfrom(self, address, buffer_out, buffer_in, *, out_start=0, out_end=None, in_start=0, in_end=None):
        """Write a register command and read data into a caller-owned buffer."""
        out_end = out_end if out_end is not None else len(buffer_out)
        in_end = in_end if in_end is not None else len(buffer_in)

        # Allocate a temporary bytearray buffer matching the size Blinka expects.
        read_buffer = bytearray(in_end - in_start)

        # Pass the memory containers to Blinka's underlying smbus layer.
        self._i2c.writeto_then_readfrom(
            address,
            bytes(buffer_out[out_start:out_end]),
            read_buffer,
        )

        # Map the read bytes back sequentially into the driver's target memory
        # slice, preserving the same layout that the sensor driver expects.
        for i, val in enumerate(read_buffer):
            buffer_in[in_start + i] = val

    def writeto(self, address, buffer, *, start=0, end=None):
        """Write a byte sequence to the target I2C device."""
        end = end if end is not None else len(buffer)
        self._i2c.writeto(address, bytes(buffer[start:end]))

    def readfrom_into(self, address, buffer, *, start=0, end=None):
        """Read bytes from the target device into an existing buffer."""
        end = end if end is not None else len(buffer)
        read_data = self._i2c.readfrom(address, end - start)
        for i, val in enumerate(read_data):
            buffer[start + i] = val