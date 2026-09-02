import sys
from types import ModuleType

# 1. Block the broken Blinka pin-detection system completely
mock_digitalio = ModuleType("digitalio")
mock_digitalio.DigitalInOut = lambda pin: None
mock_digitalio.Direction = None
mock_digitalio.Pull = None
mock_digitalio.DriveMode = None
sys.modules["digitalio"] = mock_digitalio

from adafruit_blinka.microcontroller.generic_linux.i2c import I2C as RawI2C
import adafruit_lsm6ds.lsm6dsox

# 2. Corrected I2C bridge resolving the Blinka length type conflict
class LockedI2CBus:
    def __init__(self, bus_id):
        self._i2c = RawI2C(bus_id)
    
    def try_lock(self):
        return True
        
    def unlock(self):
        pass
        
    def writeto_then_readfrom(self, address, buffer_out, buffer_in, *, out_start=0, out_end=None, in_start=0, in_end=None):
        out_end = out_end if out_end is not None else len(buffer_out)
        in_end = in_end if in_end is not None else len(buffer_in)
        
        # Allocate a temporary bytearray buffer matching the size Blinka expects
        read_buffer = bytearray(in_end - in_start)
        
        # Pass the memory containers to Blinka's underlying smbus layer
        self._i2c.writeto_then_readfrom(
            address, 
            bytes(buffer_out[out_start:out_end]), 
            read_buffer
        )
        
        # Map the read bytes back sequentially into the driver's target memory slice
        for i, val in enumerate(read_buffer):
            buffer_in[in_start + i] = val

    def writeto(self, address, buffer, *, start=0, end=None):
        end = end if end is not None else len(buffer)
        self._i2c.writeto(address, bytes(buffer[start:end]))

    def readfrom_into(self, address, buffer, *, start=0, end=None):
        end = end if end is not None else len(buffer)
        read_data = self._i2c.readfrom(address, end - start)
        for i, val in enumerate(read_data):
            buffer[start + i] = val

import math

def pitch(ax, ay, az):
    return math.atan2(-ax, math.sqrt(ay * ay + az * az))

def relative_pitch(ref_accel, moving_accel):
    ref_pitch = pitch(*ref_accel)
    moving_pitch = pitch(*moving_accel)

    angle = moving_pitch - ref_pitch

    # wrap to -pi ... +pi
    angle = math.atan2(math.sin(angle), math.cos(angle))

    return angle

# 3. Spin up the bus on /dev/i2c-1
i2c_bus = LockedI2CBus(1)

# 4. Map both physical sensors using their distinct addresses
print("\n--- Initializing Dual IMU Stream on Arch ARM ---")
sensor_A = adafruit_lsm6ds.lsm6dsox.LSM6DSOX(i2c_bus, address=0x6A)
print("[0x6A] Sensor A Connected!")

sensor_B = adafruit_lsm6ds.lsm6dsox.LSM6DSOX(i2c_bus, address=0x6B)
print("[0x6B] Sensor B Connected!")

# 5. Read data structures to prove functionality
print("\n--- Live Data Reads ---")
print(f"Sensor A (0x6A) Accel: {sensor_A.acceleration} Pitch {math.degrees(pitch(*sensor_A.acceleration))} Gyro: {sensor_A.gyro}")
print(f"Sensor B (0x6B) Accel: {sensor_B.acceleration} Pitch {math.degrees(pitch(*sensor_B.acceleration))} Gryro: {sensor_B.gyro}")


