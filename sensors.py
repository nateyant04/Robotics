"""Read the front, rear and ground sensors; guard requested movement."""
import math
from gpiozero import InputDevice, DistanceSensor
import adafruit_vl53l0x

FRONT_STOP_CM = 30.0
REAR_STOP_CM = 30.0
# Verify on the actual marked boundary before enabling motor movement.
BOUNDARY_VALUE = 0


class Sensors:
    def __init__(self, i2c):
        self.front = adafruit_vl53l0x.VL53L0X(i2c)
        self.rear = DistanceSensor(echo=24, trigger=23, max_distance=2)
        # Same InputDevice assignments as the professor's example.
        self.lines = [InputDevice(pin=p) for p in (17, 27, 22)]

    def read(self):
        front_cm = self.front.range / 10.0
        rear_cm = self.rear.distance * 100.0
        ground = tuple(s.value for s in self.lines)
        return front_cm, rear_cm, ground

    def safe(self, left, right, readings):
        front, rear, ground = readings
        forward = left > 0 or right > 0
        backward = left < 0 or right < 0
        if forward:
            if not math.isfinite(front) or not FRONT_STOP_CM < front <= 200:
                return False
            if BOUNDARY_VALUE in ground:
                return False
        if backward:
            if not math.isfinite(rear) or not REAR_STOP_CM < rear <= 200:
                return False
        return True

    def close(self):
        self.rear.close()
        for sensor in self.lines:
            sensor.close()
