"""Read the front, rear and ground sensors; guard requested movement."""
import math
from gpiozero import InputDevice, DistanceSensor
import adafruit_vl53l0x

# Initial stopping thresholds in centimetres; calibrate on the actual robot.
FRONT_STOP_CM = 30.0
REAR_STOP_CM = 30.0
# Verify on the actual marked boundary before enabling motor movement.
BOUNDARY_VALUE = 0


# Group all sensor readings and movement safety decisions in one place.
class Sensors:
    def __init__(self, i2c):
        # Initialize the front time-of-flight distance sensor on the shared I2C bus.
        self.front = adafruit_vl53l0x.VL53L0X(i2c)
        # Configure rear ultrasonic echo and trigger GPIO pins, with a 2 m maximum range.
        self.rear = DistanceSensor(echo=24, trigger=23, max_distance=2)
        # Same InputDevice assignments as the professor's example.
        self.lines = [InputDevice(pin=p) for p in (17, 27, 22)]

    # Return front and rear distances in centimetres and three digital line readings.
    def read(self):
        # Convert millimetres from the ToF sensor to centimetres.
        front_cm = self.front.range / 10.0
        # Convert metres from gpiozero to centimetres.
        rear_cm = self.rear.distance * 100.0
        # Read the three infrared sensors as digital values (0 or 1).
        ground = tuple(s.value for s in self.lines)
        return front_cm, rear_cm, ground

    # Return True only if the requested direction passes the applicable guard checks.
    def safe(self, left, right, readings):
        front, rear, ground = readings
        # Check each tread separately; a pivot can require both forward and rear checks.
        forward = left > 0 or right > 0
        backward = left < 0 or right < 0
        # Forward motion requires a valid, clear front distance and no marked boundary.
        if forward:
            # Reject non-finite, too-close or out-of-configured-range measurements.
            if not math.isfinite(front) or not FRONT_STOP_CM < front <= 200:
                return False
            # Stop forward motion if any infrared sensor detects the marked boundary.
            if BOUNDARY_VALUE in ground:
                return False
        # Reverse motion requires a valid and clear rear distance.
        if backward:
            if not math.isfinite(rear) or not REAR_STOP_CM < rear <= 200:
                return False
        # No applicable guard has blocked the requested movement.
        return True

    # Release the GPIO devices when the program exits.
    def close(self):
        self.rear.close()
        for sensor in self.lines:
            sensor.close()
