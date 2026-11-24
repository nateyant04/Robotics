"""Motor control using the lab's PCA9685 configuration."""
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor


# Encapsulate the motor controller and expose signed left/right tread commands.
class Motors:
    def __init__(self, i2c):
        # Create the PCA9685 at the I2C address supplied by the professor.
        self.pwm = PCA9685(i2c, address=0x5f)
        # Set the PWM frequency to the value used in the lab sample.
        self.pwm.frequency = 50
        # Store the four motor objects in channel order.
        self.motors = []
        # Each pair contains the two PCA9685 outputs for one motor.
        for a, b in [(15, 14), (12, 13), (11, 10), (8, 9)]:
            m = motor.DCMotor(self.pwm.channels[a], self.pwm.channels[b])
            # Use the same slow-decay motor driver mode as the provided code.
            m.decay_mode = motor.SLOW_DECAY
            self.motors.append(m)
        # Ensure all motor outputs are stopped before continuing.
        self.stop()

    # Accept independent tread speeds from -1 (reverse) to +1 (forward).
    def drive(self, left, right):
        # Positive means forward; the professor's sample uses negative throttle.
        # Clamp speeds to the valid throttle range and invert their signs for this wiring.
        self.motors[0].throttle = -max(-1.0, min(1.0, left))
        self.motors[1].throttle = -max(-1.0, min(1.0, right))

    # Set all four motor outputs to zero, including unused channels.
    def stop(self):
        for m in self.motors:
            m.throttle = 0

    # Stop movement before releasing the PWM controller.
    def close(self):
        self.stop()
        self.pwm.deinit()
