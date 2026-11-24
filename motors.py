"""Motor control using the lab's PCA9685 configuration."""
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor


class Motors:
    def __init__(self, i2c):
        self.pwm = PCA9685(i2c, address=0x5f)
        self.pwm.frequency = 50
        self.motors = []
        for a, b in [(15, 14), (12, 13), (11, 10), (8, 9)]:
            m = motor.DCMotor(self.pwm.channels[a], self.pwm.channels[b])
            m.decay_mode = motor.SLOW_DECAY
            self.motors.append(m)
        self.stop()

    def drive(self, left, right):
        # Positive means forward; the professor's sample uses negative throttle.
        self.motors[0].throttle = -max(-1.0, min(1.0, left))
        self.motors[1].throttle = -max(-1.0, min(1.0, right))

    def stop(self):
        for m in self.motors:
            m.throttle = 0

    def close(self):
        self.stop()
        self.pwm.deinit()
