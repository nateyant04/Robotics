"""Guarded control lab entry point. Run --test-sensors before driving."""
import argparse
import os
import signal
import time
# Use dummy SDL drivers so Pygame can run over SSH without a display or speakers.
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
import pygame
import board
import busio
from motors import Motors
from sensors import Sensors
from manuevers import Maneuver, FIGURE_EIGHT, REVERSE_AND_TURN

# Controller button indexes; confirm these using --test-controller.
BUTTON_FIGURE_EIGHT = 0
BUTTON_REVERSE = 1
BUTTON_CANCEL = 2
BUTTON_EXIT = 9
# Vertical axes of the left and right sticks, as in the lab sample.
LEFT_AXIS = 1
RIGHT_AXIS = 4
# Limit manual throttle to 45%; ignore small stick movements near the centre.
MAX_MANUAL_SPEED = 0.45
DEADZONE = 0.12


# Convert a joystick axis into a signed motor command.
def stick(joystick, axis):
    # Invert the axis so pushing forward gives a positive speed.
    value = -joystick.get_axis(axis)
    return 0.0 if abs(value) < DEADZONE else value * MAX_MANUAL_SPEED


# Handle SIGTERM like Ctrl+C so the finally block can stop the robot.
def interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    # Parse optional diagnostic modes from the command line.
    parser = argparse.ArgumentParser()
    parser.add_argument('--test-sensors', action='store_true')
    parser.add_argument('--test-controller', action='store_true')
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, interrupt)
    # Initialize references before setup so cleanup can handle partial initialization.
    i2c = motors = sensors = None
    # Track the current automatic maneuver and controller state.
    routine = Maneuver()
    joystick = None
    previous = time.monotonic()
    was_moving = False
    # Require neutral sticks before enabling motion.
    armed = False
    try:
        # Skip motor and sensor initialization when only testing the controller.
        if not args.test_controller:
            # Share one I2C bus between the motor controller and front distance sensor.
            i2c = busio.I2C(board.SCL, board.SDA)
            motors = Motors(i2c)
            sensors = Sensors(i2c)
        # Print sensor readings repeatedly without issuing driving commands.
        if args.test_sensors:
            while True:
                print(sensors.read(), flush=True)
                time.sleep(0.2)
        # Initialize Pygame and detect an already-connected controller.
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count():
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
        # Use a Pygame clock to limit the control loop to a target of 50 Hz.
        clock = pygame.time.Clock()
        while True:
            # Measure elapsed loop time using a clock unaffected by wall-clock changes.
            now = time.monotonic()
            dt = now - previous
            previous = now
            # Advance the maneuver timer only if movement was permitted in the previous loop.
            if was_moving and routine.active:
                routine.advance(dt)
            # Reset flags for this iteration; they are set again only when needed.
            was_moving = False
            cancelled = False
            # Process controller connection, disconnection and button events.
            for event in pygame.event.get():
                # Connect a newly detected controller and require neutral sticks again.
                if event.type == pygame.JOYDEVICEADDED and joystick is None:
                    joystick = pygame.joystick.Joystick(event.device_index)
                    joystick.init()
                    armed = False
                # Stop immediately and discard the maneuver when the active controller disconnects.
                elif event.type == pygame.JOYDEVICEREMOVED and joystick is not None:
                    if event.instance_id == joystick.get_instance_id():
                        joystick = None
                        routine.cancel()
                        armed = False
                        if motors:
                            motors.stop()
                # React to button presses once rather than restarting a routine while a button is held.
                elif event.type == pygame.JOYBUTTONDOWN and joystick is not None:
                    # Ignore button events from other controllers.
                    if event.instance_id != joystick.get_instance_id():
                        continue
                    # In diagnostic mode, report button indexes instead of moving.
                    if args.test_controller:
                        print('Button:', event.button, flush=True)
                        continue
                    # Exit the program; the finally block handles cleanup.
                    if event.button == BUTTON_EXIT:
                        return
                    # Cancel the routine, stop the motors and require neutral sticks before driving again.
                    if event.button == BUTTON_CANCEL:
                        routine.cancel()
                        cancelled = True
                        armed = False
                        motors.stop()
                    # Start a selected routine only when driving is enabled.
                    elif armed and not cancelled:
                        if event.button == BUTTON_FIGURE_EIGHT:
                            routine.start(FIGURE_EIGHT)
                        elif event.button == BUTTON_REVERSE:
                            routine.start(REVERSE_AND_TURN)
                # Report significant axis movements to help identify stick indexes.
                elif args.test_controller and event.type == pygame.JOYAXISMOTION:
                    if abs(event.value) > 0.3:
                        print('Axis:', event.axis, 'value:', round(event.value, 2), flush=True)
            # Do not execute any driving logic in controller diagnostic mode.
            if args.test_controller:
                clock.tick(50)
                continue
            # Fail safely if the controller is absent or lacks the expected axes.
            if joystick is None or joystick.get_numaxes() <= RIGHT_AXIS:
                motors.stop()
                routine.cancel()
                armed = False
                clock.tick(50)
                continue
            # Read both sticks independently for tank steering.
            manual = (stick(joystick, LEFT_AXIS), stick(joystick, RIGHT_AXIS))
            # Keep the robot stopped until both sticks return to neutral.
            if not armed:
                motors.stop()
                if manual == (0.0, 0.0) and not cancelled:
                    armed = True
                clock.tick(50)
                continue
            # An active maneuver takes priority over manual stick commands.
            command = routine.command() if routine.active else manual
            # Both manual and automatic commands pass through the same guard.
            try:
                # Obtain fresh sensor measurements and evaluate the requested direction.
                readings = sensors.read()
                safe = sensors.safe(*command, readings)
            # Treat sensor read failures as unsafe and stop the motors.
            except Exception as exc:
                motors.stop()
                print('Sensor error:', exc, flush=True)
                safe = False
            # Drive only when the guard permits it; remember whether to advance the routine timer.
            if safe and not cancelled:
                motors.drive(*command)
                was_moving = routine.active and command != (0.0, 0.0)
            else:
                motors.stop()
            clock.tick(50)
    # Allow Ctrl+C or SIGTERM to exit normally.
    except KeyboardInterrupt:
        pass
    # Release hardware resources and stop motors even if an exception occurs.
    finally:
        if motors is not None:
            motors.close()
        if sensors is not None:
            sensors.close()
        pygame.quit()
        if i2c is not None:
            i2c.deinit()


# Run main only when this file is executed directly.
if __name__ == '__main__':
    main()
