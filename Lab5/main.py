"""Guarded control lab entry point. Run --test-sensors before driving."""
import argparse
import os
import signal
import time
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
import pygame
import board
import busio
from motors import Motors
from sensors import Sensors
from manuevers import Maneuver, FIGURE_EIGHT, REVERSE_AND_TURN

BUTTON_FIGURE_EIGHT = 0
BUTTON_REVERSE = 1
BUTTON_CANCEL = 2
BUTTON_EXIT = 9
LEFT_AXIS = 1
RIGHT_AXIS = 4
MAX_MANUAL_SPEED = 0.45
DEADZONE = 0.12


def stick(joystick, axis):
    value = -joystick.get_axis(axis)
    return 0.0 if abs(value) < DEADZONE else value * MAX_MANUAL_SPEED


def interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--test-sensors', action='store_true')
    parser.add_argument('--test-controller', action='store_true')
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, interrupt)
    i2c = motors = sensors = None
    routine = Maneuver()
    joystick = None
    previous = time.monotonic()
    was_moving = False
    armed = False
    try:
        if not args.test_controller:
            i2c = busio.I2C(board.SCL, board.SDA)
            motors = Motors(i2c)
            sensors = Sensors(i2c)
        if args.test_sensors:
            while True:
                print(sensors.read(), flush=True)
                time.sleep(0.2)
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count():
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
        clock = pygame.time.Clock()
        while True:
            now = time.monotonic()
            dt = now - previous
            previous = now
            if was_moving and routine.active:
                routine.advance(dt)
            was_moving = False
            cancelled = False
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED and joystick is None:
                    joystick = pygame.joystick.Joystick(event.device_index)
                    joystick.init()
                    armed = False
                elif event.type == pygame.JOYDEVICEREMOVED and joystick is not None:
                    if event.instance_id == joystick.get_instance_id():
                        joystick = None
                        routine.cancel()
                        armed = False
                        if motors:
                            motors.stop()
                elif event.type == pygame.JOYBUTTONDOWN and joystick is not None:
                    if event.instance_id != joystick.get_instance_id():
                        continue
                    if args.test_controller:
                        print('Button:', event.button, flush=True)
                        continue
                    if event.button == BUTTON_EXIT:
                        return
                    if event.button == BUTTON_CANCEL:
                        routine.cancel()
                        cancelled = True
                        armed = False
                        motors.stop()
                    elif armed and not cancelled:
                        if event.button == BUTTON_FIGURE_EIGHT:
                            routine.start(FIGURE_EIGHT)
                        elif event.button == BUTTON_REVERSE:
                            routine.start(REVERSE_AND_TURN)
                elif args.test_controller and event.type == pygame.JOYAXISMOTION:
                    if abs(event.value) > 0.3:
                        print('Axis:', event.axis, 'value:', round(event.value, 2), flush=True)
            if args.test_controller:
                clock.tick(50)
                continue
            if joystick is None or joystick.get_numaxes() <= RIGHT_AXIS:
                motors.stop()
                routine.cancel()
                armed = False
                clock.tick(50)
                continue
            manual = (stick(joystick, LEFT_AXIS), stick(joystick, RIGHT_AXIS))
            if not armed:
                motors.stop()
                if manual == (0.0, 0.0) and not cancelled:
                    armed = True
                clock.tick(50)
                continue
            command = routine.command() if routine.active else manual
            # Both manual and automatic commands pass through the same guard.
            try:
                readings = sensors.read()
                safe = sensors.safe(*command, readings)
            except Exception as exc:
                motors.stop()
                print('Sensor error:', exc, flush=True)
                safe = False
            if safe and not cancelled:
                motors.drive(*command)
                was_moving = routine.active and command != (0.0, 0.0)
            else:
                motors.stop()
            clock.tick(50)
    except KeyboardInterrupt:
        pass
    finally:
        if motors is not None:
            motors.close()
        if sensors is not None:
            sensors.close()
        pygame.quit()
        if i2c is not None:
            i2c.deinit()


if __name__ == '__main__':
    main()
