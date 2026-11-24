#!/usr/bin/env/python3
import time
from board import SCL, SDA
import os
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor
import pygame
from gpiozero import DistanceSensor
from gpiozero.exc import PWMSoftwareFallback
import warnings
from time import sleep

MOTOR_M1_IN1 =  15      #Define the positive pole of M1
MOTOR_M1_IN2 =  14      #Define the negative pole of M1
MOTOR_M2_IN1 =  12      #Define the positive pole of M2
MOTOR_M2_IN2 =  13      #Define the negative pole of M2
MOTOR_M3_IN1 =  11      #Define the positive pole of M3
MOTOR_M3_IN2 =  10      #Define the negative pole of M3
MOTOR_M4_IN1 =  8       #Define the positive pole of M4
MOTOR_M4_IN2 =  9       #Define the negative pole of M4

Dir_forward   = 0
Dir_backward  = 1

left_forward  = 1
left_backward = 0

right_forward = 0
right_backward= 1

pwn_A = 0
pwm_B = 0

Tr = 23
Ec = 24
warnings.filterwarnings("ignore", category=PWMSoftwareFallback)
sensor = DistanceSensor(echo=Ec, trigger=Tr,max_distance=2) # Maximum detection distance 2m.
  
def map(x,in_min,in_max,out_min,out_max):
  return (x - in_min)/(in_max - in_min) *(out_max - out_min) +out_min

i2c = busio.I2C(SCL, SDA)
pwm_motor = PCA9685(i2c, address=0x5f) #default 0x40
pwm_motor.frequency = 50
 
motor1 = motor.DCMotor(pwm_motor.channels[MOTOR_M1_IN1],pwm_motor.channels[MOTOR_M1_IN2] )
motor1.decay_mode = (motor.SLOW_DECAY)
motor2 = motor.DCMotor(pwm_motor.channels[MOTOR_M2_IN1],pwm_motor.channels[MOTOR_M2_IN2] )
motor2.decay_mode = (motor.SLOW_DECAY)
motor3 = motor.DCMotor(pwm_motor.channels[MOTOR_M3_IN1],pwm_motor.channels[MOTOR_M3_IN2] )
motor3.decay_mode = (motor.SLOW_DECAY)
motor4 = motor.DCMotor(pwm_motor.channels[MOTOR_M4_IN1],pwm_motor.channels[MOTOR_M4_IN2] )
motor4.decay_mode = (motor.SLOW_DECAY)
#  motorStop()

def Motor(channel,direction,motor_speed):
  if motor_speed > 100:
    motor_speed = 100
  elif motor_speed < 0:
    motor_speed = 0
  speed = map(motor_speed, 0, 100, 0, 1.0)
  if direction == -1:
    speed = -speed
  if channel == 1:
    motor1.throttle = speed
  elif channel == 2:
    motor2.throttle = speed
  elif channel == 3:
    motor3.throttle = speed
  elif channel == 4:
    motor4.throttle = speed

def motorStop():#Motor stops
    motor1.throttle = 0
    motor2.throttle = 0
    motor3.throttle = 0
    motor4.throttle = 0

def destroy():
  motorStop()
  pwm_motor.deinit()

def checkdist():
    return (sensor.distance) *100 # Unit: cm

pygame.init()
pygame.joystick.init()
clock=pygame.time.Clock()
joystick=None

if __name__ == '__main__':
  try:     
    while True:
      distance = checkdist() 
      print("%.2f cm" %distance)
      pygame.event.pump()
      for event in pygame.event.get():
        if event.type == pygame.JOYDEVICEADDED:
          print("Controller connected")
          joystick = pygame.joystick.Joystick(event.device_index)
          joystick.init()
        elif event.type == pygame.JOYDEVICEREMOVED:
          print("Controller disconnected")
          joystick = None
          motorStop()
      if joystick is not None:
        left_stick = -joystick.get_axis(4)
        right_stick = -joystick.get_axis(1)
        left_direction = -1 if left_stick >= 0 else 1
        right_direction = -1 if right_stick >= 0 else 1
        left_speed = abs(left_stick) * 100
        right_speed = abs(right_stick) * 100
        if distance < 20:
            if right_direction = 1:
                right_speed = 0
            if left_direction = 1:
                left_speed = 0
            print("stop", left_speed, right_speed)
        Motor(1, left_direction, left_speed)
        Motor(2, right_direction, right_speed)
        if joystick.get_button(9):
          break
      
      clock.tick(50)
    destroy()
  except KeyboardInterrupt:
    destroy()
    pygame.quit()
os.system("systemctl poweroff -i")