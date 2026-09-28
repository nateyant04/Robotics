import time
import sys
import nxt
import nxt.locator
import nxt.motor
import nxt.sensor
import nxt.sensor.generic

def bumper( sensor ):
    def bumpy():
        while not sensor.get_sample():
            pass
        return True
    return bumpy

def prep():
    global brick
    global motor_shoulder
    global motor_elbow
    global touch_shoulder
    global touch_elbow
    try:
        brick = nxt.locator.find()
    except nxt.locator.BrickNotFoundError:
        print("---\n<<< Did you remember to turn the brick on? >>>\n---")
        if sys.flags.interactive:
            return
        else:
            sys.exit(0)
    motor_shoulder = brick.get_motor(nxt.motor.Port.A)
    motor_elbow = brick.get_motor(nxt.motor.Port.B)
    touch_shoulder = brick.get_sensor(nxt.sensor.Port.S1, nxt.sensor.generic.Touch)
    touch_elbow = brick.get_sensor(nxt.sensor.Port.S2, nxt.sensor.generic.Touch)

def cleanup():
    motor_shoulder.idle()
    motor_elbow.idle()
    brick.close()

def home():
    motor_shoulder.turn(-15,360,stop_turn=bumper(touch_shoulder))
    motor_elbow.turn(15,360,stop_turn=bumper(touch_elbow))
    
def centre():
    motor_shoulder.turn(15,100)
    motor_elbow.turn(-15,120)
    
def slap_left():
    motor_shoulder.turn(15,130)
    motor_elbow.turn(-60,110)
    home()
    
def slap_right():
    motor_elbow.turn(-15,270)
    motor_shoulder.turn(15,80)
    motor_elbow.turn(15,30)
    motor_shoulder.turn(15,30)
    motor_elbow.turn(15,30)
    motor_shoulder.turn(15,30)
    motor_elbow.turn(60,180)
    home()
    
def slap_forward():
    motor_elbow.turn(-15,250)
    motor_shoulder.turn(15,80)
    motor_elbow.turn(50,100)
    home()

prep()
home()
slap_forward()

#home()
#centre()
cleanup() #Don't do this until you're done, but still check it out!