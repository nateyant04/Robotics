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

prep()
home()
centre()
cleanup() #Don't do this until you're done, but still check it out!