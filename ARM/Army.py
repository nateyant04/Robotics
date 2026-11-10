from xarm.wrapper import XArmAPI
import time
arm=XArmAPI('192.168.1.232') #It will ALWAYS be at this IP address
while arm.get_err_warn_code()[1][0]:
    print("Don't forget the emergency stop button?")
    time.sleep(0.5)
    arm.clean_error()
arm.motion_enable(True) #Turn the motors on

arm.set_bio_gripper_g2_position(850,wait=True) #Easier to mark positions?

arm.set_mode(2) #'Zero grav' mode (free-positioning)
arm.set_state(0) #Ready to go!

def button():
    return arm.get_tgpio_digital(2)[1]

def getpos():
    fields=["x","y","z","roll","pitch","yaw"]
    p=arm.get_position()[1]
    return {label:value for label,value in zip(fields,p)}
print("Position the arm, then press the button just above the gripper")

#while True: #If you want indefinite readings
for i in range(10): #Mash the button to get it back to an interactive REPL
    while not button(): #Wait for button press
        pass
    print(getpos())
    while button(): #Effectively debounce
        pass