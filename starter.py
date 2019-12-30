from pymavlink import mavutil  # import mavutil
from threading import Thread
import time
import math
import pix_int
#from simple_square import square_path
import serial

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550') # Create UDP connection to Localhost
master.wait_heartbeat()  # Wait for heartbeat before continuing

#set PWM speed for RC Channel.
#Safe negative direction PWM ranges from 1250 to 1500
#Safe positive direction PWM ranges from 1500 to 1750
#PWM 1500 is neutral (off)
#Channel 1 = Pitch
#Channel 2 = Roll
#Channel 3 = Throttle
#Channel 4 = Yaw
#Channel 5 = Forward
#Channel 6 = Lateral
def set_rc_channel_pwm(channel, pwm=1500):
    if channel < 1:
        print("Channel does not exist.")
        return
    if channel < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[channel - 1] = pwm
        master.mav.rc_channels_override_send(
            master.target_system,                # target_system
            master.target_component,             # target_component
            *rc_channel_values)                  # RC channel list, in microseconds.


# Have to arm before ever moving the motors
master.arducopter_arm()

# This is how we move forward.
#set_rc_channel_pwm(5, 1600)

# And this is how we move reverse.
#set_rc_channel_pwm(5, 1600)

# This is how we turn clockwise.
#set_rc_channel_pwm(4, 1600)

# And this is how turn counter clockwise.
#set_rc_channel_pwm(4, 1400)

# This is how we move left.
#set_rc_channel_pwm(6, 1600)

# And this is how we move right.
#set_rc_channel_pwm(6, 1400)

# This is how we get temperature in Celcius
#pix_int.get_temp(master)

# This is how we set up the usb port for reading from sensor

# URL of usb port - change to that of oxidation sensor
# Conductivity port: '/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DM01N4L3-if00-port0'
# Oxidation port: '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0'
usbport_path = '/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_DM01N4L3-if00-port0'
usbport = serial.Serial(usbport_path, 9600, timeout=0)

# And then we can just pass it into the function to get conductivity data
# This is how we get conductivity

#####################################################################
def senseData():
    a= pix_int.get_conductivity(usbport)
    print(a)
    c = 0
    s_list =[]
    while(c<6):#prints 5 values neglecting the first one('OK')
    #Read Value
        if(c>0):
            a= pix_int.get_conductivity(usbport)
            a=float(a)
            s_list.append(a)
            print(a)
        c+=1
    print(s_list)
    s = sum(s_list)
    print("SUM IS:",s)
    av = s/len(s_list)
    print("AVERAGE:",av)

b= pix_int.get_conductivity(usbport)
b=float(b)
print(b)
if(b>av):
    print("SLEEPING FOR 3RS")
    time.sleep(3)
    set_rc_channel_pwm(6, 1600)
    print("SLEEPING FOR 1RS")
    time.sleep(1)
    print("RIGHT SIDE ACTIVATED")
else:
    print("SLEEPING FOR 3LS")
    time.sleep(3)
    set_rc_channel_pwm(6, 1400)
    print("SLEEPING FOR 1LS")
    time.sleep(1)
    print("LEFT SIDE ACTIVATED")
# At the end, we disarm
time.sleep(2)
print("DISARM")


###################### SQUARE PATH #############################

def set_time(seconds):
    end_time = time.time() + seconds 
    while time.time() < end_time:
        set_rc_channel_pwm(5, 1650)
        time.sleep(1)



def setNeutral():
    set_rc_channel_pwm(1, 1500)
    set_rc_channel_pwm(2, 1500)
    set_rc_channel_pwm(3, 1500)
    set_rc_channel_pwm(4, 1500)
    set_rc_channel_pwm(5, 1500)
    set_rc_channel_pwm(6, 1500)
    return


def getLatestHeading():
    msg = master.recv_match(type='VFR_HUD', blocking=True)
    return msg.heading


def angleDiff(setPoint, currentHeading):
    deltaAngle = setPoint - currentHeading
    deltaAngle = (deltaAngle + 180) % 360 - 180
    return deltaAngle



def turn(degrees):
    master.arducopter_arm()
    print "BlueRov Armed"
    heading = getLatestHeading()
    targetHeading = heading + degrees
    if targetHeading > 360:
        targetHeading = targetHeading - 360
    deltaAngle = angleDiff(heading, targetHeading)
    print("Turn Started")
    while deltaAngle not in range(-1, 2):
        if abs(deltaAngle) >= 90:
            set_rc_channel_pwm(4, 1580)
        elif abs(deltaAngle) in range(45,90):
            set_rc_channel_pwm(4, 1570)
        elif abs(deltaAngle) in range(19, 45):
            set_rc_channel_pwm(4, 1560)
        elif abs(deltaAngle) in range (0,19):
            set_rc_channel_pwm(4, 1550)
        else:
            set_rc_channel_pwm(4,1550)
        deltaAngle = angleDiff(getLatestHeading(), targetHeading)
    print("Turn Complete")
    set_rc_channel_pwm(4, 1500)
    return

def square_path():
    duration = 8
    strt_duration = duration / 4
    vertices = 0
    master.arducopter_arm()
    while vertices < 4:
        print('Vertex: %d' % vertices)
        set_time(strt_duration)
        #time.sleep(1)
        setNeutral()
        turn(90)
        setNeutral()
        #time.sleep(1)
        vertices += 1
    master.arducopter_disarm()

######################## SQUARE PATH ENDS ########################################
move = Thread(target=square_path, args=())
record = Thread(target=senseData, args=())
move.start()
record.start()
move.join()
record.join()
#####################################################################

master.arducopter_disarm()
