# Wahhaj Zahedi
# Fall 2018
from pymavlink import mavutil
from multiprocessing import Process
import subprocess
import time
import math
import json

# Create UDP connection to Localhost
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()  # Wait for heartbeat before continuing

# set PWM speed for RC Channel.
# Safe negative direction PWM ranges from 1250 to 1500
# Safe positive direction PWM ranges from 1500 to 1750
# PWM 1500 is neutral (off)
# Channel 1 = Pitch
# Channel 2 = Roll
# Channel 3 = Throttle
# Channel 4 = Yaw
# Channel 5 = Forward
# Channel 6 = Lateral
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

# Throttle Control, emulates joystick throttle based on currentSpeed PWM and targetSpeed PWM.
# Duration is used to keep at targetSpeed for x seconds
# Acceleration is used to control rate of change of speed
def throttle(channel, currentSpeed, targetSpeed, acceleration, duration):
    print('Throttling channel %d ' % channel + 'to speed %d ' %
          targetSpeed + 'for %d ' % duration + 'seconds.')
    if currentSpeed == targetSpeed:
        pass
    elif currentSpeed < targetSpeed:
        for x in range(currentSpeed, targetSpeed):
            j = x
            j -= currentSpeed
            x += int(0.2*math.e**(j*0.1107))
            if x > targetSpeed:
                x = targetSpeed
                set_rc_channel_pwm(channel, x)
                break
            set_rc_channel_pwm(channel, x)
            time.sleep(acceleration)
    else:
        for x in range(targetSpeed, currentSpeed):
            j = x
            j -= targetSpeed
            j = int(0.2*math.e**(j*0.1107))
            x -= targetSpeed - j
            x = currentSpeed - x
            if x < targetSpeed:
                x = targetSpeed
                set_rc_channel_pwm(channel, x)
                break
            set_rc_channel_pwm(channel, x)
            time.sleep(acceleration)
    time.sleep(duration)
    set_rc_channel_pwm(channel, 1500)
    return

# Set All Channels to Neutral Speed (Off)
def setNeutral():
    set_rc_channel_pwm(1, 1500)
    set_rc_channel_pwm(2, 1500)
    set_rc_channel_pwm(3, 1500)
    set_rc_channel_pwm(4, 1500)
    set_rc_channel_pwm(5, 1500)
    set_rc_channel_pwm(6, 1500)
    return

# Get latest Heading packet in degrees
def getLatestHeading():
    msg = master.recv_match(type='VFR_HUD', blocking=True)
    return msg.heading

def angleDiff(setPoint, currentHeading):
    deltaAngle = setPoint - currentHeading
    deltaAngle = (deltaAngle + 180) % 360 - 180
    return deltaAngle


def turnCW():
    heading = getLatestHeading()
    targetHeading = heading + 90
    if targetHeading > 360:
        targetHeading = targetHeading - 360
    deltaAngle = angleDiff(heading, targetHeading)
    set_rc_channel_pwm(4, 1530)
    print("Turn Started")
    while deltaAngle not in range(-3, 4):
        deltaAngle = angleDiff(getLatestHeading(), targetHeading)
    print("Turn Complete")
    set_rc_channel_pwm(4, 1500)
    return

def turnCCW():
    heading = getLatestHeading()
    targetHeading = heading - 90
    if targetHeading > 360:
        targetHeading = targetHeading - 360
    deltaAngle = angleDiff(heading, targetHeading)
    set_rc_channel_pwm(4, 1470)
    print("Turn Started")
    while deltaAngle not in range(-3, 4):
        deltaAngle = angleDiff(getLatestHeading(), targetHeading)
    print("Turn Complete")
    set_rc_channel_pwm(4, 1500)
    return


# Adaptive Sampling Path
print ('Adaptive Sampling Path Started')
loops = 0
setNeutral()
# cameraProcess = subprocess.Popen(['python', '/home/pi/startCapture.py', 'AdaptiveSampling'])
dataProcess = subprocess.Popen(['python', '/home/pi/capData.py', 'AdaptiveSampling'])
master.arducopter_arm()
length = 5
pwm = 1600

print('Edge 1')
set_rc_channel_pwm(5, 1530)
time.sleep(length)
set_rc_channel_pwm(5, 1500)
turnCCW()
set_rc_channel_pwm(5, pwm)
time.sleep(6)
set_rc_channel_pwm(5, 1500)
turnCCW()

print('Edge 2')
set_rc_channel_pwm(5, 1530)
time.sleep(length)
set_rc_channel_pwm(5, 1500)
turnCW()
set_rc_channel_pwm(5, pwm)
time.sleep(3)
set_rc_channel_pwm(5, 1500)
turnCW()

print('Edge 3')
set_rc_channel_pwm(5, 1530)
time.sleep(length)
set_rc_channel_pwm(5, 1500)
turnCCW()
set_rc_channel_pwm(5, pwm)
time.sleep(1.5)
set_rc_channel_pwm(5, 1500)
turnCCW()

print('Edge 4')
set_rc_channel_pwm(5, 1530)
time.sleep(length)
set_rc_channel_pwm(5, 1500)
turnCW()
set_rc_channel_pwm(5, pwm)
time.sleep(0.5)
set_rc_channel_pwm(5, 1500)
turnCW()

print('Edge 5')
set_rc_channel_pwm(5, 1530)
time.sleep(length)
set_rc_channel_pwm(5, 1500)
turnCCW()
set_rc_channel_pwm(5, pwm)
time.sleep(1.5)
set_rc_channel_pwm(5, 1500)
turnCCW()

print('Edge 6')
set_rc_channel_pwm(5, 1530)
time.sleep(length)
set_rc_channel_pwm(5, 1500)
turnCW()
set_rc_channel_pwm(5, pwm)
time.sleep(3)
set_rc_channel_pwm(5, 1500)
turnCW()

print('Edge 7')
set_rc_channel_pwm(5, 1530)
time.sleep(length)
set_rc_channel_pwm(5, 1500)
turnCCW()
set_rc_channel_pwm(5, pwm)
time.sleep(6)
set_rc_channel_pwm(5, 1500)
turnCCW()

print('Edge 8')
set_rc_channel_pwm(5, 1530)
time.sleep(length)
set_rc_channel_pwm(5, 1500)
turnCCW()

# while loops < 4:
#     print('Loop: %d' % loops)
#     set_rc_channel_pwm(5, 1530)
    # time.sleep(5)
    # set_rc_channel_pwm(5, 1500)
#     width = 2
#     pwm = 1550
#     adaptiveFactor = loops + width
#     if loops % 2 == 0:
#         turnCCW()
#         set_rc_channel_pwm(5, pwm)
        # time.sleep(adaptiveFactor)
        # set_rc_channel_pwm(5, 1500)
#         turnCCW()
#     else:
#         turnCW()
#         set_rc_channel_pwm(5, pwm)
        # time.sleep(adaptiveFactor)
        # set_rc_channel_pwm(5, 1500)
#         turnCW()
#     loops += 1
setNeutral()
master.arducopter_disarm()
dataProcess.terminate()
# cameraProcess.terminate()
print ('Lawnmower Path Complete')
