from math import *
from multiprocessing import Process
from threading import Thread
from pymavlink import mavutil
import time
# import pixhawk_interface as pint
import requests

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
print('Waiting for Heartbeat')
master.wait_heartbeat()
print ("Heartbeat Detected")
master.arducopter_arm()
print ("BlueROV Armed")

def set_rc_channel_pwm( channel, pwm=1500):
    if channel < 1:
        print("Channel does not exist.")
        return
    if channel < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[channel - 1] = pwm
        master.mav.rc_channels_override_send(
                                             master.target_system,  # target_system
                                             master.target_component,  # target_component
                                             *rc_channel_values)  # RC channel list, in microseconds.

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

def errorAttitude(setPoint, currentAtt):
    error = setPoint - currentAtt
    return error

def getCurrGPS():
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
    return msg.lat/10000000.0, msg.lon/10000000.0

def turn(degrees):
    master.arducopter_arm()
    heading = getLatestHeading()
    targetHeading = heading + degrees
    if targetHeading > 360:
        targetHeading = targetHeading - 360
    deltaAngle = angleDiff(heading, targetHeading)
    while deltaAngle not in range(-1, 2):
        if degrees > 0:
            if abs(deltaAngle) >= 90:
                set_rc_channel_pwm(4, 1600)
            elif abs(deltaAngle) in range(45, 90):
                set_rc_channel_pwm(4, 1580)
            elif abs(deltaAngle) in range(19, 45):
                set_rc_channel_pwm(4, 1560)
            elif abs(deltaAngle) in range(0, 19):
                set_rc_channel_pwm(4, 1550)
            else:
                set_rc_channel_pwm(4, 1550)
        else:
            if abs(deltaAngle) >= 90:
                set_rc_channel_pwm(4, 1400)
            elif abs(deltaAngle) in range(45, 90):
                set_rc_channel_pwm(4, 1420)
            elif abs(deltaAngle) in range(19, 45):
                set_rc_channel_pwm(4, 1440)
            elif abs(deltaAngle) in range(0, 19):
                set_rc_channel_pwm(4, 1450)
            else:
                set_rc_channel_pwm(4, 1550)
        deltaAngle = angleDiff(getLatestHeading(), targetHeading)
    print("Turn Complete")
    return


def bearAngle():
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    Bearing = atan2(cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1), sin(lon2-lon1)*cos(lat2))
    heading = getLatestHeading()
    Bearing = 90-degrees(Bearing)
    return Bearing


def turningNorth():
    heading = getLatestHeading()
    targetHeading = 0
    deltaAngle = angleDiff(targetHeading, heading)
    return deltaAngle


def haversine(lon1, lat1, lon2, lat2):
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    r = 6371
    return c * r

def getLatestAtt():
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    return msg.yaw

def forwardPID(speed, KP, KI, KD, duration):
    lastError = 0
    integral = 0
    iteration_time = 0.01
    lateralSpeed = 1500
    setPoint = getLatestAtt()
    set_rc_channel_pwm(5, speed)
    endTime = time.time() + duration  # Seconds
    while time.time() <= endTime:
        currentHeading = getLatestHeading()
        error = errorAttitude(setPoint, currentHeading) + 300
        integral = integral + (error*iteration_time)
        derivative = (error - lastError)/iteration_time
        lateralSpeed = KP*error + KI*integral + KD*derivative + 1500
        print lateralSpeed, error
        lastError = error
        lateralSpeed = int(lateralSpeed)
        if lateralSpeed < 1450:
            lateralSpeed = 1450
        elif lateralSpeed > 1550:
            lateralSpeed = 1550
        set_rc_channel_pwm(4, lateralSpeed)
    return

def mean(data):
    """Return the sample arithmetic mean of data."""
    n = len(data)
    if n < 1:
        raise ValueError('mean requires at least one data point')
    return sum(data)/float(n) # in Python 2 use sum(data)/float(n)

def _ss(data):
    """Return sum of square deviations of sequence data."""
    c = mean(data)
    ss = sum((x-c)**2 for x in data)
    return ss

def stddev(data, ddof=0):
    """Calculates the population standard deviation
        by default; specify ddof=1 to compute the sample
        standard deviation."""
    n = len(data)
    if n < 2:
        raise ValueError('variance requires at least two data points')
    ss = _ss(data)
    pvar = ss/(n-ddof)
    return pvar**0.5

global input
input = "STAY"
def post_reward(reward):
    #reward = reward ** 2
    global input
    # Change the IP address to the one your computer has for the robot this code runs on. 
    print(10 ** reward)
    ip_addr = "192.168.42.19"
    input = requests.post("http://" + ip_addr + ":5000", json={"reward": 10 ** reward}).text
    return

def get_temp():
    msg = master.recv_match(type='SCALED_PRESSURE3', blocking=True)
    return msg.temperature/100.0

seconds = 8

def get_temps():
    temp_list = []

    start = time.time()
    while time.time() - start < seconds:
        temp_list.append(get_temp())
        time.sleep(0.3)
    # print(stddev(temp_list) ** 2)
    return post_reward(stddev(temp_list))

def spiral():
    x = Thread(target=get_temps)
    x.start()
    s = time.time()
    while time.time() - s < seconds:
        set_rc_channel_pwm(4,1575)
    # # Move a spiral
    # for i in range(0,3):
    #     # Move a spiral
    #     set_rc_channel_pwm(5, 1550)
    #     set_rc_channel_pwm(4, 1550)
    #     time.sleep(0.3)
    #     set_rc_channel_pwm(5, 1550)
    #     set_rc_channel_pwm(4, 1550)
    #     time.sleep(0.3)
    #     set_rc_channel_pwm(5, 1550)
    #     set_rc_channel_pwm(4, 1550)
    #     time.sleep(0.3)
    #     set_rc_channel_pwm(5, 1550)
    #     set_rc_channel_pwm(4, 1550)
    #     time.sleep(0.3)
    #     set_rc_channel_pwm(5, 1550)
    #     set_rc_channel_pwm(4, 1550)
    #     time.sleep(0.3)
    #     set_rc_channel_pwm(5, 1550)
    #     set_rc_channel_pwm(4, 1550)
    #     time.sleep(0.3)
    #     set_rc_channel_pwm(5, 1550)
    #     set_rc_channel_pwm(4, 1550)
    #     time.sleep(0.3)
    #     set_rc_channel_pwm(5, 1550)
    #     set_rc_channel_pwm(4, 1550)
    #     time.sleep(0.3)
    #     set_rc_channel_pwm(5, 1550)
    #     set_rc_channel_pwm(4, 1550)
    #     time.sleep(0.3)
    #     set_rc_channel_pwm(5, 1550)
    #     set_rc_channel_pwm(4, 1550)
    #     time.sleep(0.3)
    # time.sleep(0.3)
    setNeutral()
    x.join()
    return
# print("Mean %f, STDDEV %f", mean(temp_list), stddev(temp_list))
# return post_reward(stddev(temp_list))

start_x = 3
start_y = 0
print("Turning North")
turn(turningNorth())
while True:
    print(input)
    print("Searching Grid")
    master.arducopter_arm()
    start = time.time()
    if input.upper() == "LEFT":
        while time.time() - start < seconds and not start_x == 0:
            set_rc_channel_pwm(6, 1400)
        start_x = start_x - 1 if start_x > 0 else start_x
    elif input.upper() == "DOWN" and not start_y == 0:
        while time.time() - start < seconds:
            set_rc_channel_pwm(5, 1400)
        start_y = start_y - 1 if start_y > 0 else start_y
    elif input.upper() == "UP" and not start_y == 3:
        while time.time() - start < seconds:
            set_rc_channel_pwm(5, 1600)
        start_y = start_y + 1 if start_y < 3 else start_y
    elif input.upper() == "RIGHT":
        while time.time() - start < seconds and not start_x == 3:
            set_rc_channel_pwm(6, 1600)
        start_x = start_x + 1 if start_x < 3 else start_x

    else:
        pass
 
    spiral()
    time.sleep(0.3)
    spiral()
    time.sleep(0.3)
    turn(turningNorth())  
print('Disarming BlueROV')
master.arducopter_disarm()
print('BlueROV disarmed')
print('Guided Path Ended')
