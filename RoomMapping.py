# Room-Mapping Software Package for Rover Robotics Mini 3-8-24

import os                         # Used to interface w/ ROS through subshell
import time                       # For delay commands
import math                       # For trig functions
import terminalplot as tp         # For terminal plotting
import RPi.GPIO as GPIO           # For GPIO communication
import subprocess                 # For acquiring sudo permission

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO_TRIGGER = 18  # Ultrasonic Trigger GPIO Pin
GPIO_ECHO = 24    # Ultrasonic Echo GPIO Pin
GPIO_SERVO = 16   # Servo Data GPIO pin
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_SERVO, GPIO.OUT)
angles = [1.75, 2.167, 2.584, 3.001, 3.418, 3.835, 4.252, 4.669, 5.086, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 10]
p = GPIO.PWM(GPIO_SERVO, 50)  # Initialize Servo at 50Hz
p.start(1.75)   # Send servo to starting position

def distance():     # Find Ultrasonic Distance Value
    GPIO.output(GPIO_TRIGGER, False)  # Initialization Window
    time.sleep(0.1)
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
    return distance


roverxpos = float(0)  # x-component of rover position
roverypos = float(0)  # y-component of rover position
recdist = []          # Define list for temporary ultrasonic value storage
xcords = []           # List for X-Components
ycords = []           # List for Y-Components
leftwall = 0          # Left Wall Presence
rightwall = 0         # Right Wall Presence
rotationvalue = float(.00225)  # Distance moved each increment
roverangle = 90  # Angle of Rover Relative to Starting Position

def roverscan():  # Collects 18 distances from ultrasonic
    time.sleep(1)
    for i in range(18):  # Run 18 times (for each position)
        p.ChangeDutyCycle(angles[i])  # pull angle reference from list
        time.sleep(.1)
        recdist.append(distance())  # Store distance in recdist list
    print ("Scan Complete")


def componentcompute():  # Converts distance data into x&y vector components
    global roverxpos, roverypos, roverangle
    angl = 0
    for i in range(18):  # Read list & index component values
        if recdist[i] > 95:
            angl = angl + 10
            xcords.append(0)
            ycords.append(0)
        elif recdist[i] < 20:
            angl = angl + 10
            xcords.append(0)
            ycords.append(0)
        else:
            xcord = float(math.cos(math.radians(angl + (roverangle - 90))) * recdist[i]+roverxpos)  # M*Cos(theta) = X-Comp
            ycord = float(math.sin(math.radians(angl + (roverangle - 90))) * recdist[i]+roverypos)  # M*Sin(theta) = Y-Comp
            xcords.append(xcord)
            ycords.append(ycord)
            angl = angl + 10
    print("Compute Complete")


def currentposition():  # Acquires current rover position
    output_streamx = os.popen('ros2 topic echo --once /odometry/wheels --field pose.pose.position')  # Obtain pos data string form shell
    xnew = float((output_streamx.read().partition("x:")[2]).partition("y:")[0])  # Extract pos value from string
    return xnew


def roverposition():
    global leftwall, rightwall, roverxpos, roverypos, rotationvalue, roverangle
    if recdist[5] <= 25 or recdist[6] <= 35 or recdist[7] <= 45 or recdist[8] <= 55 or recdist[9] <= 65 or recdist[10] <= 55 or recdist[11] <= 45 or recdist[12] <= 35 or recdist[13] <= 26:  # If the rover is close to a wall
        print("Front Wall Detected")
        if recdist[4] < 40:  # Check for wall on right side
            rightwall = bool(1)
        elif recdist[14] < 40:  # Check for wall on left side
            leftwall = bool(1)
        if recdist[8] <= 20 or recdist[9] <= 15 or recdist[10] <=20:  # If object is right in front
            desiredDist = float(-0.057063)  # 20cm Encoder Value
            DesiredPos = currentposition() + desiredDist
            os.system('ros2 topic pub -t 32 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 30')  # Rover Position -20cm
            time.sleep(0.1)
            offset = currentposition() - DesiredPos  # Inaccuracy of Movement, Post to Stored Pos Value
            os.system('ros2 topic pub -t 62 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}" -r 30')  # Turn to the left
            roverxpos += math.cos(math.radians(roverangle)) * (-20 - offset * 0.0028518)  # Rover X-Position  - 20.0cm
            roverypos += math.sin(math.radians(roverangle)) * (-20 - offset * 0.0028518)  # Rover Y-Position + 0.0cm
            roverangle = roverangle + 90
            time.sleep(0.1)
            leftwall, rightwall = 0, 0
            return
        if rightwall == 1 and leftwall == 1:  # If there are walls on both sides, turn around
            os.system('ros2 topic pub -t 118 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}" -r 30')  #turn around 180 degrees
            roverangle = roverangle + 180
            time.sleep(0.1)
            desiredDist = float(0.14259)  # 50cm Encoder Value
            DesiredPos = currentposition() + desiredDist
            os.system('ros2 topic pub -t 92 /cmd_vel geometry_msgs/Twist "{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 30') # Rover Position +50cm
            time.sleep(0.1)
            offset = currentposition() - DesiredPos  # Inaccuracy of Movement, Post to Stored Pos Value
            roverxpos += math.cos(math.radians(roverangle))*(50 + offset*0.0028518)  # Rover X-Position  - 0.0cm
            roverypos += math.sin(math.radians(roverangle))*(50 + offset*0.0028518)  # Rover Y-Position - 50.0cm
            leftwall, rightwall = 0, 0
            return
        if rightwall == 1 and leftwall == 0:  # If left direction is open, turn left
            os.system('ros2 topic pub -t 56 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}" -r 30')  # Turn to the left
            roverangle = roverangle + 90
            time.sleep(0.1)
            desiredDist = float(0.057063)  # 20cm Encoder Value
            DesiredPos = currentposition() + desiredDist
            os.system('ros2 topic pub -t 32 /cmd_vel geometry_msgs/Twist "{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 30')  # Rover Position +20cm
            time.sleep(0.1)
            offset = currentposition() - DesiredPos  # Inaccuracy of Movement, Post to Stored Pos Value
            roverxpos += math.cos(math.radians(roverangle))*(20 + offset*0.0028518)  # Rover X-Position  - 20.0cm
            roverypos += math.sin(math.radians(roverangle))*(20 + offset*0.0028518)  # Rover Y-Position + 0.0cm
            leftwall, rightwall = 0, 0
            return
        if rightwall == 0 and leftwall == 1:  # If right direction is open, turn right
            os.system('ros2 topic pub -t 56 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}" -r 30')  # Turn to the right
            roverangle = roverangle - 90
            time.sleep(0.1)
            desiredDist = float(0.057063)  # 20cm Encoder Value
            DesiredPos = currentposition() + desiredDist
            os.system('ros2 topic pub -t 32 /cmd_vel geometry_msgs/Twist "{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 30')  # Rover Position +20cm
            time.sleep(0.1)
            offset = currentposition() - DesiredPos  # Inaccuracy of Movement, Post to Stored Pos Value
            roverxpos += math.cos(math.radians(roverangle))*(20 + offset*0.0028518)  # Rover X-Position + 20.0cm
            roverypos += math.sin(math.radians(roverangle))*(20 + offset*0.0028518)  # Rover Y-Position + 0.0cm
            leftwall, rightwall = 0, 0
            return
        if rightwall == 0 and leftwall == 0:  # If Neither Wall Obstructed, just turn left
            os.system('ros2 topic pub -t 56 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}" -r 30')
            roverangle = roverangle + 90
            time.sleep(0.1)
            desiredDist = float(0.057063)  # 20cm Encoder Value
            DesiredPos = currentposition() + desiredDist
            os.system('ros2 topic pub -t 32 /cmd_vel geometry_msgs/Twist "{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 30')  #Rover Position +20cm
            time.sleep(0.1)
            offset = currentposition() - DesiredPos
            roverxpos += math.cos(math.radians(roverangle))*(20 + offset*0.0028518)  # Rover X-Position - 20.0cm
            roverypos += math.sin(math.radians(roverangle))*(20 + offset*0.0028518)  # Rover Y-Position + 0.0cm
            return
    if (recdist[1] <= 30):  # About to Scrape a wall
        os.system('ros2 topic pub -t 25 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}" -r 30')   # Turn left 45 degrees
        time.sleep(0.1)
        os.system('ros2 topic pub -t 32 /cmd_vel geometry_msgs/Twist "{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 30')  # Rover Position +20cm
        time.sleep(0.1)
        os.system('ros2 topic pub -t 25 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}" -r 30')  # Turn right 45 degrees
        roverxpos += 14.142
        roverypos += 14.142
        return
    if (recdist[17] <= 30):  # About to Scrape a wall
        os.system('ros2 topic pub -t 25 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}" -r 30')  # Turn right 45 degrees
        time.sleep(0.1)
        os.system('ros2 topic pub -t 32 /cmd_vel geometry_msgs/Twist "{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 30')  # Rover Position +20cm
        time.sleep(0.1)
        os.system('ros2 topic pub -t 25 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}" -r 30')   # Turn left 45 degrees
        roverxpos += 14.142
        roverypos += 14.142
        return
    else:  # If no Obstruction Detected, Advance Forward 50cm
        print("No Wall Detected", rotationvalue)
        desiredDist = float(0.14259)  # 50cm Encoder Value
        DesiredPos = currentposition() + desiredDist
        os.system('ros2 topic pub -t 92 /cmd_vel geometry_msgs/Twist "{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 30')  # Rover Position +x 50cm
        offset = currentposition() - DesiredPos  # Inaccuracy of Movement, Post to Stored Pos Value
        roverxpos += math.cos(math.radians(roverangle)) * (50 + offset * 0.0028518)  # Rover X-Position + 50.0cm
        roverypos += math.sin(math.radians(roverangle)) * (50 + offset * 0.0028518)  # Rover Y-Position + 0.0cm
        return


def main():
    roverscan()  # Obtain Coordinates
    componentcompute()
    roverposition()


for i in range(8):  # Run Program for 75 Cycles
    main()
    tp.plot(xcords, ycords)
    while len(recdist) > 0: recdist.pop()
subprocess.call(['/usr/bin/sudo', 'chmod', '777', 'RoomCords.txt'])  # Write Coordinates to File
with open('RoomCords.txt', 'a') as f:
    for i in range(len(xcords)):
        f.write('\n')
        f.write(str(xcords[i]))
        f.write(', ')
        f.write(str(ycords[i]))