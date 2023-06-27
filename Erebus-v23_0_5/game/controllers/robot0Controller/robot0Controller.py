from controller import Robot, DistanceSensor, GPS, Camera, Motor, Gyro
import math
import struct
import numpy
import cv2 as cv
import numpy as np

robot = Robot()  # Robot object
timestep = int(robot.getBasicTimeStep())

delta_t = robot.getBasicTimeStep()/1000.0    # [s]

# Robot pose
# Adjust the initial values to match the initial robot pose in your simulation
x = -0.06    # position in x [m]
y = 0.436    # position in y [m]
phi = 0.0531  # orientation [rad]

# Robot velocity and acceleration
dx = 0.0   # speed in x [m/s]
dy = 0.0   # speed in y [m/s]
ddx = 0.0  # acceleration in x [m/s^2]
ddy = 0.0  # acceleration in y [m/s^2]

# Robot wheel speeds
wl = 0.0    # angular speed of the left wheel [rad/s]
wr = 0.0    # angular speed of the right wheel [rad/s]

# Robot linear and angular speeds
u = 0.0    # linear speed [m/s]
w = 0.0    # angular speed [rad/s]

# e-puck Physical parameters for the kinematics model (constants)
R = 0.0205    # radius of the wheels: 20.5mm [m]
D = 0.0565    # distance between the wheels: 52mm [m]
# distance from the center of the wheels to the point of interest [m]
A = 0.05

# defining flag for sending message, we assume that we haven't seen a victim
# messageSent = False

# defining constants
MAX_VELOCITY = 6.28
PI = np.pi

BLACKHOLE_COLOR = b'...\xff'  # b')))\xff' b'***\xff'

# SWAMP_COLOR = b'o\xbd\xdd\xff'
# EXIT_COLOR = b'%\xfc%\xff'

# START_TIME = 0
# DURATION_TIME = 0 #if a command needs a duration to complete we can use this constant MiliSeconds

# defining Distnace sensors
FDS = robot.getDevice("FrontDistance")  # Front distance sensor
FRDS = robot.getDevice("FrontRightDistance")  # Front Right distance sensor
FLDS = robot.getDevice("FrontLeftDistance")  # Front Left distance sensor
RDS = robot.getDevice("RightDistance")  # Right distance sensor
LDS = robot.getDevice("LeftDistance")  # Left distnace sensor
distanceSensors = [FDS, FRDS, FLDS, RDS, LDS]


# defining Location sensor
location = robot.getDevice("GpsSensor")  # Location for GPS sensor'

# defining Color sensor
colorSensor = robot.getDevice("Color")  # Color Sensor

# defining Camera sensors
FCamera = robot.getDevice("FrontCamera")
RCamera = robot.getDevice("RightCamera")
LCamera = robot.getDevice("LeftCamera")
# cameras = [FCamera, RCamera, LCamera] # grouping all cameras for loops

# defining Motor wheels
leftWheel = robot.getDevice("LeftWheel motor")
rightWheel = robot.getDevice("RightWheel motor")
wheels = [leftWheel, rightWheel]  # Grouping the wheels for loops
# wheelsSpeed = [MAX_VELOCITY, MAX_VELOCITY]

# defining motor Encoders
rightEncoder = robot.getDevice("RightWheel sensor")
leftEncoder = robot.getDevice("LeftWheel sensor")
encoderSensors = [leftEncoder, rightEncoder]
oldEncoderValues = list()

# defining robot emitter
emitter = robot.getDevice("emitter")

# defining robot Gyro
gyroScope = robot.getDevice("GyroSensor")

# enabling Distance sensors
FDS.enable(timestep)
FRDS.enable(timestep)
FLDS.enable(timestep)
RDS.enable(timestep)
LDS.enable(timestep)

# enabling GPS sensor for location
location.enable(timestep)

# enabling Color Sensor
colorSensor.enable(timestep)

# enabling Camera sensors
FCamera.enable(timestep)
RCamera.enable(timestep)
LCamera.enable(timestep)

# enabling Gyro Sensor
gyroScope.enable(timestep)

# enabling encoder sensors
rightEncoder.enable(timestep)
leftEncoder.enable(timestep)

# # enabling Recognition for the cameras
# FCamera.recognitionEnable(timestep)
# RCamera.recognitionEnable(timestep)
# LCamera.recognitionEnable(timestep)


# Setting the position and velocity for the wheels
leftWheel.setPosition(float("inf"))
rightWheel.setPosition(float("inf"))

leftWheel.setVelocity(0.0)
rightWheel.setVelocity(0.0)

# starting time of the loop for getting the duration if needed
startPlayingTime = robot.getTime()

#####################################
#####  PID Class  STARTS HERE   #####
#####################################


class PID:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0

    def update(self, current_value, delta_time):

        error = self.setpoint - current_value
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error

        print(f"Error: {error}\toutput: {output}")
        print(f"integral{self.Ki * self.integral}")
        return output


#################################################
#####  MAIN PROGRAM FUNCTIONS STARTS HERE   #####
#################################################
#######################################################################
# Robot Localization functions - option 1
# Author: Felipe N. Martins

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    """Computes speed of the wheels based on encoder readings"""
    # Encoder values indicate the angular position of the wheel in radians
    wl = (encoderValues[0] - oldEncoderValues[0])/delta_t
    wr = (encoderValues[1] - oldEncoderValues[1])/delta_t

    return wl, wr


def get_robot_speeds(wl, wr, r, d):
    """Computes robot linear and angular speeds"""
    u = r/2.0 * (wr + wl)
    w = r/d * (wr - wl)

    return u, w


def linear_angular_2_wl_wr(u, w):
    wl = (2*u + w*D)/(2*R)
    wr = (2*u - w*D)/(2*R)
    return wl, wr


def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    """Updates robot pose based on heading and linear and angular speeds"""
    delta_phi = w * delta_t
    phi = phi_old + delta_phi
    phi_avg = (phi_old + phi)/2
    if phi >= np.pi:
        phi = phi - 2*np.pi
    elif phi < -np.pi:
        phi = phi + 2*np.pi

    delta_x = u * np.cos(phi_avg) * delta_t
    delta_y = u * np.sin(phi_avg) * delta_t
    x = x_old + delta_x
    y = y_old + delta_y

    return x, y, phi


#######################################################################
def getColor():
    img = colorSensor.getImage()    # Grab color sensor camera's image view
    # Return grayness of the only pixel (0-255)
    return colorSensor.imageGetGray(img, colorSensor.getWidth(), 0, 0)


def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(timestep) != -1:
        # If time elapsed (converted into ms) is greater than value passed in
        if (robot.getTime() - initTime) * 1000.0 > ms:
            break
        else:
            avoidTiles()


def turning_right():

    # speeding left wheel more to turn right
    leftWheel.setVelocity(0.6 * MAX_VELOCITY)
    rightWheel.setVelocity(-0.6 * MAX_VELOCITY)
    delay(595)
    leftWheel.setVelocity(0 * MAX_VELOCITY)
    rightWheel.setVelocity(0 * MAX_VELOCITY)
    delay(600)


def turning_left():
    leftWheel.setVelocity(-0.6 * MAX_VELOCITY)
    rightWheel.setVelocity(0.6 * MAX_VELOCITY)
    delay(595)
    leftWheel.setVelocity(0 * MAX_VELOCITY)
    rightWheel.setVelocity(0 * MAX_VELOCITY)
    delay(600)


def turning_around(color):
    if 170 < color < 180:
        print("yellow")
        leftWheel.setVelocity(-1 * MAX_VELOCITY)
        rightWheel.setVelocity(1 * MAX_VELOCITY)
        delay(2200)
        leftWheel.setVelocity(0 * MAX_VELOCITY)
        rightWheel.setVelocity(0 * MAX_VELOCITY)
        delay(400)
    else:
        leftWheel.setVelocity(-1 * MAX_VELOCITY)
        rightWheel.setVelocity(1 * MAX_VELOCITY)
        delay(650)
        leftWheel.setVelocity(0 * MAX_VELOCITY)
        rightWheel.setVelocity(0 * MAX_VELOCITY)
        delay(400)


def turning_smooth_right():
    leftWheel.setVelocity(0.6 * MAX_VELOCITY)
    rightWheel.setVelocity(-0.6 * MAX_VELOCITY)
    delay(183)
    leftWheel.setVelocity(0 * MAX_VELOCITY)
    rightWheel.setVelocity(0 * MAX_VELOCITY)
    delay(600)


def turning_smooth_left():
    leftWheel.setVelocity(-0.6 * MAX_VELOCITY)
    rightWheel.setVelocity(0.6 * MAX_VELOCITY)
    delay(183)
    leftWheel.setVelocity(0 * MAX_VELOCITY)
    rightWheel.setVelocity(0 * MAX_VELOCITY)
    delay(600)


def moveForward(color):
    if color < 80:
        leftWheel.setVelocity(0.6 * MAX_VELOCITY)
        rightWheel.setVelocity(0.6 * MAX_VELOCITY)
        delay(180)
        leftWheel.setVelocity(0 * MAX_VELOCITY)
        rightWheel.setVelocity(0 * MAX_VELOCITY)
        delay(200)
    elif 170 < color < 180:
        print("yellow")
        leftWheel.setVelocity(0.6 * MAX_VELOCITY)
        rightWheel.setVelocity(0.6 * MAX_VELOCITY)
        delay(1600)
        leftWheel.setVelocity(0 * MAX_VELOCITY)
        rightWheel.setVelocity(0 * MAX_VELOCITY)
        delay(200)
    else:
        leftWheel.setVelocity(0.6 * MAX_VELOCITY)
        rightWheel.setVelocity(0.6 * MAX_VELOCITY)
        delay(533)
        leftWheel.setVelocity(0 * MAX_VELOCITY)
        rightWheel.setVelocity(0 * MAX_VELOCITY)
        delay(200)

# def get_camera_visible_victims(camera):
#     # getting objects from the camera
#     objects = camera.getRecognitionObjects()

#     victims = list()

#     for object in objects:
#         if object.get_colors() == [1,1,1]:
#             victim_pos = object.get_position()
#             victim_image_pos = object.get_position_on_image()

#             victims.append([victim_pos,victim_image_pos,camera])

#     return victims

# def get_all_visible_victims():
#     # gathering all victims from all cameras
#     allVisibleVictims = list()

#     for camera in cameras:
#         allVisibleVictims.append(get_camera_visible_victims(camera))
#     return allVisibleVictims

# def getObjectDistance(position):
#     # calculating the Euclidean distance to object
#     return math.sqrt((position[0] ** 2) + (position[2] ** 2))

# def getClosestVictim(victims):
#     # getting the closest victim to the robot
#     shortestDistance = 999
#     closestVictim = []

#     for victim in victims:
#         dist = getObjectDistance(victim[0])
#         if dist < shortestDistance:
#             shortestDistance = dist
#             closestVictim = victim

#     return closestVictim

# def turn_right_to_victim():
#     #set left wheel speed
#     wheelsSpeed[0] = 1 * MAX_VELOCITY
#     #set right wheel speed
#     wheelsSpeed[1] = 0.8 * MAX_VELOCITY

# def turn_left_to_victim():
#     #set left wheel speed
#     wheelsSpeed[0] = 0.8 * MAX_VELOCITY
#     #set right wheel speed
#     wheelsSpeed[1] = 1 * MAX_VELOCITY

# def turnToVictim(victim):
#     if(victim[2] == RCamera):
#         turning_right()
#     elif(victim[2] == LCamera):
#         turning_left()
#     else:
#         # [x,y]
#         position_on_image = victim[1]

#         width = FCamera.getWidth()
#         center = width / 2

#         victim_x_position = position_on_image[0]
#         dx = center - victim_x_position

#         if dx < 0:
#             turn_right_to_victim()
#         else:
#             turn_left_to_victim()

# def nearObject(position):
#     return getObjectDistance(position)  < 0.10

# def stop():
#     #set left wheel speed
#     wheelsSpeed[0] = 0
#     #set right wheel speed
#     wheelsSpeed[1] = 0

# def sendMessage(robot_type, v1, v2, v3):
#     message = struct.pack('i i i c', robot_type, v1, v2, v3)
#     emitter.send(message)

# def sendVictimMessage():
#     global messageSent
#     position = location.getValues()

#     if not messageSent:
#         #robot type, position x cm, position z cm, victim type
#         sendMessage(0, int(position[0] * 100), int(position[2] * 100), b'H')
#         messageSent = True

# def stop_at_vicim():
#     # stopping at the closest victim and report
#     global messageSent
#     victims = get_all_visible_victims() #get all the victims the cameras can see


#     foundVictim = False

#     if len(victims) != 0:
#         closest_victim = getClosestVictim(victims)
#         turnToVictim(closest_victim)

#     #if we are near a victim, stop and send a message to the supervisor
#     for victim in victims:
#         if nearObject(victim[0]):
#             stop()
#             sendVictimMessage()
#             foundVictim = True

#     if not foundVictim:
#         messageSent = False

def avoidTiles():
    tileColor = getColor()

    if tileColor < 80:
        move_backwards()


def move_backwards():
    # set left wheel speed
    rightWheel.setVelocity(-0.8 * MAX_VELOCITY)
    # set right wheel speed
    leftWheel.setVelocity(-0.3 * MAX_VELOCITY)
    delay(1500)


def get_robot_speeds(wl, wr, r, d):
    """Computes robot linear and angular speeds"""
    u = r/2.0 * (wr + wl)
    w = r/d * (wr - wl)

    return u, w


NEAR = 0.068


def updating_robot():
    global wl, wr, u, w, x, y, phi, delta_t, R, D, distanceSensors, encoderSensors, oldEncoderValues
    # Update sensor readings
    dsValues = list()
    for sensor in distanceSensors:
        dsValues.append(sensor.getValue())

    encoderValues = list()
    for encoder in encoderSensors:
        encoderValues.append(encoder.getValue())    # [rad]

    # Update old encoder values if not done before
    if len(oldEncoderValues) < 2:
        for encoder in encoderSensors:
            oldEncoderValues.append(encoder.getValue())

    [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)

    # Compute robot linear and angular speeds
    [u, w] = get_robot_speeds(wl, wr, R, D)

    # Compute new robot pose
    [x, y, phi] = get_robot_pose(u, w, x, y, phi, delta_t)
    # phi = phi % (2*PI)


def turn_N_degree(degree):
    global phi, delta_t, wl, wr
    radian = ((degree * PI) / 180) + phi
    if radian >= np.pi:
        radian = radian - 2*np.pi
    elif radian < -np.pi:
        radian = radian + 2*np.pi
    KP = 0.1
    KI = 0.001
    KD = 0.0
    pid = PID(KP, KI, KD, radian)

    # last_angle = phi + 1
    while robot.step(timestep) != -1:
        print(
            f'Sim time: {robot.getTime():.3f}  Pose: x={x:.2f} m, y={y:.2f} m, phi={phi:.4f} rad.')
        # ... (update sensor readings and compute robot speeds)
        updating_robot()
        # Update the PID controller and get the output
        current_angle = phi  # get the current angle from the robot pose
        pid_output = pid.update(current_angle, delta_t)
        # if(abs(current_angle - last_angle) < 0.001):
        #     leftWheel.setVelocity(0.0)
        #     rightWheel.setVelocity(0.0)
        #     print("breaking")
        #     break
        # last_angle = phi

        # Apply the PID output to the robot
        wl = pid_output
        wr = -1 * pid_output
        leftWheel.setVelocity(wl)
        rightWheel.setVelocity(wr)


############################################
#####  MAIN PROGRAM LOOP STARTS HERE   #####
############################################
while robot.step(timestep) != -1:

    updating_robot()
    #######################################################################
    avoidTiles()

    turn_N_degree(90)
    print(
        f'Sim time: {robot.getTime():.3f}  Pose: x={x:.2f} m, y={y:.2f} m, phi={phi:.4f} rad.')
    delay(10000)
    # if(FDS.getValue() < NEAR and LDS.getValue() < NEAR and RDS.getValue() < NEAR):
    #     print("turning around")
    #     turning_around(getColor())
    # elif(FDS.getValue() < NEAR and RDS.getValue() > NEAR):
    #     print("turning right")
    #     turning_right()
    # elif((FDS.getValue() < NEAR and RDS.getValue() < NEAR and FRDS.getValue() > NEAR)):
    #     print("turning smooth right")
    #     turning_smooth_right()
    # elif((FDS.getValue() < NEAR and RDS.getValue() < NEAR and LDS.getValue() > NEAR)):
    #     print("turning left")
    #     turning_left()
    # elif((FDS.getValue() < NEAR and LDS.getValue() < NEAR and FLDS.getValue() > NEAR)):
    #     print("turning smooth left")
    #     turning_smooth_left()
    # else:
    #     print("going forward")
    #     moveForward(getColor())

    # print(f"LDS sensor: {LDS.getValue()}")
    # print(f"RDS sensor: {RDS.getValue()}")
    # print(f"FLDS sensor: {FLDS.getValue()}")
    # print(f"FRDS sensor: {FRDS.getValue()}")
    # print(f"FDS sensor: {FDS.getValue()}")
