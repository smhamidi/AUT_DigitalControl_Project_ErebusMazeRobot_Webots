from controller import Robot, DistanceSensor, GPS, Camera, Motor
# import cv2 as cv
import numpy as np
from math import pi


robot = Robot() # Robot object
timestep = int(robot.getBasicTimeStep())

MAX_VELOCITY = 6.28

# defining Distnace sensors
FDS = robot.getDevice("FrontDistance") # Front distance sensor
RDS = robot.getDevice("RightDistance") # Right distance sensor
LDS = robot.getDevice("LeftDistance") # Left distnace sensor

# defining Location sensor
location = robot.getDevice("GpsSensor") # Location for GPS sensor'

# defining Color sensor
colorSensor = robot.getDevice("Color") # Color Sensor

# defining Camera sensors
FCamera = robot.getDevice("FrontCamera")
RCamera = robot.getDevice("RightCamera")
LCamera = robot.getDevice("LeftCamera")

# defining Motor wheels
leftWheel = robot.getDevice("LeftWheel motor")
rightWheel = robot.getDevice("RightWheel motor")

# defining robot emitter
emitter = robot.getDevice("emitter")
# enabling Distance sensors
FDS.enable(timestep)
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

# Setting the position and velocity for the wheels
leftWheel.setPosition(float("inf"))
rightWheel.setPosition(float("inf"))

leftWheel.setVelocity(0.0)
rightWheel.setVelocity(0.0)

# defining FUNCTIONS
def delay(ms):
    initTime = robot.getTime()
    while robot.step(timestep) != -1:
        if(robot.getTime() - initTime) * 1000.0 > ms:
            break # break when time finished

def get_color():
    image = colorSensor.getImage() # Retrieve the image frame
    return colorSensor.imageGetGray(image, colorSensor.getWidth(), 0, 0) # return gray converted image

def turn(angle, colorCoeff):
    leftWheel.setVelocity(-0.6 * MAX_VELOCITY)
    rightWheel.setVelocity(0.6 * MAX_VELOCITY)
    delay(colorCoeff * 595)
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)
    delay(int(600/colorCoeff))

colorCoeff = {
    "yellow" : 2,
    "black" : 0,
    "others" : 1
}


# Main while loop for the robot
while robot.step(timestep) != -1:

    # Distnace sensors Value
    fd = FDS.getValue()
    rd = RDS.getValue()
    ld = LDS.getValue()

    # Location Values
    x = location.getValues()[0]
    y = location.getValues()[1]
    z = location.getValues()[2]

    # Color sensor values
    image = colorSensor.getImage() # Retrieve the image frame

    r = colorSensor.imageGetRed(image, 1, 0, 0)
    g = colorSensor.imageGetGreen(image, 1, 0, 0)
    b = colorSensor.imageGetBlue(image, 1, 0, 0)

    turn(90, colorCoeff["others"])



