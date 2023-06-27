from controller import Robot
import numpy as np

# Create a Robot object
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Define constants
MAX_VELOCITY = 6.28

# Define Distance sensors
FDS = robot.getDevice("FrontDistance")  # Front distance sensor
FRDS = robot.getDevice("FrontRightDistance")  # Front Right distance sensor
FLDS = robot.getDevice("FrontLeftDistance")  # Front Left distance sensor
RDS = robot.getDevice("RightDistance")  # Right distance sensor
LDS = robot.getDevice("LeftDistance")  # Left distance sensor

# Define Location sensor
location = robot.getDevice("GpsSensor")  # Location for GPS sensor'

# Define Color sensor
colorSensor = robot.getDevice("Color")  # Color Sensor

# Define Camera sensors
FCamera = robot.getDevice("FrontCamera")
RCamera = robot.getDevice("RightCamera")
LCamera = robot.getDevice("LeftCamera")

# Define Motor wheels
leftWheel = robot.getDevice("LeftWheel motor")
rightWheel = robot.getDevice("RightWheel motor")

# Define robot emitter
emitter = robot.getDevice("emitter")

# Define robot Gyro
gyroScope = robot.getDevice("GyroSensor")

# Enable Distance sensors
FDS.enable(timestep)
FRDS.enable(timestep)
FLDS.enable(timestep)
RDS.enable(timestep)
LDS.enable(timestep)

# Enable GPS sensor for location
location.enable(timestep)

# Enable Color Sensor
colorSensor.enable(timestep)

# Enable Camera sensors
FCamera.enable(timestep)
RCamera.enable(timestep)
LCamera.enable(timestep)

# Enable Gyro Sensor
gyroScope.enable(timestep)

# Enable Recognition for the cameras
FCamera.recognitionEnable(timestep)
RCamera.recognitionEnable(timestep)
LCamera.recognitionEnable(timestep)

# Set the position and velocity for the wheels
leftWheel.setPosition(float("inf"))
rightWheel.setPosition(float("inf"))

leftWheel.setVelocity(0.0)
rightWheel.setVelocity(0.0)

# Get the start time of the loop for duration calculation if needed
startPlayingTime = robot.getTime()

#################################################
#####  MAIN PROGRAM FUNCTIONS STARTS HERE   #####
#################################################

# Define function to get color from color sensor


def getColor():
    img = colorSensor.getImage()    # Grab color sensor camera's image view
    # Return grayness of the only pixel (0-255)
    return colorSensor.imageGetGray(img, colorSensor.getWidth(), 0, 0)

# Define function to create a delay


def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(timestep) != -1:
        # If time elapsed (converted into ms) is greater than value passed in
        if (robot.getTime() - initTime) * 1000.0 > ms:
            break
        else:
            avoidTiles()

# Define function for turning right


def turning_right():
    # Speed up left wheel more to turn right
    leftWheel.setVelocity(0.6 * MAX_VELOCITY)
    rightWheel.setVelocity(-0.6 * MAX_VELOCITY)
    delay(595)
    leftWheel.setVelocity(0 * MAX_VELOCITY)
    rightWheel.setVelocity(0 * MAX_VELOCITY)
    delay(600)

# Define function for turning left


def turning_left():
    leftWheel.setVelocity(-0.6 * MAX_VELOCITY)
    rightWheel.setVelocity(0.6 * MAX_VELOCITY)
    delay(595)
    leftWheel.setVelocity(0 * MAX_VELOCITY)
    rightWheel.setVelocity(0 * MAX_VELOCITY)
    delay(600)

# Define function for turning around based on color


def turning_around(color):
    # Delays for Yellow tiles (Swamps) are different
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

# Define function for smooth right turn


def turning_smooth_right():
    leftWheel.setVelocity(0.6 * MAX_VELOCITY)
    rightWheel.setVelocity(-0.6 * MAX_VELOCITY)
    delay(183)
    leftWheel.setVelocity(0 * MAX_VELOCITY)
    rightWheel.setVelocity(0 * MAX_VELOCITY)
    delay(600)

# Define function for smooth left turn


def turning_smooth_left():
    leftWheel.setVelocity(-0.6 * MAX_VELOCITY)
    rightWheel.setVelocity(0.6 * MAX_VELOCITY)
    delay(183)
    leftWheel.setVelocity(0 * MAX_VELOCITY)
    rightWheel.setVelocity(0 * MAX_VELOCITY)
    delay(600)

# Define function for moving forward based on color


def moveForward(color):
    if 170 < color < 180:
        # Going forward delays are different for Yellow(Swamps) tiles
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

# Define function to avoid Black Holes


def avoidTiles():
    tileColor = getColor()

    if tileColor < 80:
        move_backwards()

# Define function for moving backwards
# We will move backward if we encounter Black Holes


def move_backwards():
    # Set left wheel speed
    rightWheel.setVelocity(-0.8 * MAX_VELOCITY)
    # Set right wheel speed
    leftWheel.setVelocity(-0.3 * MAX_VELOCITY)
    delay(1500)


NEAR = 0.068

########################################################################
#####  Object Detection Starts Here (IF CAMERAS HAD RECOGNITION)   #####
########################################################################
# defining flag for sending message, we assume that we haven't seen a victim
# messageSent = False

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

############################################
#####  MAIN PROGRAM LOOP STARTS HERE   #####
############################################

while robot.step(timestep) != -1:
    # Avoid tiles
    avoidTiles()

    # Check sensor values and perform appropriate action
    if (FDS.getValue() < NEAR and LDS.getValue() < NEAR and RDS.getValue() < NEAR):
        print("turning around")
        turning_around(getColor())
    elif (FDS.getValue() < NEAR and RDS.getValue() > NEAR):
        print("turning right")
        turning_right()
    elif ((FDS.getValue() < NEAR and RDS.getValue() < NEAR and FRDS.getValue() > NEAR)):
        print("turning smooth right")
        turning_smooth_right()
    elif ((FDS.getValue() < NEAR and RDS.getValue() < NEAR and LDS.getValue() > NEAR)):
        print("turning left")
        turning_left()
    elif ((FDS.getValue() < NEAR and LDS.getValue() < NEAR and FLDS.getValue() > NEAR)):
        print("turning smooth left")
        turning_smooth_left()
    else:
        print("going forward")
        moveForward(getColor())

    # Print sensor values for debugging
    print(f"LDS sensor: {LDS.getValue()}")
    print(f"RDS sensor: {RDS.getValue()}")
    print(f"FLDS sensor: {FLDS.getValue()}")
    print(f"FRDS sensor: {FRDS.getValue()}")
    print(f"FDS sensor: {FDS.getValue()}")
