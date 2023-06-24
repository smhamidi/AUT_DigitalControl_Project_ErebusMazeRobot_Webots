from controller import Robot, DistanceSensor, GPS, Camera, Motor
import math
import struct

robot = Robot() # Robot object
timestep = int(robot.getBasicTimeStep())

# defining flag for sending message, we assume that we haven't seen a victim
messageSent = False

# defining constants
MAX_VELOCITY = 6.28

BLACKHOLE_COLOR = b'\n\n\n\xff'
SWAMP_COLOR = b'\x12\x1b \xff'
EXIT_COLOR = b'\x10\xb8\x10\xff'

START_TIME = 0
DURATION_TIME = 0 #if a command needs a duration to complete we can use this constant

# defining Distnace sensors
FDS = robot.getDevice("FrontDistance") # Front distance sensor
FRDS = robot.getDevice("FrontRightDistance") # Front Right distance sensor
FLDS = robot.getDevice("FrontLeftDistance") # Front Left distance sensor
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
cameras = [FCamera, RCamera, LCamera] # grouping all cameras for loops

# defining Motor wheels
leftWheel = robot.getDevice("LeftWheel motor")
rightWheel = robot.getDevice("RightWheel motor")
wheels = [leftWheel, rightWheel] # Grouping the wheels for loops
wheelsSpeed = [MAX_VELOCITY, MAX_VELOCITY]

# defining robot emitter
emitter = robot.getDevice("emitter")

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

# enabling Recognition for the cameras
FCamera.recognitionEnable(timestep)
RCamera.recognitionEnable(timestep)
LCamera.recognitionEnable(timestep)


# Setting the position and velocity for the wheels
leftWheel.setPosition(float("inf"))
rightWheel.setPosition(float("inf"))

leftWheel.setVelocity(0.0)
rightWheel.setVelocity(0.0)

#starting time of the loop for getting the duration if needed
startPlayingTime = robot.getTime()

#################################################
#####  MAIN PROGRAM FUNCTIONS STARTS HERE   #####
#################################################

def turning_right():
    # speeding left wheel more to turn right
    wheelsSpeed[0] = 0.6 * MAX_VELOCITY
    wheelsSpeed[1] = -0.2 * MAX_VELOCITY
   
def turning_left():
    # speeding right wheel more to turn left
    wheelsSpeed[0] = -0.2 * MAX_VELOCITY
    wheelsSpeed[1] = 0.6 * MAX_VELOCITY

def turning_around():
    # speeding right and left wheel in reverse to turn around
    wheelsSpeed[0] = -0.6 * MAX_VELOCITY
    wheelsSpeed[1] = 0.6 * MAX_VELOCITY

def get_camera_visible_victims(camera):
    # getting objects from the camera
    objects = camera.getRecognitionObjects()

    victims = list()

    for object in objects:
        if object.get_colors() == [1,1,1]:
            victim_pos = object.get_position()
            victim_image_pos = object.get_position_on_image()

            victims.append([victim_pos,victim_image_pos,camera])

    return victims

def get_all_visible_victims():
    # gathering all victims from all cameras
    allVisibleVictims = list()

    for camera in cameras:
        allVisibleVictims.append(get_camera_visible_victims(camera))
    return allVisibleVictims

def stop_at_vicim():
    # stopping at the closest victim and report
    global messageSent
    victims = get_all_visible_victims() #get all the victims the cameras can see


    foundVictim = False

    if len(victims) != 0:
        closest_victim = getClosestVictim(victims)
        turnToVictim(closest_victim)

    #if we are near a victim, stop and send a message to the supervisor
    for victim in victims:
        if nearObject(victim[0]):
            stop()
            sendVictimMessage()
            foundVictim = True

    if not foundVictim:
        messageSent = False

############################################
#####  MAIN PROGRAM LOOP STARTS HERE   #####
############################################

while robot.step(timestep) != -1:
    if (robot.getTime() - START_TIME) < DURATION_TIME:
        pass
    else:
        START_TIME = 0
        DURATION_TIME = 0

        wheelsSpeed[0] = MAX_VELOCITY
        wheelsSpeed[1] = MAX_VELOCITY

        # If we are too close to one of the left sensors then turn right
        if( (LDS.getValue()>80) or (FLDS.getValue()>80)):
            turning_right()

        # If we are too close to one of the right sensors then turn left
        elif( (RDS.getValue()>80) or (FRDS.getValue()>80)):
            turning_left()
        
        # If we are too close to a wall or an obstacle then spin
        if FDS.getValue() > 80:
            turning_around()

        # If we are close to a victim then stop and send message
        stop_at_vicim()

        # If we are close to a swamp or blackHole then go backward
        # our priority is to find the victims meaning that if there
        # is a swamp that is near a victim we will go through swamp
        avoidTiles()

        if (robot.getTime() - startPlayingTime) > 6*60:
            if colorSensor.getImage() == EXIT_COLOR:
                sendMessage(0,0,0,b'E')

        leftWheel.setVelocity(wheelsSpeed[0])
        rightWheel.setVelocity(wheelsSpeed[1])
