from controller import Robot
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


# defining constants
MAX_VELOCITY = 6.28
PI = np.pi


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

# defining Motor wheels
leftWheel = robot.getDevice("LeftWheel motor")
rightWheel = robot.getDevice("RightWheel motor")
wheels = [leftWheel, rightWheel]  # Grouping the wheels for loops

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

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    # Encoder values indicate the angular position of the wheel in radians
    wl = (encoderValues[0] - oldEncoderValues[0])/delta_t
    wr = (encoderValues[1] - oldEncoderValues[1])/delta_t

    return wl, wr


def get_robot_speeds(wl, wr, r, d):
    u = r/2.0 * (wr + wl)
    w = r/d * (wr - wl)

    return u, w


def linear_angular_2_wl_wr(u, w):
    wl = (2*u + w*D)/(2*R)
    wr = (2*u - w*D)/(2*R)
    return wl, wr


def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
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

    turn_N_degree(90)


def turning_left():
    turn_N_degree(-90)


def turning_around():
    turn_N_degree(180)


def turning_smooth_right():
    turn_N_degree(30)


def turning_smooth_left():
    turn_N_degree(-30)


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
    u = r/2.0 * (wr + wl)
    w = r/d * (wr - wl)

    return u, w


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


NEAR = 0.068
############################################
#####  MAIN PROGRAM LOOP STARTS HERE   #####
############################################
while robot.step(timestep) != -1:
    # Updating the values from sensors
    updating_robot()

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
