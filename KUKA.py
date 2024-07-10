from controller import Robot
import math

# Create the Robot instance.
robot = Robot()

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

pi = 3.1415

#Initial robot position is
x_irp = -2.4
y_irp = 0
z_irp = 0

#Cub position is
x_cp = 1
y_cp = 0.205
z_cp = 0

#Target position is
x_tp = -2.185
y_tp = 0.140
z_tp = -0.813

#Robot's wheel radius is
wheelRadius = 0.05
#Robot's wheel peremeter is
wheelPeremeter = 2*pi*wheelRadius
#distance between the center of the 1st joint and the axis of the front wheel
axis_distance = 0.0845
#distance between the center of the 1st joint and the center of the robot
arm_robotCenter_distance = (0.385-0.084)*0.5 + 0.033

#wheel base dimentions in meters
L= 0.580
W = 0.38 
H = 0.140

#KUKAS's link dimentions
Link = [0.147, 0.155, 0.135, 0.2175]

#joints
q = [0, 0, 0, 0]

#Initializations
# Inizialize base motors.
wheels = []
wheels.append(robot.getMotor("wheel1"))
wheels.append(robot.getMotor("wheel2"))
wheels.append(robot.getMotor("wheel3"))
wheels.append(robot.getMotor("wheel4"))
for wheel in wheels:
    # Activate controlling the motors setting the velocity.
    # Otherwise by default the motor expects to be controlled in force or position,
    # and setVelocity will set the maximum motor velocity instead of the target velocity.
    wheel.setPosition(float('+inf'))

# Initialize arm motors.
armMotors = []
armMotors.append(robot.getMotor("arm1"))
armMotors.append(robot.getMotor("arm2"))
armMotors.append(robot.getMotor("arm3"))
armMotors.append(robot.getMotor("arm4"))
armMotors.append(robot.getMotor("arm5"))
# Set the maximum motor velocity.
armMotors[0].setVelocity(0.2)
armMotors[1].setVelocity(0.5)
armMotors[2].setVelocity(0.5)
armMotors[3].setVelocity(0.5)

# Initialize arm position sensors.
# These sensors can be used to get the current joint position and monitor the joint movements.
armPositionSensors = []
armPositionSensors.append(robot.getPositionSensor("arm1sensor"))
armPositionSensors.append(robot.getPositionSensor("arm2sensor"))
armPositionSensors.append(robot.getPositionSensor("arm3sensor"))
armPositionSensors.append(robot.getPositionSensor("arm4sensor"))
armPositionSensors.append(robot.getPositionSensor("arm5sensor"))

for sensor in armPositionSensors:
    sensor.enable(timestep)
    
wheelPositionSensors = []
wheelPositionSensors.append(robot.getPositionSensor("wheel1sensor"))
wheelPositionSensors.append(robot.getPositionSensor("wheel2sensor"))
wheelPositionSensors.append(robot.getPositionSensor("wheel3sensor"))
wheelPositionSensors.append(robot.getPositionSensor("wheel4sensor"))

for sensor in wheelPositionSensors:
    sensor.enable(timestep)

# Initialize fingers.
fingerMotors = []
fingerMotors.append(robot.getMotor("finger1"))
fingerMotors.append(robot.getMotor("finger2"))

# Initialize finger position sensors.
fingerPositionSensors = []
fingerPositionSensors.append(robot.getPositionSensor("finger1sensor"))
fingerPositionSensors.append(robot.getPositionSensor("finger2sensor"))

for sensor in fingerPositionSensors:
    sensor.enable(timestep)

# Set the maximum motor velocity.
fingerMotors[0].setVelocity(0.03)
fingerMotors[1].setVelocity(0.03)

fingerMinPosition = fingerMotors[0].getMinPosition()
fingerMaxPosition = fingerMotors[0].getMaxPosition()

#revers kinematiks
q[1] = -65*pi/180
q[2] = -25*pi/180
q[3] = math.asin((y_cp - H - Link[0] - Link[1]*math.cos(-65*pi/180)-Link[2]*math.cos((-65-25)*pi/180))/(Link[3]))

#Distance 1
Distance1 = abs(x_cp-x_irp)

arm_length = (Link[1]*math.cos(0.5*pi + q[1]) + Link[2]*math.cos(0.5*pi + q[1] + q[2]) + Link[3]*math.cos(0.5*pi + q[1] + q[2] + q[3] + 0.245))

#the total angle in rads that need the wheels to turn to reach position to pick cube.
rad1 = (Distance1 -  arm_robotCenter_distance -arm_length )*2*pi/wheelPeremeter 

#check for z-axis displacement
if (abs(z_irp - z_cp) != 0):
    wheel1PositionSensor0 = wheelPositionSensors[0].getValue()
    #if the robot is cube's right
    if (z_irp > z_cp):
        # Rotate the robot left.
        wheels[0].setVelocity(5)
        wheels[1].setVelocity(-5)
        wheels[2].setVelocity(5)
        wheels[3].setVelocity(-5)

        # Wait until the motion is completed.
        while robot.step(timestep) != -1:
            print(wheelPositionSensors[0].getValue())
            if (wheelPositionSensors[0].getValue() - 0.87758 > 17.8):
                # Motion of 90deg rotation completed.
                break
        # Stop.
        for wheel in wheels:
            wheel.setVelocity(0.0)
            
        a2 = wheelPositionSensors[1].getValue()
        
        # Move forward.
        for wheel in wheels:
            wheel.setVelocity(5)
        print("move")    
        
        while robot.step(timestep) != -1:
            if (wheelPositionSensors[1].getValue() - a2 > abs(z_irp - z_cp)*2*pi/wheelPeremeter):
                # Motion completed.
                break
        # Stop.
        for wheel in wheels:
            wheel.setVelocity(0.0)
        print("stop")
        
        # Rotate the robot right.
        wheels[0].setVelocity(-5)
        wheels[1].setVelocity(5)
        wheels[2].setVelocity(-5)
        wheels[3].setVelocity(5)

        a3 = wheelPositionSensors[1].getValue()

        while robot.step(timestep) != -1:
            if abs(wheelPositionSensors[1].getValue() - a3) > 17.8:
                # Motion completed.
                break
        # Stop.
        for wheel in wheels:
            wheel.setVelocity(0.0)
            
    #if the robot is at cube's left
    if (z_irp < z_cp):
        # Rotate the robot right.
        wheels[0].setVelocity(-5)
        wheels[1].setVelocity(5)
        wheels[2].setVelocity(-5)
        wheels[3].setVelocity(5)

        while robot.step(timestep) != -1:
            print(wheelPositionSensors[0].getValue())
            if (abs(wheelPositionSensors[0].getValue() - 0.87758) > 17.8):
                # Motion completed.
                break
        # Stop.
        for wheel in wheels:
            wheel.setVelocity(0.0)
            
        a2 = wheelPositionSensors[0].getValue()
        
        # Move forward.
        for wheel in wheels:
            wheel.setVelocity(5)
        print("move")    
        
        while robot.step(timestep) != -1:
            if (wheelPositionSensors[0].getValue() - a2 > abs(z_irp - z_cp)*2*pi/wheelPeremeter):
                # Motion completed.
                break
        # Stop.
        for wheel in wheels:
            wheel.setVelocity(0.0)
        print("stop")
        
        # Rotate the robot left.
        wheels[0].setVelocity(5)
        wheels[1].setVelocity(-5)
        wheels[2].setVelocity(5)
        wheels[3].setVelocity(-5)

        a3 = wheelPositionSensors[0].getValue()

        # Wait for a fixed amount to step that the robot rotates.
        while robot.step(timestep) != -1:
            if abs(wheelPositionSensors[0].getValue() - a3) > 17.8:
                # Motion completed.
                break
        # Stop.
        for wheel in wheels:
            wheel.setVelocity(0.0)
    a4 = wheelPositionSensors[0].getValue()    
a4 = 0
# Move forward.
for wheel in wheels:
    wheel.setVelocity(14.81)
    
# Move arm and open gripper.
armMotors[0].setPosition(0)
armMotors[1].setPosition(q[1])
armMotors[2].setPosition(q[2])
armMotors[3].setPosition(0)
fingerMotors[0].setPosition(0.025)
fingerMotors[1].setPosition(0.025)

# Wait until the robot is in front of the box and the joints are in right position
while robot.step(timestep) != -1:
    if (wheelPositionSensors[0].getValue()-a4 > rad1 - 0.25 ) and abs(armPositionSensors[3].getValue()) < 0.01 and abs(armPositionSensors[2].getValue())+q[2] < 0.01 and abs(armPositionSensors[1].getValue())+q[1] < 0.01:
        # Motion completed.
        break

#cube's distance from arm's center when the robot is close to cube and has stopped 
arm_cube_distance = wheelPeremeter + axis_distance + wheelRadius

wheel1PositionSensor = wheelPositionSensors[0].getValue()

# Stop moving forward.
for wheel in wheels:
    wheel.setVelocity(0.0)
    
#calculating the distance in rads from start to the point it stops for the first time
#bcs whene it moves backwards the sensor dicreases.
temp1 = wheelPositionSensors[0].getValue()

armMotors[3].setPosition(q[3] + 0.245)
# Monitor the arm joint position to detect when the motion is completed
while robot.step(timestep) != -1:
    if armPositionSensors[3].getValue() < q[3] + 0.245 + 0.01:
        # Motion completed.
        break

# Close gripper.
fingerMotors[0].setPosition(0)
fingerMotors[1].setPosition(0)

# Wait until the gripper is closed.
while robot.step(timestep) != -1:
    print(fingerPositionSensors[0].getValue())
    #0.01342 is fingerPositionSensors's value when the gripper is closed
    if abs(fingerPositionSensors[0].getValue()-0.01342) < 0.001:
        # Motion completed.
        break

# Lift arm so it will not be crushed into the obstacle.
armMotors[1].setPosition(-1)

# Wait until the arm is lifted.
while robot.step(timestep) != -1:
    if abs(armPositionSensors[1].getValue() + 1) < 0.01:
        # Motion completed.
        break

#Distance 2
Distance2 = abs(x_tp - x_cp) - arm_cube_distance - arm_robotCenter_distance
rad2 = Distance2*2*pi/wheelPeremeter - 0.25

# Move backward.
for wheel in wheels:
    wheel.setVelocity(-14.81)

armMotors[3].setPosition(0)

# Wait until the robot is in place.
while robot.step(timestep) != -1:
    if abs(wheelPositionSensors[0].getValue() < temp1 - rad2 ):
        # Motion completed.
        break
    
wheel1PositionSensor = wheelPositionSensors[0].getValue()

# Stop moving backward.
for wheel in wheels:
    wheel.setVelocity(0.0)

# Rotate the robot right.
wheels[0].setVelocity(5)
wheels[1].setVelocity(-5)
wheels[2].setVelocity(5)
wheels[3].setVelocity(-5)

a = wheelPositionSensors[0].getValue()
# Move arm.
armMotors[1].setPosition(q[1])

while robot.step(timestep) != -1:
    if abs(wheelPositionSensors[0].getValue() - wheel1PositionSensor ) > 17.8:
        # Motion completed.
        break

#Distance 3
Distance3 = abs(z_cp-z_tp) - (Link[1]*math.cos(0.5*pi+q[1]) + Link[2]*math.cos(0.5*pi + q[1] + q[2]) + Link[3]*math.cos(0.5*pi + q[1] + q[2] + q[3] + 0.245) + arm_robotCenter_distance)
rad3=Distance3*2*pi/wheelPeremeter 

for wheel in wheels:
    wheel.setVelocity(0)

wheel1PositionSensor2 = wheelPositionSensors[0].getValue()

# Move forward.
for wheel in wheels:
    wheel.setVelocity(5)

while robot.step(timestep) != -1:
    if (wheelPositionSensors[0].getValue() > wheel1PositionSensor2 + rad3):
        # Motion completed.
        break
# Stop.
for wheel in wheels:
    wheel.setVelocity(0.0)
print("stop")

#armMotors[3] set to final position
armMotors[3].setPosition(q[3] + 0.245)
while robot.step(timestep) != -1:
    if armPositionSensors[3].getValue() < q[3] + 0.245 + 0.01:
        # Motion completed.
        break

# Open gripper.
fingerMotors[0].setPosition(fingerMaxPosition)
fingerMotors[1].setPosition(fingerMaxPosition)
