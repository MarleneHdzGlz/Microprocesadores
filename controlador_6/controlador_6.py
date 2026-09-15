from controller import Robot

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']

for name in wheelsNames:
    wheels.append(robot.getDevice(name))
    
speed = -1.5
for wheel in wheels:
    wheel.setPosition(float('inf'))
    wheel.setVelocity(speed)

ds = []
dsNames = ['ds_right', 'ds_left']
for i in range(2):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)
    
avoidObstacleCounter = 0
while robot.step(TIME_STEP) != -1:

    leftSpeed = 1.0
    rightSpeed = 1.0
    if avoidObstacleCounter > 0:
        avoidObstacleCounter -= 1
        leftSpeed = 1.0
        rightSpeed = -1.0
    else:  # read sensors
        for i in range(2):
            if ds[i].getValue() < 950.0:
                avoidObstacleCounter = 100
                
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)
    
    print(leftSpeed, rightSpeed)