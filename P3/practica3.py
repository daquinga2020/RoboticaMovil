from GUI import GUI
from HAL import HAL
import math
import numpy as np

def checkpoint(goalx):

    return math.fabs(goalx) <= 0.2

def detect_front_obst(laser_data):

    front = []
    ang = []

    for i in range(80, 100):
        if laser_data[i][0] < 4:
            front.append(laser_data[i][0])
            ang.append(laser_data[i][1])

    if front:
        return (True,min(front),ang[front.index(min(front))])

    return (False, 0)

def parse_laser_data (laser_data):

    laser = []
    i = 0
    while (i < 180):
        dist = laser_data.values[i]
        if dist > 10:
            dist = 10
        angle = math.radians(i-90) # because the front of the robot is -90 degrees
        laser += [(dist, angle)]
        i+=1

    return laser

def absolute2relative (x_abs, y_abs, robotx, roboty, robott):

    # robotx, roboty are the absolute coordinates of the robot
    # robott is its absolute orientation
    # Convert to relatives
    dx = x_abs - robotx
    dy = y_abs - roboty

    # Rotate with current angle
    x_rel = dx * math.cos (-robott) - dy * math.sin (-robott)
    y_rel = dx * math.sin (-robott) + dy * math.cos (-robott)

    return x_rel, y_rel



alpha = 1
beta = 1
vel_lin = 3
counter = 0
extra_W = 0

while True:

    alpha = 1
    beta = 1
    laser_right = []
    laser_left = []
    laser_front = []

    obsForce = [0,0]

    laser_data = HAL.getLaserData()
    laser = parse_laser_data(laser_data)

    laser_vectorized = []
    for i in range(180):

        x = laser[i][0] * math.cos (laser[i][1])# * -1
        y = laser[i][0] * math.sin (laser[i][1])# * -1
        v = (x, y)
        laser_vectorized += [v]
    
    laser_mean = np.mean (laser_vectorized, axis = 0)
    module_laser_mean = math.sqrt((laser_mean[0])**2 + (laser_mean[1])**2)

    ################ Car coords ###############
    robotx = HAL.getPose3d().x
    roboty = HAL.getPose3d().y
    robott = HAL.getPose3d().yaw
    #############################################

    ########## Goal coords ###############
    currentTarget = GUI.map.getNextTarget()
    GUI.map.targetx = currentTarget.getPose().x
    GUI.map.targety = currentTarget.getPose().y

    # Current target
    target = [currentTarget.getPose().x, currentTarget.getPose().y]
    GUI.showLocalTarget(target)

    ########### Checking goal ##############
    targetx_transf,targety_transf = absolute2relative(currentTarget.getPose().x, currentTarget.getPose().y, robotx, roboty, robott)

    if checkpoint(targetx_transf):
        currentTarget.setReached(True)
    ############################################################

    ########## Forces ##############

    # Obstacles direction (black line in the image below)
    if module_laser_mean < 2.4:
        obsForce = [-1/laser_mean[0], laser_mean[1]]

    front_obs = detect_front_obst(laser)

    wall = laser[0][0] <= 0.3 or laser[179][0] <= 0.3

    if front_obs[0]:
        alpha = 0.9
        beta = 1.15

        if front_obs[1] <= 0.45:
            vel_lin=0.5
            alpha = 0.75
            direction = front_obs[2]/math.fabs(front_obs[2])
            counter = 5
            extra_W = -1.46*direction
        else:
            if counter != 0:
                counter -= 1
                vel_lin += 0.5
            else:
                extra_W = 0

    # Average direction (red line in the image below)
    # Limit the force of attraction
    if targetx_transf > 6:
        targetx_transf = 6
    if math.fabs(targety_transf) > 6:
        direction  = math.fabs(targety_transf)/targety_transf
        targety_transf = 6*direction

    module_target = math.sqrt((targetx_transf)**2 + (targety_transf)**2)

    if module_target < 1:
        module_target = 1

    avgForce = [targetx_transf, targety_transf]
    ##############################################################

    # Car direction  (green line in the image below)
    direction_x = alpha*avgForce[0]/module_target + obsForce[0]*beta
    direction_y = alpha*avgForce[1]/module_target + obsForce[1]*beta

    module_direction = math.sqrt((direction_x)**2 + (direction_y)**2)
    carForce = [direction_x*module_target, direction_y*module_target]

    GUI.showForces(carForce, avgForce, obsForce)
    ########################################

    if front_obs[0] or module_target <= 1.5:
        if vel_lin > 3:
            vel_lin -= 0.34
    else:
        if vel_lin < 4.5:
            vel_lin += 0.04

    if counter == 0:    
        w = math.atan2(direction_y, direction_x) + extra_W
    else:
        w = extra_W

    if math.fabs(w) < 0.4:
        w = 0

    HAL.setV(vel_lin)
    HAL.setW(w)

    img = HAL.getImage()
    GUI.showImage(img)