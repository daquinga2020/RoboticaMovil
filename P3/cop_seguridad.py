from GUI import GUI
from HAL import HAL
import time
import math
import numpy as np
# Enter sequential code!

def checkpoint(robotx, roboty, goalx, goaly):
    return math.fabs(goalx) <= 0.2


def detect_front_obst(laser_data):
    
    front_r = []
    front = []
    ang = []
    detected = False
    for i in range(80, 100):
        """if i < 90 and laser_data[i][0] < 3.5:
            front_r.append(laser_data[i][0])
        elif i >= 90 and laser_data[i][0] < 3.5:
            front_l.append(laser_data[i][0])"""
        if laser_data[i][0] < 4:
            front.append(laser_data[i][0])
            ang.append(laser_data[i][1])
            #return (True,laser_data[i][0],laser_data[i][1])
    if front:
        return (True,min(front),ang[front.index(min(front))])
    """if front_r:
        detected = True
        min_r = min(front_r)
    else:
        min_r = 0
    
    if front_l:
        detected = True
        min_l = min(front_l)
    else:
        min_l = 0
    
    if min_r < min_l:
        direction = -1
        minimum = min_r
    else:
        direction = 1
        minimum = min_l
    
    return (detected,minimum,direction)"""
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
V = 3
counter = 0
extra_W = 0
while True:
    # Enter iterative code!
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
        if i <= 59:
            laser_right += [v]
        elif i <= 119:
            laser_front += [v]
        else:
            laser_left += [v]
            
        laser_vectorized += [v]
    mean_right = np.mean (laser_right, axis = 0)
    mean_front = np.mean (laser_front, axis = 0)
    mean_left = np.mean (laser_left, axis = 0)
    print("I:",mean_left, "F:", mean_front, "R:", mean_right)
    
    laser_mean = np.mean (laser_vectorized, axis = 0)
    
    module_obs = math.sqrt((laser_mean[0])**2 + (laser_mean[1])**2)

    
    ################ Coordenadas del coche ###############
    robotx = HAL.getPose3d().x
    roboty = HAL.getPose3d().y
    robott = HAL.getPose3d().yaw
    #print("ROBOT:", (robotx,roboty), "Ang:", robott)
    #############################################
    
    
    ########## Coordenadas de la meta ###############
    currentTarget = GUI.map.getNextTarget()
    GUI.map.targetx = currentTarget.getPose().x
    GUI.map.targety = currentTarget.getPose().y
    
    
    # Current target
    target = [currentTarget.getPose().x, currentTarget.getPose().y]
    GUI.showLocalTarget(target)

    ########### Comprobacion de llegada a la meta ##############
    targetx_transf,targety_transf = absolute2relative(currentTarget.getPose().x, currentTarget.getPose().y, robotx, roboty, robott)
    
    if checkpoint(robotx, roboty, targetx_transf, targety_transf):
        currentTarget.setReached(True)
    ############################################################
    
    
    ########## Muestreo de fuerzas ##############
    
    # Obstacles direction (black line in the image below) # Ha de ser negativa la y
    # para que la flecha vaya en sentido contrario al obstaculo
    if module_obs < 2.4:
        obsForce = [-1/laser_mean[0], laser_mean[1]]
        
    mod = math.sqrt((obsForce[0])**2 + (obsForce[1])**2)
    
    front_obs = detect_front_obst(laser)
    
    print("FRENTE:", front_obs)
    wall = laser[0][0] <= 0.3 or laser[179][0] <= 0.3
    
    if front_obs[0]:
        beta = 1.15
        alpha = 0.9
        
        if front_obs[1] <= 0.45:
            #beta = 1.31
            V=0.5
            alpha = 0.75
            print("########## OBJETO DE FRENTE DEMASIADO CERCA -----> GIIIIRAAAAAA ##########")
            direction = front_obs[2]/math.fabs(front_obs[2])
            counter = 5#15
            """if obsForce[1] < 0:
                obsForce[1] = -targety_transf
            else:
                obsForce[1] = targety_transf"""
            extra_W = -1.46*direction
        else:
            if counter != 0:
                counter -= 1
                V += 0.5
            else:
                extra_W = 0
            
        
            # W pos izq, neg der
            
    # Average direction (red line in the image below) A donde kiero que vaya Funciona
    # Acotamos el maximo del vector target
    if targetx_transf > 6:
        targetx_transf = 6
    if math.fabs(targety_transf) > 6:
        direction  = math.fabs(targety_transf)/targety_transf
        targety_transf = 6*direction
        
    module_target = math.sqrt((targetx_transf)**2 + (targety_transf)**2)
    
    if module_target < 1:
        module_target = 1
        
    avgForce = [targetx_transf, targety_transf]
    W_avg = math.atan2(targety_transf, targetx_transf)
    ##############################################################
    
    #print("ALPHA:", alpha, "BETA:", beta)
    ################################################
    
    # Car direction  (green line in the image below)
    direction_x = alpha*avgForce[0]/module_target + obsForce[0]*beta
    direction_y = alpha*avgForce[1]/module_target + obsForce[1]*beta
    
    module_direction = math.sqrt((direction_x)**2 + (direction_y)**2)
    carForce = [direction_x*module_target, direction_y*module_target]
    
    #print("VEC_OBJETIVO:", avgForce, "ANG:", W_avg, "MOD:", int(module_target))
    #print("VEC_OBS: ", obsForce, "ANG:", math.atan2(obsForce[1],obsForce[0]), "MOD:", module_obs)
    #print("VEC_RESULT:", carForce, "ANG:", math.atan2(direction_y, direction_x), "MOD:", module_direction)
    
    GUI.showForces(carForce, avgForce, obsForce)
    ########################################
    
    if front_obs[0] or module_target <= 1.5:
        if V > 3:
            V -= 0.34
        #W = 0 # W pos izq, neg der
    else:
        if V < 4.5:
            V += 0.04
            
    if counter == 0:    
        W = math.atan2(direction_y, direction_x) + extra_W
    else:
        W = extra_W
        
    if math.fabs(W) < 0.4:
        W = 0
        
    print("VEL W:", W, "VEL V:", V)
    HAL.setV(V)#1.3
    HAL.setW(W)
    
    img = HAL.getImage()
    GUI.showImage(img)
    print()
    
    
