from GUI import GUI
from HAL import HAL
import time
import math
import numpy as np
# Completa vuelta pero choca con vehiculo de frente
# Enter sequential code!

def checkpoint(robotx, roboty, goalx, goaly):
    
    
    return math.fabs(goalx) <= 0.2

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
while True:
    # Enter iterative code!
    
    obsForce = [0,0]
    
    laser_data = HAL.getLaserData()
    laser = parse_laser_data(laser_data)
    
    laser_vectorized = []
    for d, a in laser:
        
        x = d * math.cos (a)# * -1
        y = d * math.sin (a)# * -1
        v = (x, y)
        laser_vectorized += [v]
    
    laser_mean = np.mean (laser_vectorized, axis = 0)
    print("LASER:", laser_mean)
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
    
    #print("GOAL_NORMAL:", (GUI.map.targetx, GUI.map.targety))
    
    # Current target
    target = [currentTarget.getPose().x, currentTarget.getPose().y]
    GUI.showLocalTarget(target)

    ########### Comprobacion de llegada a la meta ##############
    targetx_transf,targety_transf = absolute2relative(currentTarget.getPose().x, currentTarget.getPose().y, robotx, roboty, robott)
    
    # Modulo para convertir en vector unitario target
    module_target = math.sqrt((targetx_transf)**2 + (targety_transf)**2)
    
    #currentTarget.setReached(checkpoint(robotx, roboty, targetx_transf, targety_transf))
    #print("GOAL_RELATIVE",(targetx_transf, targety_transf), module_target)
    
    if checkpoint(robotx, roboty, targetx_transf, targety_transf):
        #print("################## ALCANCE CHECKPOINT ##################")
        currentTarget.setReached(True)
    ############################################################
    
    
    ########## Muestreo de fuerzas ##############
    
    # Obstacles direction (black line in the image below) # Ha de ser negativa la y
    # para que la flecha vaya en sentido contrario al obstaculo
    #if laser_mean[0] <= 2.25:
    obsForce = [-1/laser_mean[0], laser_mean[1]]
        
    
    
    # Average direction (red line in the image below) A donde kiero que vaya Funciona
    avgForce = [targetx_transf, targety_transf]
    W_avg = math.atan2(targety_transf, targetx_transf)
    
    print("IZQ:",laser[179],"DERCH:", laser[0])
    wall = laser[0][0] <= 0.4 or laser[179][0] <= 0.4
    # Definimos la relevancia de cada fuerza b = [n * 3 for n in a]
    if wall or module_obs <= 2.0:
        alpha = 0.7
        beta = module_target/2 + 1
    else:
        alpha = 1
        beta = module_target/4 + 1
    
    obsForce = [beta * crd for crd in obsForce]
    avgForce = [alpha * crd for crd in avgForce]
    
    ################################################
    print("ALPHA:", alpha, "BETA:", beta)
    # Car direction  (green line in the image below)
    direction_x = avgForce[0] + obsForce[0]
    direction_y = avgForce[1] + obsForce[1]
    
    module_direction = math.sqrt((direction_x)**2 + (direction_y)**2)
    
    carForce = [direction_x, direction_y]
    
    print("VEC_OBJETIVO:", avgForce, "ANG:", W_avg, "MOD:", module_target)
    print("VEC_OBS: ", obsForce, "ANG:", math.atan2(obsForce[1],obsForce[0]), "MOD:", module_obs)
    print("VEC_RESULT:", carForce, "ANG:", math.atan2(direction_y, direction_x), "MOD:", module_direction)
    
    
    GUI.showForces(carForce, avgForce, obsForce)
    ############################################
    
    if module_target < 8:
        if V > 0.8:
            V -= 0.05
        #W = 0 # W pos izq, neg der
    else:
        if V < 2.8:
            V += 0.08
            
    W = math.atan2(direction_y, direction_x)

    print("VEL W:", W, "VEL V:", V)
    HAL.setV(1.2)
    HAL.setW(W)
    
    img = HAL.getImage()
    GUI.showImage(img)
    print()
    #time.sleep(2)
    
    
