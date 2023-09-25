from GUI import GUI
from HAL import HAL
import time
import math
import numpy as np
# Enter sequential code!

def checkpoint(robotx, roboty, goalx, goaly):
    distx = math.fabs(goalx) - math.fabs(robotx)
    disty = math.fabs(goaly) - math.fabs(roboty)
    print("DISTY:",disty)
    return distx <= 0.01 #or disty <= 0.1

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

V = 3
while True:
    # Enter iterative code!
    
    laser_data = HAL.getLaserData()
    laser = parse_laser_data(laser_data)
    
    laser_vectorized = []
    for d, a in laser:
        
        x = d * math.cos (a) * -1
        y = d * math.sin (a) * -1
        v = (x, y)
        laser_vectorized += [v]
    
    laser_mean = np.mean (laser_vectorized, axis = 0)
    
    module_obs = math.sqrt((laser_mean[0])**2 + (laser_mean[1])**2)
    
    #print("Media laser VECTOR OBS:", laser_mean, "MODULO:", module_obs)
    # Medida del laser obs a la derecha del coche [-1.95206293, 0.44479627]
    # Medida del laser obs a la izquierda del coche []
    
    ################ Coordenadas del coche###############
    robotx = HAL.getPose3d().x
    roboty = HAL.getPose3d().y
    robott = HAL.getPose3d().yaw
    #print("ROBOT: (", robotx, ",", roboty, ") Ang:", robott)
    #############################################
    
    
    ########## Coordenadas de la meta ###############
    currentTarget = GUI.map.getNextTarget()
    GUI.map.targetx = currentTarget.getPose().x
    GUI.map.targety = currentTarget.getPose().y
    
    # Current target
    target = [currentTarget.getPose().x, currentTarget.getPose().y]
    GUI.showLocalTarget(target)

    ########### Comprobacion de llegada a la meta ##############
    targetx_transf, targety_transf = absolute2relative(currentTarget.getPose().x, currentTarget.getPose().y, robotx, roboty, robott)
    #currentTarget.setReached(checkpoint(robotx, roboty, targetx_transf, targety_transf))
    print("POSICION:", (robotx, roboty))
    print("#######META#####",(targetx_transf, targety_transf))
    if checkpoint(robotx, roboty, targetx_transf, targety_transf):
        print("################## ALCANCE CHECKPOINT #####################")
        currentTarget.setReached(True)
        HAL.setV(0)
        HAL.setW(0)
        time.sleep(10)
    # Modulo para convertir en vector unitario target
    module_target = math.sqrt((targetx_transf)**2 + (targety_transf)**2)
    ############################################################
    
    #print("Meta en coordenadas relativas:", targetx_transf, targety_transf)
    ######
    

    ########## Muestreo de fuerzas ##############
    
    # Obstacles direction (black line in the image below) # Ha de ser negativa la y
    # para que la flecha vaya en sentido contrario al obstaculo
    obsForce = [1/laser_mean[0], -laser_mean[1]]
    
    print("Vector obstaculo: ", obsForce, "ANG:", math.atan2(-laser_mean[1],laser_mean[0]), "MOD:", module_obs)
    
    
    # Average direction (red line in the image below) A donde kiero que vaya Funciona
    avgForce = [targetx_transf/module_target, targety_transf/module_target]
    W_avg = math.atan2(targety_transf, targetx_transf)
    # Si pongo W_avg como vel_ang funciona, gira hacia el objetivo
    print("Vector unitario OBJETIVO:", avgForce, "ANG:", W_avg, "MOD:", module_target)
    #currentTarget.setReached(checkpoint(module_target))
    
    #if currentTarget.setReached(module_target <= 0.5):
        #print("###########################################################")
        #print("####################### CHECKPOINT ########################")
        #print("###########################################################")
    
    # Car direction  (green line in the image below)
    #direction_x = targetx_transf/module_target + laser_mean[0]/module_obs
    #direction_y = targety_transf/module_target + (-laser_mean[1]/module_obs)
    if module_obs < 0.8:
        relevance = 1.8
    else:
        relevance = 1
        
    direction_x = avgForce[0] + obsForce[0]*relevance
    direction_y = avgForce[1] + obsForce[1]*relevance
    module_direction = math.sqrt((direction_x)**2 + (direction_y)**2)
    
    carForce = [direction_x, direction_y]
    print("VECTOR RESULTANTE:", carForce, "ANG:", math.atan2(direction_y, direction_x), "MOD:", module_direction)
    
    
    GUI.showForces(carForce, avgForce, obsForce)
    ############################################
    
    if module_target < 8:
        if V > 0.8:
            V -= 0.05
        #W = 0 # W pos izq, neg der
    else:
        if V < 2.8:
            V += 0.08
            
    if module_obs < 1.7:
        W = math.atan2(obsForce[1], obsForce[0])
        V = 0.9
    else:
        W = math.atan2(direction_y, direction_x)
    #V = module_direction*2.5
    print("VEL W:", W, "VEL V:", V)
    HAL.setV(1)
    HAL.setW(W)
    
    img = HAL.getImage()
    GUI.showImage(img)
    print()
    #time.sleep(2)
    
    
