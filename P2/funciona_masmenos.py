from GUI import GUI
from HAL import HAL
import math
import time
import numpy as np
import argparse
import cv2

def filter_image(image):
    
    redBajo1 = np.array([0, 100, 20], np.uint8)
    redAlto1 = np.array([8, 255, 255], np.uint8)
    redBajo2=np.array([175, 100, 20], np.uint8)
    redAlto2=np.array([179, 255, 255], np.uint8)
    
    frameHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
    maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
    maskRed = cv2.add(maskRed1, maskRed2)
    maskRedvis = cv2.bitwise_and(image, image, mask= maskRed)
    
    return maskRedvis

prev_error = 0.0
int_error = 0.0

def get_ouputPID(new_reference):
    # Proportional Error
    global int_error
    global prev_error
    ref = new_reference
    direction = 0.0
    output = 0.0
    
    if ref != 0.0:
        direction = ref / math.fabs(ref)

    if math.fabs(ref) < min_ref:
        output = 0.0

    elif math.fabs(ref) > max_ref:
        output = direction * max_output

    else:
        output = direction * min_output + ref * (max_output - min_output)

    print("OUTPUT ANTES DE PID: ", output)
    # Integral Error
    int_error = (int_error + output) * 2.0 / 3.0
    print("ERROR INTEGRAL:", int_error)
    # Derivative Error
    deriv_error = output - prev_error
    prev_error = output
    print("ERROR DERIVATIVO:", deriv_error)
    print("P:",KP * output, "I:", KI * int_error, "D:", KD * deriv_error)
    output = KP * output + KI * int_error + KD * deriv_error
    
    return output
    

KP = 0.3
KD = 0.8
KI = 0.01

min_ref = 0.05
max_ref = 1.0
min_output = 0.0
max_output = 1.3

V = 2
W = 0
MAX_V = 5
MAX_W = 1.3
ind = 0
img = HAL.getImage()

while True:

    img = HAL.getImage()
    
    img_filtered = filter_image(img)
    
    hight = img.shape[0]
    width = img.shape[1]
    center_image =(int(width/2), int(hight/2))
    cv2.circle(img_filtered, center_image, 3, (255, 5, 5), 2)#Blue
    
    coordinate_sum = np.array([0,0])
    count = 0
    
    #Calculo del centro de masas
    for i in range(hight):
        for j in range(width):
            
            b = img_filtered.item(i, j, 0)
            g = img_filtered.item(i, j, 1)
            r = img_filtered.item(i, j, 2)
            
            if r > 100:
                count += 1
                coordinate_sum = coordinate_sum + [j,i]
    
    if math.isnan(coordinate_sum[0]/count) or math.isnan(coordinate_sum[1]/count):
        pixel_x = 320
        pixel_y = 0
    else:
        pixel_x = int(coordinate_sum[0]/count)
        pixel_y = int(coordinate_sum[1]/count)
    
    # Centro de masas
    cv2.circle(img_filtered, (pixel_x,pixel_y), 3, (5, 255, 5), 2)#Green
    
    print("PUNTO A SEGUIR", pixel_x, pixel_y)
    
    print("CENTRO IMG:", center_image)
    diff = pixel_x - center_image[0]
    print("DIFERENCIA EN X:", diff) #15 dif
    
    errang = (320 - pixel_x)/320
    # get_ouputPID(diff)
    
    #Vel negativa izq
    """if diff-15 > 30:
        V = 1
        W -= 0.2
    elif diff-15 < -30:
        V = 1
        W += 0.2
    else:
        W = 0
        if V < 8:
            V += 0.01"""
            
    print("Error angular:", errang)
    #V = get_output(errlin)*MAX_V
    if ind > 20:
        W = get_ouputPID(errang)*MAX_W
    else:
        W = 0
    
    if math.fabs(W) > 0.001:
        if V > 1.0:
            V -= 0.2
    elif V < 4:
        V += 0.1
    
    ind += 1
    print("VEL ANG:", W, "VEL LIN:", V)
    HAL.setW(W)
    HAL.setV(V)
    GUI.showImage(img_filtered)
    #GUI.showImage(img)
    
"""
double errlin = (ball.depth - ideal_depth_)/(dist_p-1);
double errang = (ideal_x_ - ball.x)/320;

speed.linear = (linear_pid_.get_output(errlin))*max_vel_lin;
speed.angular = (angular_pid_.get_output(errang))*max_vel_ang;
"""

