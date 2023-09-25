from GUI import GUI
from HAL import HAL
import math
import time
import numpy as np
import argparse
import cv2
#time 4:29
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
    output = ref
    
    if ref != 0.0:
        direction = ref / math.fabs(ref)

    """if math.fabs(ref) < min_ref:
        output = 0.0

    elif math.fabs(ref) > max_ref:
        output = direction * max_output

    else:
        output = direction * min_output + ref * (max_output - min_output)
    """
    output = ref
    # Integral Error
    int_error = int_error + output
    
    # Derivative Error
    deriv_error = output - prev_error
    prev_error = output
    
    print("P:",KP * output)#, "I:", KI * int_error, "D:", KD * deriv_error)
    output = KP * output + KD * deriv_error# + KI * int_error
    
    return output
    

KP = 0.0025
KD = 0.0043#0.667
KI = 0.025

min_ref = 0.015
max_ref = 1.0
min_output = 0.0
max_output = 0.8
"""
min_ref = 25
max_ref = 320
min_output = 0.0
max_output = 1.6"""

W = 0
MAX_W = 1.0

V = 3
MAX_V = 5

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
            
            #b = img_filtered.item(i, j, 0)
            #g = img_filtered.item(i, j, 1)
            r = img_filtered.item(i, j, 2)
            
            if r > 100 and i < 300:
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
    
    #print("PUNTO A SEGUIR", pixel_x, pixel_y)
    #print("CENTRO IMG:", center_image)
    diff = pixel_x - center_image[0]
    print("DIFERENCIA EN X:", (320 - pixel_x)) #15 dif
    
    #errang = (320 - pixel_x)/320
    errang=320 - pixel_x
    #Vel negativa izq
    #V = get_output(errlin)*MAX_V
    if ind > 20:
        W = get_ouputPID(errang)*MAX_W
        print("++++ERROR ANGULAR:", errang, "++++")
    else:
        W = 0
    
    """if math.fabs(errang) > 0.3:
        MAX_W +=0.2
    else:
        MAX_W = 1.3"""
        #if V > 2:
        #    V -= 0.4
    #elif V < 4.8:
        #V += 0.4
    
    ind += 1
    
    print("VEL ANG:", W, "VEL LIN:", V)
    HAL.setW(W)
    HAL.setV(V)
    
    GUI.showImage(img_filtered)
    #GUI.showImage(img)
    

