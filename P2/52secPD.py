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
        
    output = ref
    # Integral Error
    int_error = int_error + output
    
    # Derivative Error
    deriv_error = output - prev_error
    prev_error = output
    
    print("P:",KP * output, "I:", KI * int_error, "D:", KD * deriv_error)#)
    output = KP * output + KD * deriv_error + KI * int_error#
    
    return output
    
KP = 0.00681 #0.00688
KD = 0.0168 #0.7 #0.01485
KI = 0.0 #0.000322

min_ref = 0.0
max_ref = 1.0
min_output = 0.0
max_output = 0.8

W = 0
MAX_W = 1.0

V = 9
MAX_V = 5

ind = 0
img = HAL.getImage()
while True:

    img = HAL.getImage()
    
    img_filtered = filter_image(img)
    
    hight = img.shape[0]
    width = img.shape[1]
    center_image =(int(width/2), int(hight/2))
    cv2.circle(img, center_image, 3, (255, 255, 255), 2)#WHITE
    
    coordinate_sum = np.array([0,0])
    count = 0
    
    #Calculo del centro de masas
    for i in range(hight-215):
        for j in range(width):
            
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
    cv2.circle(img, (pixel_x,pixel_y), 3, (5, 255, 5), 2)#Green
    
    diff = pixel_x - center_image[0]
    
    #errang = (320 - pixel_x)/320
    errang=320 - pixel_x
    #Vel negativa izq
    #V = get_output(errlin)*MAX_V
    if ind > 20:
        W = get_ouputPID(errang)*MAX_W
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
    
    #GUI.showImage(img_filtered)
    GUI.showImage(img)
    
"""from GUI import GUI
from HAL import HAL
import math
import numpy as np
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

def calculate_centermass(img, h, w):

    coordinate_sum = np.array([0,0])
    count = 0

    # Calculation of the center of mass of the line
    for i in range(h-215):
        for j in range(w):

            r = img.item(i, j, 2)
            if r > 100:
                count += 1
                coordinate_sum = coordinate_sum + [j,i]

    if math.isnan(coordinate_sum[0]/count) or math.isnan(coordinate_sum[1]/count):
        x = 320
        y = 0
    else:
        x = int(coordinate_sum[0]/count)
        y = int(coordinate_sum[1]/count)

    return x, y

prev_error = 0.0
int_error = 0.0

def get_ouputPID(new_reference):

    # Proportional Error
    global int_error
    global prev_error
    output = new_reference

    # Integral Error
    int_error = int_error + output

    # Derivative Error
    deriv_error = output - prev_error
    prev_error = output

    print("P:",KP * output, "I:", KI * int_error, "D:", KD * deriv_error)
    output = KP * output + KD * deriv_error + KI * int_error

    return output

Tiempo = 52 seg
KP = 0.00681 #0.00688
KD = 0.01683 #0.7 #0.01485
KI = 0.0 #0.000322

W = 0
V = 9

ind = 0
img = HAL.getImage()

hight = img.shape[0]
width = img.shape[1]
center_image =(int(width/2), int(hight/2))

while True:

    img = HAL.getImage()
    img_filtered = filter_image(img)

    # Calculation of the center of mass of the line
    pixel_x,pixel_y = calculate_centermass(img_filtered, hight, width)

    cv2.circle(img, center_image, 3, (255, 255, 255), 2)
    cv2.circle(img, (pixel_x,pixel_y), 3, (5, 255, 5), 2)

    errang = center_image[0] - pixel_x

    if ind > 25:
        W = get_ouputPID(errang)
    else:
        W = 0

    ind += 1

    print("VEL ANG:", W, "VEL LIN:", V)
    HAL.setW(W)
    HAL.setV(V)

    GUI.showImage(img)"""

