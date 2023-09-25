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

prev_error_v = 0.0
#int_error_v = 0.0

def get_outputPID_W(new_reference):
    
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

    # Integral Error
    int_error = (int_error + output) * 2.0 / 3.0
    
    # Derivative Error
    deriv_error = output - prev_error
    prev_error = output
    #print("P:",KP * output, "I:", KI * int_error, "D:", KD * deriv_error)
    output = KP * output + KI * int_error + KD * deriv_error
    
    return output
    

def get_outputPD_V(new_reference_v):
    
    # Proportional Error
    global int_error_v
    global prev_error_v
    
    ref_v = new_reference_v
    output_v = 0.0
    
    if ref_v < min_ref_v:
        output_v = 0.0

    elif ref_v > max_ref_v:
        output_v = max_output_v

    else:
        output_v = min_output_v + ref_v * (max_output_v - min_output_v)

    #print("P DE LINEAL: ", output_v)
    # Integral Error
    #int_error_v = (int_error_v + output_v) * 2.0 / 3.0
    #print("ERROR INTEGRAL DE LINEAL:", int_error_v)
    # Derivative Error
    deriv_error_v = output_v - prev_error_v
    prev_error_v = output_v
    #print("ERROR DERIVATIVO DE LINEAL:", deriv_error_v)
    print("P_lineal:",KP * output_v)#, "D_lineal:", KD * deriv_error_v)
    output_v = KP_v * output_v + KD_v * deriv_error_v
    
    return output_v
    

# Valores para W angular
KP = 0.287
KD = 0.635
KI = 0.021
# Valores para V lineal
KP_v = 0.269
KD_v = 0.445
#KI_v = 0.005

# Ajustes para controlador PID angular
min_ref = 0.0625
max_ref = 1.0
min_output = 0.0
max_output = 1.3

W = 0
MAX_W = 1.3

# Ajustes para controlador PID lineal
min_ref_v = 0.0
max_ref_v = 1.0
min_output_v = 1.3
max_output_v = 6.0

#V_ideal = 4.0
MAX_V = 3.5

ind = 0
img = HAL.getImage()
dist_pix = 132
max_pix_x = 0
max_pix_y = 0

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
            
            r = img_filtered.item(i, j, 2)
            
            if r > 100:
                count += 1
                coordinate_sum = coordinate_sum + [j,i]
                
                if i == 280:
                    max_pix_x = int(coordinate_sum[0]/count)
                    max_pix_y = int(coordinate_sum[1]/count)
    
    if math.isnan(coordinate_sum[0]/count) or math.isnan(coordinate_sum[1]/count):
        pixel_x = 320
        pixel_y = 0
    else:
        pixel_x = int(coordinate_sum[0]/count)
        pixel_y = int(coordinate_sum[1]/count)
    
    if not math.isnan(max_pix_x) or not math.isnan(max_pix_y):
        distance = math.sqrt((max_pix_x-pixel_x)**2+(max_pix_y-pixel_y)**2)
        cv2.circle(img_filtered, (max_pix_x,max_pix_y), 3, (255, 255, 255), 2) # White
        cv2.line(img_filtered, (pixel_x,pixel_y), (max_pix_x,max_pix_y) , (255, 255, 255),  2)
    
    
    # Centro de masas
    cv2.circle(img_filtered, (pixel_x,pixel_y), 3, (5, 255, 5), 2) # Green
    
    diff = pixel_x - center_image[0]
    
    # Error angular y lineal
    errang = (320 - pixel_x)/320
    errlin = 1-math.fabs(errang)
    print("Error angular:", errang)
    print("Error lineal:", 1-math.fabs(errang))
    
    #W negativa izq, W post der
    
    
    
    if ind > 20:
        W = get_outputPID_W(errang)*MAX_W
    else:
        W = 0
    
    if math.fabs(W) > 0.035:
        if MAX_V > 0.95:
            MAX_V -= 0.15
        else:
            MAX_V += 0.015
    elif MAX_V < 3.5:
        MAX_V += 0.3
        
    V = get_outputPD_V(errlin)*MAX_V
    
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

