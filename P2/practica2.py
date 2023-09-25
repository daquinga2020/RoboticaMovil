from GUI import GUI
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

KP = 0.00967
KD = 0.02463
KI = 0.000001215

W = 0
V = 10.5

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

    if ind > 15:
        W = get_ouputPID(errang)
    else:
        W = 0

    ind += 1

    print("VEL ANG:", W, "VEL LIN:", V)
    HAL.setW(W)
    HAL.setV(V)

    GUI.showImage(img)