import time
from GUI import GUI
from HAL import HAL

def follow_walls():

    t_initial = time.time()
    wall = False
    count_f = 0
    state_f = "SEARCHING"

    while time.time()-t_initial < 10:
        laser_data_f = HAL.getLaserData()

        if state_f == "SEARCHING":

            if not laser_data_f.values[90] <= 0.5:
                HAL.setV(2)
                HAL.setW(0)
            else:
                state_f = "TURNING"
                print("FOLLOW WALL: SEARCHING -----> TURNING")

        if state_f == "TURNING":
            HAL.setV(0)

            if laser_data_f.values[179] < 0.4 and count_f > 15:
                wall = True
            else:
                HAL.setW(1)

            if wall:
                state_f = "SEARCHING"
                count_f = 0
                wall = False
                print("FOLLOW WALL: TURNING -----> SEARCHING")

        count_f += 1


DISTANCE_DETECTED_ = 0.4
state_base = "FORWARD"
count = 0
portion = int(180/4)
measures = int(180/2)
change_state = 0
angle = 90

while True:

    laser_data = HAL.getLaserData()
    detected_obj = False

    # Object Detection
    for i in range(measures):
        dist = laser_data.values[i+portion]
        if dist < DISTANCE_DETECTED_:
            angle = i+portion
            detected_obj = True
            break

    if angle < 90:
        W = -0.8
    else:
        W = 0.8

    if state_base == "FORWARD":

        HAL.setV(2.5)
        HAL.setW(-0.3)

        if count > 20:
            change_state = 0

        if detected_obj and not change_state > 8:
            state_base = "BACKWARD"
            count = 0
            print("FORWARD ----> BACKWARD")

        if change_state > 8:
            change_state = 0
            state_base = "FOLLOW WALLS"
            print("FORWARD -----> FOLLOW WALLS")

    elif state_base == "BACKWARD":

        HAL.setV(-1)
        HAL.setW(W)

        if detected_obj:
            count = 0

        if count > 5:
            count = 0
            state_base = "TURNING"
            print("BACKWARD ----> TURNING")

    elif state_base == "TURNING":

        HAL.setW(W)
        HAL.setV(0)

        if detected_obj:
            count = 0

        if count > 10:
            count = 0
            state_base = "FORWARD"
            change_state += 1
            print("TURNING ----> FORWARD")

    elif state_base == "FOLLOW WALLS":
        follow_walls()
        state_base = "FORWARD"
        print("FOLLOW WALLS ----> FORWARD")

    count += 1