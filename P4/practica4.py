from GUI import GUI
from HAL import HAL
from MAP import MAP
import numpy as np
from queue import PriorityQueue
import math

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


def get_neighbours(point):
    
    (x,y) = point
    neighbours =  [(x-1, y), (x+1, y), (x, y-1), (x, y+1), (x+1, y+1), (x-1, y+1), (x+1, y-1), (x-1, y-1)]

    return neighbours


pth_planning = PriorityQueue()
gradient_planning = PriorityQueue()

max_weight = 0

goal_found = False
path_found = False
gradient_done = False
widen_walls = False
reached = False
normalize = True

map_array = np.full((400,400),0)
obstacles = list()
path = list()

location_taxi = (HAL.getPose3d().x,HAL.getPose3d().y)
location_taxi = MAP.rowColumn(location_taxi)
location_taxi = (location_taxi[0], location_taxi[1])
neightbours_taxi = get_neighbours(location_taxi)
neightbours_taxi.append(location_taxi)

def do_GradientMap(map_img):

    global goal_found
    global gradient_done
    global gradient_planning
    global location_taxi
    global map_array
    global obstacles
    global max_weight
    expanded = list()
    nghb_visited = list()

    while goal_found and not gradient_done and not gradient_planning.empty():
        # Sacar el nodo c de la cola de prioridad
        current_node = gradient_planning.get()

        # Si c == localizacion del taxi, finalizar
        if current_node[1] == location_taxi:
            gradient_done = True
            break

        # Si c == obstáculo, guardar en otra lista
        if current_node[1] not in obstacles:
            if map_img[current_node[1][1],current_node[1][0]] == 0:
                obstacles.append(current_node[1])
                map_array[current_node[1][1],current_node[1][0]] = 0
                continue

        if current_node[1] in expanded:
            continue

        expanded.append(current_node[1])

        # Nodos vecinos
        neighbours = get_neighbours(current_node[1])
        count = 0
        for nghb in neighbours:
            count += 1

            if not (0 <= nghb[0] <= 399 and 0 <= nghb[1] <= 399):
                continue

            if map_img[nghb[1], nghb[0]] == 0:
                map_array[nghb[1], nghb[0]] = 0
                obstacles.append(nghb)
                continue

            # Asignar peso a los vecinos de c si anteriormente no han sido asignados
            if nghb in expanded or nghb in nghb_visited:
                continue

            if map_array[nghb[1], nghb[0]] != 0:
                continue

            if count >= 5:
                cost = 1
            else:
                cost = 0.6

            nghb_visited.append(nghb)
            map_array[nghb[1],nghb[0]] = current_node[0] + cost
            if current_node[0] + cost > max_weight:
                max_weight = current_node[0] + cost

            # Insertar vecinos de c en la cola de prioridad
            gradient_planning.put((current_node[0] + cost, nghb))

        GUI.showNumpy(map_array)

def widing_walls(npixel_wall, extra_cost, map_img):

    global widen_walls
    global gradient_done
    global neightbours_taxi
    global obstacles
    global map_array
    global max_weight
    widen_pixels = obstacles
    nghb_visited = list()
    thickened_pixels = list()

    while not widen_walls and gradient_done:

        if npixel_wall <= 0:
            widen_walls = True
            break

        for wall in widen_pixels:
            neighbours = get_neighbours(wall)

            for nghb in neighbours:
                if not (0 <= nghb[0] <= 399 and 0 <= nghb[1] <= 399):
                    continue
                if nghb in neightbours_taxi or nghb in nghb_visited or nghb in obstacles:
                    continue
                if map_img[nghb[1], nghb[0]] == 0:
                    continue

                nghb_visited.append(nghb)
                thickened_pixels.append(nghb)
                map_array[nghb[1]][nghb[0]] = map_array[nghb[1]][nghb[0]] + extra_cost

                if map_array[nghb[1]][nghb[0]] > max_weight:
                    max_weight = map_array[nghb[1]][nghb[0]]

        widen_pixels = thickened_pixels
        thickened_pixels = list()
        extra_cost -= 5
        npixel_wall -= 1

def normalize_map():

    global map_array
    global normalize

    if normalize:
        copy_map = np.copy(map_array)
        for i in range(400):
            for j in range(400):
                if copy_map[j][i] != 0:
                    copy_map[j][i] = copy_map[j][i]*254/(max_weight-1)

        normalize = False

        GUI.showNumpy(copy_map)

def find_path(goal_map):

    global gradient_done
    global path_found
    global pth_planning
    global obstacles
    global path
    expanded = list()

    while gradient_done and not path_found and not pth_planning.empty():

        current_node = pth_planning.get()

        if current_node[1] == goal_map:
            path_found = True
            break

        if current_node[1] in expanded or current_node[1] in obstacles:
            continue

        expanded.append(current_node[1])

        neighbours = get_neighbours(current_node[1])

        nghb_lowcost = 10000

        for nghb in neighbours:
            if not (0 <= nghb[0] <= 399 and 0 <= nghb[1] <= 399):
                continue

            if nghb in expanded:
                continue

            if nghb == goal_map:
                path.append([nghb[0],nghb[1]])
                nghb_lowcost = 10000
                path_found = True
                break

            cost = map_array[nghb[1],nghb[0]]
            if nghb_lowcost > cost and cost != 0:
                nghb_lowcost = cost
                next_node = nghb

        if nghb_lowcost != 10000:
            path.append([next_node[0],next_node[1]])
            pth_planning.put((nghb_lowcost, next_node))

        GUI.showPath(path)

def navigate2goal():

    global path
    global reached
    v = 2.5
    local_reached = True
    ind = 0
    
    while not reached:
        taxi_x = HAL.getPose3d().x
        taxi_y = HAL.getPose3d().y
        theta_taxi = HAL.getPose3d().yaw

        if local_reached:
            ind += 4 # Radio del coche
            local_reached = False

        if ind >= len(path)-1:
            ind = len(path)-1

        x_abs = (path[ind][1] - 200)*1.25
        y_abs = (path[ind][0] - 200)*1.25

        goalx_rel,goaly_rel = absolute2relative(x_abs, y_abs, taxi_x, taxi_y, theta_taxi)
        module_goal = math.sqrt((goalx_rel)**2 + (goaly_rel)**2)

        if module_goal <= 1.5:
            local_reached = True
            if ind == len(path)-1:
                reached = True
                HAL.setV(0)
                HAL.setW(0)
                break

        w = math.atan2(goaly_rel, goalx_rel)
        if math.fabs(w) <= 0.05:
            w = 0
            if v <= 5:
                v += 0.5
        else:
            v = 2.5

        HAL.setV(v)
        HAL.setW(w)

while True:

    map_img = MAP.getMap()
    goal_world = GUI.getTargetPose()

    # Insertar la localizacion del taxi en la cola de prioridad, con peso 1 (peso minimo)
    if not goal_world is None and not goal_found:
        goal_map = MAP.rowColumn(goal_world)
        goal_map = (goal_map[0], goal_map[1])

        gradient_planning.put((1, goal_map))
        map_array[goal_map[1], goal_map[0]] = 1
        goal_found = True

        do_GradientMap(map_img)

    if not widen_walls and gradient_done:
        npixel_wall = 4
        extra_cost = 35

        widing_walls(npixel_wall, extra_cost, map_img)

    if gradient_done and not path_found:
        normalize_map()
        
        pth_planning.put((map_array[location_taxi[1], location_taxi[0]], location_taxi))
        path.append([location_taxi[0],location_taxi[1]])
        
        find_path(goal_map)

    if path_found:
        navigate2goal()

    HAL.setV(0)
    HAL.setW(0)