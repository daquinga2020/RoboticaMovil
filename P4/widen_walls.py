from GUI import GUI
from HAL import HAL
from MAP import MAP
import numpy as np
from queue import PriorityQueue
import cv2
import time

#def get_neighbors(vector):
    #neighbors = []
    # [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
    #if vector[0] == 399 and vector[: # lateral derecho
        
    
# La imagen del mapa tiene una resolución de 400x400 píxeles e 
# indica si hay un obstáculo o no por su color. 
# El mapa en el mundo Gazebo tiene su centro en [0, 0] 
# y tiene un ancho y alto de 500 metros. 
# Por lo tanto, cada uno de los píxeles de la imagen del mapa 
# representa una celda en el mundo Gazebo con un ancho y una altura de 1,25 metros.
# Medidas en mapa
## 255 -> blanco -> no hay obstaculo
## 0 -> negro -> hay obstaculo, peso mucho mayor
# Los valores más pequeños tienen mas prioridad



pth_planning = PriorityQueue()
gradient_planning = PriorityQueue()

path = []
node_path = []

cost_node_path = 0
wall_pixel = 800

goal_found = False
path_found = False
gradient_done = False
widen_walls = False

map_array = np.full((400,400),0)
obstacles = list()
expanded = list()
nghb_visited = list()

location_taxi = (HAL.getPose3d().x,HAL.getPose3d().y) # Coordenadas del taxi en el mundo
location_taxi = MAP.rowColumn(location_taxi)
location_taxi = (location_taxi[0], location_taxi[1])
print("LOCALIZACION DEL TAXI",location_taxi)
time.sleep(10)

while True:
    # Enter iterative code!
    map_img = MAP.getMap()
    goal_world = GUI.getTargetPose() # Devuelve las coords en el mundo
    
    # Paso 1: inserte el nodo de destino en la cola de prioridad
    if not goal_world is None and not goal_found:
        print("Goal En Mundo:", goal_world)
        
        goal_map = MAP.rowColumn(goal_world)
        goal_map = (goal_map[0], goal_map[1])
        gradient_planning.put((1, goal_map))
        print("Goal En Mapa:",goal_map)
        map_array[goal_map[1], goal_map[0]] = 1
        goal_found = True
        
    # Para hacer el mapa gradiente
    while goal_found and not gradient_done and not gradient_planning.empty():
        # Paso 2: c = sacar el nodo de la cola de prioridad
        current_node = gradient_planning.get()
        
        # Paso 3: si c == iniciar el nodo Finalizar
        if current_node[1] == location_taxi:
            gradient_done = True
            print("ENCONTRE META", map_array[location_taxi[1],location_taxi[0]])
            break
        
        # Paso 4: si c == obstáculo Guardar en otra lista e ir al Paso 2
        if current_node[1] not in obstacles:
            if map_img[current_node[1][1],current_node[1][0]] == 0:
                obstacles.append(current_node[1])
                map_array[current_node[1][1],current_node[1][0]] = 0
                continue
        
        if current_node[1] in expanded:
            continue
        
        expanded.append(current_node[1])
            
        (x,y) = current_node[1]
        # Consigo los nodos vecinos
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1), (x+1, y+1), (x-1, y+1), (x+1, y-1), (x-1, y-1)]
        count = 0
        for nghb in neighbors:
            count += 1
            if not (0 <= nghb[0] <= 399 and 0 <= nghb[1] <= 399):
                continue

            if map_img[nghb[1], nghb[0]] == 0:
                map_array[nghb[1], nghb[0]] = 0
                continue
            
            # Paso 5: asignar peso a los vecinos de c si anteriormente sin asignar
            if nghb in expanded and nghb in nghb_visited:
                continue
            
            if count >= 5:
                cost = 0.8
            else:
                cost = 0.4
                
            nghb_visited.append(nghb)
            map_array[nghb[1],nghb[0]] = current_node[0] + cost
            
            # Paso 6: inserte vecinos de c en la cola de prioridad
            gradient_planning.put((current_node[0] + cost, nghb))
        
        GUI.showNumpy(map_array)
        # Paso 7: vaya al Paso 2
    
    
    while not widen_walls and gradient_done:
        
        if wall_pixel <= 0:
            widen_walls = True
            break
        
        for i in range(400):
            for j in range(400):
                if wall_pixel == 800:
                    detected_wall = (map_array[j][i] == 0)
                else:
                    detected_wall = (map_array[j][i] == wall_pixel+400)

                if detected_wall:
                    neighbors = [(i-1, j), (i+1, j), (i, j-1), (i, j+1), (i+1, j+1), (i-1, j+1), (i+1, j-1), (i-1, j-1)]
                    for nghb in neighbors:
                        if not (0 <= nghb[0] <= 399 and 0 <= nghb[1] <= 399):
                            continue
                        if map_array[nghb[1]][nghb[0]] == 0:
                            continue
                        if map_array[nghb[1]][nghb[0]] == wall_pixel+400:
                            continue
                        
                        if wall_pixel != 0:
                            map_array[nghb[1]][nghb[0]] = wall_pixel
                            print("NUEVO VALOR:",map_array[nghb[1]][nghb[0]])
                            
                GUI.showNumpy(map_array)
        
            
        wall_pixel -= 400
    
    GUI.showNumpy(map_array)
    
    
    if gradient_done and not path_found:
        print(location_taxi)
        pth_planning.put((map_array[location_taxi[1], location_taxi[0]], location_taxi))
        path.append([location_taxi[0],location_taxi[1]])
        nghb_visited = list()
        expanded = list()
    
    while gradient_done and not path_found and not pth_planning.empty():
        
        current_node = pth_planning.get()
        
        if current_node[1] == goal_map:
            path_found = True
            print("ENCONTRE CAMINO")
            print(path)
            break
        
        if current_node[1] in expanded or current_node[1] in obstacles:
            continue
        
        expanded.append(current_node[1])
        (x,y) = current_node[1]
        # Obtengo los nodos vecinos
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1), (x+1, y+1), (x-1, y+1), (x+1, y-1), (x-1, y-1)]
        
        nghb_lowcost = 10000
        
        for nghb in neighbors:
            if not (0 <= nghb[0] <= 399 and 0 <= nghb[1] <= 399):
                continue
            
            if nghb in expanded:# and nghb in nghb_visited:
                continue
            
            #nghb_visited.append(nghb)
            cost = map_array[nghb[1],nghb[0]]
            if nghb_lowcost > cost and cost != 0:
                nghb_lowcost = cost
                next_node = nghb
                #print("NODO SIGUIENTE:",next_node)
                
        if nghb_lowcost != 10000:        
            path.append([next_node[0],next_node[1]])
            pth_planning.put((cost, next_node))
            
        
        GUI.showPath(path) #-> cuando ya tenga la ruta, array2D con los puntos
    
    
    if path_found:
        
        GUI.showPath(path)
        time.sleep(60)
    
    
    
    
    
    
    
    
    
    
