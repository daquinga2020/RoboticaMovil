from GUI import GUI
from HAL import HAL
from MAP import MAP
import numpy as np
from queue import PriorityQueue
import cv2
import time
# PINTA EL GRADIENTE
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
current_pth = PriorityQueue()

path = []

goal_found = False
path_found = False
gradient_done = False

map_array = np.full((400,400),0)
obstacles = list()
expanded = list()
nghb_visited = list()

location_taxi = (HAL.getPose3d().x,HAL.getPose3d().y) # Coordenadas del taxi en el mundo
location_taxi = MAP.rowColumn(location_taxi)
location_taxi = (location_taxi[0], location_taxi[1])
print(location_taxi)
time.sleep(10)

while True:
    # Enter iterative code!
    map_img = MAP.getMap()
    goal_world = GUI.getTargetPose() # Devuelve las coords en el mundo
    
    # MAP.rowColumn(vector) -> devuelve indice en coords mapa
    # correspondiente al vector en coords del mundo, por ejemplo
    # el getPose.
    
    
    
    # Paso 1: inserte el nodo de destino en la cola de prioridad
    if not goal_world is None and not goal_found:
        #pth_planning.put((0, goal_world)) # Almacenamos el destino 
        print("Goal En Mundo:", goal_world)
        
        goal_map = MAP.rowColumn(goal_world)
        pth_planning.put((1, goal_map))
        print("Goal En Mapa:",goal_map)
        map_array[goal_map[1], goal_map[0]] = 1
        goal_found = True
        
    # Para hacer el mapa gradiente
    while goal_found and not gradient_done and not pth_planning.empty():
        # Paso 2: c = sacar el nodo de la cola de prioridad
        current_node = pth_planning.get()
        #print("NODO ACTUAL", current_node[1], location_taxi)
        # Paso 3: si c == iniciar el nodo Finalizar
        if current_node[1] == location_taxi:
            gradient_done = True
            print("ENCONTRE META")
            break
        
        #print("COMPRUEBO SI EL NODO ES OBSTACULO")
        # Paso 4: si c == obstáculo Guardar en otra lista e ir al Paso 2
        if current_node[1] not in obstacles:
            if map_img[current_node[1][1],current_node[1][0]] == 0:
                obstacles.append(current_node[1])
                map_array[current_node[1][1],current_node[1][0]] = 0
                continue
        
       #print("COMPRUEBO SI EL NODO YA SE HA EXPANDIDO")
        if current_node[1] in expanded:
            continue
        
        expanded.append(current_node[1])
            
        (x,y) = current_node[1]
        # Get neighbors
        #print("OBTENGO LOS VECINOS DEL NODO ACTUAL")
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
        
        for nghb in neighbors:
            
            if not (0 <= nghb[0] <= 399 and 0 <= nghb[1] <= 399):
                #continue
            #print("COMPRUEBO SI EL VECINO ES UN OBSTACULO")
            if map_img[nghb[1], nghb[0]] == 0:
                map_array[nghb[1], nghb[0]] = 0
                #obstacles.append(nghb)
                continue
            
            # Paso 5: asignar peso a los vecinos de c si anteriormente sin asignar
            if nghb in expanded and nghb in nghb_visited:
                continue
            nghb_visited.append(nghb)
            #print("ASIGNO VALOR AL MAPA")
            map_array[nghb[1],nghb[0]] = current_node[0]+0.8
            #temp_pth = path + [nghb]
            #current_pth.put((current_node[0]+0.8, temp_pth))
            
            # Paso 6: inserte vecinos de c en la cola de prioridad
            pth_planning.put((current_node[0]+0.8, nghb))
        
        path = current_pth.get()[1]
        print("CAMINO ACTUAL",path)
        print("")
        #print("")
        GUI.showNumpy(map_array)
    
        # Paso 7: vaya al Paso 2
    
    GUI.showNumpy(map_array)
    #GUI.showPath(path) #-> cuando ya tenga la ruta, array2D con los puntos
    
    
    
    
    
    
    
    
    
    
    
    
