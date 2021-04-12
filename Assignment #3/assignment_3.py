import math
import matplotlib.pyplot as plt
import numpy as np
import random
import numpy.linalg as LA
from math import sqrt

path_angles = "C:/Users/pabli/Desktop/Lab inteligencia/Assignment #3/angles.txt"
path_distances = "C:/Users/pabli/Desktop/Lab inteligencia/Assignment #3/distances.txt"

def read_txt_data(path):
    """Funcion para leer archivos que tienen los datos del sensor recopilados en el simulador.
    Los datos deben estar separados por coma (así vienen por defecto)"""
    my_file = open(path, "r")
    content = my_file.read()
    content_list = content.split(",")
    my_file.close()
    content_list = list(map(float, content_list))
    return content_list

##Data leida
angles = read_txt_data(path_angles)
distances = read_txt_data(path_distances)

###PREPROCESAMIENTO

def coordenadasXY(angles, dist, ang_actual=0): 
    '''A partir de la lista de ángulos y distancias que mide el sensor, la función retorna una tupla que contiene 
    la lista con las coordenadas en X y otra lista con las coordenadas en Y '''
    angles = list(angles)
    angles =  [x+ang_actual*math.pi/180 for x in angles]
    X_list = []
    Y_list = []
    max_d = max(dist) #5.0 #valor máximo que se sale de los limites (el sensor capta el vacio como limite)
    assert len(angles) == len(dist)
    for i in range(len(angles)):
        d = dist[i]
        ang = angles[i]
        if d >= max_d: #preprocesar valores del extremo
            pass
        else:
            x = d * math.cos(ang)
            y = d * math.sin(ang)
            X_list.append(x)
            Y_list.append(y)
    return X_list, Y_list

def plotCoordenadas(coord, XSelect = None, YSelect = None, XTest = None, YTest = None, plot = True): 
    '''Esta función recibe una tupla de dos elementos, que corresponden a las listas de las coordenadas X e Y obtenidas a partir de las distancias
    y ángulos medidos por el sensor y grafica los puntos correspondientes en el plano XY. Además recibe lista o coordenadas de puntos que son
    graficados con otro color para resaltar '''
    X = coord[0]
    Y = coord[1]
    plt.plot(X,Y,'*')

    if XTest != None and YTest != None:
        plt.plot(XTest, YTest, 'm*')

    if XSelect != None and YSelect != None:
        plt.plot(XSelect, YSelect, 'r*-')
        #plt.plot(XSelect, YSelect, 'r-')
    if plot:
        plt.figure(1)
        plt.grid()
        plt.xlabel('Coordenada X')
        plt.ylabel('Coordenada Y')
        plt.legend(['Puntos detectados', 'Puntos cercanos', 'Puntos aleatorios'], loc='upper right')
        plt.title("Puntos detectados por sensor del robot en world.cave")
        plt.show()

def get_random_points(coordX, coordY):
    assert len(coordX) == len(coordY)
    nMAx = len(coordX)-1
    random_index_1 = random.randint(0, nMAx)
    random_index_2 = random.randint(0, nMAx)
    while random_index_1 == random_index_2:
        random_index_2 = random.randint(0, nMAx)
    #print(random_index_1, random_index_2)
    random_X1, random_Y1 = coordX[random_index_1], coordY[random_index_1]
    random_X2, random_Y2 = coordX[random_index_2], coordY[random_index_2]
    return random_X1, random_Y1, random_X2, random_Y2

#Distancia de punto p3 a recta formada por p1-p2
def distancePointLine(p1, p2, p3):
    '''Distancia desde el punto P3 hacia la línea formada por P1 y P2'''
    dist = LA.norm(np.cross(p2-p1, p1-p3))/LA.norm(p2-p1) 
    return dist

#Distancia de punto E a segmento AB
def minDistance(A, B, E):
 
    # vector AB
    AB = [None, None]
    AB[0] = B[0] - A[0]
    AB[1] = B[1] - A[1]
 
    # vector BP
    BE = [None, None]
    BE[0] = E[0] - B[0]
    BE[1] = E[1] - B[1]
 
    # vector AP
    AE = [None, None]
    AE[0] = E[0] - A[0]
    AE[1] = E[1] - A[1]
 
    # Variables to store dot product
 
    # Calculating the dot product
    AB_BE = AB[0] * BE[0] + AB[1] * BE[1]
    AB_AE = AB[0] * AE[0] + AB[1] * AE[1]
 
    # Minimum distance from
    # point E to the line segment
    reqAns = 0
 
    # Case 1
    if (AB_BE > 0) :
 
        # Finding the magnitude
        y = E[1] - B[1]
        x = E[0] - B[0]
        reqAns = sqrt(x * x + y * y)
 
    # Case 2
    elif (AB_AE < 0) :
        y = E[1] - A[1]
        x = E[0] - A[0]
        reqAns = sqrt(x * x + y * y)
 
    # Case 3
    else:
 
        # Finding the perpendicular distance
        x1 = AB[0]
        y1 = AB[1]
        x2 = AE[0]
        y2 = AE[1]
        mod = sqrt(x1 * x1 + y1 * y1)
        reqAns = abs(x1 * y2 - y1 * x2) / mod
     
    return reqAns

def countNearPoints(dMax, coordLinea, coordX, coordY):
    #coordLinea: coordenadas de la línea (dos puntos la forman) --> get_random_points
    #dMax: valor máximo de distancia (umbral para contar un punto)

    n = 0 #inicialización del contador de puntos cercanos
    X1, Y1, X2, Y2 = coordLinea
    selected_X = []
    selected_Y = []
    #Puntos random que forman la línea
    p1 = np.array((X1, Y1))
    p2 = np.array((X2, Y2))
    for i in range(len(coordX)):
        X = coordX[i]
        Y = coordY[i]
        p = np.array((X,Y)) #Punto desde el que se mide la distancia a la línea (cada punto medido)
        if X1 == X or X2 == X: 
            pass #No se mide la distancia de los puntos a la linea formada por ellos mismos
        else:
            #d = distancePointLine(p1, p2, p) #distancia medida desde p a la linea formada por p1 y p2
            d = minDistance(p1, p2, p)
            if d<=dMax: #si la distancia es menor al maximo seteado, se cuenta el punto
                n+=1
                selected_X.append(X)
                selected_Y.append(Y)
    #plotCoordenadas((coordX,coordY), [X1, X2], [Y1, Y2], selected_X, selected_Y)
    return n, selected_X, selected_Y
    
coords = coordenadasXY(angles, distances) #Data procesada para obtener coordenadas X e Y de cada punto detectado
#plotCoordenadas(coords) #Graficar puntos obtenidos
coord_X = coords[0]
coord_Y = coords[1]  

#RandomPoints = get_random_points(coord_X, coord_Y)
#X1, Y1, X2, Y2 = RandomPoints
#print(countNearPoints(0.5, RandomPoints, coord_X, coord_Y))

###RANSAC
def runRANSAC(coordX, coordY, kNeed, dMax, LTimes, plot = False):
    '''Se -corre- el algoritmo RANSAC para los puntos de la forma (coordX[i], coordY[i])'''
    itNum = 0 #Número de iteraciones que han transcurrido
    found = False
    while (not found):
        if itNum == LTimes:
            break
        RandomPoints = get_random_points(coordX, coordY) #Obtener puntos random
        countNear = countNearPoints(dMax, RandomPoints, coordX, coordY)
        KObtenido = countNear[0] #Número de puntos que se ajustan al segmento
        X1, Y1, X2, Y2 = RandomPoints
        selected_X = countNear[1]
        selected_Y = countNear[2]

        if kNeed <= KObtenido: #Si se encuentra una línea que se ajusta a kNeed o más puntos,
                               #se elige la línea y se termina el algoritmo.
            found = True
            print('Iteración ', itNum, 'exitosa - KObtenido = ',KObtenido)
            plotCoordenadas((coordX,coordY), [X1, X2], [Y1, Y2], selected_X, selected_Y, plot)
        else: 
            print('Iteración ', itNum, 'fallida - KObtenido = ',KObtenido)
            pass
        itNum+=1
    if found:
        return RandomPoints #se retornan los puntos obtenidos en caso que el ransac se exitoso
    else:
        pass #si no se obtiene, no se retorna nada

#runRANSAC(coord_X, coord_Y, 20, 0.5, 20, True)

def getRANSACparams(coordX, coordY, kNeed, dMax, nIterat):
    itNum = 0 #Número de iteraciones que han transcurrido
    itSucceded = 0 #Iteraciones exitosas
    sumProb = 0
    assert len(coordX) == len(coordY)
    pointsTotal = len(coordX)
    while itNum < nIterat:
        RandomPoints = get_random_points(coordX, coordY) #Obtener puntos random
        KObtenido = countNearPoints(dMax, RandomPoints, coordX, coordY)[0] #Número de puntos que se ajustan al segmento
        sumProb += KObtenido/pointsTotal #puntos ajustados/puntos total
        if kNeed <= KObtenido: #Si se encuentra una línea que se ajusta a kNeed, se considera una iteración exitosa.
            itSucceded+=1
            #print('Iteración ', itNum, 'exitosa - KObtenido = ',KObtenido)
        else: 
            pass
            #print('Iteración ', itNum, 'fallida - KObtenido = ',KObtenido)
        itNum+=1
    pFail =  (itNum - itSucceded)/itNum #probabilidad de que una iteración sea fallida.
    pG = sumProb/itNum
    L = math.log2(pFail)/math.log2(1-pG**2)
    print('pFail = ', pFail)
    print('pG = ', pG)
    print('L = ', L)
    return L

#getRANSACparams(coord_X, coord_Y, 10, 0.1, 1000)

def executeRANSAC(coordX, coordY, kNeed, dMax, niteratTest):
    L = round(getRANSACparams(coordX, coordY, kNeed, dMax, niteratTest))
    return runRANSAC(coordX, coordY, kNeed, dMax, L, plot = True )

#Obtener L, pFail y pG para distintos valores
#for i in range(20,31,1):
    #for j in np.arange(0.1, 1.7, 0.1):
        #print('----------------------------------------')
        #print('K = ', i, '--- d = ', j)
        #getRANSACparams(coord_X, coord_Y, i, j, 1000)


###Pruebas mostradas en el informe

#K = 20; d = 1.0
#executeRANSAC(coord_X, coord_Y, 20, 1.0, 1000)

#K = 22; d = 0.5
#executeRANSAC(coord_X, coord_Y, 22, 0.5, 1000)

#K = 25; d = 0.1
#executeRANSAC(coord_X, coord_Y, 25, 0.1, 1000)

#K = 30; d = 0.8
executeRANSAC(coord_X, coord_Y, 30, 0.8, 1000)
