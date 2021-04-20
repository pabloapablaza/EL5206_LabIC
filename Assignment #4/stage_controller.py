#!/usr/bin/env python3
# Autor> Ruben Claveria Vega
# Curso> EL5206 Laboratorio de Inteligencia Computacional y Robotica

# En lugar de 'testpy', deberia ir el nombre del paquete creado.
#import roslib; roslib.load_manifest('testpy')
import robot_utilities
import time
import rospy
import math
import numpy
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry     
from geometry_msgs.msg import Twist
from std_msgs.msg import String 

class Controller:
	def go_to(R , target_x, target_y, path_mode = 'straight'):
		#Constantes robot
		A = 0.0011
		w = 0.3963
		
		#Valores iniciales (posiciòn actual)
		x0 = R.pose_x
		y0 = R.pose_y
		Th0 = R.angle
		Th0_rad = Th0 * math.pi/180
		
		#Variables input
		x1 = target_x
		y1 = target_y 
		
		#Variables calculadas
		dx = x1 - x0
		dy = y1 - y0
		print("dx", dx)
		print("dy", dy)
		
		####MOV RECTO####
		if path_mode == 'straight':
			print('Going straight to: (' +str(target_x)+' , 					'+str(target_y) + ')')
			
			#dTh = math.atan(dy/dx)
			
			#correcciòn de àngulos segùn cuadrantes
			if dx == 0: 
				dTh = 2*math.pi + math.pi/2 - Th0_rad
			if dy == 0:
				dTh = 2*math.pi - Th0_rad
			elif dx> 0 and dy >0:
				dTh = 2*math.pi + math.atan(dy/dx) - Th0_rad
				print(1)
			elif dx < 0 and dy > 0:
				dTh = math.pi + math.atan(dy/dx) - Th0_rad
				print(2)
			elif dx < 0 and dy < 0:
				dTh = math.pi + math.atan(dy/dx) - Th0_rad
				print(3)
			elif dx > 0 and dy < 0: 
				dTh = 2*math.pi + math.atan(dy/dx) - Th0_rad
				print(4)

			nR1 = (dTh * w)/(2 * A)
			nL1 = -nR1 
			print ("nR1: ",nR1,"nL1: ",nL1) 
			R.nSteps(nL1,nR1)
							
			x0 = R.pose_x
			y0 = R.pose_y
			x1 = target_x
			y1 = target_y 
		
			#Variables calculadas
			dx = x1 - x0
			dy = y1 - y0
			d = math.sqrt(dx**2 + dy**2)
			nR2 = d/A
			nL2 = nR2
			print ("nR2: ", nR2, "nL2: ", nL2) 
			R.nSteps(nL2,nR2)
			print("angulo", R.angle)
			print('Arrived at: (' +str(R.pose_x)+' , '+ str(R.pose_y) + ')')
		####MOV CURVO####
		if path_mode == 'smooth':
			print('Going smoothly to: (' +str(target_x)+' , '+ str(target_y) + ')')
			
			#rotar el robot en su eje
			print("angulo inicial", R.angle)
			##Ajuste de angulos
			tg = math.atan(dy/dx)
			if dx > 0 and dy > 0:
				ThF = 2*math.pi - (math.pi/2 - tg)
				dTh = ThF - Th0_rad
				dTh1 = math.pi
				print("c", 1)
			elif dx < 0 and dy > 0:
				ThF = math.pi + (math.pi/2 + tg)
				dTh = ThF - Th0_rad
				dTh1 = -math.pi
				print("c", 2)
			elif dx < 0 and dy < 0:
				ThF = math.pi - (math.pi/2 - tg)
				dTh = ThF - Th0_rad
				dTh1 = math.pi
				print("c", 3)
			elif dx > 0 and dy < 0:
				ThF = math.pi/2 + tg
				dTh = ThF - Th0_rad
				dTh1 = - math.pi
				print("c", 4)
			
			if dTh < 0:
				dTh = (2*math.pi + dTh)
			
			print("dTh", dTh*180/math.pi)
			nR1 = (dTh * w)/(2 * A)
			nL1 = -nR1
			R.nSteps(nL1,nR1)
			print("angulo final", R.angle)
			
			
			#moverse en curva hacia el punto de destino
			x0 = R.pose_x
			y0 = R.pose_y
			x1 = target_x
			y1 = target_y 
		
			#Variables calculadas
			dx = x1 - x0
			dy = y1 - y0
			
			d = math.sqrt(abs(dx)**2 + abs(dy)**2) #distancia entre los puntos 
			assert d>0
			r = d/2 #radio de la circunferencia
			deltaS = r * math.pi #perimetro de la curva
			#dTh1 = math.pi; #angulo entre extremos
			
			nL = deltaS/A - dTh1 * w/(2*A)
			nR = (dTh1 * w)/A + nL
			print ("nR: ",nR,"nL: ",nL) 
			R.nSteps(nL,nR)
			print("angulo", R.angle)
			print('Arrived at: (' +str(R.pose_x)+' , '+ str(R.pose_y) + ')')
		####MOV PLANIFICADO####
		if path_mode == 'planned':
			print('Going planned to: (' +str(target_x)+' , '+ str(target_y) + ')')
			
			#Calcular fuerza de atracciòn
			
			def attractiveFrz(poseX, poseY, targetX, targetY):
			    #posicion del robot
			    x0 = poseX
			    y0 = poseY
			    
			    #posicion objetivo
			    x1 = targetX
			    y1 = targetY
			    kAtt = 1
			    rMax = 2

			    xAtt = x1 - x0
			    yAtt = y1 - y0
			    Rd = math.sqrt(xAtt**2 + yAtt**2)
			    
			    PAtt = kAtt*Rd #PAtt es la fuerza atractiva en la posicion x0, y0 para llegar a x1, y1
			    

			    if Rd <= rMax:
			    	PAtt = kAtt*Rd
			    elif Rd > rMax:
			    	PAtt = kAtt*rMax
			    	xAtt = kAtt*(rMax/Rd)*xAtt
			    	yAtt = kAtt*(rMax/Rd)*yAtt
			    return PAtt, xAtt, yAtt

			def repulsiveFrz(angles, dist, angActual):
			    dist =  list(dist)
			    angles = list(angles)
			    angles =  [x+angActual*math.pi/180 for x in angles] #reescalar angulos segun pos de referencia
			    anglesProc = []
			    distProc = []
			    max_d = max(dist)
			    assert len(dist) == len(angles)
			    for i in range(len(angles)):
			    	d = dist[i]
			    	ang = angles[i]
			    	if d >= max_d: #preprocesar valores del extremo
			    		pass
		    		else: #si no esta en el extremo se procesa.
		    			anglesProc.append(ang)
		    			distProc.append(d)
	    			#se tiene una lista con angulos y distancias
			    Krep = 0.001
			    assert len(anglesProc) == len(distProc)
			    F = 0
			    S = 0
			    for i in range(len(anglesProc)):
			    	f1 = 1/(distProc[i]**2) * math.cos(anglesProc[i])
			    	s1 = 1/(distProc[i]**2) * math.sin(anglesProc[i])
			    	F+=f1
			    	S+=s1
			    Xrep = Krep * F * math.cos(angActual) - Krep * S * math.sin(angActual)
			    Yrep = Krep * F * math.sin(angActual) + Krep * S * math.cos(angActual)
			    return Xrep, Yrep
			    
			def netFrz(attFrz, repFrz):
			    xAtt = attFrz[1]
			    yAtt = attFrz[2]
			    xRep = repFrz[0]
			    yRep = repFrz[1]
			    Px = xAtt - xRep
			    Py = yAtt - yRep
			    P = math.sqrt(Px**2 + Py**2)
			    direction = math.atan2(Py, Px) 
			    return Px, Py, P
			    
			x0 = R.pose_x
			y0 = R.pose_y
			Th0 = R.angle
			Th0_rad = Th0 * math.pi/180
			
			x1 = target_x
			y1 = target_y
			
			angl = R.laser_angles
			dist = R.distances
			
			#Variables input
			
			#print('x0, y0', x0, y0)    
			
			#print('Fuerzas atractivas', attFrz)
			#print('Xatt = ', attFrz[1])
			#print('Yatt = ', attFrz[2])
			
			#-------------------------------------------
			
			#print('Fuerzas repulsivas', repFrz)
			#print('Xrep', repFrz[0])
			#print('Yrep', repFrz[1])
			
			#-----------------------------------------
			
			#print('Fuerza Neta', fuerzaNeta)
			#print('Xtot', fuerzaNeta[0])
			#print('Ytot', fuerzaNeta[1])
			#--------------------------------------------
			
			attFrz = attractiveFrz(x0, y0, x1, y1)
			repFrz = repulsiveFrz(angl, dist, Th0_rad)
			fuerzaNeta = netFrz(attFrz, repFrz)
			P = fuerzaNeta[2]
			d = math.sqrt(abs(x0 - x1)**2 + abs(x0 - x1)**2)
			
			while P>0.5 and d > 0.2:
				x0 = R.pose_x #x actual
				y0 = R.pose_y #y actual 
				Th0 = R.angle #Angulo actual
				Th0_rad = Th0 * math.pi/180
				
				x1 = target_x
				y1 = target_y
				
				angl = R.laser_angles
				dist = R.distances
				
				attFrz = attractiveFrz(x0, y0, x1, y1)
				repFrz = repulsiveFrz(angl, dist, Th0_rad)
				fuerzaNeta = netFrz(attFrz, repFrz)
				print(fuerzaNeta[0], fuerzaNeta[1])
				dx = fuerzaNeta[0]
				dy = fuerzaNeta[1]
				
				if dx == 0: 
					dTh = 2*math.pi + math.pi/2 - Th0_rad
				if dy == 0:
					dTh = 2*math.pi - Th0_rad
				elif dx> 0 and dy >0:
					#dTh = 2*math.pi + math.atan(dy/dx) - Th0_rad
					dTh = math.atan2(dy,dx) - Th0_rad

				elif dx < 0 and dy > 0:
					dTh = math.pi + math.atan(dy/dx) - Th0_rad

				elif dx < 0 and dy < 0:
					dTh = math.pi + math.atan(dy/dx) - Th0_rad

				elif dx > 0 and dy < 0: 
					dTh = 2*math.pi + math.atan(dy/dx) - Th0_rad
					
				nR1 = (dTh * w)/(2 * A)
				nL1 = -nR1
				print ("nR1: ",nR1,"nL1: ",nL1) 
				R.nSteps(nL1,nR1)
				
				dt = 0.2
				dx = dx * dt #Px
				dy = dy * dt #Py
				
				d = math.sqrt(dx**2 + dy**2)
				nR2 = d/A
				nL2 = nR2
				print ("nR2: ", nR2, "nL2: ", nL2) 
				R.nSteps(nL2,nR2)
				print('Arrived at: (' +str(R.pose_x)+' , '+ str(R.pose_y) + ')')
				
				time.sleep(2)
				
				x0 = R.pose_x #medir x actual
				y0 = R.pose_y #medir y actual 
				Th0 = R.angle #medir Angulo actual
				Th0_rad = Th0 * math.pi/180
				
				x1 = target_x
				y1 = target_y
				
				angl = R.laser_angles
				dist = R.distances
				
				attFrz = attractiveFrz(x0, y0, x1, y1)
				repFrz = repulsiveFrz(angl, dist, Th0_rad)
				fuerzaNeta = netFrz(attFrz, repFrz)
				angl = R.laser_angles
				dist = R.distances
				
				d = math.sqrt(abs(R.pose_x-target_x)**2 + abs(R.pose_y-target_y)**2)
				P = fuerzaNeta[2]
				
				
				
			
			'''while P > 2 and d > 0.2:
				x0 = R.pose_x
				y0 = R.pose_y
				Th0 = R.angle
				Th0_rad = Th0 * math.pi/180
				#Variables input
				x1 = target_x
				y1 = target_y
				attFrz = attractiveFrz(x0, y0, x1, y1)
				repFrz = repulsiveFrz(angl, dist, Th0_rad)
				fuerzaNeta = netFrz(attFrz, repFrz)
				
				dx = fuerzaNeta[0] #Px
				dy = fuerzaNeta[1] #Py
				
				#dTh = math.atan(dy/dx)
				
				#correcciòn de àngulos segùn cuadrantes
				if dx == 0: 
					dTh = 2*math.pi + math.pi/2 - Th0_rad
				if dy == 0:
					dTh = 2*math.pi - Th0_rad
				elif dx> 0 and dy >0:
					dTh = 2*math.pi + math.atan(dy/dx) - Th0_rad
					print(1)
				elif dx < 0 and dy > 0:
					dTh = math.pi + math.atan(dy/dx) - Th0_rad
					print(2)
				elif dx < 0 and dy < 0:
					dTh = math.pi + math.atan(dy/dx) - Th0_rad
					print(3)
				elif dx > 0 and dy < 0: 
					dTh = 2*math.pi + math.atan(dy/dx) - Th0_rad
					print(4)
					
				#d = math.sqrt(abs(R.pose_x-target_x)**2 + abs(R.pose_y-target_y)**2)
				
				x0 = R.pose_x
				y0 = R.pose_y
				Th0 = R.angle
				Th0_rad = Th0 * math.pi/180
				#Variables input
				x1 = target_x
				y1 = target_y
				attFrz = attractiveFrz(x0, y0, x1, y1)
				repFrz = repulsiveFrz(angl, dist, Th0_rad)
				fuerzaNeta = netFrz(attFrz, repFrz)

				nR1 = (dTh * w)/(2 * A)
				nL1 = -nR1 
				print ("nR1: ",nR1,"nL1: ",nL1) 
				R.nSteps(nL1,nR1)
				
				dt = 0.05
				dx = fuerzaNeta[0]*dt #Px
				dy = fuerzaNeta[1]*dt #Py
				
				d = math.sqrt(dx**2 + dy**2)
				nR2 = d/A
				nL2 = nR2
				print ("nR2: ", nR2, "nL2: ", nL2) 
				R.nSteps(nL2,nR2)
				print("angulo", R.angle)
				print('Arrived at: (' +str(R.pose_x)+' , '+ str(R.pose_y) + ')')
				d = math.sqrt(abs(R.pose_x-target_x)**2 + abs(R.pose_y-target_y)**2)
				print(d)
				fuerzaNeta = netFrz(attFrz, repFrz)
				P = fuerzaNeta[2]
				print(P)
				time.sleep(2) '''
	
			
	def __init__(self):

		# inicializa nodo ROS
		rospy.init_node('Simulation');

		# robotID = ID del robot que se desea controlar
		# si hay solo un robot, robotID debe ser ""
		# si hay mas de un robot, robotID debe ser de la forma "/robot_0", "/robot_1", etc.
		robotID = ""

		# Posicion y orientacion iniciales.
		# IMPORTANTE: deben ser consistentes con la del archivo .world
		init_x = -6.0
		init_y = -6.0
		init_angle = 90.0

		# creacion de un objeto de tipo Robot
		R = robot_utilities.Robot(robotID, init_x, init_y, init_angle)

		# Subscripcion a los topicos de interes: odometria (Odometry) y sensor de distancia (LaserScan)
		rospy.Subscriber(robotID+'/odom', Odometry, R.odom_callback)
		rospy.Subscriber(robotID+'/base_scan', LaserScan, R.ranger_callback)
		# Observacion: Las funciones de callback se ejecutan cada vez que el programa recibe informacion 
		# desde un topico. Se utilizan, generalmente, para actualizar datos.
		# Pueden utilizarse funciones de callback programadas por el alumno, 
		# diferentes a las provistas por la clase Robot

		# Se recomienda que este ciclo vaya antes de todas las instrucciones de control
		# Sirve para asegurar que el Robot recibio su primera lectura de sensor
		while not rospy.is_shutdown() and len(R.distances)==0:
			R.rate.sleep()

		# Ciclo de ejemplo. 
		# Si se va a controlar el robot con una funcion de robot_utilities en vez de con el ciclo:
		# opcion 1: cambiar True por False en la condicion del while
		# opcion 2: comentar el bloque de codigo
		while not rospy.is_shutdown() and False:
			# Define velocidad lineal [metros/segundo] del robot durante la iteracion
			R.cmd.linear.x = 0.2
			
			# Velocidad angular [radianes/segundo]; rango aceptable: [-pi/2, pi/2 ], aprox. [-1.57, 1.57]
			R.cmd.angular.z = 0.0 

			# indica a Stage la velocidad que el robot tendra durante un ciclo
			R.cmd_pub.publish(R.cmd)
			
			# Ejemplo de como rescatar datos captados por los sensores
			# En este caso solo se imprimen, pero la idea es utilizarlos en decisiones de control
			print( "_______________________")
			print( 'Lasers [m]: '+str(R.distances))
			print( 'angulo lase [grados]: '+str(R.laser_angles))
			#R.show_distance()
	
			# Mantiene la regularidad temporal (de acuerdo al rate definido en robot_utilities.Robot)
			R.rate.sleep()
		

		# Funciones de ejemplo (uncomment para usar): 
		
		#nR_1 = 2856
		#nL_1 = 2284.8
		#print("nL = ",nL_1)
		#print("nR = ",nR_1)
		
		#x_fin = 12
		#y_fin = 12
		#angl = R.laser_angles
		#dist = R.distances
		
		#print(angl)
		#print(dist)

		#crd = coordenadasXY(angl, dist)
		#print("coordenadas")
		#print(crd)

		#Controller.go_to(R,-3.5,-3.5)
		#Controller.go_to(R,10,11)
		#Controller.go_to(R,4,-1)
		#Controller.go_to(R,0,7)
		#Controller.go_to(R,-5,-2)
		
		Controller.go_to(R,0,0,"planned")
		#Controller.go_to(R,6,5,"smooth")
		#Controller.go_to(R,6,5,"smooth")
		#Controller.go_to(R,6,5,"smooth")
		#Controller.go_to(R,16,8)
		#Controller.go_to(R,16,8)
		#Controller.go_to(R,16,8)
		
		#Controller.go_to(R,11,-2)
		#Controller.go_to(R,11,-2)
		#Controller.go_to(R,11,-2)
		#Controller.go_to(R,14,8)
		#Controller.go_to(R,14,-10)
		
		#print("----VALORES INICIALES----")
		#print(R.pose_x)
		#print(R.pose_y)
		#print(R.angle)
		#print(R.perim)
		#R.nSteps(1453,2585)
		#print("----VALORES FINALES----")
		#print(R.pose_x)
		#print(R.pose_y)
		#print(R.angle)
		#print(R.perim)
		
		
		
				#R.nSteps(1500,1500)
		#R.nSteps(900,1500)
		#R.moveTill(math.pi*5.0/2, 0.5, 0.1)
        
if __name__ == "__main__": Controller()
