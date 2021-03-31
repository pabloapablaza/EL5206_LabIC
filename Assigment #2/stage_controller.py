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
			print('Going straight to: (' +str(target_x)+' , '+ str(target_y) + ')')
			
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
		
		x_fin = 12
		y_fin = 12
		#Controller.go_to(R,8,8,"smooth")
		#Controller.go_to(R,11,9,"smooth")
		#Controller.go_to(R,11,12,"smooth")
		Controller.go_to(R,7,9,"smooth")
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
