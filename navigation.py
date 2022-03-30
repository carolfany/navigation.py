#!/usr/bin/env python
# -*- coding: utf-8 -*-
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2, tan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
from tf2_msgs.msg import TFMessage
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import numpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2, tan
import math
from time import sleep
from std_msgs.msg import String
import time
angMax=0
angMin=0
count=0
vet = []
q_x = []
q_y = []
q_z = []
q_w = []
q = ()
euler = ()
roll = 0.0
pitch = 0.0
yaw = 0.0
global prev_error 
prev_error = 0.0
global sum_error
sum_error = 0.0
global i
global estado
estado = 'none'



def callback_imu(imu): # Lê a IMU do robô e transforma seus valores de quarternion p/ ângulos de Euler 
	global q_x
	q_x = imu.orientation.x
	global q_y
	q_y = imu.orientation.y
	global q_z
	q_z = imu.orientation.z
	global q_w	
	q_w = imu.orientation.w
	global q	
	q = imu.orientation
	global euler
	global roll, pitch, yaw
	#quat = Quaternion(q_x, q_y, q_z, q_w)
	explicit_quat = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
	(roll, pitch, yaw) = euler_from_quaternion(explicit_quat)
	#print roll, pitch, yaw

def callback_laser(laser): 
	global vet
	vet = list(laser.ranges) #Pega as distancias medidas do tópico
	global angle_min
	angle_min = laser.angle_min  #pega o menor ângulo do laser 
	global angle_increment
	angle_increment = laser.angle_increment  #pega o valor do incremento do laser

def callback_state(state): #lê o tópido do estado publicado pela máquina de estados
	global estado
	estado = state.data


def navigation():
	vel = Twist()   # Variável de velocidade
	rospy.init_node("navigation",anonymous = True)          # Inicia nó do pacote
 	rospy.Subscriber("/scan",LaserScan, callback_laser)     # Subscreve no nó do laser
	rospy.Subscriber("/imu_data",Imu, callback_imu)         # Subscreve no nó da IMU
	rospy.Subscriber("/state", String, callback_state)      # Subscreve no nó da Maquina de estados
	pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1)   # Nó onde publica a velocidade
	rate = rospy.Rate(10)
	raio_robo = 0.60
	global sum_error
	global prev_error
	isRev = +1 #  se = -1, robô está no modo reverso
	v = rospy.get_param('v')   #Velocidade linear do robô
	vc = rospy.get_param('vc') #Velocidade linear do robô
	print (v)
	

	while not rospy.is_shutdown():    
		#vel.linear.x = 0.8
    	vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		
		#roll positivo tem que virar pra direita
		#roll negativo vira pra esquerda
		#print roll, pitch, yaw
		error_imu = (roll*180/pi)
		if (error_imu > 0):
			error_imu = error_imu -180
		elif (error_imu < 0):
			error_imu = error_imu +180
		
		if (len(vet) == 0):
			rate.sleep()
			continue  
     
		R_menor = 100 
		L_menor = 100
		frente = 100
    	theta_menor = 0
    
    	for i in range(0, len(vet)): 
			d = vet[i]   
			if(math.isnan(d)): # Se uma das distâncias for não numérica (nan), passa pra próxima
				continue 
			teta = angle_min + i * angle_increment # Calcula o valor do ângulo teta
			if (-pi/10 < teta < pi/10):
				if (frente > d) and (d > 0):
					frente = d
			if ((teta < 0) and (teta > -3*pi/4)): # Lado direito do robô, ignorando a traseira dele 
				if (d < R_menor) and (d > 0): # Procura a menor distância e salva ela na variável R_menor
					R_menor = d	
			if ((teta < 3*pi/4) and (teta > 0)): # Lado esquerdo do robô, ignorando a traseira dele
				if (d < L_menor) and (d > 0):  # Procura a menor distância e salva ela na variável L_menor
					L_menor = d
			continue
		
    	error_laser = L_menor - R_menor # Calcula a diferença das distâncias entre o robô e cada parede.
    
    	a = rospy.get_param('a') # Constante para definir importância do Laser
    
    	#estado = 'StandardControl'
	    if (estado == 'StandardControl'):
      		print 'Standard Navigation ON'
    		error = a*(error_laser*250) + (error_imu*10/4) #normaliza e soma os erros
      		kp = rospy.get_param('kp')
		    isRev = +1
      
      
    	#estado = 'RevStandardControl'
	    if (estado == 'RevStandardControl'):
		    print 'Reverse Standard Navigation ON'
		    error = a*(error_laser*250) - (error_imu*10/4) #Como o robô está de ré, o erro da IMU precisa ser invertido
	    	kp = rospy.get_param('kp')
		    isRev = -1
      
      
    	#estado = 'BifurcationControl' 
    	if (estado == 'BifurcationControl'):
		    print 'BifurcationControl ON'
      		erro_theta = theta_menor - pi/2
		    print(theta_menor*180/pi)
		    error_laser = L_menor  # Para virar para esquerda, o lado direito é ignorado
		    error = a*(erro_theta*250) + (error_imu*10/4)
      		kp = 0.0158
		    isRev = +1
      
      
   		#estado = 'RevBifurcationControl'
    	if (estado == 'RevBifurcationControl'):
		    print 'RevBifurcationControl ON'
      		erro_theta = theta_menor + pi/2
		    print(theta_menor*pi/180)
		    error_laser = L_menor 
		    error = a*(erro_theta*250) - (error_imu*10/4) #Como o robô está de ré, o erro da IMU precisa ser invertido
		    kp = 0.0158
		    isRev = -1
	
			
	if ((error_laser< -0.02) or (error_laser > 0.02)) or ((error_imu < -2) or (error_imu > 2)):
		vel.angular.z = error*(kp) # Equação da velocidade angular 0.0056
		vel.linear.x = vc*(isRev)
		#vel.angular.z = max(-0.4, min(vel.angular.z, 0.4)) # Limita velocidade angular em -0.2 ou 0.2
		pub.publish(vel) # Publica a velocidade
		print 'Controlando'
		#print error_imu
	else:
		vel.angular.z = 0 # Se estiver dentro da faixa do setpoint, velocidade angular = 0
		vel.linear.x = v*(isRev)
		pub.publish(vel)
		print 'Posição boa'
	
	sum_error = sum_error + error
	
	prev_error = error
	
	rate.sleep()
              
if __name__== '__main__':
	try:
		navigation()
	except rospy.ROSInterruptException:
		pass
