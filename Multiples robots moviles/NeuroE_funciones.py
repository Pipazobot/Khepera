# tu puedes pipe

import sim
import numpy as np

def position_control(dis, Oc):
	# Segun el paper 'Khepera IV library for robotic control education using V-REP'

	# Constantes
	W_max = np.pi/4 # [rad/s]
	V_max = 0.05    # [m/s]
	k_R = 0.05      # radio para estacionarse

	if (dis>0.05):
		w_ang = W_max*np.sin(Oc)
		v_abs = (V_max/k_R)*dis
		if v_abs>V_max:
			v_abs = V_max
	else:
		v_abs = 0
		w_ang = 0
	return v_abs, w_ang

def BraitenbergControl(Sensors ):
	[V_L , V_R] = [0,0]
	if abs(sum(Sensors))>0.04:
		# 'Sensors' es un vector de 1x8
		W_Braitenberg = np.array(
		[ [+1  , 0] ,
		  [+0.7,-0.7] ,
		  [0   ,-1] ,
		  [-0.15,-0.15] ,
		  [-1  , 0] ,
		  [-0.7,+0.7] ,
		  [0   ,+1] ,
		  [+0.7,+0.7] ,
		]	) # Izquierdo, Derecho
		# Dismunir el tamaño de la matriz
		W_Braitenberg = W_Braitenberg*6

		[V_L , V_R] = np.matmul(Sensors , W_Braitenberg)
	return [V_L , V_R]  # Lo que se le debe sumar a cada motor

def getSensorsData( clientID, sensors_han):
	sensor_val = []
	for i in range(8):
		handle = sensors_han[i]
		_ ,det_State,det_Point, _ , _ =sim.simxReadProximitySensor(clientID,handle,sim.simx_opmode_buffer)
		if det_State:
			sensor_val.append((0.2-np.linalg.norm(det_Point))/0.2)
		else:
			sensor_val.append(0) 
	return sensor_val

def Hey(clientID, robot_h, target_h ,sensor_h):

	#print('clientID: ',clientID, end='  ')
	#print('robot_h: ',robot_h, end='  ')
	#print('target_h: ',target_h, end='  ')

	pos = sim.simxGetObjectPosition(clientID,target_h,-1,sim.simx_opmode_buffer)
	robot_x = pos[1][0]
	robot_y = pos[1][1]
	#print('robot_h: ',robot_h, end='  ')
	#print('[X,Y]: ',np.around(robot_x,2), np.around(robot_y,2), end=' ' )

	pos = sim.simxGetObjectPosition(clientID,robot_h,-1,sim.simx_opmode_buffer)
	tar_x = pos[1][0]
	tar_y = pos[1][1]
	#print('[X,Y]: ',np.around(tar_x,2), np.around(tar_y,2) , end='  ')

	# Orientacion del robot
	_ = sim.simxGetObjectOrientation(clientID, robot_h,-1,sim.simx_opmode_streaming)
	ori_body = sim.simxGetObjectOrientation(clientID,robot_h,-1,sim.simx_opmode_buffer)
	theta = ori_body[1][2]

	# cosas matematicas
	dis = np.sqrt( (tar_x - robot_x )**2 + ( tar_y - robot_y)**2 )      
	alpha = np.arctan2(tar_y - robot_y , tar_x - robot_x)
	if alpha < 0:
		alpha = alpha+2.0*np.pi
	Oc = np.arctan2(np.sin(alpha-theta),np.cos(alpha-theta))
	Oc = Oc -np.pi
	#print('[dis, Oc] = ',np.around(dis,2), np.around(Oc,2) )

	[v_abs, w_ang] = position_control(dis, Oc)

	# Transformar de velocidad angular a VR y VL
	V_L = (v_abs-(w_ang*0.1)/2)*48
	V_R = (v_abs+(w_ang*0.1)/2)*48

	# Medir sensores - los sensores estan midiendo bien
	sensor_val = getSensorsData( clientID, sensor_h)
	#print('Sensor_val: ',np.around(sensor_val,2))

	# Aca debe ir la wea de obstaculos braitenberg
	V_sumar_L, V_sumar_R = BraitenbergControl( sensor_val )
	[VL, VR] = [ V_L+V_sumar_L , V_R+V_sumar_R ]
	
	return [VL, VR]
