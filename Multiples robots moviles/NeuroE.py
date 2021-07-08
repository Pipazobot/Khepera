
import sim
import numpy as np
import NeuroE_funciones as CT

def updateVelocities( V_L, V_R , handle_L, handle_R):
	sim.simxSetJointTargetVelocity(clientID, handle_L, V_L , sim.simx_opmode_oneshot)
	sim.simxSetJointTargetVelocity(clientID, handle_R, V_R , sim.simx_opmode_oneshot)
	#print(V_L, V_R)

def cambiar_target(self, X, Y):
		self.xp = X
		self.yp = Y
		Z = -0.005
		new_position = [X,Y,Z]
		errorCode = sim.simxSetObjectPosition(self.clientID,self.target,-1,new_position,sim.simx_opmode_oneshot)


# Cosas Iniciales Coppelia
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
	print ('-- Nos Conectamos con CoppeliaSim --')
sim.simxSynchronous(clientID,True)
sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)

# robot movil 1
[ _ , motorLeft_1] = sim.simxGetObjectHandle(clientID, 'K4_Left_Motor', sim.simx_opmode_oneshot_wait)
[ _ , motorRight_1] = sim.simxGetObjectHandle(clientID, 'K4_Right_Motor', sim.simx_opmode_oneshot_wait)
[ _ , robotMovil] = sim.simxGetObjectHandle(clientID, 'K4_Case', sim.simx_opmode_oneshot_wait)
_ = sim.simxGetObjectPosition(clientID,robotMovil,-1,sim.simx_opmode_streaming)

# robot movil 2
[ _ , motorLeft_2] = sim.simxGetObjectHandle(clientID, 'K4_Left_Motor#0', sim.simx_opmode_oneshot_wait)
[ _ , motorRight_2] = sim.simxGetObjectHandle(clientID, 'K4_Right_Motor#0', sim.simx_opmode_oneshot_wait)
[ _ , robotMovil_2] = sim.simxGetObjectHandle(clientID, 'K4_Case#0', sim.simx_opmode_oneshot_wait)
_ = sim.simxGetObjectPosition(clientID,robotMovil_2,-1,sim.simx_opmode_streaming)

# robot movil 3
[ _ , motorLeft_3] = sim.simxGetObjectHandle(clientID, 'K4_Left_Motor#1', sim.simx_opmode_oneshot_wait)
[ _ , motorRight_3] = sim.simxGetObjectHandle(clientID, 'K4_Right_Motor#1', sim.simx_opmode_oneshot_wait)
[ _ , robotMovil_3] = sim.simxGetObjectHandle(clientID, 'K4_Case#1', sim.simx_opmode_oneshot_wait)
_ = sim.simxGetObjectPosition(clientID,robotMovil_3,-1,sim.simx_opmode_streaming)


# Handle de los sensores - robot movil 1
sensors_handle = {}
for i in range(8):
	handle = 'K4_Infrared_{}'.format(i+1)
	_ , sensors_handle[i] = sim.simxGetObjectHandle(clientID, handle, sim.simx_opmode_oneshot_wait)
	# Decirle a los sensores que empiecen a transmitir 'Streaming' su informacion
	_ , _ , _ , _ , _ =sim.simxReadProximitySensor(clientID,sensors_handle[i],sim.simx_opmode_streaming)

# Handle de los sensores - robot movil 2
sensors2_handle = {}
for i in range(8):
	handle = 'K4_Infrared_{}#0'.format(i+1)
	_ , sensors2_handle[i] = sim.simxGetObjectHandle(clientID, handle, sim.simx_opmode_oneshot_wait)
	# Decirle a los sensores que empiecen a transmitir 'Streaming' su informacion
	_ , _ , _ , _ , _ =sim.simxReadProximitySensor(clientID,sensors2_handle[i],sim.simx_opmode_streaming)

# Handle de los sensores - robot movil 3
sensors3_handle = {}
for i in range(8):
	handle = 'K4_Infrared_{}#1'.format(i+1)
	_ , sensors3_handle[i] = sim.simxGetObjectHandle(clientID, handle, sim.simx_opmode_oneshot_wait)
	# Decirle a los sensores que empiecen a transmitir 'Streaming' su informacion
	_ , _ , _ , _ , _ =sim.simxReadProximitySensor(clientID,sensors3_handle[i],sim.simx_opmode_streaming)

# Handle target
_, Target_h = sim.simxGetObjectHandle(clientID, 'Objetivo',sim.simx_opmode_oneshot_wait) 
_, Target2_h = sim.simxGetObjectHandle(clientID, 'Objetivo#0',sim.simx_opmode_oneshot_wait) 
_, Target3_h = sim.simxGetObjectHandle(clientID, 'Objetivo#1',sim.simx_opmode_oneshot_wait) 

_ = sim.simxGetObjectPosition(clientID,Target_h,-1,sim.simx_opmode_streaming)
_ = sim.simxGetObjectPosition(clientID,Target2_h,-1,sim.simx_opmode_streaming)
_ = sim.simxGetObjectPosition(clientID,Target3_h,-1,sim.simx_opmode_streaming)

# Variables importantes hasta ahora
# clientID, sensors_handle,motorLeft_1, motorRight_1

# Cambiar ubicacion del target
X = -1.27 + 0.7
Y = 0.12 - 0
Z = -0.005
new_position = [X,Y,Z]
#_ = sim.simxSetObjectPosition(clientID,Target_h,-1,new_position,sim.simx_opmode_oneshot)


# Now step a few time
for _ in range(690):

	# mover el robot movil 1
	Accion = CT.Hey(clientID, robotMovil, Target_h, sensors_handle)
	#Accion = [0,0]
	updateVelocities( Accion[0], Accion[1], motorLeft_1, motorRight_1)

	# mover el robot movil 2
	Accion = CT.Hey(clientID, robotMovil_2, Target2_h, sensors2_handle)
	#Accion = [7,7]
	updateVelocities( Accion[0], Accion[1], motorLeft_2, motorRight_2)

	# mover el robot movil 3
	Accion = CT.Hey(clientID, robotMovil_3, Target3_h, sensors3_handle)
	#Accion = [7,7]
	updateVelocities( Accion[0], Accion[1], motorLeft_3, motorRight_3)

	sim.simxSynchronousTrigger(clientID);

# Detener la simulación
sim.simxStopSimulation(clientID,sim.simx_opmode_blocking)
sim.simxFinish(clientID)

print('___EL programa termino___')