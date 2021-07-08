

import numpy as np
#import kheperaposition_control
import kheperaposition_obstacle
import NeuroE_funciones
#import RedNeuronal_2

def actualizar_buffer(buff, state):
	# 'state' es el nuevo estado que se va a ingresar
	N = len(state)
	size_buffer = int( len(buff)/len(state) )

	# Movemos los ultimos 'size_buffer-1' estados a la derecha
	buff[N:size_buffer*N] = np.copy( buff[0:(size_buffer-1)*N] )

	# Ingresamos el nuevo estado (por la izquierda)
	buff[0:N] = state

	return buff

def crear_buffer(state):	
	N = len(state)
	size_buffer = 3
	conj_estados = np.zeros( size_buffer*N)
	for idx in range(size_buffer): # por cada fila de la matriz "conj_estados"
		id_ini = idx*N
		id_fin = (idx+1)*N
		conj_estados[id_ini: id_fin] = state

	return conj_estados




# Crear el ambiente
env = kheperaposition_obstacle.KheperaPositionObstacle()

estado = env.reset()
env.cambiar_target( 0, 0.5)	

# Obtener posicion del robot movil
xr,yr, _ = env.getPositionRobot()
state = [xr,yr]
buffer_estados = crear_buffer(state)

# Posición del target
xt = np.around(env.xp,5) #TArget
yt = np.around(env.yp,5) 
Target_pos = [xt , yt]

# Cosas para medic IAE y ITSE
paso = 0
dis_lista = []

# Variables para evitar colision de objetos
t_alarma = 0 


camino_robot = []
done = False

while not(done):

	# Decidir la accion
	Vel_ruedas  = NeuroE_funciones.Principal(buffer_estados, Target_pos, paso, estado, t_alarma)


	# Realizar accion
	estado , reward, done, _ = env.step( Vel_ruedas )
	dis, Oc = estado[0:2] # Para medic IAE y ITSE

	# Guardar dis  y Oc para despues graficarlos
	dis_lista.append(dis)
	paso += 1

	# Guardar el camino que recorre el robot (PIPE)
	xr,yr, _ = env.getPositionRobot( )
	pos_khep =  [ xr, yr ]
	camino_robot.append(pos_khep)

	# Actualizar buffer de estados
	buffer_estados = actualizar_buffer(buffer_estados, [xr, yr])

env.close()


print("====== El programa termino ===========")
print('Se demoro {} segudos en llegar al Target'.format(paso*0.05))

# guardar como archivo de numpy, datos que se van a graficar
#np.save('V_actuadores_PIPE', V_actuadores)
np.save('Dis_lista_PIPE',dis_lista)
np.save('N_pasos_totales_PIPE',paso)
np.save('Camino_robot_PIPE', camino_robot)




# el tiempo de simulación de coppelia es: dt = 50 [ms]
#t = 0.05* np.arange(paso) # asi lo calulaban los otros
#d = np.array(dis_lista)

#IAE = np.trapz(abs(d),t)
#ISE = np.trapz(d**2,t)
#ITAE = np.trapz(t*abs(d),t)
#ITSE = np.trapz(t*(d**2),t)

#print('IAE: ',IAE)
#print('ISE: ',ISE)
#print('ITAE: ',ITAE)
#print('ITSE: ',ITSE)



