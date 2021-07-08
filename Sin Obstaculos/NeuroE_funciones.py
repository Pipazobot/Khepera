import numpy as np

#print('Control_Posicion Reportandose!!')

def resta_vectores(lista1, lista2):
	""" se realia la operación RESTA = lista2 - lista1  """
	RESTA = np.zeros(len(lista1))
	for idx in range(len(lista1)):
		RESTA[idx] = lista2[idx] - lista1[idx]
	return RESTA

def calcular_velocidad(time):

	A = 2.38
	t_up = 30

	# calcular el argumento de la funcion exponencial
	B = -np.log(0.1)/t_up
	B = np.around(B,3)

	# Para cualquier 't' indicar la velocidad
	vel = A - A*np.exp(-B*time)
	vel = np.around(vel, 3)
	return [vel , vel]

def aplicar_diferencia_a_velocidad(vel, angulo_error):
	# si voy directo hacia el target,no tengo que girar
	if angulo_error==0:
		return vel 

	# Recta
	#Kd = 10 
	#dif_nueva = abs(angulo_error)*Kd/180 

	# Curva exponencial
	A = 3 # positivo # La diferencia frente a 180°
	B = 0.03 # positivo
	dif_nueva = A - A*np.exp(-B*abs(angulo_error))
	dif_nueva = np.around(dif_nueva, 3)


	# Dividir la diferencia entre las dos ruedas
	if angulo_error>0:
		vel[0] += - np.around( dif_nueva/2 ,4) # Izquierda
		vel[1] += + np.around( dif_nueva/2 ,4) # Derecha

	if angulo_error<0:
		vel[0] += + np.around( dif_nueva/2 ,4) # Izquierda
		vel[1] += - np.around( dif_nueva/2 ,4) # Derecha

	# Si es que me paso del maximo 7.5
	if abs(vel[0])>7.5 or abs(vel[1]>7.5):

		por_lo_que_me_pase = np.max(vel) - 7.5
		vel += -por_lo_que_me_pase

	return vel

def Agente_manual(t, buff):

	pos_2 = buff[0:2] # t=0
	pos_1 = buff[4:6] # t=-1
	pos_0 = buff[8:10] #t=-2
	pos_target = buff[2:4]

	# Si parte del reposo
	if (pos_0==pos_2).all():
		empujon = [0.2,0.2]
		return empujon

	# Obetener el angulo de error
	angulo_error = angulo_de_error_con_signo(pos_target, pos_0, pos_2)
	#print('Angulo de error {} °'.format(angulo_error))

	
	# Relacionar el angulo de error con la velocidad correspondiente
	vel_const = calcular_velocidad(t) #la velocidad "absoluta"
	vel_const = aplicar_diferencia_a_velocidad(vel_const, angulo_error)
	vel_const = np.clip(vel_const, -7.5, +7.5)

	return vel_const

def angulo_de_error_con_signo(target, x_0, x_1):

	# Mover el punto de referencia, X_0 va a ser el origen
	vec1 = resta_vectores(x_0, x_1)
	vec2 = resta_vectores(x_0, target)
	angulo = angulo_entre_vectores(vec1, vec2)

	lado = pa_que_lado_esta_el_target(x_0, x_1, target)

	if lado == 'Izquierda':
		return +angulo
	elif lado == 'Derecha':
		return -angulo
	elif lado == 'Misma linea':
		return 0

def pa_que_lado_esta_el_target(x_0, x_1, target):
	""" Se hace un cambio de base
		la velocidad actual del robot es V= x_1 - X_0
		y en el fondo solo quiero saber si el target esta a la Derecha o Izquierda de mi Velocidad actual
	"""
	#print('X_0 y x_1: ', [x_0 ,x_1])
	x_0 = np.array(x_0)
	x_1 = np.array(x_1)

	# Primer vector base 
	B_1 = x_1 - x_0

	# Calcular el vector B2a y despues ver cual utilizar
	# Encontrar el 2do vector base: [k_1, k_2]
	# el vector base B2 tiene |B2|=1 y ademas es ortogonal a B1
	if B_1[0]==0:
		B_2_a = np.array([1,0])
	else:
		k_2 = np.sqrt( 1/((B_1[1]**2 / B_1[0]**2) + 1) )
		k_1 = -k_2*B_1[1] / B_1[0]
		B_2_a = np.array([k_1, k_2])

	# ¿¿¿¿¿ Usar el vector B_2a o B_2b ????
	Vel = x_1 - x_0 
	#print('VELOCIDAD = ',Vel)
	if Vel[0]<0:
		B_2 = B_2_a # seguir usando el B_2a 
	elif Vel[0]>0:
		B_2 = -B_2_a # cambiar a B_2b

	# Caso especial
	elif Vel[0]==0:
		if Vel[1]>0:
			B_2 = B_2_a # seguir usando el B_2a 
		if Vel[1]<0:
			B_2 = -B_2_a # cambiar a B_2b


	# Matriz de cambio de base
	Matriz_P = np.zeros((2,2))
	Matriz_P[:,1] = B_1
	Matriz_P[:,0] = B_2
	P_inversa = np.linalg.inv(Matriz_P)

	# Transformar el target
	target -= x_0
	pto_mapeado = np.matmul(P_inversa, target)

	if pto_mapeado[0]>0:
		return 'Derecha'
	if pto_mapeado[0]<0:
		return 'Izquierda'
	if pto_mapeado[0]==0:
		return 'Misma linea'

def angulo_entre_vectores(vec1, vec2):

	# Sacar el producto punto
	prod_punto = 0
	for idx in range(len(vec1)):
		prod_punto += vec1[idx]*vec2[idx]

	# Obtener magnitud de cada vector
	angulo = np.arccos( prod_punto/( magnitud(vec1) * magnitud(vec2) ) )
	ang_grad = np.around( angulo*180/np.pi ,3)	

	return ang_grad

def magnitud(vec):
	suma = 0
	for idx in range(len(vec)):
		suma += ( vec[idx] )**2
	return np.sqrt(suma)


# Principal
def Principal(bufffer_estados, target_pos, time):

	# Posición del robot movil
	pos_2 = bufffer_estados[0:2] #t=0
	pos_0 = bufffer_estados[4:6] #t=-2

	# Si parte del reposo
	if (pos_0==pos_2).all():
		empujon = [0.2,0.2]
		return empujon  

	#print('bufffer_estados: ', bufffer_estados)
	#print('target_pos: ',target_pos)

	# Obetener el angulo de error
	angulo_error = angulo_de_error_con_signo(target_pos, pos_0, pos_2)
	#print('Angulo de error {} °'.format(angulo_error))

	# Relacionar el angulo de error con la velocidad correspondiente
	Vel_ruedas = calcular_velocidad(time) #la velocidad "absoluta"	
	Vel_ruedas = aplicar_diferencia_a_velocidad(Vel_ruedas, angulo_error)
	Vel_ruedas = np.clip(Vel_ruedas, -7.5, +7.5)

	return Vel_ruedas


# == Para revisar que este archivo este funcionando bien ==
#Principal([0,0,0,0,0,0] , [0.8, 0.04], time=10)
