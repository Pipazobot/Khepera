""" IPC
"""

from __future__ import print_function
import kheperaposition_control
import numpy as np
import math
import sys


def IPC_accion(d, Oc, ErrorAcumulado):

    # Constantes del IPC
    Vmax=0.05
    Wmax=math.pi/4

    Kr_Prop=0.025   
    K1=0.1
    Kp=0.75
    Ki=0.00007 
    PI=3.1415


    #Recibe como variable 'Oc'
    error = math.atan2(math.sin(Oc), math.cos(Oc))
    p = ((math.pi-math.fabs(error))/math.pi)
    v = min(K1*d*p, Vmax)
    if p > 0.9 and d > Kr_Prop:
        v = Vmax
    if d < Kr_Prop:
        v = 0
        w = 0
    ErrorAcumulado = error + ErrorAcumulado
    w = Kp*math.sin(Oc)+Ki*ErrorAcumulado
    Vl = (v-(w*0.1)/2)*48
    Vr = (v+(w*0.1)/2)*48
    action = [Vl, Vr]

    return action, ErrorAcumulado

# Crear el ambiente
env = kheperaposition_control.KheperaPositionControl()
d, Oc = env.reset()
env.cambiar_target(-1.2, 0)

steps=800
env.MaxSteps = steps #800
done = False

ErrorAcumulado=0

# Cosas para medic IAE y ITSE
paso = 0
dis_lista = []
camino_robot = []

V_actuadores = []

while not(done):

    #Decidir la acción
    accion, ErrorAcumulado = IPC_accion(d, Oc, ErrorAcumulado)

    # Realizar acción
    state, reward, done, info = env.step(accion)
    d, Oc = state

    # Guardar el voltaje que se envio a los actuadores
    V_actuadores.append(accion) 

    # Guardar dis  y Oc para despues graficarlos
    dis_lista.append(d)
    paso += 1

    # Guardar el camino que recorre el robot (PIPE)
    xr,yr, _ = env.getPositionRobot( )
    pos_khep = [ xr, yr ]
    camino_robot.append(pos_khep)

env.close()

print('_____Termino la simulación__________')
print('Se demoro {} segundos '.format(paso*0.05))

# guardar como archivo de numpy, datos que se van a graficar
np.save('V_actuadores_IPC', V_actuadores)
#np.save('Dis_lista_IPC',dis_lista)
#np.save('N_pasos_totales_IPC',paso)
#np.save('Camino_robot_IPC', camino_robot)

