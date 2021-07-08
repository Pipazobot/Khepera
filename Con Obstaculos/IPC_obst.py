""" IPC
Vamos a adaptar el IPc con el braitenberg, para que pueda evitar obstaculos
"""

from __future__ import print_function
#   import kheperaposition_control
import kheperaposition_obstacle
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

def BraitenbergControl(Sensors ):
    # 'Sensors' es un vector de 1x8
    W_Braitenberg = np.array(
    [ [+1  , 0] ,
      [+0.7,-0.7] ,
      [0   ,-1] ,
      [-0.7,-0.5] ,
      [-1  , 0] ,
      [-0.7,+0.7] ,
      [0   ,+1] ,
      [+0.7,+0.7] ,
    ]   ) # Izquierdo, Derecho

    # Dismunir el tamaño de la matriz
    W_Braitenberg = W_Braitenberg*10

    [V_L , V_R] = np.matmul(Sensors , W_Braitenberg)
    return [V_L , V_R]  # Lo que se le debe sumar a cada motor

# Crear el ambiente
#env = kheperaposition_control.KheperaPositionControl()
env = kheperaposition_obstacle.KheperaPositionObstacle()

estado = env.reset()
dis, Oc = estado[0:2]
env.cambiar_target( 0 , 0.5)

steps=800
#env.MaxSteps = steps #800
done = False

ErrorAcumulado=0

# Cosas para medic IAE y ITSE
paso = 0
dis_lista = []
camino_robot = []

V_actuadores = []

while not(done):

    #Decidir la acción
    [V_L , V_R], ErrorAcumulado = IPC_accion(dis, Oc, ErrorAcumulado)

    # Aca debe ir la wea de obstaculos braitenberg
    V_sumar_L, V_sumar_R = BraitenbergControl( estado[4:] )
    accion = [ V_L+V_sumar_L , V_R+V_sumar_R ]

    # Realizar acción
    estado, reward, done, info = env.step(accion)
    dis, Oc = estado[0:2]
    
    # Guardar el voltaje que se envio a los actuadores
    #V_actuadores.append(accion) 

    # Guardar dis  y Oc para despues graficarlos
    dis_lista.append(dis)
    paso += 1

    # Guardar el camino que recorre el robot (PIPE)
    xr,yr, _ = env.getPositionRobot( )
    pos_khep = [ xr, yr ]
    camino_robot.append(pos_khep)

env.close()

print('_____Termino la simulación__________')
print('Se demoro {} segundos '.format(paso*0.05))

# guardar como archivo de numpy, datos que se van a graficar
#np.save('V_actuadores_IPC', V_actuadores)
np.save('Dis_lista_IPC',dis_lista)
np.save('N_pasos_totales_IPC',paso)
np.save('Camino_robot_IPC', camino_robot)

