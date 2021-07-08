import kheperaposition_obstacle
import numpy as np
import math
import sys


def Villela_control(dis, Oc):
    Vmax=0.05
    Wmax=math.pi/4
    L=0.1
    Kr_V_RL=0.05   # radio de seguridad

    w = Wmax*math.sin(Oc)
    if dis > Kr_V_RL:
        v = Vmax
    else:
        v = dis*(Vmax/Kr_V_RL)
    if dis < 0.02:
        v = 0
        w = 0

    Vl = (v-(w*0.1)/2)*48
    Vr = (v+(w*0.1)/2)*48
    return [Vl, Vr]

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
env = kheperaposition_obstacle.KheperaPositionObstacle()

estado = env.reset()
dis, Oc = estado[0:2]
env.cambiar_target( 0 , 0.5)

steps=800
done = False

# Cosas para medic IAE y ITSE
paso = 0
dis_lista = []
camino_robot = []

while not(done):

    #Decidir la acción
    [V_L , V_R] = Villela_control(dis, Oc)

    # Aca debe ir la wea de obstaculos braitenberg
    V_sumar_L, V_sumar_R = BraitenbergControl( estado[4:] )
    accion = [ V_L+V_sumar_L , V_R+V_sumar_R ]

    # Realizar acción
    estado, reward, done, info = env.step(accion)
    dis, Oc = estado[0:2]

    # Guardar dis  y Oc para despues graficarlos
    dis_lista.append(dis)
    paso += 1

    # Guardar el camino que recorre el robot (PIPE)
    xr,yr, _ = env.getPositionRobot( )
    camino_robot.append( [xr,yr] )

env.close()

print('_____Termino la simulación__________')
print('Se demoro {} segundos '.format(paso*0.05))

# guardar como archivo de numpy, datos que se van a graficar
np.save('Dis_lista_Villela',dis_lista)
np.save('N_pasos_totales_Villela',paso)
np.save('Camino_robot_Villela', camino_robot)