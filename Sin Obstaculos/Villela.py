import kheperaposition_control
import numpy as np
import math
import sys

Vmax=0.05
Wmax=math.pi/4
L=0.1
Kr_V_RL=0.05   # radio de seguridad

def random_agent(steps=900):
    episode_reward = 0

    # Crear el ambiente
    env = kheperaposition_control.KheperaPositionControl()

    # Cambiar el target
    env.cambiar_target(-1.2, 0.02)


    #env.MaxSteps = steps
    state = env.reset()
    d, Oc = state
    states = [] 
    #env.render()
    
    data = {}
    paso = []
    infoOc = []
    infoD = []
    Lineal = []
    Angular = []
    xc = []
    yc = []
    camino_robot = [] # pipe modificacion
    V_actuadores = [] # pipe modificacion

    for e in range(steps):
        w = Wmax*math.sin(Oc)
        if d > Kr_V_RL:
            v = Vmax
        else:
            v = d*(Vmax/Kr_V_RL)
        if d < 0.02:
            v = 0
            w = 0
        states.append([state, e])
        Vl = (v-(w*0.1)/2)*48
        Vr = (v+(w*0.1)/2)*48
        action = [Vl, Vr]

        # Realizar acción
        state, reward, done, info = env.step(action)
        d, Oc = state
        
        # Guardar el voltaje que se envio a los actuadores
        V_actuadores.append(action)

        # Guardar el camino que recorre el robot (PIPE)
        xr,yr, _ = env.getPositionRobot( )
        pos_khep = [xr, yr]
        camino_robot.append(pos_khep)

        #env.render()
        print(reward, end='\r')
        sys.stdout.flush()
        episode_reward += reward
        
        x = info['xc']
        y = info['yc']
        
        paso.append(e)
        infoOc.append(float(Oc))
        infoD.append(d)
        Lineal.append(float(v))
        Angular.append(float(w))
        xc.append(np.array(x))
        yc.append(np.array(y))
        
        if done:
            print('Reward of the episode is: ',episode_reward)
            break
    
    env.close()
    data={'paso' : paso,
          'Oc' : infoOc,
          'Distance' : infoD,
          'Lineal' : Lineal,
          'Angular' : Angular,
          'xc' : xc,
          'yc' : yc
    }
    return data, camino_robot, V_actuadores


data1 , camino_robot, V_actuadores = random_agent()

print('_______ Termino la simulación_________')


# Guardar cosas neccesarias
np.save('V_actuadores_Villela', V_actuadores)
#np.save('Camino_robot_Villela', camino_robot)
#np.save('N_pasos_totales_Villela', max(data1['paso']) )
#np.save('Dis_lista_Villela', data1['Distance'])





d, t, v, w = [np.array(data1['Distance']), 0.05*np.array(data1['paso']), np.array(data1['Lineal']), np.array(data1['Angular'])]

IAE = np.trapz(abs(d),t)
ISE = np.trapz(d**2,t)
ITAE = np.trapz(t*abs(d),t)
ITSE = np.trapz(t*(d**2),t)

Villela = [ISE, IAE, ITSE, ITAE]

""" Modificaciones de pipe """

#print('IAE: ',IAE)
#print('ISE: ',ISE)
#print('ITAE: ',ITAE)
#print('ITSE: ',ITSE)
