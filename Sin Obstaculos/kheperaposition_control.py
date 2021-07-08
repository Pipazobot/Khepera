#  10 diciembre de 2020

import gym
from gym import error, spaces, utils
from gym.utils import seeding
try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')
import sys
import time
import numpy as np
import matplotlib.pyplot as mlp
import random
import math 

"""
    Description:
    	The objetive of this environment is to drive the wheeled mobile robot from its
    	current position to a predefined target point.    

    Source:
        This environment corresponds to the implementation of the Khepera IV library
        for robotic control education using V-REP for the position control experiment.

    Observation:
        Type: Box(3)
        Num Observation               Min             Max
        0   Distance to the target     0              1.41
        1   Error Angle               -pi              pi

    Actions:
        Type: Box (2)
        Num Action                    Min             Max
        0   Velocity L Wheel         -7.55            7.55
        1   Velocuty R Wheel         -7.55            7.55        

    Reward:
    	reward = -(d**2)

    Starting State:
        
    Episode Termination:
        When the robot reaches the target point, when the number of steps of the episode
        are greater than the maximum quantity of steps allowed or when the robot leaves
        the maximum allowed area (square with side 2 meters).

"""  

class KheperaPositionControl(gym.Env):
	metadata = {'render.modes':['human']}

	def __init__(self):

		sim.simxFinish(-1) # just in case, close all opened connections
		self.clientID = sim.simxStart('127.0.0.1',19997,True,True,5000,5)
		if self.clientID!=-1:  #check if client connection successful	
			print('Connected to remote API server')
			# enable the synchronous mode on the client:
			sim.simxSynchronous(self.clientID,True)
		else:
			print('Connection not successful')
			sys.exit('Could not connect')

		# Action Space
		self.action_space = spaces.Discrete(3)
		# Observation Space
		self.observation_space = spaces.Box(
			low=np.array([0.,-np.pi,-np.pi], dtype=np.float32),
			high=np.array([5,np.pi,np.pi], dtype=np.float32),
			dtype=np.float32
		)

		# Objetcs in the Simulation Scene

		errorCode, self.right_motor = sim.simxGetObjectHandle(self.clientID, 'K4_Right_Motor',sim.simx_opmode_oneshot_wait)
		errorCode, self.left_motor = sim.simxGetObjectHandle(self.clientID, 'K4_Left_Motor',sim.simx_opmode_oneshot_wait)
		errorCode, self.khepera = sim.simxGetObjectHandle(self.clientID, 'Khepera_IV',sim.simx_opmode_oneshot_wait)
		errorCode, self.target = sim.simxGetObjectHandle(self.clientID, 'Target',sim.simx_opmode_oneshot_wait)  
		errorCode, self.camera = sim.simxGetObjectHandle(self.clientID, 'Vision_sensor',sim.simx_opmode_oneshot_wait)

		self.viewer = None
		self.seed()
		self.steps = 0
		self.MaxSteps = 1200
		self.radius = 0.8
		self.angle = 0
		self.Randomize = False
		self.xp, self.yp = self.getPositionTarget()
		self.ResetSimulationScene()

	def ResetSimulationScene(self):
		errorCode, resolution, image = sim.simxGetVisionSensorImage(self.clientID,self.camera, 0, sim.simx_opmode_streaming)
		pos = sim.simxGetObjectPosition(self.clientID,self.khepera,-1,sim.simx_opmode_streaming)
		ori_body = sim.simxGetObjectOrientation(self.clientID,self.khepera,-1,sim.simx_opmode_streaming)
		tar = sim.simxGetObjectPosition(self.clientID,self.target,-1,sim.simx_opmode_streaming)

	def seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]

	def getPositionRobot(self):
		pos = sim.simxGetObjectPosition(self.clientID,self.khepera,-1,sim.simx_opmode_buffer)
		ori_body = sim.simxGetObjectOrientation(self.clientID,self.khepera,-1,sim.simx_opmode_buffer)
		theta = ori_body[1][2]
		if theta < 0:
			theta = theta+2.0*np.pi
		xc = pos[1][0]   
		yc = pos[1][1]
		return xc,yc,theta

	def getPositionTarget(self):
		tar = sim.simxGetObjectPosition(self.clientID,self.target,-1,sim.simx_opmode_buffer)
		xp = tar[1][0]
		yp = tar[1][1]
		return xp,yp

	def updateVelocities(self,Vl,Vr):
		sim.simxSetJointTargetVelocity(self.clientID,self.left_motor,Vl, sim.simx_opmode_oneshot)
		sim.simxSetJointTargetVelocity(self.clientID,self.right_motor,Vr, sim.simx_opmode_oneshot)

	def get_obs(self):
		xc,yc,theta = self.getPositionRobot()
		xp = self.xp
		yp = self.yp

		d = np.sqrt(((xp-xc)**2)+((yp-yc)**2))      
		alpha = np.arctan2(yp-yc,xp-xc)
		if alpha < 0:
			alpha = alpha+2.0*np.pi
		#Oc = np.arctan2(np.sin(alpha-theta),np.cos(alpha-theta))
		Oc = alpha - theta
		return d,Oc

	def step(self,action):
		d,Oc = self.get_obs()
		observation = np.array([d,Oc])
		reward = -(d**2)

		if d>=0.05:
			done = False
			Vl,Vr = action
		else:
			Vl = 0
			Vr = 0
			done = True
			reward = self.MaxSteps - self.steps
			
		self.updateVelocities(Vl,Vr)
		
		if d>=2 or self.steps > (self.MaxSteps-1):
			done=True
		if not done:
			sim.simxSynchronousTrigger(self.clientID)
			self.steps+=1
		xc,yc,theta = self.getPositionRobot()
		info = {'xc':xc,
				'yc':yc}

		return observation, reward, done, info

	def reset(self):
		# stop the simulation:
		sim.simxStopSimulation(self.clientID,sim.simx_opmode_blocking)
		time.sleep(0.1)
		sim.simxSynchronous(self.clientID,True)
		# start the simulation:
		sim.simxStartSimulation(self.clientID,sim.simx_opmode_blocking)
		self.steps = 0
		self.ResetSimulationScene()
		if self.Randomize:
			ran_angle = random.random()*360
			self.xp, self.yp = self.change_target_angle(ran_angle)
		else:
			self.xp, self.yp = self.getPositionTarget()
		#self.xp, self.yp = self.change_target_angle(self.angle)
		d,Oc = self.get_obs()
		observation = np.array([d,Oc])
		return observation

	def getCameraImage(self):
		errorCode, resolution, image = sim.simxGetVisionSensorImage(self.clientID, self.camera, 0, sim.simx_opmode_buffer)
		im = np.array(image,dtype=np.uint8)
		im.resize([resolution[1],resolution[0],3])
		return im

	def render(self, mode='human'):
		img = self.getCameraImage()
		if mode == 'human':
			from gym.envs.classic_control import rendering
			if self.viewer is None:
				self.viewer = rendering.SimpleImageViewer()
			self.viewer.imshow(img)
			return self.viewer.isopen

	def close(self):
		if self.viewer:
			self.viewer.close()
			self.viewer = None
		sim.simxStopSimulation(self.clientID,sim.simx_opmode_blocking)
		# Now close the connection to CoppeliaSim:
		sim.simxFinish(self.clientID)

	def change_target_angle(self, angle):
		X = math.cos(math.radians(angle))*self.radius
		Y = math.sin(math.radians(angle))*self.radius
		Z = -0.005
		new_position = [X,Y,Z]
		errorCode = sim.simxSetObjectPosition(self.clientID,self.target,-1,new_position,sim.simx_opmode_oneshot)
		return X,Y

	def change_target_position(self, radius):
		# Esta funcion creo que la hizo el fansico
		minimum = -radius
		maximum = radius
		X = minimum + (maximum - minimum) * random.random()
		Y = math.sqrt(self.radius**2 - X**2)*(-1)**(random.sample([0,1],1)[0])
		Z = -0.005
		new_position = [X,Y,Z]
		errorCode = sim.simxSetObjectPosition(self.clientID,self.target,-1,new_position,sim.simx_opmode_oneshot)
		return X,Y

	def cambiar_target(self, X, Y):
		self.xp = X
		self.yp = Y
		Z = -0.005
		new_position = [X,Y,Z]
		errorCode = sim.simxSetObjectPosition(self.clientID,self.target,-1,new_position,sim.simx_opmode_oneshot)
		#print('-------°°-- Cambiaste el target --°°-----------')
		#print('new_position ',new_position)

