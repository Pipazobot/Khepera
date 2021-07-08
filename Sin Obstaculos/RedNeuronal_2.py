
"""
Libreria personal, genera una red neuronal

Video de Sebastian Lague -  https://www.youtube.com/watch?v=8bNIkfRJZpo
"""

import numpy as np


class totalmente_conectada():

	def __init__(self,arquitectura):
		# 'arquitectura' es una lista que indica cuantas neuronas hay en cada capa
		# ej: [3,5,1]

		self.arquitectura = arquitectura
		self.pesos = []
		self.bias = []
		for idx in range(len(arquitectura)-1):
			filas = arquitectura[idx+1]
			col = arquitectura[idx]
			w_matrix = np.random.standard_normal( size=(filas,col) )
			b_matrix = np.random.standard_normal( size=(filas, 1 ) )
			self.pesos.append(w_matrix)
			self.bias.append(b_matrix)

	def predict(self,input_vector):
		""" 'input_vetor' es la entrada. Debe ser un vector nomas,  input_vector = [4,2,6] 
			La activación de cada capa es ReLU, y la ultima es Softmax					"""
		if not(len(input_vector)==self.arquitectura[0]):
			raise ValueError('--Error-- el vector de entrada de dim:{} es distinto de {} dim entrada de la red '
				.format(len(input_vector),self.arquitectura[0]))


		for idx in range(len(self.pesos)):
			dot_product = np.dot(self.pesos[idx], input_vector)
			dot_product = np.reshape(dot_product, (len(dot_product),1))
			dot_product += self.bias[idx]

			#funcion de activacion
			if idx == (len(self.pesos)-1):
				out = self.Lineal(dot_product) # ultima capa
			else:
				out = self.ReLU(dot_product) # capas intermedias
			input_vector = np.copy(out)

		return out

	def Softmax(self, x):
		return np.exp(x)/sum(np.exp(x))

	def Sigmoid(self, x):
		return 1/(1+np.exp(-x))

	def ReLU(self, x):
		return np.maximum(0,x)

	def Lineal(self,x):
		return x

	def n_de_param(self):
		arquitectura=self.arquitectura
		param=0
		for idx in range(len(arquitectura)-1):
			pesos = arquitectura[idx]*arquitectura[idx+1]
			bias = arquitectura[idx+1]
			param=pesos+bias+param
		return param

	def Insertar_parametros(self,vect_red):
		"""Ejemplo de vect_red
		vect_red = [0.8,-0.2,0.5,0.1,-0.6,0.8,-0.1,0.2,0.5,-0.3,0.2,-0.1,5]
		"""
		if self.n_de_param()!=len(vect_red):
			print('----Error----')
			print('El Vector_red es de largo {}, pero deberia ser de largo {}'.format(len(vect_red),self.n_de_param()))
		else:
			pos=0
			for capa,n in enumerate(self.arquitectura):
				if capa<len(self.arquitectura)-1:
					for fila in range(self.pesos[capa].shape[0]):
						for col in range(self.pesos[capa].shape[1]):
							self.pesos[capa][fila][col]=vect_red[pos]
							pos += 1
			for capa,n in enumerate(self.arquitectura):
				if capa<len(self.arquitectura)-1:
					for fila in range(self.bias[capa].shape[0]):
						for col in range(self.bias[capa].shape[1]):
							self.bias[capa][fila][col]=vect_red[pos]
							pos += 1

	def sacar_parametros(self):

		vector = np.zeros((self.n_de_param()))
		indice = 0
		for idx in range(len(self.arquitectura)-1):
			for fila in self.pesos[idx]:
				for w in fila:
					vector[indice] = w
					indice += 1

		for idx in range(len(self.arquitectura)-1):
			for bias_value in self.bias[idx]:
				vector[indice] = bias_value
				indice += 1
		return vector


	def activaciones(self,input_vector):
		""" Devuleve las activaciones en una prediccion """
		activaciones = []
		for idx in range(len(self.pesos)):
			dot_product = np.dot(self.pesos[idx], input_vector)
			dot_product = np.reshape(dot_product, (len(dot_product),1))
			dot_product += self.bias[idx]

			#funcion de activacion
			if idx == (len(self.pesos)-1):
				out = self.Softmax(dot_product)
			else:
				out = self.ReLU(dot_product)
			input_vector = np.copy(out)
			activaciones.append(out)
			
		return activaciones
