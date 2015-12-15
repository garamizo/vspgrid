import numpy
import math
from tf import transformations as tfs

def rotx(q): return numpy.matrix(tfs.rotation_matrix(q, [1, 0, 0]))
def roty(q): return numpy.matrix(tfs.rotation_matrix(q, [0, 1, 0]))
def rotz(q): return numpy.matrix(tfs.rotation_matrix(q, [0, 0, 1]))
def trans(xyz): return numpy.matrix(tfs.translation_matrix(xyz))

class Robot(object):

	def get_jacobian(self, q):
		
		EPS = 1e-5

		jac = numpy.zeros((self.dof, self.dof))
		cart0 = self.get_cart(self.get_matrix(q))
		for k in range(0, self.dof):
			qeps = q
			qeps[k] += EPS
			jac[:,k] = (self.get_cart(self.get_matrix(qeps)) - cart0)/EPS

		return jac

	def increment(self, v)

		jac = self.get_jacobian(self.q)
		dev = abs(numpy.linalg.det(jac))
		dq = numpy.linalg.tensorsolve(jac, v)
		
		return dq, dev



class Widowx(Robot):

	def __init__(self):

		self.dof = 5
		self.q = [0]*self.dof

	def get_matrix(self, q):

		T1 = rotz(q[0]) * trans([0, 0, 0.125])
		T2 = roty(q[1]) * trans([0.04825, 0, 0.14203])
		T3 = roty(q[2]) * trans([0.14203, 0, 0])
		T4 = roty(q[3]) * trans([0.0715, 0, 0])
		T5 = rotx(q[4]) * trans([0.043, 0, 0])

		return T1*T2*T3*T4*T5

	def get_cart(self, matrix):

		x = matrix[0,3]
		y = matrix[1,3]
		z = matrix[2,3]
		yaw = math.atan2(matrix[1,0], matrix[0,0])
		pitch = math.asin(-matrix[2,0])

		return numpy.array([x, y, z, yaw, pitch])





