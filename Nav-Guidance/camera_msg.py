
import pickle

class Line:
	def __init__(self, equation):
		self.equation = equation
	def __str__(self):
		return self.equation	

class CameraMsg:

	def __init__(self, local_map_val = None,  pickled_values = None):
		if(pickled_values!=None):
			unpickled = pickle.loads(pickled_values)
			self.local_map = unpickled.local_map
			return
		self.local_map = local_map_val
		

	def pickleMe(self):
		return pickle.dumps(self)

	def __str__(self):
		return "local_map = {}".format(self.local_map)

	def setLocalMap(self, local_map):
		self.local_map = local_map

	def getLocalMap(self):
		return self.local_map