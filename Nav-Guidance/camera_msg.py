
import pickle

class CameraMsg:

	def __init__(self, local_map_val = None,  pickled_values = None):
		if(pickled_values!=None):
			# print(pickled_values.data)
			unpickled = pickle.loads(pickled_values)
			# print(unpickled.local_map)
			self.local_map = unpickled.local_map
			return
		self.local_map = local_map_val		

	def pickleMe(self):
		# print(type(self.local_map))
		return pickle.dumps(self, protocol=0)

	def __str__(self):
		return "local_map = {}".format(self.local_map)

	def setLocalMap(self, local_map):
		self.local_map = local_map

	def getLocalMap(self):
		return self.local_map