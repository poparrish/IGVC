import pickle

class GPSMsg:

	def __init__(self, pickled_values = None, latlong = (0,0)):
		if(pickled_values!=None):
			unpickled = pickle.loads(pickled_values)
			self.latlong = unpickled.latlong
			return
		self.latlong = latlong

	def pickleMe(self):
		return pickle.dumps(self)

	def __str__(self):
		return str(self.latlong)