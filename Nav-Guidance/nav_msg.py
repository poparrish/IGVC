import pickle

class CameraMsg:

	def __init__(self, pickled_values = None):
		if(pickled_values!=None):
			unpickled = pickle.loads(pickled_values)
			return
		possible_forward = [None]

	def pickleMe(self):
		return pickle.dumps(self)
