
import pickle

class Line:
	def __init__(self, equation):
		self.equation = equation
	def __str__(self):
		return self.equation	

class CameraMsg:

	def __init__(self, pickled_values = None):
		if(pickled_values!=None):
			unpickled = pickle.loads(pickled_values)
			self.left_line = unpickled.left_line
			self.right_line = unpickled.right_line
			self.clear_area = unpickled.clear_area
			return
		self.left_line = None
		self.right_line = None
		self.clear_area = None

	def pickleMe(self):
		return pickle.dumps(self)

	def __str__(self):
		return "left_line = {}, right_line = {}, clear_area = {}".format(self.left_line, self.right_line, self.clear_area)

	def setLeftLine(self, equation):
		self.left_line = Line(equation)

	def setRightLine(self, equation):
		self.right_line = Line(equation)

	def setClearArea(self, clear_area):
		self.clear_area = clear_area