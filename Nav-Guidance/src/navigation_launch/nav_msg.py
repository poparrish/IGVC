import pickle


class NavMsg:

    def __init__(self, camera, gps, lidar, stop, heading):
        self.camera = camera
        self.gps = gps
        self.lidar = lidar
        self.stop = stop
        self.heading = heading

    def pickle(self):
        return pickle.dumps(self)

    @staticmethod
    def unpickle(value):
        return pickle.loads(value)
