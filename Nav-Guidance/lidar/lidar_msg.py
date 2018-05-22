import pickle


class LidarMsg:
    def __init__(self, scans):
        self.scans = scans

    def __str__(self):
        return vars(self)

    @staticmethod
    def unpickle(data):
        return pickle.loads(data)
