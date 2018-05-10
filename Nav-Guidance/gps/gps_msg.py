import pickle


class GPSMsg:
    def __init__(self, lat, lon, heading, fixed=False):
        self.lat = lat
        self.lon = lon
        self.heading = heading
        self.fixed = fixed

    def __str__(self):
        return vars(self)

    @staticmethod
    def unpickle(data):
        return pickle.loads(data)
