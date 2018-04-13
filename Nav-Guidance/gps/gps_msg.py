class GPSMsg:
    def __init__(self, lat, lon, heading):
        self.lat = lat
        self.lon = lon
        self.heading = heading

    def __str__(self):
        return vars(self)
