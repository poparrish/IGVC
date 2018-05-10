class GPSMsg:
    def __init__(self, lat, lon, heading, fixed):
        self.lat = lat
        self.lon = lon
        self.heading = heading
        self.fixed = fixed

    def __str__(self):
        return vars(self)
