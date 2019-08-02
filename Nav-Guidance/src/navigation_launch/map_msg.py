import cv2


class MapData:
    def __init__(self, transform, map_data):
        self.transform = transform
        self.map_bytes = map_data

    def __add__(self, other):
        return MapData(transform=self.transform,
                       map_data=cv2.bitwise_not(cv2.add(cv2.bitwise_not(self.map_bytes), cv2.bitwise_not(other.map_bytes))))
