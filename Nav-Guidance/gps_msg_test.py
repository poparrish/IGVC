import pickle
from gps_msg import GPSMsg

print("Running Tests")
gps_point = GPSMsg()
print(gps_point)
ps1 = gps_point.pickleMe()
print(GPSMsg(pickled_values = ps1))
print("Creating with data")

gps_point = GPSMsg(latlong = (-123.123132,132.12321))
print(gps_point)
ps1 = gps_point.pickleMe()
print(GPSMsg(pickled_values = ps1))