import rospy
import pickle
from std_msgs.msg import String

def dummynav(dgps):
	return {'gps': dgps,
			'lidar': [],
			'camera': []
	}

def dummygps(heading, lat, lon):
	return {'lat': lat,
			'lon': lon,
			'heading': heading,
			'satellites': 5,
			'fixed': True}

firstgps = dummygps(10,10,10)
firstnav = dummynav(firstgps)

rospy.init_node('dummy')
navp = rospy.Publisher('NAV', String, queue_size=3)
gpsp = rospy.Publisher('GPS', String, queue_size=3)

def pubnav(navp, np):
	print('publishing nav')
	navp.publish(pickle.dumps(np))

def pubgps(gpsp, np):
	print('publishing gps')
	gpsp.publish(pickle.dumps(np))

pubnav(navp, firstnav)

pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
pubgps(gpsp, firstgps)
