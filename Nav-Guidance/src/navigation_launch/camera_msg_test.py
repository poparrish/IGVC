import pickle
from camera_msg import CameraMsg
import numpy as np

print("Running Tests")
test = np.zeros([10,10,3])

test_msg = CameraMsg(local_map_val = test)
# print(test_msg)
test_msg_pickle = test_msg.pickleMe()
# print(test_msg_pickle)
print(type(test_msg_pickle))
testmsg2 = CameraMsg(pickled_values = test_msg_pickle)
# print(testmsg2)