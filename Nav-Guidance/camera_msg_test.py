import pickle
from camera_msg import CameraMsg

print("Running Tests")

test_object = CameraMsg()
print("created object")
print(test_object)
test_dumps = test_object.pickleMe()
print("pickled and unpicked object")
print(CameraMsg(test_dumps))
test_object.setLeftLine("leftLine")
test_object.setRightLine("rightLine")
print("changed parameters")
print(test_object)
test_dumps2 = test_object.pickleMe()
print("unpicked again")

print(CameraMsg(test_dumps2))
