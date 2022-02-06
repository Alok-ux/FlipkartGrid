import cv2
import csv
import rospy
import rospkg

from camera_driver.msg import GridPoseArray
from cv_bridge import CvBridge, CvBridgeError

cv_bridge = CvBridge()
data = {}

path = rospkg.RosPack().get_path('grid_phase2_controller') + '/data/'

with open(path + 'coordinates_backup.csv', 'r') as file:
    reader = csv.reader(file)
    for x, y, tx, ty in reader:
        data[(int(x), int(y))] = [int(tx), int(ty)]

for x in [2, 6, 10]:
    for y in range(1, 14, 1):
        data[x, y][0] -= 10

for x in [5, 9, 13]:
    for y in range(1, 14, 1):
        data[x, y][0] += 10

for y in [1]:
    for x in range(1, 15, 1):
        data[x, y][1] += 10

for y in [5]:
    for x in range(1, 15, 1):
        data[x, y][1] += 2 #7

for y in [9]:
    for x in range(1, 15, 1):
        data[x, y][1] += 0 #7        

for y in [4, 8]:
    for x in range(1, 15, 1):
        data[x, y][1] -= 0 #5

for y in [12]:
    for x in range(1, 15, 1):
        data[x, y][1] -= 5
        
with open(path + 'coordinates.csv', 'w') as file:
    writer = csv.writer(file)
    for pose in data:
        writer.writerow([pose[0], pose[1], data[pose][0], data[pose][1]])

def callback(msg):
    try:
        image = cv_bridge.imgmsg_to_cv2(msg.image)
        for pose in data:
            cv2.circle(image, data[pose], 2, (0, 0, 255), -1)
        cv2.imshow('image', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('killed')
    except CvBridgeError as e:
        print(e)

rospy.init_node('demo')
rospy.Subscriber('grid_robot/poses', GridPoseArray, callback)

try:
    rospy.spin()
except rospy.ROSInterruptException as e:
    print(e)
cv2.destroyAllWindows()