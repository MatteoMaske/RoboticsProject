#! /usr/bin/env python

from ultralytics import YOLO
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
import sensor_msgs.msg
from PIL import Image
from cv_bridge import CvBridge
from py_publisher.msg import BlockDetected #custom messages
from py_publisher.msg import BlockInfo
from std_msgs.msg import Byte, Int16
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

# result.boxes.xyxy   # box with xyxy format, (N, 4)
# result.boxes.xywh   # box with xywh format, (N, 4)
# result.boxes.xyxyn  # box with xyxy format but normalized, (N, 4)
# result.boxes.xywhn  # box with xywh format but normalized, (N, 4)
# result.boxes.conf   # confidence score, (N, 1)
# result.boxes.cls    # cls, (N, 1)

ZED_LEFT_TOPIC = "/ur5/zed_node/left/image_rect_color"
ZED_DEPTH_TOPIC = "/ur5/zed_node/depth/depth_registered"
SLEEP_RATE = 1

WEIGHT = '/home/stefano/ros_ws/src/visionData/best.pt'
IMAGE = '/home/stefano/ros_ws/src/visionData/1.jpg'

IMGSZ = 1280

pointX, pointY = 0,0

def findCenter(result):
    block = BlockInfo()
    block.id = 1 #TODO: implement a function that generates unique id
    block.objectClass = int(result.boxes.cls.tolist()[0]) #class of the block

    #find the center of the square for zed api
    x1 = int(result.boxes.xyxy[0][0])
    y1 = int(result.boxes.xyxy[0][1])
    x2 = int(result.boxes.xyxy[0][2])
    y2 = int(result.boxes.xyxy[0][3])
    #find the center of the square
    x = int((x1 + x2) / 2)
    y = int((y1 + y2) / 2)

    print("xx: ", x)
    print("yy: ", y)

    global pointX
    global pointY
    pointX = x
    pointY = y

def detect(image):
    model = YOLO(WEIGHT)
    results = model.predict(source=image, imgsz=IMGSZ, stream=True)
    for result in results:
        if not len(result) == 0:
            findCenter(result)
        else:
            print("No blocks detected")

def callback(data):
    rospy.loginfo(rospy.get_caller_id())
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    # cv2.imshow('image',image)
    # cv2.waitKey(0)
    #cropImage() # need to crop the table
    cv2.imwrite('img.jpeg',image)
    detect(image)

def talker():
    rospy.init_node('publisher',anonymous=True)
    pub = rospy.Publisher('vision/vision_detection', BlockDetected, queue_size=100)

    block = BlockInfo()
    block.id = Int16(1)
    block.objectClass = Byte(1)
    block.position = Point(1,1,1)

    msg = BlockDetected()
    msg.blockDetected.append(block)
    
    pub.publish(msg)

def receivePointcloud(msg):
    points_list = []
    #this is the center of the image plane
    # center_x = int(msg.width / 2)
    # center_y = int(msg.height / 2)

    print("X: ",pointX)
    print("Y: ", pointY)

    for data in point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=False, uvs=[(pointX, pointY)]):
        points_list.append([data[0], data[1], data[2]])

    #print("Data Optical frame: ", points_list)

    #black magic
    base_offset = [0.5,0.35,1.75]
    x_c = [-0.9,0.24,-0.35]

    rotation = np.array([[0., -0.49948, 0.86632],
 	                     [-1., 0., 0.],
 	                     [-0., -0.86632, -0.49948]])
    
    #coordinates in the world frame
    pointW = rotation.dot(points_list[0]) + x_c + base_offset
    print("World: ", pointW)

if __name__ == '__main__':

    rospy.init_node('publisher',anonymous=True)
    rospy.Subscriber(ZED_LEFT_TOPIC, sensor_msgs.msg.Image, callback) #subscribe to zed image
    rospy.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, receivePointcloud) #subscribe to zed point cloud

    # talker()

    while not rospy.is_shutdown():
        rospy.sleep(SLEEP_RATE)

############################################################################################
"""
Matrix to store data for each block:

        class   confidence    box
block1    0       0.9999    (x, y)
block2    1       0.4678    (x, y)
block3    8       0.6534    (x, y)
"""

# model = YOLO('best.pt')

# results = model.predict(source='1.jpg', imgsz=IMGSZ, stream=True)

# matrix = np.empty((0,4))

# for result in results:

#     blockClass = int(result.boxes.cls.tolist()[0])
#     confidence = round(result.boxes.conf.tolist()[0],4)

#     #print("Block: " + str(i) + "\n   Class: " + str(blockClass) + "\n   Confidence: " + str(confidence))

#     x1 = int(result.boxes.xyxy[0][0])
#     y1 = int(result.boxes.xyxy[0][1])
#     x2 = int(result.boxes.xyxy[0][2])
#     y2 = int(result.boxes.xyxy[0][3])

#     #find the center of the square
#     x = int((x1 + x2) / 2)
#     y = int((y1 + y2) / 2)

#     matrix = np.vstack((matrix, [blockClass, confidence, x, y]))
#     #print(matrix)

# img = cv2.imread('1.jpg')
# for i in range(matrix.shape[0]):

#     x = int(matrix[i][2])
#     y = int(matrix[i][3])

#     img = cv2.circle(img, (x, y), 5, (255, 0, 0), -1)

# cv2.imshow('image', img)
# cv2.waitKey(0)
############################################################################################
