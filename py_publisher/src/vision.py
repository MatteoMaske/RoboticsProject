#! /usr/bin/env python

from ultralytics import YOLO
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
import subprocess
import sensor_msgs.msg
from PIL import Image
from cv_bridge import CvBridge
from py_publisher.msg import BlockDetected #custom messages
from py_publisher.msg import BlockInfo

# result.boxes.xyxy   # box with xyxy format, (N, 4)
# result.boxes.xywh   # box with xywh format, (N, 4)
# result.boxes.xyxyn  # box with xyxy format but normalized, (N, 4)
# result.boxes.xywhn  # box with xywh format but normalized, (N, 4)
# result.boxes.conf   # confidence score, (N, 1)
# result.boxes.cls    # cls, (N, 1)

"""
Matrix to store data for each block:

        class   confidence    box
block1    0       0.9999    (x, y)
block2    1       0.4678    (x, y)
block3    8       0.6534    (x, y)
"""

ZED_LEFT_TOPIC = "/ur5/zed_node/left/image_rect_color"
ZED_DEPTH_TOPIC = "/ur5/zed_node/depth/depth_registered"
SLEEP_RATE = 10

WEIGHT = '/home/stefano/ros_ws/src/visionData/best.pt'
IMAGE = '/home/stefano/ros_ws/src/visionData/1.jpg'

IMGSZ = 1280

def detect(image):
    model = YOLO(WEIGHT)
    results = model.predict(source=image, imgsz=IMGSZ, stream=True)
    if not len(list(results)) == 0:
        for result in results:
            #blockClass = int(result.boxes.cls.tolist()[0])
            print("Class: " + str(result.boxes))

def callback(data):
    rospy.loginfo(rospy.get_caller_id())
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    # cv2.imshow('image',image)
    # cv2.waitKey(0)
    #cropImage() # need to crop the table
    detect(image)

def listener():
    #anonymous=True rospy generates a unique name for the node so we can have multiple listeners
    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber(ZED_LEFT_TOPIC, sensor_msgs.msg.Image, callback)

    while not rospy.is_shutdown():
        rospy.sleep(SLEEP_RATE)

def talker():
    print("NOT IMPLEMENTED")
    # rospy.init_node('publisher',anonymous=True)
    # pub = rospy.Publisher('vision/vision_detection', BlockDetected, queue_size=100)
    # block = BlockInfo()
    # block.objectClass = 1
    # block.position = (1,2,3)
    # block.id = 1

    # msg = BlockDetected()
    # msg.blockDetected.append(block)

    

    # while True:
    #     print("Publishing")
    #     pub.publish(msg)
    #     rospy.sleep(SLEEP_RATE)

if __name__ == '__main__':
    listener()
    #talker()


############################################################################################
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
