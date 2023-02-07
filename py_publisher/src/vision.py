#! /usr/bin/env python

from ultralytics import YOLO
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from PIL import Image
import sensor_msgs.msg
from cv_bridge import CvBridge
from py_publisher.msg import BlockDetected #custom messages
from py_publisher.msg import BlockInfo
from std_msgs.msg import Byte, Int16
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import math
import message_filters

"""
TODO:
    - implement a function that generates unique id
    - if pixels don't change, don't publish
    done - check if the z is right [0.85 - 0.90]
    - crop the image to see only the table
"""

# result.boxes.xyxy   # box with xyxy format, (N, 4)
# result.boxes.xywh   # box with xywh format, (N, 4)
# result.boxes.xyxyn  # box with xyxy format but normalized, (N, 4)
# result.boxes.xywhn  # box with xywh format but normalized, (N, 4)
# result.boxes.conf   # confidence score, (N, 1)
# result.boxes.cls    # cls, (N, 1)

ZED_LEFT_TOPIC = "/ur5/zed_node/left/image_rect_color"
ZED_POINT_CLOUD_TOPIC = "/ur5/zed_node/point_cloud/cloud_registered"
# ZED_DEPTH_TOPIC = "/ur5/zed_node/depth/depth_registered"
SLEEP_RATE = 10

WEIGHT = '/home/stefano/ros_ws/src/visionData/best.pt'
# IMAGE = '/home/stefano/ros_ws/src/visionData/1.jpg'

IMGSZ = 1280

MIN_Z = 0.85
MAX_Z = 0.90

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

    blockClass = int(result.boxes.cls.tolist()[0])

    info = {
        'id': 0,
        'class': blockClass,
        'x': x,
        'y': y
    }

    return info

def detect(image):
    model = YOLO(WEIGHT)
    results = model.predict(source=image, imgsz=IMGSZ)
    list = []
    if not (len(results[0]) == 0):
        for i in range(len(results[0])):
            list.append(findCenter(results[0][i]))
    else:
        print("No blocks detected")

    # print("List:\n", list)
    return list

def buildMsg(listCoord):

    msg = BlockDetected()

    for i in range(len(listCoord)):
        block = BlockInfo()
        block.id = Int16(listCoord[i]['id'])
        block.objectClass = Byte(listCoord[i]['class'])
        block.position = Point(listCoord[i]['x'], listCoord[i]['y'], listCoord[i]['z'])
        msg.blockDetected.append(block)

    print("Message:\n", msg)
    return msg

def receivePointcloud(msg, list):
    points_list = []
    listCoord = {
        'id': list['id'],
        'class': list['class'],
        'x': 0,
        'y': 0,
        'z': 0
    }

    x = list['x']
    y = list['y']

    #if the point didn't change, don't do anything
    # if not (abs(x - prevX) < 10 and abs(y - prevY) < 10): # need to check if the point didn't changed

    for data in point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=False, uvs=[(x, y)]):
        points_list.append([data[0], data[1], data[2]]) #coordinates in the camera frame

    #black magic
    base_offset = [0.5,0.35,1.75]
    x_c = [-0.9,0.24,-0.35]

    rotation = np.array([[0., -0.49948, 0.86632],
                        [-1., 0., 0.],
                        [-0., -0.86632, -0.49948]])
    
    #coordinates in the world frame
    pointW = rotation.dot(points_list[0]) + x_c + base_offset
    # print("World coord: ", pointW)

    #check if the point is a NaN
    if not (math.isnan(pointW[0]) and math.isnan(pointW[1]) and math.isnan(pointW[2])):
        # buildMsg(pointW)
        listCoord['x'] = pointW[0]
        listCoord['y'] = pointW[1]
        listCoord['z'] = pointW[2]
    else:
        print("Nan detected")
    
    # else:
    #     print("Point not changed")

    return listCoord

def talker(msg):
    rospy.init_node('publisher',anonymous=True)
    pub = rospy.Publisher('vision/vision_detection', BlockDetected, queue_size=100) #publish to planner
    
    print("Publishing message:\n", msg)
    pub.publish(msg)

def callback(img, pointCloud):
    #convert image to cv2
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')

    #need to crop the image to get only the table
    # cropImage()

    #detect blocks with YOLO
    list = detect(image)

    #get coordinates from x,y pixels
    listCoord = []
    for i in range(len(list)):
        if list[i]['z'] >= MIN_Z or list[i]['z'] <= MAX_Z:
            listCoord.append(receivePointcloud(pointCloud, list[i]))
        else:
            print("Block out of z range")

    #build the massage to publish it
    msg = buildMsg(listCoord)
    # print("Message:\n", msg)
    
    #publish the message
    talker(msg)

if __name__ == '__main__':

    rospy.init_node('publisher',anonymous=True)

    imageSub = message_filters.Subscriber(ZED_LEFT_TOPIC, sensor_msgs.msg.Image) #subscribe to zed image
    pointCloudSub = message_filters.Subscriber(ZED_POINT_CLOUD_TOPIC, PointCloud2) #subscribe to zed point cloud
    ts = message_filters.TimeSynchronizer([imageSub, pointCloudSub], 1)
    ts.registerCallback(callback)

    while not rospy.is_shutdown():
        rospy.sleep(SLEEP_RATE)
