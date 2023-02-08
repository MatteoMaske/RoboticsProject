#! /usr/bin/env python

from ultralytics import YOLO
import cv2
import numpy as np
import rospy
from PIL import Image
import sensor_msgs.msg
from cv_bridge import CvBridge
# from py_publisher.msg import BlockDetected #custom messages
from py_publisher.msg import BlockInfo
from std_msgs.msg import Byte, Int16, Bool, String
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import math
import message_filters

"""
TODO:
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
PLANNER_DETECTION_REQUEST_TOPIC = "/planner/detection_request"
# ZED_DEPTH_TOPIC = "/ur5/zed_node/depth/depth_registered"

SLEEP_RATE = 10

WEIGHT = '/home/stefano/ros_ws/src/visionData/best.pt'
# IMAGE = '/home/stefano/ros_ws/src/visionData/1.jpg'

IMGSZ = 1280

#Z limits of the table
MIN_Z = 0.8
MAX_Z = 0.95
#X limits of the table
MAX_X = 0.5

#Detection request sent by planner to enable the vision to publish
detectionRequest = False

def findCenter(result):

    #Find the center of the square for zed api
    x1 = int(result.boxes.xyxy[0][0])
    y1 = int(result.boxes.xyxy[0][1])
    x2 = int(result.boxes.xyxy[0][2])
    y2 = int(result.boxes.xyxy[0][3])
    #Find the center of the square
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
    #Get weights
    model = YOLO(WEIGHT)
    #Detect blocks
    results = model.predict(source=image, imgsz=IMGSZ)

    list = []
    #If results are not empty append the center of the square to the list
    if not (len(results[0]) == 0):
        for i in range(len(results[0])):
            list.append(findCenter(results[0][i]))
    else:
        print("No blocks detected")

    # print("List:\n", list)
    return list

def buildMsg(block):

    # print("Block:\n", block)

    msg = BlockInfo()
    msg.blockId = Int16(0) #Int16(block['id'])
    msg.blockClass = Byte(block['class'])
    msg.blockPosition = Point(block['x'], block['y'], block['z'])

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

    #If the point didn't change, don't do anything
    # if not (abs(x - prevX) < 10 and abs(y - prevY) < 10): # need to check if the point didn't changed

    for data in point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=False, uvs=[(list['x'], list['y'])]):
        points_list.append([data[0], data[1], data[2]]) #coordinates in the camera frame

    #Black magic
    base_offset = [0.5,0.35,1.75]
    x_c = [-0.9,0.24,-0.35]

    rotation = np.array([[0., -0.49948, 0.86632],
                        [-1., 0., 0.],
                        [-0., -0.86632, -0.49948]])
    
    #Coordinates in the world frame
    pointW = rotation.dot(points_list[0]) + x_c + base_offset
    # print("World coord: ", pointW)

    #Check if the point is a NaN
    if not (math.isnan(pointW[0]) and math.isnan(pointW[1]) and math.isnan(pointW[2])):
        listCoord['x'] = pointW[0]
        listCoord['y'] = pointW[1]
        listCoord['z'] = pointW[2]
    else:
        print("Nan detected")

    # else:
    #     print("Point not changed")

    print("Listcoord:\n", listCoord)

    #Check if the point is on the table
    if listCoord['z'] >= MIN_Z and listCoord['z'] <= MAX_Z and listCoord['x'] <= MAX_X:
        return listCoord
    else:
        return None

def talker(msg):
    rospy.init_node('publisher',anonymous=True)
    pub = rospy.Publisher('vision/vision_detection', BlockInfo, queue_size=10) #Publish to planner
    
    rospy.sleep(1)

    if pub.get_num_connections() > 0:
        print("Publishing message:\n", msg)
        pub.publish(msg)
    else:
        print("No subscribers")

def callback(img, pointCloud):

    global detectionRequest

    print("Waiting for detection request")

    #Wait for planner detection request
    if detectionRequest:

        print("Detection request received")

        #Convert image to cv2
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')

        #need to crop the image to get only the table
        # cropImage()

        #Detect blocks with YOLO
        list = detect(image)

        #Get coordinates from x,y pixels
        listCoord = []
        for i in range(len(list)):
            block = receivePointcloud(pointCloud, list[i])
            if block != None: #check z limits
                listCoord.append(block)

        #print("len(listCoord): ", len(listCoord))
        #print("listCoord:\n", listCoord)

        #Keep only the nearest block to the camera
        if len(listCoord) > 1: #If more than one block detected
            block = min(listCoord, key=lambda x: x['x'])
        elif len(listCoord) == 1: #If only one block detected
            block = listCoord[0]
        else:
            print("No blocks detected")
            block = { #If no blocks detected, send a message with 0 coordinates
                'id': 0,
                'class': 0,
                'x': 0,
                'y': 0,
                'z': 0
            }
        #print("Block:\n", block)

        #Build the massage to publish it
        msg = buildMsg(block)
        # print("Message:\n", msg)
        
        #Disable the callback
        detectionRequest = False

        #Publish the message
        talker(msg)

def listenerDetectionReq(msg): #Listen to planner detection request
    print("Detection request received")
    global detectionRequest
    detectionRequest = True

if __name__ == '__main__':

    rospy.init_node('publisher',anonymous=True)

    imageSub = message_filters.Subscriber(ZED_LEFT_TOPIC, sensor_msgs.msg.Image) #Subscribe to zed image
    pointCloudSub = message_filters.Subscriber(ZED_POINT_CLOUD_TOPIC, PointCloud2) #Subscribe to zed point cloud
    ts = message_filters.TimeSynchronizer([imageSub, pointCloudSub], 1)
    ts.registerCallback(callback)

    rospy.Subscriber(PLANNER_DETECTION_REQUEST_TOPIC, Bool, listenerDetectionReq, queue_size=10) #Subscribe to planner detectionRequest

    while not rospy.is_shutdown():
        rospy.sleep(SLEEP_RATE)
