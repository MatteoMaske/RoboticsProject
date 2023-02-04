#TODO:
# 1. Get the image from the zed camera using the topic
# 2. Crop the image to the size of the object
# 3. Use the cropped image to detect the object using yolo
# 4. Use the topic to get the point in the world frame of the object as you see in ur5_generic.py
# 5. Create an ad-hoc service/topic to send the point to move.cpp


#!/usr/bin/env python
# license removed for brevity
import rospy
import torch
from std_msgs.msg import String
import sensor_msgs.msg
#import pyzed.sl as sl
import cv2
from PIL import Image
import numpy as np
import time
import subprocess

ZED_LEFT_TOPIC = "/ur5/zed2/left/image_rect_color"
ZED_DEPTH_TOPIC = "/ur5/zed2/depth/depth_registered"
SLEEP_RATE = 5

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
'''
This function retrieves only part of the image from the camera
@image the image to be cropped`
@pointA the top left point of the rectangle
@pointB the bottom right point of the rectangle
@return the cropped image
'''
def filterImage(image, pointA, pointB):
    if image is Image.Image:
        image_array = np.asarray(image)
    image_array_cropped = image_array[pointA[0]:pointB[0], pointA[1]:pointB[1]]
    return Image.fromarray(image_array_cropped)


'''
Function that initializes the camera and it's parameters
'''
def initCamera():
    # Initialize camera
    print("Initializing camera...")
    zed = sl.Camera()

    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.sdk_verbose = False
    init_params.camera_resolution = sl.RESOLUTION.VGA
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_minimum_distance = 0.1
    init_params.depth_maximum_distance = 10
    err = zed.open(init_params)

    # Try to open the camera
    if err != sl.ERROR_CODE.SUCCESS:
        print("An unexpected error occurred while opening the camera.")
        exit(1)

    # Object detection parameters
    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking = False
    obj_param.enable_mask_output = True
    obj_param.image_sync = True

    zed_serial = zed.get_camera_information().serial_number
    print("Camera serial number: ", zed_serial)
    
    err = zed.enable_object_detection(obj_param)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Object detection initialization failed.")
        zed.close()
        exit(1)

    return zed

def onImageReceived(data):
    rospy.loginfo(f"Received a message of size {data.height}x{data.width} at time {data.header.stamp}")
    image = Image.fromarray(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1))
    image.show()
    rospy.sleep(.5)
    ######
    # Crop the image before returning it
    cropped_image = image.crop((data.width/2, data.height/2, data.width, data.height))
    ######
    cropped_image.show()
    rospy.sleep(SLEEP_RATE)
    return image

if __name__ == '__main__':
    # change ros rate to 10
    ret = subprocess.call("rosrun rostime", shell=True)
    while not rospy.is_shutdown():
        ## Connect to the zed topic to get info about the images in np array format
        rospy.Subscriber(ZED_LEFT_TOPIC, sensor_msgs.msg.Image, onImageReceived, queue_size=1)
        #rospy.Subscriber(ZED_DEPTH_TOPIC, sensor_msgs.msg.Image, callback)
        try:
            talker()
        except rospy.ROSInterruptException:
            pass
    