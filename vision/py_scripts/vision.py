#!/usr/bin/env python
# license removed for brevity
import rospy
import torch
from std_msgs.msg import String
import pyzed.sl as sl

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

if __name__ == '__main__':
    capture_thread = Thread(target=torch_thread,
                            kwargs={'weights': opt.weights, 'img_size': opt.img_size, "conf_thres": opt.conf_thres})

    zed = initCamera()



    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    