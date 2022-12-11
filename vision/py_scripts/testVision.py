import torch
import pyzed.sl as sl
import numpy as np
import math
##
# Questo è solo un po' di lavoro iniziale per comprendere meglio le API di ZED
##

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
    init_params.camera_resolution = sl.RESOLUTION.WVGA
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
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
'''
This function retrieves only the image from the camera
@param camera: the zed camera object
'''
def captureImage(camera):
    image = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    if (camera.grab() == sl.ERROR_CODE.SUCCESS):
        camera.retrieve_image(image, sl.VIEW.LEFT)
        timestamp = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE)
        print("Image resolution : {0} x {1} || Image timestamp : {2}\n".format(image.get_width(), image.get_height(), timestamp))
    return image
'''
This function retrieves only the depth (in meters) map from the camera
@param camera: the zed camera object
'''
def getDepth(camera):
    depth = sl.Mat()
    if (camera.grab() == sl.ERROR_CODE.success):
        camera.retrieve_measure(depth, sl.MEASURE.DEPTH)
    return depth
'''
This function retrieves both the image and the depth map from the camera
@param camera: the zed camera object
'''
def getImageandDepth(camera):
    image = sl.Mat()
    depth = sl.Mat()
    if (camera.grab() == sl.ERROR_CODE.SUCCESS):
        camera.retrieve_image(image, sl.VIEW.LEFT)
        camera.retrieve_measure(depth, sl.MEASURE.DEPTH)
        timestamp = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE)
        print("Image resolution : {0} x {1} || Image timestamp : {2}\n".format(image.get_width(), image.get_height(), timestamp))
    return image, depth
'''
This function retrieves the depth of a certain point in the image
@param depth_map: the depth map of the image or the point cloud
@param point: the point in the image
@param depth_mode: the depth mode of the camera
'''
def getDistForPoint(depth_map, point, depth_mode):
    distance = -1
    if (depth_mode == sl.MEASURE.DEPTH):
        distance = depth_map.get_value(point[0], point[1])
    elif (depth_mode == sl.MEASURE.XYZRGBA):
        err, point_cloud_val = depth_map.get_value(point[0], point[1])
        distance = math.sqrt(point_cloud_val[0]**2, point_cloud_val[1]**2, point_cloud_val[2]**2)
    else :
        print("Error: Unknown depth mode")
    if (distance == -1):
        print("Error: Could not get distance for point")
    if np.isnan(distance):
        print("Error: Maybe you're too close to the object?")
    if np.isinf(distance):
        print("Error: Maybe you're too far from the object?")
    return distance

'''
This function detects objects in the image and then prints the 
information about them.
@param zed: the zed camera object
'''
def captureData(zed):
    #It will update the detection at each frame, to change this set the
    # image_sync parameter to False in the object detection parameters in
    # the initCamera function
    objects = sl.Objects()
    obj_runtimeParameters = sl.ObjectDetectionRuntimeParameters()
    while zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed_error = zed.retrieve_objects(objects, obj_runtimeParameters)
        if zed_error == sl.ERROR_CODE.SUCCESS:
            for i in range(objects.object_list.size()):
                obj = objects.object_list[i]
                print("Object ID: {0} || Label: {1} || Confidence: {2} || Position: {3}".format(obj.id, obj.label, obj.confidence, obj.position))
                print("Bounding box: {0}".format(obj.bounding_box_2d))
                print("Tracking state: {0}")
    zed.disable_object_detection()
    return objects

if __name__ == "__main__":
    zed = initCamera()
    image = captureImage(zed)
    depth = getDepth(zed)
    image, depth = getImageandDepth(zed)
    dist = getDistForPoint(zed, (0,0), sl.MEASURE.DEPTH)
    objects = captureData(zed)
    zed.close()