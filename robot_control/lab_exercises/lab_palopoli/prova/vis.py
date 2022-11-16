#!/usr/bin/env python
import rospy as ros
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import params as conf

class image_converter:

    def __init__(self):

        self.image_pub = ros.Publisher("/z_base_camera/camera/rgb/image_raw",Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = ros.Subscriber("/z_base_camera/camera/rgb/image_raw",Image,self.callback)
        self.net = cv.dnn.readNet('yolov5n.onnx')
        file = open('coco.txt')
        self.classes = file.read().split('\n')
        print(self.classes)
        

    
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        blob = cv.dnn.blobFromImage(cv_image, scalefactor= 1/255, size=(640,640), mean= [0,0,0], swapRB= True, crop=False)
        self.net.setInput(blob)
        detections = self.net.forward()[0]

        rows = detections.shape[0]

        img_width, img_height = cv_image.shape[1], cv_image.shape[0]

        x_scale = img_width/640
        y_scale = img_height/640
        classes_ids = []
        confidences = []
        boxes = []

        for i in range(rows):
            row = detections[i]
            confidence = row[4]
            if confidence > 0.5:
                classes_score = row[5:]
                ind = np.argmax(classes_score)
                if classes_score[ind] > 0.5:
                    classes_ids.append(ind)
                    confidences.append(confidence)
                    cx, cy, w, h = row[:4]
                    x1 = int((cx-w/2)*x_scale)
                    y1 = int((cy-h/2)*y_scale)
                    width = int(w*x_scale)
                    height = int(h*y_scale)
                    box = np.array([x1,y1,width,height])
                    boxes.append(box)

        indices = cv.dnn.NMSBoxes(boxes,confidences,0.5,0.5)

        for i in indices:
            x1,y1,w,h = boxes[i]
            label = self.classes[classes_ids[i]]
            conf = confidences[i]
            text = label +"{:.2f}".format(conf)
            cv.rectangle(cv_image,(x1,y1),(x1+w,y1+h),(255,0,0),2)
            cv.putText(cv_image,text,(x1,y1-2),cv.FONT_HERSHEY_COMPLEX, 0.7, (0,0,255),2)



        #cx,cy, w,h, confidence, class_scores
        #class_ids, condifence_scores, bounding_boxes



        cv.imshow("Image window", cv_image)
        cv.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
    
    def format_yolov5(self, source):

        # put the image in square big enough
        col, row, _ = source.shape
        _max = max(col, row)
        resized = np.zeros((_max, _max, 3), np.uint8)
        resized[0:col, 0:row] = source
        
        # resize to 640x640, normalize to [0,1[ and swap Red and Blue channels
        result = cv.dnn.blobFromImage(resized, 1/255.0, (640, 640), swapRB=True)
        
        return result

if __name__ == '__main__':
    ic = image_converter()
    ros.init_node('image_converter', anonymous=True)
    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()
