#!/usr/bin/env python
# license removed for brevity
import rospy
import torch
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    #model
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    
    dir = '/home/robo/Downloads/IMG_20210130_153000.jpg'
    imgs = [dir]  # batch of images
    res = model(imgs)
    res.show()
    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    