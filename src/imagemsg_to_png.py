#!/usr/bin/env python
###############################################################################
# Duckietown - Project Unicorn ETH
# Author: Simon Schaefer
# Subscribe and store compressed image. 
###############################################################################
import cv2
from cv_bridge import CvBridge, CvBridgeError
from os.path import isdir
import rospy
from sensor_msgs.msg import CompressedImage

class Main(): 

    def __init__(self): 
        self.bridge = CvBridge()
        topic = rospy.get_param("/imagemsg_to_png/img_topic")
        rospy.Subscriber(topic, CompressedImage, self.callback)
        self.i = 0
        self.storage_path = rospy.get_param("/imagemsg_to_png/storage_dir")
        if not isdir(self.storage_path): 
            raise OSError("Invalid storage path !")
        rospy.spin()
        
    def callback(self, data):
        """Store message data as png."""
        try: 
            frame = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logfatal(e)
        name = self.storage_path + "/" + str(self.i) + ".png"
        cv2.imwrite(name, frame)
        self.i += 1

if __name__ == '__main__':
    rospy.init_node('converter_imagemsg_png', anonymous=True)
    try:
        Main()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
