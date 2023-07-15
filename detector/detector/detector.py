import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

model = YOLO('/home/chen/Documents/crane/detector/model/best.pt')  # load a custom model

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'raw_img',
            self.imageCallback,10)
        self.subscriptions

    def imageCallback(self,msg):
        cv_img = CvBridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("frame" , cv_img)
        cv2.waitKey(3)
        results = model(cv_img)
        print(results)
    
def main():
    rclpy.init()
    # print('Hi from detector.')
    image_sub_ = ImageSubscriber()
    rclpy.spin(image_sub_)

    image_sub_.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
