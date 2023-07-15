import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import torch

model = YOLO('/home/chen/Documents/crane/detector/model/best.pt')

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'raw_img',
            self.imageCallback,1)
        self.subscriptions
        self.cvbridge = CvBridge()

    def imageCallback(self,msg):
        cv_img = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
        results = model.predict(cv_img,conf=0.35)
        res_plotted = results[0].plot()
        cv2.imshow("result", res_plotted)
        cv2.waitKey(1)

        xyxy = results[0].boxes.xyxy
        cls = results[0].boxes.cls
        if len(xyxy) != 6:
            return
        cls=cls.reshape(len(cls),1)
        c = torch.cat([xyxy,cls],dim=1)
        c = c.sort(0,False)[0]
        print(c)

        
        # for i in range(0,6):
        #     xyxy[i]=cls[i]
        # print(xyxy)
        # print(type(boxes))
        # print(boxes)

        # rect = torch.tensor()
        # for box in boxes:
        #     print(box.xyxy)
        #     print(box.cls)
        # print("--------------")
        # print(results[0])



def main():
    rclpy.init()
    # print('Hi from detector.')
    image_sub_ = ImageSubscriber()
    rclpy.spin(image_sub_)

    image_sub_.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
