import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import torch
from interfaces.msg import Serial

model = YOLO('/home/chen/Documents/crane_detection/detector/model/best.pt')

dic={7:1,11:2,13:3,14:4,19:5,21:6,22:7,25:8,
       26:9,28:10,35:11,37:12,38:13,41:14,42:15,44:16,49:17,50:18,52:19,56:20}

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'raw_img',
            self.imageCallback,1)
        self.subscriptions
        self.cvbridge = CvBridge()
        self.publisher_ = self.create_publisher(
            Serial,
            'type',
            1
        )
        self.sendmsg = Serial()
        self.sendmsg.type = 0
        self.correctPics = 0
        self.type = 0
        
    def imageCallback(self,msg):
        cv_img = self.cvbridge.imgmsg_to_cv2(msg, "bgr8")
        results = model.predict(cv_img,conf=0.5)
        res_plotted = results[0].plot()
        cv2.imshow("result", res_plotted)
        cv2.waitKey(1)

        xyxy = results[0].boxes.xyxy
        cls = results[0].boxes.cls
        sum = torch.sum(cls)
        msg = Serial()
        if len(xyxy) == 6 and sum == 3:
            cls=cls.reshape(len(cls),1)
            x=xyxy[:,0:1]
            indices = x.sort(0,False)[1]
            c=torch.gather(cls, 0, indices)
            msg.object1 = int(c[0][0])
            msg.object2 = int(c[1][0])
            msg.object3 = int(c[2][0])
            msg.object4 = int(c[3][0])
            msg.object5 = int(c[4][0])
            msg.object6 = int(c[5][0])
            type = int(c[0][0])*32+int(c[1][0])*16+int(c[2][0])*8+int(c[3][0])*4+int(c[4][0])*2+int(c[5][0])
            if type > 7:
                msg.more = 0
            else:
                msg.more = 1
            msg.type =  dic[type]
        else:
            self.correctPics = 0
            
        if msg.type == self.type:
            self.correctPics += 1
            if self.correctPics > 3:
                self.sendmsg = msg
        else:
            self.type = msg.type
            self.correctPics = 0
                    
        self.publisher_.publish(self.sendmsg)


def main():
    rclpy.init()
    image_sub_ = ImageSubscriber()
    rclpy.spin(image_sub_)
    image_sub_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
