# https://dhanuzch.medium.com/using-opencv-with-gazebo-in-robot-operating-system-ros-part-2-read-and-decode-a-qr-code-ed6ce5c298ca

# pip install pyzbar
# pip install setuptools==58.2.0

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge
import cv2

from pyzbar.pyzbar import decode

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
               
        self.br = CvBridge();
        self.currentTarget = ""
        self.landed = []
        self.direction = [0, 0]
        
        # Camera Subscriber
        self.camera_subscription = self.create_subscription(
            Image,
            'camera',
            self.camera_callback,
            10)
        self.camera_subscription # prevent unused variable warning
        
        # Movement Publisher
        self.movement_publisher = self.create_publisher(
            Int16MultiArray,
            'direction',
            10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        msg = Int16MultiArray()
        msg.data = self.direction
        
        self.movement_publisher.publish(msg)
        self.get_logger().info('Publishing: {}'.format(msg.data))
        
        
    def camera_callback(self, data):
        # self.get_logger().info('Receiving Image')
        cv_image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        (rows, cols, channels) = cv_image.shape
        
        image = cv_image
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        thres = 40
        img_bw = cv2.threshold(gray, thres, 255, cv2.THRESH_BINARY)[1]
        
        qr_result=decode(img_bw)
        
        droneViewMiddleX = int(cols/2)
        droneViewMiddleY = int(rows/2)
    
        if len(qr_result) !=0 :
            
            for result in qr_result:
                qr_data = result.data.decode('utf-8')
                
                if qr_data not in self.landed:
                
                    if self.currentTarget == "":
                        self.currentTarget = qr_data
                    elif self.currentTarget == qr_data:
                        
                        (x, y, w, h) = result.rect
                        
                        rectMiddleX = x + w/2
                        rectMiddleY = y + h/2
                        
                        cv2.line(image, (droneViewMiddleX, droneViewMiddleY), (int(rectMiddleX), int(rectMiddleY)), (255, 0, 0), 2)
                        
                        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 4)
                        
                        text = "{}".format(qr_data)
                        
                        cv2.putText(image, text, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        self.get_logger().info('QrCode data: {}'.format(qr_data))
                        
                        self.direction = [rectMiddleX - droneViewMiddleX, rectMiddleY - droneViewMiddleY]

        elif self.currentTarget == "":
            self.direction = []      
                    
        cv2.circle(image, (droneViewMiddleX, droneViewMiddleY), 5, (0, 0, 255), -1)
        cv2.imshow("Drone Camera", image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriber()

    rclpy.spin(camera_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()