# https://dhanuzch.medium.com/using-opencv-with-gazebo-in-robot-operating-system-ros-part-2-read-and-decode-a-qr-code-ed6ce5c298ca

# pip install pyzbar
# pip install setuptools==58.2.0

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import numpy as np

from pyzbar.pyzbar import decode

import math

class ImageSubscriber(Node):
    def __init__(self):
        
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.listener_callback,
            10)
        self.subscription
        
        # Create subscribers for drone state
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_local_position = VehicleLocalPosition()
        self.br = CvBridge();
        self.currentTarget = "B"
        self.timeAsTarget = 0
        self.estimatedPosition = [0, 0, 0]
        
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_globl_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        
    def _rotationMatrixToEulerAngles(self,R):
    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    
        def isRotationMatrix(R):
            Rt = np.transpose(R)
            shouldBeIdentity = np.dot(Rt, R)
            I = np.identity(3, dtype=R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6        
        assert (isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])
        
    def listener_callback(self, data):
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
                
                if self.currentTarget == "":
                    self.currentTarget = qr_data
                elif self.currentTarget == qr_data:
                    
                    (x, y, w, h) = result.rect
                    
                    corners = result.polygon
                    
                    fx, fy = 800, 800  # Focal lengths
                    cx, cy = 320, 240  # Principal point
                    K = np.array([[fx, 0, cx],
                                [0, fy, cy],
                                [0, 0, 1]], dtype=np.float32)
                    
                    ret = cv2.solvePnP(np.array([[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0]], dtype=np.float32), np.array(corners, dtype=np.float32), K, None)

                    rvec, tvec = ret[1], ret[2]
                    
                    x1 = tvec[0]
                    y1 = tvec[1]
                    z1 = tvec[2]
                    
                    cv2.drawFrameAxes(image, K, None, rvec, tvec, 2)
                    
                    R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                    R_tc = R_ct.T
                    
                    R_flip      = np.zeros((3,3), dtype=np.float32)
                    R_flip[0,0] = 1.0
                    R_flip[1,1] =-1.0
                    R_flip[2,2] =-1.0
                    
                    roll_code, pitch_code, yaw_code = self._rotationMatrixToEulerAngles(R_flip*R_tc)
                    
                    pos_camera = -R_flip*R_tc*tvec
                    
                    
                    print("Marker X = %.1f  Y = %.1f  Z = %.1f"%(tvec[0], tvec[1], tvec[2]))
                    
                    
                    str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
                    cv2.putText(image, str_position, (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)        
                    
                    #-- Print the marker's attitude respect to camera frame
                    str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_code),math.degrees(pitch_code),
                                        math.degrees(yaw_code))
                    cv2.putText(image, str_attitude, (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

                    str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
                    cv2.putText(image, str_position, (0, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

                    #-- Get the attitude of the camera respect to the frame
                    roll_camera, pitch_camera, yaw_camera = self._rotationMatrixToEulerAngles(R_flip*R_tc)
                    str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                        math.degrees(yaw_camera))
                    cv2.putText(image, str_attitude, (0, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
                    
                    rectMiddleX = x + w/2
                    rectMiddleY = y + h/2
                    
                    cv2.line(image, (droneViewMiddleX, droneViewMiddleY), (int(rectMiddleX), int(rectMiddleY)), (255, 0, 0), 2)
                    
                    cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 4)
                    
                    text = "{}".format(qr_data)
                    
                    cv2.putText(image, text, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    #self.get_logger().info('QrCode data: {}'.format(qr_data))
                    
                    x_uav = -y1
                    y_uav = x1
                    z_uav = z1
                    
                    if -self.vehicle_local_position.z >= 5.0:
                        z_uav = -self.vehicle_local_position.z * 100
                    
                    angle_x = math.atan2(x_uav, z_uav)
                    angle_y = math.atan2(y_uav, z_uav)
                    
                    print(f"Altitude: {z_uav}")
                    print(f"QRCODE found x = {x_uav} cm y = {y_uav} cm -> angle x = {angle_x*(180.0/math.pi)} angle y = {angle_y*(180.0/math.pi)}")
                    
                    c = math.cos(self.vehicle_local_position.heading)
                    s = math.sin(self.vehicle_local_position.heading)
                    
                    north = c * x_uav - s * y_uav
                    east = s * x_uav + c * y_uav
                    
                    print(f"QRCODE N = {north} cm E = {east} cm Yaw = {self.vehicle_local_position.heading*(180.0/math.pi)}")
                    
                    earth_radius = 6378137.0
                    
                    dLat = north*0.01 / earth_radius
                    dLon = east*0.01 / (earth_radius * math.cos(math.pi * self.vehicle_local_position.ref_lat / 180.0))
                    
                    newLat = self.vehicle_local_position.ref_lat + dLat * (180.0 / math.pi)
                    newLon = self.vehicle_local_position.ref_lon + dLon * (180.0 / math.pi)
                    
                    print(f"QRCODE Lat = {newLat} Lon = {newLon}")
                    
                    self.timeAsTarget += 0
                elif self.timeAsTarget > 20:
                    self.timeAsTarget = 0
                    self.currentTarget = qr_data
                    
        cv2.circle(image, (droneViewMiddleX, droneViewMiddleY), 5, (0, 0, 255), -1)
        cv2.imshow("Drone Camera", image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()