# https://dhanuzch.medium.com/using-opencv-with-gazebo-in-robot-operating-system-ros-part-2-read-and-decode-a-qr-code-ed6ce5c298ca

# pip install pyzbar
# pip install setuptools==58.2.0

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import VehicleLocalPosition
from pyzbar.pyzbar import decode
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image


class ImageSubscriber(Node):
    def __init__(self):
        self.br = CvBridge()

        qos_profile = QoSProfile(
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        super().__init__("vehicle_local_position_subscriber")
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        ))

        super().__init__("camera_info_subscriber")
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo, "camera_info", self.camera_info_callback, qos_profile
        )

        super().__init__("camera_subscriber")
        self.camera_subscriber = self.create_subscription(
            Image, "camera", self.image_callback, 10
        )

        self.camera_matrix = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )

        self.distortion_coefficients = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        self.currentTarget = ""
        self.timeAsTarget = 0


    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.get_logger().info(f"Received position: {vehicle_local_position}")

    def camera_info_callback(self, data):
        self.camera_matrix = np.array(data.k).reshape(3, 3)
        self.distortion_coefficients = np.array(data.d)

        self.get_logger().info(f"Camera Matrix {self.camera_matrix}")
        self.get_logger().info(f"Distortion Coefficients {self.distortion_coefficients}")

        self.get_logger().info("Camera Info Received")
        self.destroy_subscription(self.camera_info_subscriber)

    def image_callback(self, data):
        cv_image = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")


        (rows, cols, channels) = cv_image.shape

        image = cv_image

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        thres = 40
        img_bw = cv2.threshold(gray, thres, 255, cv2.THRESH_BINARY)[1]

        qr_result = decode(img_bw)

        droneViewMiddleX = int(cols / 2)
        droneViewMiddleY = int(rows / 2)

        if len(qr_result) != 0:

            for result in qr_result:
                qr_data = result.data.decode("utf-8")

                if self.currentTarget == "":
                    self.currentTarget = qr_data
                elif self.currentTarget == qr_data:

                    (x, y, w, h) = result.rect

                    corners = result.polygon

                    undistortedCorners = cv2.undistortPoints(
                        np.array(corners, dtype="double"),
                        self.camera_matrix,
                        self.distortion_coefficients,
                    )

                    for c in corners:
                        cv2.circle(image, c, 3, (0, 0, 255), -1)

                        makerSize = 1
                        halfSize = makerSize / 2

                        objectPoints = np.array(
                            [
                                [-halfSize, halfSize, 0],
                                [halfSize, halfSize, 0],
                                [halfSize, -halfSize, 0],
                                [-halfSize, -halfSize, 0],
                            ]
                        )

                        _, rvec, tvec = cv2.solvePnP(
                            objectPoints,
                            undistortedCorners,
                            self.camera_matrix,
                            self.distortion_coefficients,
                        )

                        # Annotate the image
                        cv2.drawFrameAxes(
                            image,
                            self.camera_matrix,
                            self.distortion_coefficients,
                            rvec,
                            tvec,
                            1,
                        )

                        # Calculate the rotation matrix
                        R, _ = cv2.Rodrigues(rvec)
                        quaternion = cv2.RQDecomp3x3(R)


                        # print the target pose
                        self.get_logger().info(f"Target Position: {str(tvec)}\n")
                        self.get_logger().info(f"Target Rotation: {str(quaternion[0])}\n")

                        # Annotate the image with the target position and marker id
                        cv2.putText(
                            image,
                            "Target Position: " + str(tvec),
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (0, 255, 0),
                            2,
                            cv2.LINE_AA,
                        )

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


if __name__ == "__main__":
    main()
