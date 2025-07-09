#!/usr/bin/env python3
import cv2
import rclpy
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
from cv_bridge import CvBridge

class PublishCamera(Node):
    def __init__(self):
        super().__init__("camera_node")

        # Camera device number found through ~ls /dev/ | grep video
        self.cameraDeviceNumber = 4
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)

        self.bridge = CvBridge()

        # Image Publisher
        self.publisher = self.create_publisher(
            CompressedImage,
            "camera",
            20
        )

        # To publish the image after 0.02 seconds for a smooth video
        self.timer = self.create_timer(
            0.02, 
            self.timer_cb
        )

    
    def timer_cb(self):
        # Getting the frame and resizing it
        success, frame = self.camera.read()
        frame = cv2.resize(frame, (820, 640), interpolation=cv2.INTER_CUBIC)

        # Publishing the ros image message
        if success:
            # Encode the frame to JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    cameraObject = PublishCamera()
    rclpy.spin(cameraObject)
    cameraObject.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()