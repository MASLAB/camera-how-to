import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge
import numpy as np

# Min and max HSV thresholds for green
GREEN_THRESHOLD = ([70,50,50], [90,255,255])

class CubeDetectNode(Node):
    def __init__(self):
        super().__init__("cube_detect")
        self.get_logger().info("Starting cube detector node")

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(CompressedImage, "image_raw/compressed", self.image_callback, 10)
        self.cube_image_pub = self.create_publisher(CompressedImage, "cube_image/compressed", 10)
        
    def image_callback(self, msg: CompressedImage):
        # this function is called whenever an image is received.
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # OpenCV needs bounds as numpy arrays
        lower_bound = np.array(GREEN_THRESHOLD[0])
        upper_bound = np.array(GREEN_THRESHOLD[1])

        # Threshold the HSV image to get only green color
        # Mask contains a white on black image, where white pixels
        # represent that a value was within our green threshold.
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Find contours (distinct edges between two colors) in mask using OpenCV builtin
        # This function returns 2 values, but we only care about the first

        # Note: In some OpenCV versions this function will return 3 values, in which
        # case the second is the contours value. If you have one of those versions of
        # OpenCV, you will get an error about "unpacking values" from this line, which you
        # can fix by adding a throwaway variable before contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # If we have contours...
        if len(contours) != 0:
            # Find the biggest countour by area
            c = max(contours, key = cv2.contourArea)

            # Get a bounding rectangle around that contour
            x,y,w,h = cv2.boundingRect(c)

            # Draw the rectangle on our frame
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
        
        cube_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.cube_image_pub.publish(cube_image_msg)



def main(args=None):
    rclpy.init()
    node = CubeDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()