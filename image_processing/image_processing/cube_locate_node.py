import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge
import numpy as np

# Example data from /image_raw/compressed_mouse_left
PTS_IMAGE_PLANE = [[218, 338],
                   [489, 136],
                   [113, 176],
                   [513, 227],
                   [199, 151]]

# Example data from corresponding point in real world
PTS_GROUND_PLANE = [[0.20,  0.00],
                    [0.38, -0.08],
                    [0.32,  0.04],
                    [0.26, -0.06],
                    [0.36,  0.02]]

# Min and max HSV thresholds for green
GREEN_THRESHOLD = ([70,50,50], [90,255,255])

class CubeLocateNode(Node):
    def __init__(self):
        super().__init__("cube_locate")
        self.get_logger().info("Starting cube location node")

        self.bridge = CvBridge()

        #Initialize data into a homography matrix
        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

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

            # Get center at the bottom of the contour as where the cube touches
            center_x = int(x+w/2)
            center_y = int(y+h)

            world_x, world_y = self.transformUvToXy(center_x, center_y)

            # Draw circle to indicate
            cv2.circle(frame, (center_x, center_y), 3, (255,0,0), 3)
            # Draw text to describe location
            cv2.putText(frame, "%.2fm, %.2fm" % (world_x, world_y), (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2)
        
        cube_image_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.cube_image_pub.publish(cube_image_msg)

    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.

        Units are in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y


def main(args=None):
    rclpy.init()
    node = CubeLocateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()