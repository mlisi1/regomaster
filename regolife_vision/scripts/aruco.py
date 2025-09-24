#!/usr/bin/env python3
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
import rclpy

import cv2
import numpy as np



class RegolifeVision(Node):

    def __init__(self):

        super().__init__("RegolifeVision")

        self.image_sub = self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.image_callback, 10)
        self.pub = self.create_publisher(CompressedImage, "/camera/image_rectified/compressed", 10)

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.pts_world = np.array([
            [0.0, 0.0],    # Marker 1
            [0.0, 955],    # Marker 2
            [981, 955],    # Marker 3
            [981, 0.0]     # Marker 4
        ], dtype=np.float32)

        self.first_image = True

        # self.declare_parameter("param1", 40)
        # self.declare_parameter("param2", 50)
        # self.declare_parameter("minDist", 90)
        # self.declare_parameter("min_radius", 0)
        # self.declare_parameter("max_radius", 300)


        self.declare_parameter("param1", 140)
        self.declare_parameter("param2", 30)
        self.declare_parameter("minDist", 90)
        self.declare_parameter("min_radius", 0)
        self.declare_parameter("max_radius", 100)




    def image_callback(self, msg):


        if not self.first_image:
            return

        self.get_logger().info("Received image")

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)


        if ids is not None and len(ids) >= 4:
            centers = []

            for c in corners:
                pts = c[0]  # shape (4,2), the 4 corners of this marker
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                centers.append([cx, cy])
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

            centers = np.array(centers, dtype=np.float32)
            # sorted_idx = np.lexsort((centers[:,0], centers[:,1]))
            # pts_img = centers[sorted_idx]


            id_to_center = {id_: centers[i] for i, id_ in enumerate(ids.flatten())}

            # Make sure all expected IDs exist
            expected_ids = [0, 1, 2, 3]  # adjust to your marker IDs
            if not all(id_ in id_to_center for id_ in expected_ids):
                self.get_logger().warn("Some expected markers not detected")
                self.first_image = True
                return

            # Order: bottom-left, bottom-right, top-right, top-left
            pts_img = np.array([
                id_to_center[0],
                id_to_center[1],
                id_to_center[2],
                id_to_center[3]
            ], dtype=np.float32)

            # Compute homography
            H, _ = cv2.findHomography(pts_img, self.pts_world)

            # Warp image into rectified ground plane
            rectified = cv2.warpPerspective(frame, H, (981, 955))

            self.get_logger().info("Computed rectified image")

            gray_rect = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray_rect, (5,5), 0)


            circles = cv2.HoughCircles(
                gray_rect,
                cv2.HOUGH_GRADIENT,
                dp=1,
                minDist=self.get_parameter("minDist").get_parameter_value().integer_value,      
                param1=self.get_parameter("param1").get_parameter_value().integer_value,         
                param2=self.get_parameter("param2").get_parameter_value().integer_value,       
                minRadius=self.get_parameter("min_radius").get_parameter_value().integer_value,       
                maxRadius=self.get_parameter("max_radius").get_parameter_value().integer_value
            )

            if type(circles) == type(None):

                self.get_logger().warn("No circles found, retrying")
                self.first_image = True
                return


            self.get_logger().info(f'Found {circles.shape[1]} circles')


            if circles is not None:
                circles = np.uint16(np.around(circles))
                for (x, y, r) in circles[0, :]: 
                    # Draw circle
                    cv2.circle(rectified, (x, y), r, (0, 255, 0), 2)
                    cv2.circle(rectified, (x, y), 2, (0, 0, 255), 3)
                    self.get_logger().info(f'Circle at position {x}, {y}')
                    label = f"x:{x}, y:{y}, r:{r}"
                    cv2.putText(
                        rectified,
                        label,
                        (x + r + 5, y),  # position (to the right of the circle)
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,              # font scale
                        (255, 0, 0),      # text color (blue)
                        1,                # thickness
                        cv2.LINE_AA
                    )





            _, buffer = cv2.imencode('.jpg', rectified)
            out_msg = CompressedImage()
            out_msg.header = msg.header
            out_msg.format = "jpeg"
            out_msg.data = np.array(buffer).tobytes()

            self.pub.publish(out_msg)

        else :

            self.get_logger().warn("Some expected markers not detected")
            self.first_image = True
            return






















def main(args=None):

    rclpy.init(args=args)

    node = RegolifeVision()

    rclpy.spin(node)






if __name__ == "__main__":

    main()