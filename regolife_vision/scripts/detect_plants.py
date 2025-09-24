#!/usr/bin/env python3
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
import rclpy

import cv2
import numpy as np
import json
import os
from datetime import datetime



class PlantDetector(Node):

    def __init__(self):

        super().__init__("PlantDetector")

        self.image_sub = self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.image_callback, 10)
        self.pub = self.create_publisher(CompressedImage, "/camera/vision_debug/compressed", 10)

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.declare_parameter("detections_save_path", "~/.regolife")
        self.save_path = self.get_parameter("detections_save_path").get_parameter_value().string_value

        save_dir = os.path.expanduser(self.save_path)
        os.makedirs(save_dir, exist_ok=True)

        self.declare_parameter("aruco0_position", [0.0, 0.0])
        self.declare_parameter("aruco1_position", [0.0, 0.0])
        self.declare_parameter("aruco2_position", [0.0, 0.0])
        self.declare_parameter("aruco3_position", [0.0, 0.0])

        self.pts_world = np.array([
            self.get_parameter("aruco0_position").get_parameter_value().double_array_value,
            self.get_parameter("aruco1_position").get_parameter_value().double_array_value,
            self.get_parameter("aruco2_position").get_parameter_value().double_array_value,
            self.get_parameter("aruco3_position").get_parameter_value().double_array_value,
        ], dtype=np.float32)

        self.declare_parameter("param1", 140)
        self.declare_parameter("param2", 30)
        self.declare_parameter("minDist", 90)
        self.declare_parameter("min_radius", 0)
        self.declare_parameter("max_radius", 100)

        self.declare_parameter("buffer_length", 10)

        self.buffer_length = self.get_parameter("buffer_length").get_parameter_value().integer_value

        self.images_buffer = []
        self.already_detected = False
        self.plants_raw_positions = {}



    def image_callback(self, msg):

        if len(self.images_buffer) >= self.buffer_length:
            self.perform_detection()
            return
        
        self.get_logger().debug("Received image")
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.images_buffer.append(frame)



    def compute_rectified(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is None or len(ids) < 4:
            
            self.get_logger().warn("Some expected markers not detected")
            return None

        centers = []

        for c in corners:
            pts = c[0]  
            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))
            centers.append([cx, cy])
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

        centers = np.array(centers, dtype=np.float32)
        id_to_center = {id_: centers[i] for i, id_ in enumerate(ids.flatten())}

        # Make sure all expected IDs exist
        expected_ids = [0, 1, 2, 3]  # adjust to your marker IDs
        if not all(id_ in id_to_center for id_ in expected_ids):
            self.get_logger().warn("Some expected markers not detected")
            self.first_image = True
            return None

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
        self.get_logger().debug("Computed rectified image")

        return rectified




    def perform_detection(self):

        if self.already_detected:
            return

        self.get_logger().info(f"Perfroming detection on {self.buffer_length} images")

        for f, frame in enumerate(self.images_buffer):

            rectified = self.compute_rectified(frame)

            if rectified is None:
                self.already_detected = False
                return
            

            gray_rect = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)

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
                self.already_detected = False
                return


            self.get_logger().info(f'Found {circles.shape[1]} circles')

            self.plants_raw_positions[f] = []

            if circles is not None:
                circles = np.uint16(np.around(circles))
                for (x, y, r) in circles[0, :]: 
                    # Draw circle
                    self.plants_raw_positions[f].append((x,y,r))

        average = self.average_circles(50)
        self.save_to_json(average)

        for (x, y, r) in average: 
            # Draw circle
            x = int(x)
            y = int(y)
            r = int(r)
            cv2.circle(rectified, (x, y), r, (0, 255, 0), 2)
            cv2.circle(rectified, (x, y), 2, (0, 0, 255), 3)
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
        # out_msg.header = msg.header
        out_msg.format = "jpeg"
        out_msg.data = np.array(buffer).tobytes()

        self.pub.publish(out_msg)
        self.already_detected = True

        self.destroy_node()
        rclpy.shutdown()



    def average_circles(self, max_distance=50):
        """
        plants_raw_positions: dict {frame_idx: [(x, y, r), ...]}
        max_distance: maximum distance to consider two circles the same
        """
        tracks = []  # list of lists, each inner list is detections of one plant

        for frame_idx in self.plants_raw_positions.keys():

            detections = self.plants_raw_positions[frame_idx]

            if not tracks:
                # first frame, initialize one track per detection
                for d in detections:
                    tracks.append([d])
                continue

            # otherwise, try to match to existing tracks
            used = set()
            for d in detections:
                x, y, r = map(float, d)
                best_track, best_dist = None, float('inf')

                for track in tracks:
                    last_x, last_y, last_r = map(float, track[-1])
                    dist = np.hypot(x - last_x, y - last_y)
                    if dist < best_dist and dist < max_distance:
                        best_dist = dist
                        best_track = track

                if best_track is not None:
                    best_track.append(d)
                    used.add(d)
            
            # new detections that didn’t match → create new tracks
            for d in detections:
                if d not in used:
                    tracks.append([d])

        # now compute averages
        averaged = []
        for track in tracks:
            arr = np.array(track)
            mean = arr.mean(axis=0)  # average x, y, r
            averaged.append(tuple(mean))

        return averaged
    

    def save_to_json(self, detections):

        
        
        detections = {
            "plants": [
                {"id": i, "x": float(x), "y": float(y), "d": float(2*r)}
                for i, (x, y, r) in enumerate(detections)
            ]
        }

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        save_path = os.path.join(os.path.expanduser(self.save_path), f"plants_{timestamp}.json")
        self.get_logger().info(f'Saving detections in {os.path.expanduser(save_path)}')
        with open(save_path, "w") as f:
            json.dump(detections, f, indent=2)






















def main(args=None):

    rclpy.init(args=args)

    node = PlantDetector()

    rclpy.spin(node)






if __name__ == "__main__":

    main()