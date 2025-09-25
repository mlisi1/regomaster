#!/usr/bin/env python3
from rclpy.node import Node 
import rclpy

from geometry_msgs.msg import PointStamped

import json
import os
import time


class CheckPlants(Node):

    def __init__(self):

        super().__init__("CheckPlants")  

        self.declare_parameter("x_offset", 60.0)
        self.declare_parameter("y_offset", 67.0)

        self.x_offset = self.get_parameter("x_offset").get_parameter_value().double_value
        self.y_offset = self.get_parameter("y_offset").get_parameter_value().double_value

        self.declare_parameter("json_path", "~/.regolife")

        self.plants_path = self.get_parameter("json_path").get_parameter_value().string_value

        self.point_pub = self.create_publisher(PointStamped, "position", 10)

        plants = self.load_plants()

        self.check_plants(plants)

        self.destroy_node()
        rclpy.shutdown()






    def load_plants(self):

        folder = os.path.expanduser(self.plants_path)
    
        if not os.path.exists(folder) or not os.path.isdir(folder):
            self.get_logger().error(f"Folder '{folder}' does not exist or is not a directory.")
            self.destroy_node()
            rclpy.shutdown()

        # List all JSON files
        json_files = [f for f in os.listdir(folder) if f.endswith('.json') and os.path.isfile(os.path.join(folder, f))]

        
        if not json_files:
            self.get_logger().error(f"No JSON files found in folder '{folder}'.")
            self.destroy_node()
            rclpy.shutdown()

        # Sort files by name (assumes timestamped filenames like plants_YYYYMMDD_HHMMSS.json)
        json_files.sort(reverse = True)
        
        # Most recent = last one after sorting
        most_recent_file = json_files[-1]
        
        # Optionally, load its content
        with open(os.path.join(folder, most_recent_file), 'r') as f:
            data = json.load(f)
        
        return data["plants"]
    

    def check_plants(self, plants):

        for p in plants:

            self.get_logger().info(f'Checking plant with id {p["id"]} at {p["x"]}, {p["y"]}')
            self.get_logger().info(f'Checking plant with id {p["id"]} at {p["x"]- self.x_offset}, {p["y"]- self.y_offset}')

            msg = PointStamped()

            msg.point.x = p["x"] - self.x_offset
            msg.point.y = p["y"] - self.y_offset

            self.point_pub.publish(msg)

            time.sleep(30)





    

        






def main(args = None):

    rclpy.init(args=args)

    node = CheckPlants()

    rclpy.spin(node)





if __name__ == "__main__":
    main()