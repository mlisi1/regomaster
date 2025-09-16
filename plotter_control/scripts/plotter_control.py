#!/usr/bin/env python3
from rclpy.node import Node 
import rclpy

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32

import serial
import time
import re

error_messages = {
                20: "Invalid or unsupported command",
                22: "Power supply not supported"
                # Aggiungi altri codici di errore se necessario
            }

class PlotterControl(Node):

    def __init__(self):

        super().__init__("PlotterControl")

        self.declare_parameter("device", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)

        self.declare_parameter("x_lim", 0)
        self.declare_parameter("y_lim", 0)
        self.declare_parameter("speed", 1000)

        self.x_lim = self.get_parameter("x_lim").get_parameter_value().integer_value
        self.y_lim = self.get_parameter("y_lim").get_parameter_value().integer_value
        self.speed = self.get_parameter("speed").get_parameter_value().integer_value

        dev = self.get_parameter("device").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value

        self.ser = self.connect_to_plotter(dev, baud_rate)

        if self.ser == None:
            raise serial.SerialException()
        
        self.send_gcode("$X")
        self.send_gcode("$Y")
        self.send_gcode("G21")
        self.send_gcode("G90")

        self.get_logger().info("Plotter initialized")

        self.point_sub = self.create_subscription(PointStamped, "position", self.position_callback, 10)
        self.status_timer = self.create_timer(0.1, self.status_callback)
        self.z_publisher = self.create_publisher(Float32, "z_position", 10)

        self.actual_position = PointStamped()
        self.z_position = Float32()


            
    def connect_to_plotter(self, dev, baud_rate):


        for attempt in range(0,5):
            try:
                self.get_logger().info(f"Attempt {attempt+1}/5 to connect on {dev}...")
                ser = serial.Serial(dev, baud_rate, timeout=2)
                time.sleep(2)  # wait for controller reset

                if ser.is_open:
                    self.get_logger().info("Connected to plotter")
                    ser.reset_input_buffer()

                    startup = ser.readline().decode(errors="ignore").strip()
                    if startup:
                        self.get_logger().info(f"Controller startup: {startup}")

                    ser.write(b"\n")   # wake up
                    ser.write(b"$I\n")
                    reply = ser.readline().decode(errors="ignore").strip()

                    if reply and reply == 'ok':
                        self.get_logger().info(f"Connection successful. Controller response: {reply}")
                        return ser
                    else:
                        self.get_logger().warn(f"Unexpected response: {repr(reply)}")
                        ser.close()
            except serial.SerialException as e:
                self.get_logger().warn(f"Connection failed: {e}")

            time.sleep(2)

        self.get_logger().error("Could not connect after retries")

        return None
    

    def send_gcode(self, code):

        self.get_logger().debug(f"Sending command {code}")
        self.ser.write((code + '\n').encode())
        response = self.read_response()
        self.get_logger().debug(f"Response: {response}")
        return self.check_response(response)
    
    def check_response(self, response):
        if response.startswith('error:'):
            error_code = int(response.split(':')[1])            
            self.get_logger().warn(f"Error {error_code}: {error_messages.get(error_code, 'Unknown error')}")
            return False
        return True
    
    def read_response(self):
        responses = []
        response = self.ser.readline().decode().strip()
        responses.append(response)

        while self.ser.in_waiting:
            response = self.ser.readline().decode().strip()
            responses.append(response)
        return '\n'.join(responses)
    
    def get_position(self):
        self.ser.write("?".encode())
        response = self.read_response()
        match = re.search(r"MPos:([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+)", response)
        if match:
            state, x, y= match.groups()
            return {
                'state': state,
                'x': float(x),
                'y': float(y),
            }
        else:
            self.get_logger().warn("Unable to interpret position")
            return None
        

    def move(self, x, y, ignore_limits = False):

        if not ignore_limits:
            if not (x <= self.x_lim):
                self.get_logger().warn(f"Requested X movement ({x}) is outside plotter limits. Ignoring")
                return False 

            if not (y <= self.y_lim):
                self.get_logger().warn(f"Requested Y movement ({y}) is outside plotter limits. Ignoring")
                return False 
            
        return self.send_gcode(f"G1 X{x} Y{y}  F{self.speed}")
        

    def position_callback(self, msg):

        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        self.get_logger().info(f"Moving to {x}, {y}, {z}")


        self.move(x, y, True)
        self.read_response()

        self.z_position.data = z
        self.z_publisher.publish(self.z_position)

    def status_callback(self):

        state = self.get_position()

        self.get_logger().info(f"{state}")
        






def main(args = None):

    rclpy.init(args=args)

    node = PlotterControl()

    rclpy.spin(node)





if __name__ == "__main__":
    main()