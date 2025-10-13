#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import minimalmodbus
import csv
import os
from datetime import datetime
import asyncio
from tapo import ApiClient
from threading import Thread
from std_msgs.msg import Bool



class LightNode(Node):

    def __init__(self):

        super().__init__('LightNode')

        self.declare_parameter("user", "regolife2025@gmail.com")
        self.declare_parameter("password", "RegoRego2025!")
        self.declare_parameter("ip", "192.168.0.165")

        self.user = self.get_parameter("user").get_parameter_value().string_value
        self.password = self.get_parameter("password").get_parameter_value().string_value
        self.ip = self.get_parameter("ip").get_parameter_value().string_value

        self.client = ApiClient(self.user, self.password)       

        Thread(target=lambda: asyncio.run(self.turn_off()), daemon=True).start()

        self.sub = self.create_subscription(Bool, "lights", self.callback, 10)

        self.get_logger().info("Initialized")



    def callback(self, msg):

        if msg.data:
            
            Thread(target=lambda: asyncio.run(self.turn_on()), daemon=True).start()
        
        else:

            Thread(target=lambda: asyncio.run(self.turn_off()), daemon=True).start()


    async def turn_on(self):

        device = await self.client.p105(self.ip)
        await device.on()
        self.get_logger().info("Turning on")

    async def turn_off(self):

        device = await self.client.p105(self.ip)
        await device.off()
        self.get_logger().info("Turning off")






   



def main(args=None):
    rclpy.init(args=args)
    node = LightNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()