#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import minimalmodbus
import csv
import os
from datetime import datetime




class RS485Scanner(Node):

    def __init__(self):

        super().__init__('RS485Scanner')

        self.declare_parameter("device", "")
        self.declare_parameter("baud_rate", 9600)
        self.declare_parameter("interval", 3600.0)

        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.interval = self.get_parameter("interval").get_parameter_value().double_value

        if self.device == "":

            self.get_logger().error("Device not set. Shutting down")
            rclpy.shutdown()

        self.declare_parameter("sensors_save_path", "~/.regolife")
        self.save_path = self.get_parameter("sensors_save_path").get_parameter_value().string_value

        save_dir = os.path.expanduser(self.save_path)
        os.makedirs(save_dir, exist_ok=True)
        

        self.timer = self.create_timer(self.interval, self.scan_addresses)

        self.get_logger().info("Scanner initialized")



    def scan_addresses(self):


        self.get_logger().info("Scanning RS485 addresses")
        results = []

        for addr in range(1, 32 + 1):

            instrument = minimalmodbus.Instrument(self.device, addr)
            instrument.serial.baudrate = self.baud_rate
            instrument.serial.timeout = 1.0
            instrument.mode = minimalmodbus.MODE_RTU
            try:
                value = instrument.read_register(0x00, 0, 3) 
                results.append({
                    "timestamp": datetime.now().isoformat(),
                    "id": addr,
                    "value": value/10
                })
            except Exception:
                self.get_logger().warn(f"Can't read sensor with id {addr}")

        if not results:
            self.get_logger().error("No sensor found")
        else:
            for r in results:
                self.get_logger().info(
                    f"[{r['timestamp']}] ID {r['id']} -> Humidity: {r['value']}"
                )

            save_path = os.path.join(os.path.expanduser(self.save_path), f"state.csv")
            self.get_logger().info(f'Saving detections in {os.path.expanduser(save_path)}')

            file_exists = os.path.isfile(save_path)
            with open(save_path, mode="a", newline="") as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=["timestamp", "id", "value"])
                if not file_exists:
                    writer.writeheader()
                writer.writerows(results)



def main(args=None):
    rclpy.init(args=args)
    node = RS485Scanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()