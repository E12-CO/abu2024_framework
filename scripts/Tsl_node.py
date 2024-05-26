#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from abu_interfaces.msg import Tsl


class TslPublisher(Node):

    def __init__(self):
        super().__init__('Tsl_publisher')
        self.publisher_ = self.create_publisher(Tsl, 'Tsl', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ser = self.Create(115200,'/dev/arduino')
    def Create(self,baudrate,port):
        return serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
    def Readsensor(self,obj: serial.Serial):
        collect = []
        result = []

        regis = obj.read().hex()
        if regis != "aa":
            return None
        regis = obj.read().hex()
        if regis != "bb":
            return None
        for i in range(128):
            collect.append(obj.read())
        for j in range(0, 128, 2):
            result.append((int.from_bytes(collect[j+1], byteorder='big') << 8) + int.from_bytes(collect[j], byteorder='big'))
            #result.append((collect[j] << 8) + collect[j + 1])
        return result

    def timer_callback(self):
        msg = Tsl()
        data = self.Readsensor(self.ser)
        if data is None:
            return
        for i in range(64):
            msg.sequence[i] = data[i]
        self.publisher_.publish(msg)
        self.get_logger().info("complete publish")


def main(args=None):
    rclpy.init(args=args)
    Tsl_publisher = TslPublisher()
    rclpy.spin(Tsl_publisher)
    Tsl_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
