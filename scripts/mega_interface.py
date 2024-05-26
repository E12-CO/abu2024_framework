#!/usr/bin/env python3
from abu_interfaces.srv import Mega
import serial
import rclpy
from rclpy.node import Node


class MegaService(Node):

    def __init__(self):
        super().__init__('mega_service')
        self.ser = serial.Serial('/dev/mega',baudrate = 115200,timeout=1)
        self.srv = self.create_service(Mega, 'ball_command', self.callback)

    def callback(self, request, response):
        message = request.req + "\n"
        incoming = ''
        self.ser.write(message.encode())
        self.ser.write("holding\n".encode())
        while(incoming == "Fault\n" or incoming == '' or incoming == "Fault"):
            self.ser.write("holding\n".encode())
            incoming = self.ser.readline().decode().strip()
            self.get_logger().info('%s' % str(incoming))
        print(incoming)
        #r_decode = incoming.decode('utf-8')
        #print(r_decode)
        response.res = incoming
        self.get_logger().info('Incoming request %s\n' % (response.res))
        return response


def main():
    rclpy.init()
    mega_service = MegaService()

    rclpy.spin(mega_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
