#!/usr/bin/env python3
from abu_interfaces.srv import Mega
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
class DetectService(Node):

    def __init__(self):
        super().__init__('detect_service')
        self.srv = self.create_service(Mega, 'see_ball', self.callback)
        self.i = 0

    @staticmethod
    def calculate_distance(focal_length, actual_diameter, perceived_diameter):
        return (focal_length * actual_diameter) / perceived_diameter

    def capture_pic(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 360)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        if not cap.isOpened():
            self.get_logger().error("Failed to open camera")
            return None
        status, img = cap.read()
        cap.release()
        if not status:
            self.get_logger().error("Failed to capture image from camera")
            return None
        cv2.imwrite(f'img{self.i}.jpg', img)
        self.i += 1
        return img

    def detect_objects(self, frame):
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        actual_diameter = 0.083
        focal_length = (83 * 800) / 102
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:
                x, y, w, h = cv2.boundingRect(contour)
                perceived_diameter = max(w, h)
                distance = self.calculate_distance(focal_length, actual_diameter, perceived_diameter)
                detected_objects.append((x + (w / 2), y + (h / 2), distance))
        return detected_objects

    def result(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 360)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        if not cap.isOpened():
            self.get_logger().error("Failed to open camera")
            return "not detected"
        while(1):
            _,frame = cap.read()
        #if frame is None:
            #return "not detected"
            objects = self.detect_objects(frame)
            if len(objects) > 0:
                cap.release()
                return ' '.join([f"{x} {y} {distance}" for x, y, distance in objects])
            #return "not detected"
        cap.release()
        return "not detected"
    def callback(self, request, response):
        self.get_logger().info('Incoming request %s\n' % (request.req))
        response.res = self.result()
        self.get_logger().info('Processed result %s\n' % (response.res))
        return response

def main():
    rclpy.init()
    detect_service = DetectService()
    rclpy.spin(detect_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
