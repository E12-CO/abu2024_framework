#!/usr/bin/env python3
import ast
import rclpy
import cv2
import numpy as np
from rclpy.node import Node

from std_msgs.msg import String


class ColortrackPublisher(Node):

    def __init__(self):
        super().__init__('colortrack_publisher')
        self.publisher_ = self.create_publisher(String, 'frame', 10)
        timer_period = 0.03
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.video = cv2.VideoCapture(0)
        self.declare_parameters(
        namespace='',
        parameters=[
            ('white','[229,225,230]'), #line
            ('orange','[255,170,77]'), # area1
            ('green','[80,158,47]'), #area2
            ('yellow','[254,209,65]'), #area3
            ('red','[244,54,76]'), #red team
            ('blue','[48,127,226]'), #blue team
            ('purple','[161,90,149]'), # fail rice
            ('numbers','64'), #number of segment
            ('start_point_roi','(0,200)'), # start point roi frame
            ('end_point_roi','(640,280)') # end point roi frame
        ])
    def segmentation(self,roi,numbers):
        list_of_segment = []
        for i in range(numbers):
            list_of_segment.append(roi[:, (640 // 64) * i:(640 // 64) * (i + 1)])
        return list_of_segment

    def get_most_highest_frequency(self,list_of_segment):
        highest_frequency = []
        for i in range(len(list_of_segment)):
            img_temp = list_of_segment[i]
            unique, counts = np.unique(img_temp.reshape(-1, 3), axis=0, return_counts=True)
            img_temp[:, :, 0], img_temp[:, :, 1], img_temp[:, :, 2] = unique[np.argmax(counts)]
            highest_frequency.append(img_temp)
        return highest_frequency

    def concat_image(self,frame, result):
        for i in range(64):
            frame[300:380, (640 // 64) * i:(640 // 64) * (i + 1)] = result[i]
        return frame

    def Update(self):
        msg = String()
        tmp = ''
        numbers = int(self.get_parameter('numbers').get_parameter_value().string_value)
        start_point_roi = ast.literal_eval(self.get_parameter('start_point_roi').get_parameter_value().string_value)
        end_point_roi = ast.literal_eval(self.get_parameter('end_point_roi').get_parameter_value().string_value)
        self.get_logger().info(f"{ast.literal_eval(self.get_parameter('white').get_parameter_value().string_value)}")
        if self.video.isOpened():
            ret,frame = self.video.read()
            roi = frame[start_point_roi[1]:end_point_roi[1], start_point_roi[0]:end_point_roi[0]]
            result = self.get_most_highest_frequency(self.segmentation(roi,numbers))
            for i in range(len(result)):
                tmp += str(result[i][0,0])
                if i != len(result)-1:
                    tmp+=','
            tmp += '$'
        msg.data = tmp
        return msg
    def timer_callback(self):
        msg = self.Update()
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    track_publisher = ColortrackPublisher()

    rclpy.spin(track_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

