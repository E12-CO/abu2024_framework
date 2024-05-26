#!/usr/bin/env python3

import cv2
import numpy as np
import serial
import time

# ROS2 stuffs
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg


left_top = (305,45)
left_buttom = (305,430)
right_buttom = (530,430)
right_top = (530,45)


class BallFeed(Node):

	def __init__(self):
		super().__init__('BallFeedNode')
		self.start_flag = 0
		self.sub_cmd = self.create_subscription(
			StringMsg,
			'ball_feed_cmd',
			self.cmd_callback,
			10)
			
		self.pub_status = self.create_publisher(
			StringMsg,
			'ball_feed_ar',
			10)

	def cmd_callback(self, msg):
		cmd_str = msg.data
		
		match cmd_str:
			case 'start':# Start roller motor
				print('receive start command')
				self.start_flag = 1 	
			
			case 'stop':# Stop all operation
				print('receive stop command')
				self.start_flag = 2
			
			case 'out':# release ball
				print('receive out command')
				ser.write(b'O\n')
				
			case _:
				self.start_flag = 0
		
	def publish_ball_ar(self, ar):
		msg = StringMsg()
		msg.data = ar
		self.pub_status.publish(msg)
                       
def detect_and_draw_balls(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # HSV color tuning param 
    lower_purple = np.array([105, 41, 76])
    upper_purple = np.array([146, 93, 255])

    mask_purple = cv2.inRange(hsv, lower_purple, upper_purple)

    contours, _ = cv2.findContours(mask_purple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    flag = False
    for contour in contours:
        area = cv2.contourArea(contour)
        min_area_threshold = 1000 
        if area > min_area_threshold:
            x, y, w, h = cv2.boundingRect(contour)
            color_name = "Purple"
            flag = True
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    return frame,flag



def main(args=None):
	ser = serial.Serial(port='/dev/mega',
	baudrate=115200,
	timeout=0.03,
	write_timeout=0.03)
	state = 0
	ser.write(b'E\n')
	cap = cv2.VideoCapture('/dev/ballcheck')
	rclpy.init(args=args)
	ball_feed_node = BallFeed()
	while rclpy.ok():
		ret, frame = cap.read()

		if frame is not None :
			frame_with_rectangles,flag = detect_and_draw_balls(frame[45:430,305:530])
			print(state,flag)

			if ball_feed_node.start_flag == 2: # Enter stop mode
				state = 20

			if state == 0: # Waiting state
				if ball_feed_node.start_flag == 1:
					ball_feed_node.start_flag = 0
					state = 1

			elif state == 1: # Start ball
				ser.write(b'S\n')
				state = 2

			elif state == 2: # Polling wait for ball   
				ser.write(b'H\n')
				message = ser.readline()
				#print(message)
				if message == b'T':
					state = 3

			elif state == 3: # Accept or Reject ball
				if not flag :# Accept the ball
					ser.write(b'A\n')
					ball_feed_node.publish_ball_ar('Accept')
				else :# Reject the ball
					ser.write(b'R\n')
					ball_feed_node.publish_ball_ar('Reject')
				state = 4

			elif state == 4: # Wait for ball out
				ser.write(b'I\n')
				message = ser.readline()
				if message == b'T':
					ball_feed_node.publish_ball_ar('OUT')
					state = 0

			elif state == 20: # Stop state
				ball_feed_node.start_flag = 0
				ser.write(b'E\n')
				state = 0 # Went back to idle

			cv2.imshow('Check Ball In', frame_with_rectangles)
		
		else: # Null frame
			print('Null frame error, quiting...')
			break

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

		rclpy.spin_once(ball_feed_node, timeout_sec=0.1)  # Process ROS events

	cap.release()
	cv2.destroyAllWindows()	
	
if __name__ == '__main__':
	main()

