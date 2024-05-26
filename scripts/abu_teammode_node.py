#!/usr/bin/env python3

from gpiozero import Button
from time import sleep

# ROS2 stuffs
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg
from geometry_msgs.msg import Twist

btn_red_team = 12
btn_blu_team = 16
btn_start    = 20
btn_retry    = 21

state = 0
start = ""
team = ""

class abu_teammode(Node):

	def __init__(self):
		super().__init__('AbuTeamModeNode')
		self.pub_teammode = self.create_publisher(
			StringMsg,
			'teammode',
			10)

	def publish_teammode(self, team_str, mode_str):
		msg = StringMsg()
		msg.data = team_str + ',' + mode_str
		self.pub_teammode.publish(msg)
#		print('Team : '+team_str+' Mode : '+mode_str)


def button_callback(button_name):
	global state, team, mode

	# State 0, Select team
	if state == 0:
		if button_name == 1:
			print("Select: red team")
			team = "red"
			state = 1
			sleep(1)
		elif button_name == 2:
			print("Select: blue team")
			team = "blue"
			state = 1
			sleep(1)


	# Stat 1, Select play mode
	elif state == 1:
		if button_name == 3:
			print("Select: start")
			mode = "start"
			state = 2
			sleep(1)
		elif button_name == 4:
			print("Select: retry")
			mode = "retry"
			state = 2
			sleep(1)


	# State 2, For stopping the robot and Jump to state 3
	elif state == 2 :
		state = 3

def main(args=None):
	global state, team, mode

	running = False

	red_team_button = Button(btn_red_team, hold_time=4,pull_up=True)
	blue_team_button = Button(btn_blu_team, hold_time=4,pull_up=True)
	start_button = Button(btn_start, hold_time=4,pull_up=True)
	retry_button = Button(btn_retry, hold_time=4,pull_up=True)

	red_team_button.when_pressed = lambda: button_callback(1)
	blue_team_button.when_pressed = lambda: button_callback(2)
	start_button.when_pressed = lambda: button_callback(3)
	retry_button.when_pressed = lambda: button_callback(4)

	rclpy.init(args=args)
	abu_teammode_node = abu_teammode()
	abu_teammode_node.get_logger().info('Starting Button team mode selector node')


	team = 'red'
	mode = 'retry'
	state = 2

	while rclpy.ok():
		#print(state)
		if state == 2 and not running :  # Send out start/retry command
			abu_teammode_node.publish_teammode(team, mode)
			abu_teammode_node.get_logger().info('Got Team: '+team+' Mode: '+mode)
			running = True

		elif state == 3 and running: # Send out stop mode
			abu_teammode_node.publish_teammode('stop', 'stop')
			abu_teammode_node.get_logger().info('STOP robot !')
			running = False

		rclpy.spin_once(abu_teammode_node, timeout_sec=0.1)  # Process ROS events

	

if __name__ == "__main__":
	main()
