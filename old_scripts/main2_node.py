#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
class MainNode(Node):
    def __init__(self):
        super().__init__("main_node")
        self.arrays = [[0, 0, 0] for _ in range(64)]
        self.color_range_black = (0,0,0,50,50,50)
        self.linear_vel = [0,0,0]
        self.angular_vel = [0,0,0]
        self.declare_parameters(
            namespace='',
            parameters=[
                ('team','red'), # value can have 'red' and 'blue'
                ('start','start') # value can have 'start' and 'retry'
            ])
        self.sub_frame = self.create_subscription(
            String,
            'frame',
            self.frame_callback,
            10
        )
        self.publisher_vel = self.create_publisher(Twist, 'cmd_vel', 10)

    def frame_callback(self, msg):
        # Process the received message here
        elements = msg.data[:-1].split(',')
        rows = [row.strip('[').strip(']') for row in elements]
        result = [[int(num) for num in row.split()] for row in rows]
        self.arrays = result
        #self.arrays.reverse()
        #self.get_logger().info("Received message: %s" % str(self.arrays))

    def Publish_msg_Twist(self,x,y,z,an_x,an_y,an_z):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.x = an_x
        msg.angular.y = an_y
        msg.angular.z = an_z
        self.publisher_vel.publish(msg)
    def tracking(self,represent):
        point = round(len(represent)/2)
        index = (represent[point][1]-32)+(represent[point][0]-31)
        return index
    def getmax_interest(self,result,color_range,numbers_of_acception):
        count = 0
        index = 0
        activate = False
        out = []
        #print(result[0])
        for i in range(len(result)):
            r,g,b = result[i]
            if  color_range[0] <= r <= color_range[3] and color_range[1] <= r <= color_range[4]  and color_range[2] <= b <= color_range[5] :
                activate = True
            else:
                if activate and i-index+1>= numbers_of_acception:
                    out.append([index,i-1])
                activate = False
                index = i
        return out

    def filter_intervals(self,output):
        max_difference = max(interval[1] - interval[0] for interval in output)
        filtered_intervals = [interval for interval in output if interval[1] - interval[0] == max_difference]
        return filtered_intervals

def main(args=None):
    rclpy.init(args=args)
    main_node = MainNode()
    while rclpy.ok():
            #for use to load parameter
        team = main_node.get_parameter('team').get_parameter_value().string_value
        start = main_node.get_parameter('start').get_parameter_value().string_value
        main_node.get_logger().info("team : %s , start : %s" % (team,start))
        rclpy.spin_once(main_node, timeout_sec=0.1)  # Process ROS events

if __name__ == '__main__':
    main()
