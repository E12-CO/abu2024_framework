#!/usr/bin/env python3

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from abu_interfaces.srv import Position
import time
state = 1
class MainNode(Node):
    def __init__(self):
        super().__init__("main_node")
        self.cap = cv2.VideoCapture('/dev/trackline')
        self.arrays = [[0, 0, 0] for _ in range(64)]
        self.color_range_black = (0,0,0,50,50,50)
        self.color_range_white = (150,130,30,250,190,70)
        self.linear_vel = [0,0,0]
        self.angular_vel = [0,0,0]
        self.lower_white = np.uint8([0, 0, 0])
        self.upper_white = np.uint8([179,100, 255])
        # for use with real compettion = [179,100,255]
        self.declare_parameters(
            namespace='',
            parameters=[
                ('team','blue'), # value can have 'red' and 'blue'
                ('start','start') # value can have 'start' and 'retry'
            ])
        self.pos_cli = self.create_client(Position,'cmd_pos')
        while not self.pos_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Cmd_pos service not avaliable, waiting again...')
        self.pos_req = Position.Request()
        self.publisher_vel = self.create_publisher(Twist, 'cmd_vel', 10)

    def send_pos_request(self,speed_linear,distance,orientation):
        self.pos_req.x = speed_linear
        self.pos_req.y = distance
        self.pos_req.yaw = orientation
        future = self.pos_cli.call_async(self.pos_req)
        rclpy.spin_until_future_complete(self,future)
        return future

    def Publish_msg_Twist(self,x,y,z,an_x,an_y,an_z):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.x = an_x
        msg.angular.y = an_y
        msg.angular.z = an_z
        self.publisher_vel.publish(msg)
    def align_line(self,frame):
        if frame is None:
            return
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,100,150)
        lines = cv2.HoughLines(edges,1,np.pi/180,180)
        if lines is None :
            return
        lt = []
        for r_theta in lines:
            arr = np.array(r_theta[0],dtype=np.float64)
            r,theta = arr
            if cv2.norm((r,theta)) < 100:
                continue
            lt.append(theta*(180/math.pi) if theta*(180/math.pi) <= 90 else theta*(180/math.pi) - 180)
        lt = [e/90 for e in lt]
        #print(lt[0])
        if len(lt) == 0:
            return
        return lt[0]
    def classify_type_turn(self,frame):
        if frame is None:
            return
        frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(frame,self.lower_white,self.upper_white)
        left = mask[mask.shape[0]//4:mask.shape[0]*3//4,:mask.shape[1]//2]
        right = mask[mask.shape[0]//4:mask.shape[0]*3//4,mask.shape[1]//2:]
        turn_left = cv2.countNonZero(left)/(left.shape[0]*left.shape[1])
        turn_right = cv2.countNonZero(right)/(right.shape[0]*right.shape[1])
        #cv2.waitKey(1)
        return turn_left,turn_right
        #if self.detect_junction(frame) and
    def detect_junction(self,frame):
        if frame is None :
            return
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,50,255)
        lines = cv2.HoughLines(edges, 1, np.pi/180,100)
        if lines is None:
            return False
        lt = []
        for r_theta in lines:
            arr = np.array(r_theta[0], dtype=np.float64)
            r, theta = arr
            if theta*(180/math.pi) > 80 and theta*(180/math.pi) < 100 :
                lt.append(theta*(180/math.pi))
        out = self.classify_type_turn(frame)
        cv2.imshow("frame1",gray)
        cv2.imshow("frame2",edges)
        cv2.waitKey(1)
       # if  1 <= len(lt) <= 2 :
            #return True
        return False
    def ReadImage(self):
        if not self.cap.isOpened():
            return
        frame = self.cap.read()[1]
        #print(frame)
        return frame
    def TurnLeft(self,back):
        self.Publish_msg_Twist(0.0,0.0,0.0,0.0,0.0,0.0)
        future = self.send_pos_request(back,-back,0.0)
        time.sleep(2)
        future = self.send_pos_request(0.0,0.0,90.0)
        time.sleep(2)
    def TurnRight(self,back):
         self.Publish_msg_Twist(0.0,0.0,0.0,0.0,0.0,0.0)
         self.send_pos_request(back,-back,0.0)
         time.sleep(2)
         self.send_pos_request(0.0,0.0,-90.0)
         time.sleep(2)
    def Goahead(self,distance):
         self.send_pos_request(distance,distance,0.0)
         time.sleep(2)

    def tracking(self,frame):
        if frame is None :
            return
        frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(frame,self.lower_white,self.upper_white)
        contours, hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                #print(" " + str(cx) + "," + str(cy)+" "+str(state))
                cz_err = self.align_line(frame)
                #print(cz_err)
                if cz_err is None :
                    vel_com_ang_z = 0.0
                else:
                    vel_com_ang_z = cz_err * -3.0
                cx_err = cx - 320      # scaling from (0 to 639) to (-320 to 319)
                cx_err = cx_err / 320  # Linearization to -1 to 1
                angle = cx_err * -0.35
                vel_compo_x = 0.5 * np.cos(angle)
                vel_compo_y = 0.5 * np.sin(angle)
                #cv2.imshow("frame",mask)
                #cv2.imshow("frame2",frame)
                #cv2.waitKey(1)
                #print(cz_err,vel_com_ang_z)
#                print("Angle :" + str(angle) + "Vel X :" + str(vel_compo_x) + "Vel Y :" + str(vel_compo_y))
                self.Publish_msg_Twist(vel_compo_x, vel_compo_y, 0.0, 0.0, 0.0,vel_com_ang_z)
                return frame,mask

        return frame,mask
def main(args=None):

    global state,key,team,start
    index = 0
    future = None
    detectable = True
    detectbox = [0,0,1,0,0,0] # default blue start point = index-1 , retry point = index+1
    time_stamp = time.time()
    rclpy.init(args=args)
    main_node = MainNode()
    main_node.get_logger().info("start...")
    team = main_node.get_parameter('team').get_parameter_value().string_value
    start = main_node.get_parameter('start').get_parameter_value().string_value
    if start  == "retry":
        state = 2

    if team == "blue" :  # 0 represent left , 1 represent right
        detectbox = [0,0,1,0,0,0]
    else:
        detectbox = [1,1,0,1,1,1]

    try:
        while rclpy.ok():
            print(state,index,team,start,detectbox[index],detectable)
            frame = main_node.ReadImage()
            out = main_node.classify_type_turn(frame)
            param = [0,0] if out is None else out
            #main_node.get_logger().info("start : %s team : %s" % (start,team))
            if state == 1 or state == 2 or state == 4:
                if param[detectbox[index]] >= 0.75 and detectable == True:
                    state += 1
                    detectable = False
                main_node.tracking(frame)
            if state == 3:
                if team == "blue":
                    main_node.TurnLeft(0.2)
                    main_node.Goahead(0.2)
                else :
                    main_node.TurnRight(0.2)
                    main_node.Goahead(0.2)
                state = 4
                #pass # if red : turn right else : turn left
            elif state == 5 :
                if team == "blue":
                    main_node.TurnRight(0.4)
                    main_node.Goahead(0.2)
                else :
                    main_node.TurnRight(0.4)
                    main_node.Goahead(0.2)
                state += 1
                #pass # if red : turn left else : turn right
            elif state == 6 :
                break
            if detectable == False and time.time() - time_stamp > 1 :
                detectable = True
            #print(state,detectable,out)
            #rclpy.spin_once(main_node, timeout_sec=0.005)  # Process ROS events
    except KeyboardInterrupt:
        print("Terminating Node...")
        main_node.Publish_msg_Twist(0.0,0.0,0.0,0.0,0.0,0.0)
        time.sleep(3)
        main_node.destroy_node()
        rclpy.shutdown()
        main_node.cap.release()
        cv2.destroyAllWindows()
        exit(0)
    main_node.cap.release()
    cv2.destroyAllWindows()
    main_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
