import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from math import atan2, sqrt
from abu_interfaces.srv import MaskPoint, MaskPoint_Response

class DeadReckoning(Node):
    def __init__(self):
        super().__init__('dead_reckoning')
        self.subscription_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.subscription_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.masked_point = None

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def imu_callback(self, msg):
        quaternion = msg.orientation
        self.current_yaw = self.quaternion_to_yaw(quaternion)

    def quaternion_to_yaw(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return yaw

    def update(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.publisher_cmd_vel.publish(twist)

    def mask_point(self, request):
        self.masked_point = (request.x, request.y)
        self.reverse_to_masked_point()
        return MaskPoint_Response(success=True)

    def reverse_to_masked_point(self):
        if self.masked_point is None:
            self.get_logger().warn("No masked point set.")
            return False

        # Call reverse path task to navigate back to the masked point
        self.reverse_path_task([self.masked_point])

    def reverse_path_task(self, reverse_path):
        rate = self.create_rate(10)
        for point in reverse_path:
            target_x, target_y = point
            while abs(self.current_x - target_x) > 0.1 or abs(self.current_y - target_y) > 0.1:
                # Calculate linear and angular speeds to move towards the target point
                distance = sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
                angle_to_target = atan2(target_y - self.current_y, target_x - self.current_x)
                angular_error = angle_to_target - self.current_yaw
                self.linear_speed = 0.1 * distance
                self.angular_speed = 0.3 * angular_error
                self.update()
                rate.sleep()

def main():
    rclpy.init()
    dead_reckoning = DeadReckoning()
    rclpy.spin(dead_reckoning)
    dead_reckoning.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
