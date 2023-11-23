import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class TurtleController(Node):

    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.twist_msg = Twist()

    def move_forward(self):
        self.twist_msg.linear.x = 2.0
        self.twist_msg.angular.z = 0.0
        self.publisher_.publish(self.twist_msg)
        self.get_logger().info('Moving forward...')
        time.sleep(2)

    def turn_left(self):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = math.radians(120) / 1.0
        self.publisher_.publish(self.twist_msg)
        self.get_logger().info('Turning left...')
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)

    turtle_controller = TurtleController()

    for _ in range(3):
        turtle_controller.move_forward()
        turtle_controller.turn_left()

    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
