import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class TurtleBotController:
    def __init__(self):
        self.node = rclpy.create_node('turtlebot_controller')
        self.vel_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.keyword_subscription = self.node.create_subscription(String, 'recognized_text', self.keyword_callback, 10)
        self.cmd_vel = Twist()

    def keyword_callback(self, msg):
        recognized_keywords = msg.data.split()
        print("Command:", recognized_keywords)
        
        for keyword in recognized_keywords:
            if keyword == "front":
                self.move_forward()
            elif keyword == "back":
                self.move_backward()
            elif keyword == "left":
                self.turn_left()
            elif keyword == "right":
                self.turn_right()
            elif keyword == "circle":
                self.make_circle()
            elif keyword == "square":
                self.make_square()
            elif keyword == "stop":
                self.stop()


    def move_forward(self):
        self.cmd_vel.linear.x = 0.2
        self.cmd_vel.angular.z = 0.0
        self.publish_and_sleep(2.0)

    def move_backward(self):
        self.cmd_vel.linear.x = -0.2
        self.cmd_vel.angular.z = 0.0
        self.publish_and_sleep(2.0)
    def make_square(self):
        for x in range(4):
            self.move_forward()
            self.turn_left()         
    def turn_left(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 90 * (3.14159265359 / 180.0)/2.0
        self.publish_and_sleep(2.0)

    def turn_right(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = -90 * (3.14159265359 / 180.0)/2.0
        self.publish_and_sleep(2.0)

    def make_circle(self):
        self.cmd_vel.linear.x = 0.628318531
        self.cmd_vel.angular.z = 0.628318531
        self.publish_and_sleep(10.0)

    def stop(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.vel_publisher.publish(self.cmd_vel)

    def publish_and_sleep(self, duration):
        self.vel_publisher.publish(self.cmd_vel)
        time.sleep(duration)
        self.stop()


def main():
    rclpy.init()
    print("Listening for recognized keywords and controlling the TurtleBot...")

    controller = TurtleBotController()
    try:
        rclpy.spin(controller.node)
    except KeyboardInterrupt:
        pass
    finally:
        controller.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
