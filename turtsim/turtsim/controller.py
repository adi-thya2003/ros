import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class ControllerNode:
    def __init__(self):
        self.node = rclpy.create_node('controller')
        self.vel_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)  # Adjust the topic as needed
        self.keyword_subscription = self.node.create_subscription(String, 'recognized_text', self.keyword_callback, 10)
        self.cmd_vel = Twist()

    def keyword_callback(self, msg):
        recognized_keywords = msg.data.split()
        print("Command:", recognized_keywords)

        # Define velocity values based on recognized keywords
        for keyword in recognized_keywords:
            if keyword == "circle":
                self.cmd_vel.linear.x = 0.0
                angular_velocity = (2 * 3.14159265359) 
                self.cmd_vel.angular.z = angular_velocity
            elif keyword == "front":
                self.cmd_vel.linear.x = 2.434734307
                self.cmd_vel.angular.z = 0.0
            elif keyword == "back":
                self.cmd_vel.linear.x = -2.434734307
                self.cmd_vel.angular.z = 0.0
            elif keyword == "left":
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z =  90 * (3.14159265359 / 180.0)
            elif keyword == "right":
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = - 90 * (3.14159265359 / 180.0)
            elif keyword == "stop":
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0           

        self.vel_publisher.publish(self.cmd_vel)

        # Sleep for 1 second to ensure the robot stops
        time.sleep(2.0)
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.vel_publisher.publish(self.cmd_vel)

def main():
    rclpy.init()
    print("Listening for recognized keywords and controlling the robot...")

    controller_node = ControllerNode()
    try:
        rclpy.spin(controller_node.node)
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
