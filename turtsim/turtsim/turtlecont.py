import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

def keyword_callback(msg, vel_publisher):
    recognized_keywords = msg.data.split()  # Split the recognized keywords into a list
    print("Command:", recognized_keywords)

    # Create a Twist message to control the turtle's velocity
    cmd_vel = Twist()

    # Define velocity values based on recognized keywords
    for keyword in recognized_keywords:
        if keyword == "circle":
            cmd_vel.linear.x  = 6.283185307
            angular_velocity = (2 * 3.14159265359) 
            cmd_vel.angular.z = angular_velocity
        elif keyword == "front":
            cmd_vel.linear.x  = 2.434734307
            cmd_vel.angular.z = 0.0
        elif keyword == "back":
            cmd_vel.linear.x  = -2.434734307
            cmd_vel.angular.z = 0.0
        elif keyword == "left":
            cmd_vel.linear.x  = 0.0
            cmd_vel.angular.z=  90 * (3.14159265359 / 180.0)
        elif keyword == "right":
            cmd_vel.linear.x  = 0.0
            cmd_vel.angular.z = - 90 * (3.14159265359 / 180.0)
        elif keyword == "stop":
            cmd_vel.linear.x  = 0.0
            cmd_vel.angular.z = 0.0           

    vel_publisher.publish(cmd_vel)

    # Sleep for 1 second to ensure the robot stops
    time.sleep(2.0)
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.0
    vel_publisher.publish(cmd_vel)

def main():
    rclpy.init()
    node = rclpy.create_node('turtle_controller')

    vel_publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
    keyword_subscriber = node.create_subscription(String, 'recognized_text', lambda msg: keyword_callback(msg, vel_publisher), 10)

    print("Listening for recognized keywords and controlling the turtle...")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
