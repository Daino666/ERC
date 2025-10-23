import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def joy_callback(msg, publisher):
    # Assuming:
    #   axes[1] → forward/back (x-axis)
    #   axes[0] → left/right (y-axis)
    linear_x = msg.axes[1]    # forward (+1), backward (-1)
    angular_z = msg.axes[0]   # left (+1), right (-1)
    
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('joystick_teleop')

    publisher = node.create_publisher(Twist, 'cmd_vel', 10)
    node.create_subscription(Joy, 'joy', lambda msg: joy_callback(msg, publisher), 10)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
