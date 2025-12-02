#!/usr/bin/env python3
import rclpy
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("pure_pursuit_avoidance")
    node.get_logger().info("Pure Pursuit Avoidance Started")
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
