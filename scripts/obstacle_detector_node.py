#!/usr/bin/env python3
import rclpy
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("obstacle_detector")
    node.get_logger().info("Obstacle Detector Started")
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
