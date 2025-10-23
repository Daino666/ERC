import rclpy
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math




global LA
global wheel_base
global points
global car_yaw 
global car_global_axis

car_yaw = None
car_global_axis = None

points  =  [[7, 11], [5,6], [4,3], [8, 12]]
wheel_base = .6
LA = 1


def choosing_point(points):
    if points is not None:
     
     return points[1] 


def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def order_points(points):
    ordered = [car_global_axis]
    remaining = points.copy()

    while remaining:
        last = ordered[-1]
        nearest = min(remaining, key=lambda p: distance(p, last))
        ordered.append(nearest)
        remaining.remove(nearest)

    ordered.append(car_global_axis)  # add start point at the end
    return ordered[1:]



def yaw_callback (odom) : 
    global car_yaw 
    global car_global_axis

    ## Quaternion from imu data 
    qx = odom.pose.pose.orientation.x
    qy = odom.pose.pose.orientation.y
    qz = odom.pose.pose.orientation.z
    qw = odom.pose.pose.orientation.w

    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y

    car_global_axis = [x, y]

    ## convert to euler to get yaw 
    r = R.from_quat ([qx,qy,qz,qw])
    __ , __ , car_yaw = r.as_euler('xyz')


def point_global_to_local(point_global_axis, car_yaw, car_axis ):


    dx = point_global_axis[0] -car_axis[0]
    dy = point_global_axis[1] - car_axis[1]

    point_local_x = dy * np.sin(car_yaw) + dx *np.cos(car_yaw)
    point_local_y = dy * np.cos(car_yaw) - dx *np.sin(car_yaw)


    

    return  point_local_x, point_local_y


    

def calc_curv( point_local_y , lock_ahead= LA):


    curvature = (2*(point_local_y))/(lock_ahead*lock_ahead)

    return curvature


def generate_command(curvature, linear_speed=1.0):

    angular_speed = curvature * linear_speed

    cmd = Twist()
    cmd.linear.x = float(linear_speed)
    cmd.angular.z = float(angular_speed)

    return cmd






def main(args = None):
    global car_yaw

    rclpy.init(args=args)
    node =  rclpy.create_node("PPC")
    
    node.create_subscription(Odometry, "/odom" , yaw_callback, 10 )
    vel_pub = node.create_publisher(Twist, "/cmd_vel" , 10)

    
    
    def timer_callback():
        if car_yaw is None or car_global_axis is None:
            node.get_logger().warn("Waiting for odometry...")
            return
        
        
        arranged_points = order_points(points)

        selected_point =  choosing_point(arranged_points)

        local_point_X , local_point_Y = point_global_to_local(selected_point, car_yaw, car_global_axis)

        curv = calc_curv(local_point_Y)

        cmd = generate_command(curvature= curv)

        vel_pub.publish(cmd)
        node.get_logger().info("vel is " + str(cmd.linear.x))
        node.get_logger().info("angular vel is "+ str(cmd.angular.z))
        
        




    timer = node.create_timer(0.1 , timer_callback)
    rclpy.spin(node)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()






if __name__ == '__main__':

    main()