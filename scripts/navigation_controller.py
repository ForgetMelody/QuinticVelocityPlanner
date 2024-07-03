#!/usr/bin/env python3

import rospy
import threading
import math
from geometry_msgs.msg import Twist,PoseStamped,Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import numpy as np

class PolynomialQuintic:
    def __init__(self, t0, t1, q0, q1, v0=0.0, v1=0.0, a0=0.0, a1=0.0):
        coeffs = self.__ComputeQuinticCoeffs(t0, t1, q0, q1, v0, v1, a0, a1)
        self.poly = np.poly1d(coeffs[::-1])

    @classmethod
    def __ComputeQuinticCoeffs(cls, t0, t1, q0, q1, v0, v1, a0, a1):
        T = t1 - t0
        T2 = T * T
        h = q1 - q0
        k0 = q0
        k1 = v0
        k2 = 0.5 * a0
        k3 = (20. * h - (8. * v1 + 12. * v0) * T -
              (3 * a0 - a1) * T2) / (2. * T * T2)
        k4 = (-30. * h + (14*v1 + 16*v0)*T+(3*a0 - 2*a1)*T2) / (2. * T2 * T2)
        k5 = (12 * h - 6*(v1 + v0) * T + (a1 - a0) * T2) / (2 * T2 * T2 * T)
        return (k0, k1, k2, k3, k4, k5)


class NavigationController:
    def __init__(self):
        # Initialize node
        rospy.init_node('navigation_controller_node')

        # Get parameters
        self.max_vel = rospy.get_param("max_vel")
        self.reverse_x = rospy.get_param("reverse_x")
        self.reverse_y = rospy.get_param("reverse_y")
        self.tolerance = rospy.get_param('tolerance')
        self.time_step = rospy.get_param('time_step')
        self.relative_target = rospy.get_param("relative_target")
        self.speed_offset_rate = rospy.get_param("speed_offset_rate")
        self.publish_rate = rospy.get_param("publish_rate")
        print("max_vel:", self.max_vel)
        print("reverse_x:", self.reverse_x)
        print("reverse_y:", self.reverse_y)
        print("tolerance:", self.tolerance)
        print("publish_rate:", self.publish_rate)
        print("time_step:", self.time_step)
        print("speed_offset_rate:", self.speed_offset_rate)
        print("relative_target:", self.relative_target)

        # State variables
        self.yaw = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.current_position = None
        self.start_point = None
        self.target_point = None
        self.total_distance = 0.0
        self.current_distance = 0.0
        self.navigation_thread = None
        self.stop_flag = False

        # Publishers and Subscribers
        cmd_topic = rospy.get_param("cmd_topic")
        odom_topic = rospy.get_param("odom_topic")
        target_topic = rospy.get_param("target_topic")
        self.cmd_vel_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        self.finish_pub = rospy.Publisher('/finish_nav',String , queue_size=1)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1)
        self.target_sub = rospy.Subscriber(target_topic, PoseStamped, self.target_callback,queue_size=1)
        self.rviz_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.target_callback,queue_size=1)

    def odom_callback(self, msg: Odometry):
        self.current_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)
        if self.target_point is not None:
            self.current_distance = math.sqrt((self.target_point.x - self.current_position.x) ** 2 + (self.target_point.y - self.current_position.y) ** 2)

    def target_callback(self, msg:PoseStamped):
        # update start point and target point
        self.start_point = Point(self.current_position.x, self.current_position.y, 0)
        self.target_point = msg.pose.position
        print("yaw:",self.yaw)
        if self.relative_target:
            self.target_point.x += self.start_point.x
            self.target_point.y += self.start_point.y
        # update distance
        self.total_distance = math.sqrt((self.target_point.x - self.start_point.x) ** 2 + (self.target_point.y - self.start_point.y) ** 2)
        self.current_distance = math.sqrt((self.target_point.x - self.current_position.x) ** 2 + (self.target_point.y - self.current_position.y) ** 2)
        #stop previous navigation
        while self.navigation_thread and self.navigation_thread.is_alive():
            if self.las_time is None or (rospy.Time.now().to_sec() - self.las_time) > 0.3:
                self.stop_flag = True
                rospy.sleep(0.1)
            else:
                return
        print("start new navigation: ({}, {}) -> ({}, {})".format(self.start_point.x, self.start_point.y, self.target_point.x, self.target_point.y))
        self.navigation_thread = threading.Thread(target=self.navigate_to_target)
        self.las_time = rospy.Time.now().to_sec()
        self.navigation_thread.start()


    def navigate_to_target(self):
        rate = rospy.Rate(self.publish_rate)  # 100 Hz
        # t_est = self.current_distance / 2.0 # 2 m /s
        qx0,qx1 = self.current_position.x,self.target_point.x
        qy0,qy1 = self.current_position.y,self.target_point.y
        self.total_distance = math.sqrt((self.target_point.x - self.start_point.x) ** 2 + (self.target_point.y - self.start_point.y) ** 2)
        if self.total_distance < 0.001:
            self.stop_robot()
            self.publish_finish_msg()
            return
        vx0,vx1 = self.vel_x,0
        vy0,vy1 = self.vel_y,0
        a0,a1 = 0,0
        t_est = (self.total_distance * 3) ** (1/3) #预估一个较快的大概的速
        start_time = rospy.Time.now().to_sec()
        while True:
            t0,t1 = 0,t_est
            xploy = PolynomialQuintic(t0,t1,qx0,qx1,vx0,vx1,a0,a1)
            yploy = PolynomialQuintic(t0,t1,qy0,qy1,vy0,vy1,a0,a1)
            ts = np.linspace(t0,t1,int(self.publish_rate*t1))
            vx,vy = xploy.poly.deriv(1)(ts),yploy.poly.deriv(1)(ts)
            # 判断所有速度都低于最大速度
            vaild_vel = True
            # if abs(np.array(vx)).max() > self.max_vel_x or abs(np.array(vy)).max() > self.max_vel_y:
            # 所有xy矢量和速度小于阈值
            if np.sqrt(abs(np.array(vx)).max()**2 + abs(np.array(vy)).max()**2) > self.max_vel:
                vaild_vel = False
            if vaild_vel:
                break
            else:
               t_est  += self.time_step
        print("plan_time_cost:",rospy.Time.now().to_sec()-start_time)
        print("path_time:",t_est)
        # print("qx0,qx1,qy0,qy1,vx0,vx1,vy0,vy1,a0,a1:",qx0,qx1,qy0,qy1,vx0,vx1,vy0,vy1,a0,a1)
        reversed_x_ = -1 if self.reverse_x else 1
        reversed_y_ = -1 if self.reverse_y else 1
        # 速度限制
        
        for i in range(len(ts)):
            if i > 30 and self.stop_flag:
                self.stop_flag = False
                break
            # Calculate rotation matrix based on current yaw
            cos_yaw = np.cos(self.yaw)
            sin_yaw = np.sin(self.yaw)
            rotation_matrix = np.array([[cos_yaw, sin_yaw],
                                        [-sin_yaw, cos_yaw]])
            
            # Rotate velocity vector
            local_vel = np.array([vx[i], vy[i]])
            global_vel = rotation_matrix @ local_vel

            cmd_vel = Twist()
            cmd_vel.linear.x = global_vel[0]*self.speed_offset_rate*reversed_x_
            cmd_vel.linear.y = global_vel[1]*self.speed_offset_rate*reversed_y_
            cmd_vel.linear.z = 0
            cmd_vel.angular.x = 0
            cmd_vel.angular.y = 0
            cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(cmd_vel)
            self.vel_x = vx[i]
            self.vel_y = vy[i]
            # rospy.loginfo("cmd_vel: ({}, {})".format(cmd_vel.linear.x, cmd_vel.linear.y))
            rate.sleep()
        self.stop_robot()
        print("navigation finished")
        self.publish_finish_msg()
        
    def publish_finish_msg(self):
        msg = String()
        msg.data = ''
        self.finish_pub.publish(msg)


    def stop_robot(self):
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        nav_controller = NavigationController()
        nav_controller.run()
    except rospy.ROSInterruptException:
        pass
