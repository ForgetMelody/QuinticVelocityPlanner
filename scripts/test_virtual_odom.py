#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist, Point, Quaternion,PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import Header
import math
import time

class VirtualOdometry:
    def __init__(self):
        # Initialize node
        rospy.init_node('virtual_odometry')

        # Get parameters
        self.rate = 100
        self.frame_id = 'map'
        self.child_frame_id = 'base_link'

        # Initialize position and orientation
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Initialize velocities
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # Subscribers and Publishers
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.rviz_pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.rviz_pose_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)

        self.last_time = time.time()
        self.path = Path()

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z

    def rviz_pose_callback(self, msg:PoseWithCovarianceStamped):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.th = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
        #调整th方向为y轴为0，顺时针0-2pi
        print("th",self.th)
        # self.x = msg.pose.position.x
        # self.y = msg.pose.position.y
        # self.th = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]
        self.publish_msg()

    def update_odometry(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        self.publish_msg()

    def publish_msg(self):
         # Create quaternion from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.frame_id

        # Set the position
        odom.pose.pose.position = Point(self.x, self.y, 0.0)
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        # Set the velocity
        odom.child_frame_id = self.child_frame_id
        odom.twist.twist = Twist()
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        # Publish the Odometry message
        self.odom_pub.publish(odom)

        # Publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            odom_quat,
            rospy.Time.now(),
            self.child_frame_id,
            self.frame_id
        )

        # Publish the path
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = self.frame_id
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = self.frame_id
        pose_stamped.pose = odom.pose.pose
        self.path.poses.append(pose_stamped)
        self.path_pub.publish(self.path)

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update_odometry()
            rate.sleep()

if __name__ == '__main__':
    try:
        virtual_odom = VirtualOdometry()
        virtual_odom.run()
    except rospy.ROSInterruptException:
        pass