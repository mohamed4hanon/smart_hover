import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import serial
import struct
import os
import math

# حل مشكلة الذاكرة المشتركة في ROS2 Jazzy
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

class HoverboardDriver(Node):
    def __init__(self):
        super().__init__('mohamed_hover_driver')
        
        # إعدادات الاتصال
        self.port = '/dev/ttyUSB0'
        self.baud = 115200
        self.START_FRAME = 0xABCD 
        
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.get_logger().info(f'✅ Connected to Hoverboard at {self.port}')
        except Exception as e:
            self.get_logger().error(f'❌ Serial Error: {e}')
            return

        # متغيرات Odometry
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = self.get_clock().now()

        # ناشر الـ Odometry ومبث الـ TF
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # الاشتراك في الأوامر والتايمر
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        self.last_received = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.update_odometry) # تحديث الموقع بتردد 20Hz

    def send_packet(self, steer, speed):
        try:
            checksum = (self.START_FRAME ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)) & 0xFFFF
            payload = struct.pack('<HhhH', self.START_FRAME, steer, speed, checksum)
            self.ser.write(payload)
        except Exception as e:
            self.get_logger().error(f'Send Error: {e}')

    def listener_callback(self, msg):
        self.last_received = self.get_clock().now()
        self.v = msg.linear.x
        self.w = msg.angular.z
        
        speed = int(self.v * 300) 
        steer = int(self.w * 200)
        self.send_packet(steer, speed)

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # حساب الموقع الجديد (Dead Reckoning)
        delta_x = (self.v * math.cos(self.th)) * dt
        delta_y = (self.v * math.sin(self.th)) * dt
        delta_th = self.w * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # بث التحويل المكاني (TF) من odom إلى base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = self.yaw_to_quaternion(self.th)
        self.tf_broadcaster.sendTransform(t)

        # نشر رسالة Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        # نظام الأمان
        if (current_time - self.last_received).nanoseconds / 1e9 > 0.5:
            self.v, self.w = 0.0, 0.0
            self.send_packet(0, 0)

    def yaw_to_quaternion(self, yaw):
        """تحويل زاوية الـ Yaw إلى Quaternion لـ ROS2"""
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = HoverboardDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_packet(0, 0)
        node.destroy_node()
        rclpy.shutdown()
