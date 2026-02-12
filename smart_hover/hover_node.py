import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import os

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
            # تم تقليل timeout لزيادة استجابة التحكم
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.get_logger().info(f'✅ Connected to Hoverboard at {self.port}')
        except Exception as e:
            self.get_logger().error(f'❌ Serial Error: {e}')
            return

        # الاشتراك في أوامر الحركة
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        
        # نظام الأمان: التوقف إذا انقطع الاتصال
        self.last_received = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.check_timeout)

    def send_packet(self, steer, speed):
        """تغليف البيانات وإرسالها للهوفر بورد"""
        try:
            # حساب الـ Checksum (XOR)
            checksum = (self.START_FRAME ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)) & 0xFFFF
            # بناء الحزمة: H=uint16, h=int16
            payload = struct.pack('<HhhH', self.START_FRAME, steer, speed, checksum)
            self.ser.write(payload)
        except Exception as e:
            self.get_logger().error(f'Send Error: {e}')

    def listener_callback(self, msg):
        self.last_received = self.get_clock().now()
        
        # تحويل السرعات (يمكنك تعديل المعاملات 300 و 200 حسب رغبتك)
        speed = int(msg.linear.x * 300) 
        steer = int(msg.angular.z * 200)
        
        self.send_packet(steer, speed)

    def check_timeout(self):
        """إيقاف المحركات إذا لم نصل رسالة cmd_vel منذ أكثر من 0.5 ثانية"""
        now = self.get_clock().now()
        elapsed = (now - self.last_received).nanoseconds / 1e9
        
        if elapsed > 0.5:
            self.send_packet(0, 0)

def main(args=None):
    rclpy.init(args=args)
    node = HoverboardDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping...')
    finally:
        # إرسال أمر توقف نهائي عند الإغلاق
        node.send_packet(0, 0)
        node.destroy_node()
        rclpy.shutdown()
