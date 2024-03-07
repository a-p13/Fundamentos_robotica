import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

import math, time

class My_Publisher(Node):
    def _init_(self):
        super()._init_('signal_generator')
        self.sine_publisher = self.create_publisher(Float32, 'signal', 10)
        self.time_publisher = self.create_publisher(Float32, 'time', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Talker node successfully initialized')
        self.start_time = 1
        
    def timer_callback(self):
        t = self.start_time
        msg = Float32()
        msg.data = math.sin(t)
        self.sine_publisher.publish(msg)
        self.get_logger().info('Sine')
        
        self.time_publisher.publish(msg)
        self.get_logger().info('Time')
        self.start_time += 0.01
        
    
def main(args=None):
    rclpy.init(args=args)
    m_p = My_Publisher()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
