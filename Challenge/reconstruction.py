import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import scipy as sp
from scipy import signal
import math

class My_Subscriber(Node):
    def __init__(self):
        super().__init__('reconstruction')
        self.sub = self.create_subscription(Float32MultiArray, 'signal_params', self.listener_callback, 10)
        self.publisher = self.create_publisher(Float32, 'signal_reconstructed', 10)
        self.timer = self.create_timer(1e-2, self.timer_callback)
        self.get_logger().info('Listener mode initialized!!')
        
        self.current_signal  = 0
        self.amplitude = Float32()
        self.frequency = Float32()
        self.offset = Float32()
        self.time = Float32()
        
    def listener_callback(self, msg):
        self.current_signal = msg.data[0]
        self.amplitude = msg.data[1]
        self.frequency = msg.data[2]
        self.offset = msg.data[3]
        self.time = msg.data[4]
    
    def timer_callback(self):
        t = self.time
        new_msg = Float32()

        if self.current_signal == 1:
            new_msg.data = self.amplitude*math.sin(2*math.pi*self.frequency*t) + self.offset #FUNCTION SINE
            self.get_logger().info('Sine')
        
        elif self.current_signal == 2:
            new_msg.data = self.amplitude * sp.signal.square(2 * math.pi * self.frequency * t) + self.offset  #FUNCION SQUARE
            self.get_logger().info('Square')
        
        elif self.current_signal == 3:
            new_msg.data = self.amplitude * sp.signal.sawtooth(2 * math.pi * self.frequency * t) + self.offset  #FUNCION SAWTOOTH
            self.get_logger().info('Sawtooth')
                           
        self.publisher.publish(new_msg)
        self.get_logger().info('Reconstructed Signal')
   
        
def main(args=None):
    rclpy.init(args=args)
    m_s = My_Subscriber()
    rclpy.spin(m_s)
    m_s.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
        
