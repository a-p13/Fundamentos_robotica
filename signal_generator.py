import rclpy
import scipy as sp
from scipy import signal
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float32MultiArray
import math

class My_Publisher(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('type', 1),
                ('amplitude', 0.5),
                ('frequency', 2.0),
                ('offset', 0.0),
            ]
        )
        self.signal_publisher = self.create_publisher(Float32, 'signal', 10)
        self.timer = self.create_timer(1e-3, self.timer_callback)
        
        self.params_publisher = self.create_publisher(Float32MultiArray, 'signal_params', 10)
        self.timer2 = self.create_timer(1e-2, self.timer_callback2)
        
        self.get_logger().info('Talker node successfully initialized')
        self.start_time = 1e-3
        
    
    def timer_callback(self):
        msg = Float32()
        t = self.start_time
        self.current_signal = self.get_parameter('type').get_parameter_value().integer_value  
        self.amplitude_value = self.get_parameter('amplitude').get_parameter_value().double_value
        self.frequency_value = self.get_parameter('frequency').get_parameter_value().double_value
        self.offset_value = self.get_parameter('offset').get_parameter_value().double_value
        
        if self.current_signal == 1:
            msg.data = self.amplitude_value*math.sin(2*math.pi*self.frequency_value*t) + self.offset_value #FUNCTION SINE
            self.get_logger().info('Sine')
        
        elif self.current_signal == 2:
            msg.data = self.amplitude_value * sp.signal.square(2 * math.pi * self.frequency_value * t) + self.offset_value  #FUNCION SQUARE
            self.get_logger().info('Square')
        
        elif self.current_signal == 3:
            msg.data = self.amplitude_value * sp.signal.sawtooth(2 * math.pi * self.frequency_value * t) + self.offset_value  #FUNCION SAWTOOTH
            self.get_logger().info('Sawtooth')
        
        self.signal_publisher.publish(msg)
        self.start_time += 0.01
        
    def timer_callback2(self):
        array = Float32MultiArray()   
         
        array.data.append(self.current_signal) 
        array.data.append(self.amplitude_value) 
        array.data.append(self.frequency_value) 
        array.data.append(self.offset_value) 
        array.data.append(self.start_time) 
        self.params_publisher.publish(array)
        self.get_logger().info('Parameters sent')
    
def main(args=None):
    rclpy.init(args=args)
    m_p = My_Publisher()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
