import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

class Signal_subscriber(Node):
    def __init__(self):
        super().__init__('listener_node') #Creates a node called listener
        self.sub = self.create_subscription(Float32, 'sinosoidal_signal', self.listener_callback, 10) 
        self.sub
        self.publisher = self.create_publisher(Float32, 'amplified_signal', 10)
        #self.get_logger().info('Listening to the sinosoidal signal')
        
    def listener_callback(self, msg):
        new_msg = Float32()
        new_msg.data = msg.data * 2
        self.publisher.publish(new_msg)
        self.get_logger().info('Amplified signal: {}'.format(msg.data))

        
def main(args = None):
    rclpy.init(args=args)
    SinosoidalSubscriber = Signal_subscriber()
    rclpy.spin(SinosoidalSubscriber)
    SinosoidalSubscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
