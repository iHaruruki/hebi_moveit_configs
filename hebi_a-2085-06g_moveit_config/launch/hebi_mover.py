import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import time

class SequenceController(Node):
    def __init__(self):
        super().__init__('sequence_controller')
        self.pub1 = self.create_publisher(Empty, '/exec_move1', 10)
        self.pub2 = self.create_publisher(Empty, '/exec_move2', 10)
        
        self.get_logger().info('Ready. Press "1" to start sequence.')
        self.timer = self.create_timer(0.1, self.check_input)
        
    def check_input(self)
        user_input = input("Start sequence? (1/q): ")
        if user_input == "1":
            self.execute_sequence()
            
    def execute_sequence(self):
        self.get_logger().info('Running Move 1')
        self.pub1.publish(Empty())
        
        time.sleep(10.0)
        
        self.get_logger().info('Running Move 2')
        self.pub2.publish(Empty())
        self.get_logger().info('Sequence Finished.')
        
def main():
    rclpy.init()
    node = SequenceController()
    rclcpy.spin(node)
        
