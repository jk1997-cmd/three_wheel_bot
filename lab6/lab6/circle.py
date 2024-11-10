import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CircleMover(Node):
    def _init_(self):
        super()._init_('circle_mover')
        # Create a publisher for the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Set the movement duration
        self.duration = 10  # Duration in seconds for moving in a circle
        
        # Create a Twist message for circular movement
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.5  # Adjust for speed
        self.vel_msg.angular.z = 0.5  # Adjust for circle radius

        # Start moving in a circle
        self.move_in_circle()

    def move_in_circle(self):
        # Record the start time
        start_time = time.time()

        # Move the robot in a circle for the specified duration
        while time.time() - start_time < self.duration:
            # Publish the velocity command
            self.publisher_.publish(self.vel_msg)
            # Sleep to maintain a 10 Hz rate
            rclpy.spin_once(self, timeout_sec=0.1)

        # Stop the robot after the movement
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.publisher_.publish(self.vel_msg)
        self.get_logger().info("Finished moving in a circle.")

def main(args=None):
    rclpy.init(args=args)
    node = CircleMover()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()