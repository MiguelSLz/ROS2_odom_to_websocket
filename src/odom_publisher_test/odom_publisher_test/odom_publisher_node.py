import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Header

class OdomPublisherNode(Node):
    def __init__(self):
        super().__init__('odom_publisher_node')
        self.publisher_ = self.create_publisher(Odometry, '/rov/odometry', 10)
        timer_period = 0.75  # segundos
        self.timer = self.create_timer(timer_period, self.publish_odom)
        self.get_logger().info('Publicador de Odometria iniciado.')

    def publish_odom(self):
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'rov/odometry'

        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 2.0
        msg.pose.pose.position.z = 3.0

        msg.twist.twist.linear.x = 0.5
        msg.twist.twist.angular.z = 0.1

        self.publisher_.publish(msg)
        self.get_logger().info('Mensagem de odometria publicada!')

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
