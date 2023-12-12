import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointCloudModifierNode(Node):
    def __init__(self):
        super().__init__('pointcloud_transformer')

        self.left_sub = self.create_subscription(PointCloud2, '/two/cloud', self.left_callback, 10)
        self.left_pub = self.create_publisher(PointCloud2, '/left/cloud', 10)
        self.right_sub = self.create_subscription(PointCloud2, '/three/cloud', self.right_callback, 10)
        self.right_pub = self.create_publisher(PointCloud2, '/right/cloud', 10)
        
        # self.two_1 = self.create_subscription(LaserScan, '/two/cloud_1', self.left_callback, 10)
        # self.two_1_pub = self.create_publisher(LaserScan, '/left/scan_1', 10)
        # self.two_2 = self.create_subscription(LaserScan, '/two/cloud_2', self.left_callback, 10)
        # self.two_2_pub = self.create_publisher(LaserScan, '/left/scan_2', 10)
        # self.two_3 = self.create_subscription(LaserScan, '/two/cloud_3', self.left_callback, 10)
        # self.two_3_pub = self.create_publisher(LaserScan, '/left/scan_3', 10)
        # self.two_4 = self.create_subscription(LaserScan, '/two/cloud_4', self.left_callback, 10)
        # self.two_4_pub = self.create_publisher(LaserScan, '/left/scan_4', 10)

        # self.three_1 = self.create_subscription(LaserScan, '/three/cloud_1', self.right_callback, 10)
        # self.three_1_pub = self.create_publisher(LaserScan, '/right/scan_1', 10)
        # self.three_2 = self.create_subscription(LaserScan, '/three/cloud_2', self.right_callback, 10)
        # self.three_2_pub = self.create_publisher(LaserScan, '/right/scan_2', 10)
        # self.three_3 = self.create_subscription(LaserScan, '/three/cloud_3', self.right_callback, 10)
        # self.three_3_pub = self.create_publisher(LaserScan, '/right/scan_3', 10)
        # self.three_4 = self.create_subscription(LaserScan, '/three/cloud_4', self.right_callback, 10)
        # self.three_4_pub = self.create_publisher(LaserScan, '/right/scan_4', 10)

    def left_callback(self, msg):
        modified_msg = msg
        modified_msg.header.frame_id = 'left_cloud'
        self.left_pub.publish(modified_msg)

    def two_1_callback(self, msg):
        modified_msg = msg
        modified_msg.header.frame_id = 'left_scanner_1'
        self.two_1_pub.publish(modified_msg)

    def two_2_callback(self, msg):
        modified_msg = msg
        modified_msg.header.frame_id = 'left_scanner_2'
        self.two_2_pub.publish(modified_msg)

    def two_3_callback(self, msg):
        modified_msg = msg
        modified_msg.header.frame_id = 'left_scanner_3'
        self.two_3_pub.publish(modified_msg)

    def two_4_callback(self, msg):
        modified_msg = msg
        modified_msg.header.frame_id = 'left_scanner_4'
        self.two_4_pub.publish(modified_msg)

    def right_callback(self, msg):
        modified_msg = msg
        modified_msg.header.frame_id = 'right_cloud'
        self.right_pub.publish(modified_msg)

    def three_1_callback(self, msg):
        modified_msg = msg
        modified_msg.header.frame_id = 'right_scanner_1'
        self.three_1_pub.publish(modified_msg)
    
    def three_2_callback(self, msg):
        modified_msg = msg
        modified_msg.header.frame_id = 'right_scanner_2'
        self.three_2_pub.publish(modified_msg)
    
    def three_3_callback(self, msg):
        modified_msg = msg
        modified_msg.header.frame_id = 'right_scanner_3'
        self.three_3_pub.publish(modified_msg)

    def three_4_callback(self, msg):
        modified_msg = msg
        modified_msg.header.frame_id = 'right_scanner_4'
        self.three_4_pub.publish(modified_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudModifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
