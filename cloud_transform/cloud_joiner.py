import ctypes
import math
import rclpy
import struct
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer
import tf2_ros
from sensor_msgs_py import point_cloud2 as pc2
from octomap_msgs.msg import Octomap

class PintCloudJoiner(Node):
    def __init__(self):
        super().__init__('pointcloud_joiner')
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        self.subscriber1 = Subscriber(self, PointCloud2, '/two/cloud')
        self.subscriber2 = Subscriber(self, PointCloud2, '/three/cloud')
        self.synchronizer = ApproximateTimeSynchronizer([self.subscriber1, self.subscriber2], 10, slop=10)
        self.synchronizer.registerCallback(self.callback)
        self.combined_pub = self.create_publisher(PointCloud2, '/all/cloud', 10)
        self.max_value = 0
        self.lidar_value = 8192

        self.subscription = self.create_subscription(
            Octomap,
            '/octomap_full',  # Replace with your OctoMap topic
            self.save_octomap, 
            10)
        
    def save_octomap(self, msg):
        filename = '/doc/code/ROBO/octomap.bt'  # Replace with your desired file name and format (e.g., .bt for OctoMap)
        with open(filename, 'wb') as file:
            file.write(msg.data)
        self.get_logger().info(f"Octomap saved to {filename}")


    def callback1(self, msg):
        self.all_pub.publish(msg)

    def callback(self, left_msg, right_msg):
        left_translation = np.array([0, 0, 1.5])
        left_rotation = np.array([math.pi/2, math.pi, math.pi/2])
        left_msg_pts = self.transform_cloud(left_msg, left_translation, left_rotation, return_byte=True)
        right_translation = np.array([0, 0, 1.5])
        right_rotation = np.array([-math.pi/2, math.pi, -math.pi/2])
        right_msg_pts = self.transform_cloud(right_msg, right_translation, right_rotation, return_byte=True)
        all_points = left_msg_pts + right_msg_pts
        new_points = pc2.create_cloud(left_msg.header, left_msg.fields, all_points)
        new_points.header.frame_id = 'base_link'
        self.combined_pub.publish(new_points)

    
    def transform_cloud(self, input_cloud, translation, rotation, return_byte=False):
        transformation_matrix = self.compose_transformation_matrix(translation, rotation)
        points = []
        for x, y, z, i in pc2.read_points(input_cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            point = np.array([x, y, z], dtype=np.float32)
            point = np.append(point, 1.0)  # Homogeneous coordinates
            transformed_point = np.dot(transformation_matrix, point)
            # print(f'point: {point}, transformed_point: {transformed_point}')
            nx, ny, nz = map(np.float32, transformed_point[:3])
            points.append([nx, ny, nz, i])  # Only x, y, z
        if return_byte:
            return points
        header = input_cloud.header
        print(input_cloud.fields)
        new_points = pc2.create_cloud(header, input_cloud.fields, points)
        return new_points

    def compose_transformation_matrix(self, translation, rotation):
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, 3] = translation
        roll, pitch, yaw = rotation
        rotation_matrix = self.euler_to_rotation_matrix(roll, pitch, yaw)
        transformation_matrix[:3, :3] = rotation_matrix
        return transformation_matrix

    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        rotation_matrix = np.array([
            [math.cos(yaw) * math.cos(pitch), math.cos(yaw) * math.sin(pitch) * math.sin(roll) - math.sin(yaw) * math.cos(roll),
             math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll)],
            [math.sin(yaw) * math.cos(pitch), math.sin(yaw) * math.sin(pitch) * math.sin(roll) + math.cos(yaw) * math.cos(roll),
             math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll)],
            [-math.sin(pitch), math.cos(pitch) * math.sin(roll), math.cos(pitch) * math.cos(roll)]
        ])
        return rotation_matrix

def main(args=None):
    rclpy.init(args=args)
    node = PintCloudJoiner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()