import rclpy
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import struct
import numpy as np
from sensor_msgs_py import point_cloud2 as pc2

class PointCloudSaver:
    def __init__(self):
        self.node = rclpy.create_node('open3d_pcd_saver')
        self.subscription = self.node.create_subscription(PointCloud2, '/all/cloud', self.point_cloud_callback, 10)
    
    def point_cloud_callback(self, msg):
        for x, y, z, intensity in pc2.read_points(msg, skip_nans=True):
            print(f"x: {x:.2f}, y: {y:.2f}, z: {z:.2f}, intensity: {intensity:.2f}")
        
        # Combine XYZ coordinates into a single array
        pc_data = np.stack((x, y, z), axis=-1)
    
        # Convert ROS2 PointCloud2 message to Open3D PointCloud
        o3d_cloud = o3d.geometry.PointCloud()
    
        # Save the Open3D PointCloud to a PCD file
        o3d.io.write_point_cloud('point_cloud.pcd', o3d_cloud)
    
    def run(self, args=None):
        rclpy.init(args=args)
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        
        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    point_cloud_saver = PointCloudSaver()
    point_cloud_saver.run()
