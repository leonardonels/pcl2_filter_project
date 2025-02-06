import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2

class PointCloudFilter:
    def __init__(self):
        self.node = rclpy.create_node('pointcloud_publisher')
        self.subscriber = self.node.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.callback,
            10
        )
        self.publisher = self.node.create_publisher(PointCloud2, '/lidar/filtered_pointcloud', 10)

    def callback(self, msg):
        # msg.data -> numpy array
        point_cloud_data = np.frombuffer(msg.data, dtype=np.float32)
        
        # Get pointcloud dimensions
        height = msg.height
        width = msg.width
        
        # Reshape to work with rows
        point_cloud_data = point_cloud_data.reshape(height, width, -1)
        
        # Define vertical intervals
        upper_range = height // 3
        lower_range = 2 * (height // 3)
        
        # Reduce density for low rays and high rays by 4 times
        upper_filtered = point_cloud_data[:upper_range:4]
        middle = point_cloud_data[upper_range:lower_range] # Kept default 
        lower_filtered = point_cloud_data[lower_range::4]  
        
        # Connect all intervals
        filtered_data = np.vstack((upper_filtered, middle, lower_filtered))
        
        # Compute the new height
        new_height = upper_filtered.shape[0] + middle.shape[0] + lower_filtered.shape[0]
        
        # Create the new message
        filtered_msg = PointCloud2()
        filtered_msg.header = msg.header
        filtered_msg.height = new_height
        filtered_msg.width = width
        filtered_msg.fields = msg.fields.copy() if hasattr(msg, 'fields') else []
        filtered_msg.is_bigendian = msg.is_bigendian
        filtered_msg.point_step = msg.point_step
        filtered_msg.row_step = msg.row_step * (height // new_height)  # Increase row_step because we are skipping rows
        filtered_msg.is_dense = msg.is_dense
        
        filtered_msg.data = filtered_data.reshape(-1).tobytes()
        
        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    pointcloud_filter = PointCloudFilter()
    try:
        rclpy.spin(pointcloud_filter.node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            pointcloud_filter.node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
