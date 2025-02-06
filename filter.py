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
        self.publisher = self.node.create_publisher(PointCloud2, '/output/filtered_pointcloud', 10)

    def callback(self, msg):
        # msg.data -> numpy array
        point_cloud_data = np.frombuffer(msg.data, dtype=np.float32)
        
        # Get pointcloud dimensions
        height = msg.height
        width = msg.width
        
        # Select only the fourth row (reduce the number of lidar rows by 4 times)
        filtered_data = point_cloud_data.reshape(height, width, -1)[::4]
        
        # Change the hight for th enew message
        new_height = height // 4
        
        # Create the new message
        filtered_msg = PointCloud2()
        filtered_msg.header = msg.header
        filtered_msg.height = new_height
        filtered_msg.width = width
        filtered_msg.fields = msg.fields.copy() if hasattr(msg, 'fields') else []
        filtered_msg.is_bigendian = msg.is_bigendian
        filtered_msg.point_step = msg.point_step
        filtered_msg.row_step = msg.row_step * 4  # Aumenta row_step poiché saltiamo righe
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
        pointcloud_filter.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
