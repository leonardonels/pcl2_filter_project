import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2

class PointCloudFilter:
    def __init__(self):
        self.node = rclpy.create_node('pointcloud_ublisher')
        self.subscriber = self.node.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.callback,
            10
        )
        self.publisher = self.node.create_publisher(PointCloud2, '/output/filtered_pointcloud', 10)

    def callback(self, msg):
        filtered_msg = PointCloud2()
        filtered_msg.header = msg.header
        filtered_msg.height = msg.height
        filtered_msg.width = msg.width
        filtered_msg.fields = msg.fields.copy() if hasattr(msg, 'fields') else []
        filtered_msg.is_bigendian = msg.is_bigendian
        filtered_msg.point_step = msg.point_step
        filtered_msg.row_step = msg.row_step
        filtered_msg.is_dense = msg.is_dense
        
        filtered_msg.data = msg.data

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
