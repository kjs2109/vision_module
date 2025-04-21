# convert_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from sensor_msgs_py.point_cloud2 import create_cloud

class PointCloud2Converter(Node):
    def __init__(self):
        super().__init__('pcd_type_converter_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/raw_lidar',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(
            PointCloud2,
            '/converted_lidar',
            10
        )

    def callback(self, msg):
        num_points = msg.height * msg.width

        dtype_input = np.dtype([
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("intensity", np.float32),
            ("ring", np.uint16),
            ("timestamp", np.float64),
        ])

        points = np.frombuffer(bytes(msg.data), dtype=dtype_input, count=num_points)

        intensity_scaled = (np.clip(points["intensity"], 0.0, 1.0) * 255).astype(np.uint8)
        return_type = np.zeros((num_points,), dtype=np.uint8)
        channel = points["ring"]

        dtype_target = np.dtype([
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("intensity", np.uint8),
            ("return_type", np.uint8),
            ("channel", np.uint16),
        ])

        new_points = np.zeros(num_points, dtype=dtype_target)
        new_points["x"] = points["x"]
        new_points["y"] = points["y"]  # 좌표계 변환
        new_points["z"] = points["z"]
        new_points["intensity"] = intensity_scaled
        new_points["return_type"] = return_type
        new_points["channel"] = channel

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.UINT8, count=1),
            PointField(name="return_type", offset=13, datatype=PointField.UINT8, count=1),
            PointField(name="channel", offset=14, datatype=PointField.UINT16, count=1),
        ]

        cloud_msg = create_cloud(msg.header, fields, new_points)
        self.publisher.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloud2Converter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
