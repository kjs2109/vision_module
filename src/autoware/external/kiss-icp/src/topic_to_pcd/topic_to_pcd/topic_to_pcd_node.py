#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('topic_to_pcd_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/kiss/local_map',  # 구독할 토픽 이름
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # PointCloud2 -> 파이썬 리스트 변환
        cloud_points = list(pc2.read_points(msg, skip_nans=True))
        xyz = []
        for p in cloud_points:
            # p: (x, y, z, intensity, ...) 형태일 수도 있으니 인덱스에 맞게 사용
            xyz.append([p[0], p[1], p[2]])

        # open3d Geometry 생성
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)

        # 여기서 파일 저장(매 callback마다 덮어쓰고 싶지 않다면, 로직 개선)
        o3d.io.write_point_cloud("output.pcd", pcd)
        self.get_logger().info("PointCloud saved to output.pcd")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
