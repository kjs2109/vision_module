#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d

# 표준 서비스: 성공/실패 결과와 메시지를 간단히 반환하는 Trigger
from std_srvs.srv import Trigger  

class SaveMapService(Node):
    def __init__(self):
        super().__init__('save_pcd_map_service_node')
        
        # (1) 마지막으로 수신된 PointCloud2를 보관할 변수
        self.latest_cloud = None

        # (2) /kiss/local_map 구독
        #     실제 운영 시 QoS(History, Reliability 등)를 필요에 따라 설정 가능
        self.subscription = self.create_subscription(
            PointCloud2,
            # '/kiss/local_map',
            # "/lio_sam/mapping/map_local",
            "/lio_sam/mapping/map_global",
            self.local_map_callback,
            10
        )

        # (3) /save_local_map 서비스 서버 생성
        self.srv = self.create_service(
            Trigger,
            'save_local_map_service',
            self.save_map_callback
        )

        self.get_logger().info("SaveMapService node has started. Waiting for /kiss/local_map messages...")

    def local_map_callback(self, msg: PointCloud2):
        """ /kiss/local_map 메시지를 받을 때마다 마지막 클라우드 정보 업데이트 """
        self.latest_cloud = msg
        self.get_logger().debug("Local map updated with new PointCloud2 data")

    def save_map_callback(self, request, response):
        """ 사용자가 /save_local_map 서비스를 호출하면, 현재 latest_cloud를 .pcd 로 저장 """
        if self.latest_cloud is None:
            response.success = False
            response.message = "No local_map data received yet!"
            return response

        cloud_points = list(pc2.read_points(self.latest_cloud, skip_nans=True))

        xyz = []
        for p in cloud_points:
            xyz.append([p[0], p[1], p[2]])

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)

        filename = "last_local_map.pcd"
        success = o3d.io.write_point_cloud(filename, pcd)

        if success:
            response.success = True
            response.message = f"Local map saved to {filename}"
        else:
            response.success = False
            response.message = "Failed to write PCD file"

        return response

def main(args=None):
    rclpy.init(args=args)
    node = SaveMapService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
