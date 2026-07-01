"""测试工具：发布一个固定的抓取候选到 /grasp_candidates。

用于仿真/无相机时验证 grasp_executor 的取放流程。
发布一个 UR5 可达、夹爪竖直向下的抓取位姿（基座系）。
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from bin_picking_interfaces.msg import GraspCandidate, GraspCandidateArray


class TestGraspPublisher(Node):

    def __init__(self):
        super().__init__('publish_test_grasp')
        self.declare_parameter('base_frame', 'base_link')
        # 默认抓取位姿：base 前方 0.4m、高 0.3m，夹爪朝下(绕X转180°: qx=1)
        self.declare_parameter('pose', [0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0])
        self.declare_parameter('width', 0.03)

        self.base_frame = self.get_parameter('base_frame').value
        self.pose = list(self.get_parameter('pose').value)
        self.width = self.get_parameter('width').value

        self.pub = self.create_publisher(
            GraspCandidateArray, '/grasp_candidates', 10)
        self.timer = self.create_timer(1.0, self.publish_once)
        self.get_logger().info('开始发布测试抓取候选到 /grasp_candidates (1Hz)')

    def publish_once(self):
        arr = GraspCandidateArray()
        arr.header.frame_id = self.base_frame
        arr.header.stamp = self.get_clock().now().to_msg()

        gc = GraspCandidate()
        ps = PoseStamped()
        ps.header = arr.header
        x, y, z, qx, qy, qz, qw = self.pose
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        gc.grasp_pose = ps
        gc.width = float(self.width)
        gc.score = 1.0
        gc.object_id = 'test'
        arr.grasps.append(gc)

        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = TestGraspPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
