"""手眼标定精度验证节点（阶段1.2）。

标定完成后，用它检查"相机看到的点，机器人能不能准确走到"：
  在 RViz2 里用 "Publish Point"（话题 /clicked_point）点一下场景中的某个
  已知物理点（如标定板角点、桌上一个尖点），本节点把它经 TF 从相机系
  转到基座系，打印坐标并发布:
    /handeye_verify/marker       (visualization_msgs/Marker)  基座系下的球，RViz 可视化
    /handeye_verify/target_pose  (geometry_msgs/PoseStamped)  夹爪竖直向下、对准该点的目标位姿

验证方法（安全、无自动运动）:
  1) 在 RViz MotionPlanning 面板把目标设成 /handeye_verify/target_pose（或手动对齐），
     Plan + Execute（**低速、手握急停**）。
  2) 量 TCP 实际到达点与真实物理点的偏差，**应在毫米级**。
  3) 偏差大 => 标定不准，重做标定。

也提供服务 /handeye_verify/transform_point：喂一个相机系点，返回基座系点，便于脚本化核对。

参数:
  camera_frame  相机系（点的输入所在系，默认取 clicked_point 的 header）
  base_frame    机器人基座系
  down_orientation  目标位姿是否强制夹爪竖直向下(qx=1)，默认 True
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  注册 PointStamped 的 do_transform


class HandeyeVerify(Node):

    def __init__(self):
        super().__init__('handeye_verify')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('down_orientation', True)

        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.down = self.get_parameter('down_orientation').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointStamped, '/clicked_point', self.on_point, 10)
        self.pub_marker = self.create_publisher(
            Marker, '/handeye_verify/marker', 10)
        self.pub_pose = self.create_publisher(
            PoseStamped, '/handeye_verify/target_pose', 10)

        self.get_logger().info(
            '手眼验证节点就绪：在 RViz 用 "Publish Point" 点一个已知物理点，'
            f'我会把它从 {self.camera_frame} 转到 {self.base_frame} 并发布目标位姿。')

    def on_point(self, msg: PointStamped):
        src_frame = msg.header.frame_id or self.camera_frame
        # 用点自带的时间戳查 TF；失败退回最新可用 TF。
        try:
            base_pt = self.tf_buffer.transform(
                msg, self.base_frame, timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(
                f'TF 转换失败 {src_frame}->{self.base_frame}: {e}；'
                f'确认手眼静态 TF 已发布（system.launch 或 handeye publish）。')
            return

        x, y, z = base_pt.point.x, base_pt.point.y, base_pt.point.z
        self.get_logger().info(
            f'点 [{src_frame}] ({msg.point.x:.4f}, {msg.point.y:.4f}, '
            f'{msg.point.z:.4f}) -> [{self.base_frame}] '
            f'({x:.4f}, {y:.4f}, {z:.4f})')

        self._publish_marker(x, y, z, base_pt.header.stamp)
        self._publish_pose(x, y, z, base_pt.header.stamp)

    def _publish_marker(self, x, y, z, stamp):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = stamp
        m.ns = 'handeye_verify'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.02
        m.color.r = 1.0
        m.color.a = 0.9
        self.pub_marker.publish(m)

    def _publish_pose(self, x, y, z, stamp):
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = stamp
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        if self.down:
            # 夹爪竖直向下：绕基座 X 轴转 180° -> qx=1
            ps.pose.orientation.x = 1.0
            ps.pose.orientation.w = 0.0
        else:
            ps.pose.orientation.w = 1.0
        self.pub_pose.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeVerify()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
