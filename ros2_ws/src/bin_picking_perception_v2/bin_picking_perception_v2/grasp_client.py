"""学习型抓取检测客户端（阶段二，替代 model_free_grasp / grasp_planner 链路）。

服务端跑 HGGD / EconomicGrasp / AnyGrasp 等在 GraspNet-1B 上训练的抓取网络，
本节点送 RGB-D，取回相机系下的 6-DoF 抓取，再:
  TF 转基座系 -> 按夹爪行程/接近方向过滤 -> 排序 -> 发布

输出与 model_free_grasp / grasp_planner **完全一致**，下游不改:
  /grasp_candidates (bin_picking_interfaces/GraspCandidateArray, 基座系)
  /grasp_markers    (visualization_msgs/MarkerArray)

⚠️ 抓取坐标系约定（与服务端 wrapper 的契约）:
  服务端返回的位姿必须满足 z 轴=接近方向、x 轴=手指闭合方向
  （graspnetAPI 原始约定是 x 轴=接近，wrapper 里必须转换，见
   inference/wrappers/grasp_wrapper.py）。
  grasp_executor 沿位姿 z 轴回退生成预抓取位，约定错则动作全错。
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener

from bin_picking_interfaces.msg import GraspCandidate, GraspCandidateArray
from bin_picking_grasp.geometry_utils import (
    matrix_to_pose, quaternion_to_matrix)

from bin_picking_perception_v2.rgbd_client import RGBDClientNode


class GraspClient(RGBDClientNode):

    def __init__(self):
        super().__init__('grasp_client')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('object_id', 'unknown')
        self.declare_parameter('max_grasps', 10)
        self.declare_parameter('min_score', 0.2)
        # 夹爪可行性过滤
        self.declare_parameter('gripper_max_width', 0.05)   # EPGC-50 行程50mm
        self.declare_parameter('gripper_min_width', 0.002)
        # 接近方向过滤: 抓取 z 轴与基座 -Z 的夹角上限(deg)。
        # 俯视料框场景夹爪只能从上方伸入，侧向抓取不可达。
        self.declare_parameter('max_approach_angle_deg', 60.0)
        # 工作区过滤（基座系），料框外的候选丢弃
        self.declare_parameter('workspace_min', [0.2, -0.3, 0.0])
        self.declare_parameter('workspace_max', [0.8, 0.3, 0.4])

        gp = self.get_parameter
        self.base_frame = gp('base_frame').value
        self.object_id = gp('object_id').value
        self.max_grasps = gp('max_grasps').value
        self.min_score = gp('min_score').value
        self.grip_max = gp('gripper_max_width').value
        self.grip_min = gp('gripper_min_width').value
        self.max_approach_rad = math.radians(
            gp('max_approach_angle_deg').value)
        self.ws_min = np.array(gp('workspace_min').value)
        self.ws_max = np.array(gp('workspace_max').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            GraspCandidateArray, '/grasp_candidates', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/grasp_markers', 10)

    def _base_from_frame(self, frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, frame, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'TF 查询失败 base<-{frame}: {e}',
                                   throttle_duration_sec=2.0)
            return None
        T = np.eye(4)
        q = tf.transform.rotation
        T[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        T[0, 3] = tf.transform.translation.x
        T[1, 3] = tf.transform.translation.y
        T[2, 3] = tf.transform.translation.z
        return T

    def on_rgbd(self, rgb, depth_m, K, header):
        T_base_cam = self._base_from_frame(header.frame_id)
        if T_base_cam is None:
            return

        res = self.client.call({
            'model': 'grasp',
            'rgb': rgb,
            'depth': depth_m,
            'K': K,
        })
        if res is None:
            self.get_logger().warn('推理服务无响应（超时）',
                                   throttle_duration_sec=5.0)
            return
        if not res.get('ok'):
            self.get_logger().warn(f'抓取推理失败: {res.get("msg")}',
                                   throttle_duration_sec=5.0)
            return

        grasps = []
        for g in res.get('results', []):
            score = float(g.get('score', 0.0))
            width = float(g.get('width', 0.0))
            if score < self.min_score:
                continue
            if not (self.grip_min <= width <= self.grip_max):
                continue
            T_base_grasp = T_base_cam @ np.asarray(
                g['pose'], dtype=np.float64).reshape(4, 4)
            # 接近方向可达性: z 轴与竖直向下夹角
            approach = T_base_grasp[:3, 2]
            cos_down = float(-approach[2])
            if cos_down < math.cos(self.max_approach_rad):
                continue
            # 工作区过滤
            p = T_base_grasp[:3, 3]
            if np.any(p < self.ws_min) or np.any(p > self.ws_max):
                continue
            grasps.append((T_base_grasp, width, score))

        grasps.sort(key=lambda x: x[2], reverse=True)
        self._publish(grasps[:self.max_grasps], header.stamp)

    def _publish(self, grasps, stamp):
        out = GraspCandidateArray()
        out.header.frame_id = self.base_frame
        out.header.stamp = stamp
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, (T, width, score) in enumerate(grasps):
            ps = PoseStamped()
            ps.header.frame_id = self.base_frame
            ps.header.stamp = stamp
            ps.pose = matrix_to_pose(T)

            gc = GraspCandidate()
            gc.grasp_pose = ps
            gc.width = float(width)
            gc.score = float(score)
            gc.object_id = self.object_id
            out.grasps.append(gc)

            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp = stamp
            m.ns = 'learned_grasp'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose = ps.pose
            m.scale.x = 0.05
            m.scale.y = 0.008
            m.scale.z = 0.008
            m.color.g = 1.0 if i == 0 else 0.0
            m.color.r = 0.0 if i == 0 else 1.0
            m.color.a = 1.0
            markers.markers.append(m)

        self.pub.publish(out)
        self.pub_markers.publish(markers)
        if grasps:
            self.get_logger().info(
                f'学习型抓取 {len(grasps)} 个，最佳 score={grasps[0][2]:.3f}, '
                f'宽度={grasps[0][1]*1000:.1f}mm', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = GraspClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
