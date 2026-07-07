"""无模型抓取节点（测试用，不依赖 CAD）。

用途：手头只有螺丝/螺母等小物件、**没有 CAD 模型**时，仍能生成抓取并跑通取放闭环，
用于验证 相机→标定→MoveIt→夹爪 整条链路。

与阶段2(基于模型)的关系：
  - 阶段2 走 perception_node(PPF/ICP) + grasp_planner，需要 CAD。
  - 本节点是**平行替代方案**，一个节点直接从点云算出俯视抓取，
    发到同一个 /grasp_candidates 话题；下游 grasp_executor / pick_loop / gripper_driver
    **完全不用改**。两条路线互不影响，按需二选一启动。

原理（俯视二指抓取，model-free）：
  点云 -> (TF 转到基座系) -> 裁剪工作区 -> 去主平面(桌面/框底) -> DBSCAN 聚类
  对每个物体簇：
    - 抓取位置 = 簇质心(x,y)，z = 簇顶 - 抓取深度
    - 接近方向 = 竖直向下(夹爪 z 轴 = -Z_base)
    - 手指闭合方向 = 物体水平投影的**次主轴**(PCA 短边)，即夹住较窄的一侧
    - 开口宽度 = 物体沿闭合方向的尺寸(近似最小外接宽度)
  按"越靠上(越在堆顶)越优先"打分排序。

输出:
  /grasp_candidates (bin_picking_interfaces/GraspCandidateArray, 基座系)
  /grasp_markers    (visualization_msgs/MarkerArray)  RViz 可视化

⚠️ 局限：只做俯视抓取；对又大又平/超出夹爪行程的物体会跳过；不区分物体种类。
"""

from __future__ import annotations

import math

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from bin_picking_interfaces.msg import GraspCandidate, GraspCandidateArray

from bin_picking_perception import pose_estimation as pe
from bin_picking_perception.pointcloud_utils import pointcloud2_to_o3d


def _quat_to_matrix(x, y, z, w) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _matrix_to_quat(R: np.ndarray):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class ModelFreeGrasp(Node):

    def __init__(self):
        super().__init__('model_free_grasp')

        # ---- 参数 ----
        self.declare_parameter('input_topic', '/camera/depth_registered/points')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('object_id', 'unknown')
        # 工作区裁剪（**基座系** m，点云会先转到基座系再裁）——按你的桌面/料框位置改
        self.declare_parameter('workspace_min', [0.2, -0.3, 0.0])
        self.declare_parameter('workspace_max', [0.8, 0.3, 0.4])
        self.declare_parameter('voxel_size', 0.002)          # 小物件用更细体素
        self.declare_parameter('plane_distance', 0.005)      # 去桌面/框底阈值
        self.declare_parameter('remove_plane', True)
        self.declare_parameter('cluster_eps', 0.008)         # 小物件聚类邻域
        self.declare_parameter('cluster_min_points', 30)
        self.declare_parameter('max_objects', 10)
        self.declare_parameter('process_every_n', 5)
        # 夹爪与抓取几何
        self.declare_parameter('gripper_max_width', 0.05)    # EPGC-50 行程50mm
        self.declare_parameter('gripper_min_width', 0.002)
        self.declare_parameter('width_margin', 0.005)        # 目标宽度在物体尺寸上加的余量
        self.declare_parameter('grasp_depth', 0.005)         # 抓取点从簇顶下沉多少
        self.declare_parameter('max_grasp_extent', 0.06)     # 次主轴宽度超此判为不可抓/背景

        gp = self.get_parameter
        self.input_topic = gp('input_topic').value
        self.base_frame = gp('base_frame').value
        self.object_id = gp('object_id').value
        self.ws_min = list(gp('workspace_min').value)
        self.ws_max = list(gp('workspace_max').value)
        self.voxel = gp('voxel_size').value
        self.plane_dist = gp('plane_distance').value
        self.remove_plane = gp('remove_plane').value
        self.cluster_eps = gp('cluster_eps').value
        self.cluster_min_points = gp('cluster_min_points').value
        self.max_objects = gp('max_objects').value
        self.process_every_n = gp('process_every_n').value
        self.grip_max = gp('gripper_max_width').value
        self.grip_min = gp('gripper_min_width').value
        self.width_margin = gp('width_margin').value
        self.grasp_depth = gp('grasp_depth').value
        self.max_extent = gp('max_grasp_extent').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            GraspCandidateArray, '/grasp_candidates', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/grasp_markers', 10)
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, 5)

        self._frame_count = 0
        self.get_logger().info(
            f'无模型抓取节点已启动，订阅 {self.input_topic}，输出基座系 /grasp_candidates')

    # ---------- TF ----------
    def _base_from_cloud(self, cloud_frame):
        """查询 base<-cloud_frame 的 4x4；相同则单位阵；失败 None。"""
        if not cloud_frame or cloud_frame == self.base_frame:
            return np.eye(4)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, cloud_frame, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(
                f'TF 查询失败 base<-{cloud_frame}: {e}', throttle_duration_sec=2.0)
            return None
        T = np.eye(4)
        q = tf.transform.rotation
        T[:3, :3] = _quat_to_matrix(q.x, q.y, q.z, q.w)
        T[0, 3] = tf.transform.translation.x
        T[1, 3] = tf.transform.translation.y
        T[2, 3] = tf.transform.translation.z
        return T

    # ---------- 主回调 ----------
    def cloud_callback(self, msg: PointCloud2):
        self._frame_count += 1
        if self._frame_count % self.process_every_n != 0:
            return

        T_base_cloud = self._base_from_cloud(msg.header.frame_id)
        if T_base_cloud is None:
            return

        scene = pointcloud2_to_o3d(msg)
        if len(scene.points) == 0:
            return
        scene.transform(T_base_cloud)                     # -> 基座系

        scene = pe.crop_workspace(scene, self.ws_min, self.ws_max)
        if self.remove_plane:
            scene = pe.remove_dominant_plane(scene, self.plane_dist)
        clusters = pe.cluster_objects(
            scene, eps=self.cluster_eps, min_points=self.cluster_min_points)

        grasps = []
        for cluster in clusters[:self.max_objects]:
            g = self._grasp_from_cluster(cluster)
            if g is not None:
                grasps.append(g)

        grasps.sort(key=lambda x: x[2], reverse=True)     # 按 score 降序
        self._publish(grasps, msg.header.stamp)

    def _grasp_from_cluster(self, cluster: o3d.geometry.PointCloud):
        """由单个物体簇算一个俯视抓取。返回 (T_base_grasp(4x4), width, score) 或 None。"""
        pts = np.asarray(cluster.points)
        if len(pts) < self.cluster_min_points:
            return None

        centroid = pts.mean(axis=0)
        top_z = float(pts[:, 2].max())

        # 水平面内 PCA：次主轴=短边(手指闭合方向)，主轴=长边
        xy = pts[:, :2] - pts[:, :2].mean(axis=0)
        cov = xy.T @ xy / len(xy)
        eigvals, eigvecs = np.linalg.eigh(cov)            # 升序
        minor2d = eigvecs[:, 0]                           # 短边方向
        proj_minor = xy @ minor2d
        extent_minor = float(proj_minor.max() - proj_minor.min())  # 短边宽度

        # 可行性：短边宽度必须落在夹爪行程内
        target_width = extent_minor + self.width_margin
        if target_width > self.grip_max or extent_minor > self.max_extent:
            return None
        target_width = float(min(self.grip_max, max(self.grip_min, target_width)))

        # 构造夹爪位姿：z 轴向下(接近)，x 轴=手指闭合方向(短边)，y=z×x
        z_axis = np.array([0.0, 0.0, -1.0])
        x_axis = np.array([minor2d[0], minor2d[1], 0.0])
        nx = np.linalg.norm(x_axis)
        if nx < 1e-9:
            return None
        x_axis = x_axis / nx
        y_axis = np.cross(z_axis, x_axis)
        R = np.column_stack([x_axis, y_axis, z_axis])

        T = np.eye(4)
        T[:3, :3] = R
        T[0, 3] = float(centroid[0])
        T[1, 3] = float(centroid[1])
        T[2, 3] = float(top_z - self.grasp_depth)

        # 打分：越靠上(堆顶)越优先，尺寸越合适略加分
        fit = 1.0 - min(1.0, extent_minor / self.grip_max)
        score = 0.8 * top_z + 0.2 * fit
        return (T, target_width, float(score))

    def _publish(self, grasps, stamp):
        out = GraspCandidateArray()
        out.header.frame_id = self.base_frame
        out.header.stamp = stamp
        markers = MarkerArray()

        # 先发一个 DELETEALL，清掉上一帧残留箭头
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, (T, width, score) in enumerate(grasps):
            qx, qy, qz, qw = _matrix_to_quat(T[:3, :3])
            ps = PoseStamped()
            ps.header.frame_id = self.base_frame
            ps.header.stamp = stamp
            ps.pose.position.x = float(T[0, 3])
            ps.pose.position.y = float(T[1, 3])
            ps.pose.position.z = float(T[2, 3])
            ps.pose.orientation.x = float(qx)
            ps.pose.orientation.y = float(qy)
            ps.pose.orientation.z = float(qz)
            ps.pose.orientation.w = float(qw)

            gc = GraspCandidate()
            gc.grasp_pose = ps
            gc.width = float(width)
            gc.score = float(score)
            gc.object_id = self.object_id
            out.grasps.append(gc)

            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp = stamp
            m.ns = 'model_free_grasp'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose = ps.pose
            m.scale.x = 0.05
            m.scale.y = 0.008
            m.scale.z = 0.008
            m.color.g = 1.0 if i == 0 else 0.0    # 最佳抓取绿色，其余红
            m.color.r = 0.0 if i == 0 else 1.0
            m.color.a = 1.0
            markers.markers.append(m)

        self.pub.publish(out)
        self.pub_markers.publish(markers)
        if grasps:
            self.get_logger().info(
                f'生成 {len(grasps)} 个无模型抓取，最佳 score={grasps[0][2]:.3f}, '
                f'宽度={grasps[0][1]*1000:.1f}mm', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ModelFreeGrasp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
</content>
