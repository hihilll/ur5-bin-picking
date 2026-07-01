"""感知节点：订阅 Gemini2 点云 -> 预处理 -> PPF/ICP 位姿估计 -> 发布零件位姿。

发布:
  /detected_objects  (bin_picking_interfaces/ObjectPoseArray)
  /detected_markers  (visualization_msgs/MarkerArray)  用于 RViz2 可视化
  TF: 每个识别到的零件广播一个坐标系 object_<i>

阶段一只做"识别 + 位姿"，抓取规划在 bin_picking_grasp 包。
"""

from __future__ import annotations

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster

from bin_picking_interfaces.msg import ObjectPose, ObjectPoseArray

from bin_picking_perception import pose_estimation as pe
from bin_picking_perception.pointcloud_utils import (
    pointcloud2_to_o3d, load_cad_as_pointcloud)


def matrix_to_quaternion(R: np.ndarray):
    """3x3 旋转矩阵 -> 四元数 (x, y, z, w)。"""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        # ---- 参数 ----
        self.declare_parameter('input_topic', '/camera/depth_registered/points')
        self.declare_parameter('cad_model_path', '')
        self.declare_parameter('object_id', 'part')
        self.declare_parameter('model_scale', 0.001)      # CAD mm -> m
        self.declare_parameter('model_sample_points', 20000)
        self.declare_parameter('voxel_size', 0.003)       # 3mm
        self.declare_parameter('workspace_min', [-0.3, -0.3, 0.2])
        self.declare_parameter('workspace_max', [0.3, 0.3, 1.0])
        self.declare_parameter('plane_distance', 0.005)
        self.declare_parameter('cluster_eps', 0.01)
        self.declare_parameter('cluster_min_points', 100)
        self.declare_parameter('max_objects', 5)
        self.declare_parameter('min_fitness', 0.4)        # 配准质量阈值
        self.declare_parameter('process_every_n', 5)      # 每 N 帧处理一次

        gp = self.get_parameter
        self.input_topic = gp('input_topic').value
        self.object_id = gp('object_id').value
        self.voxel = gp('voxel_size').value
        self.ws_min = list(gp('workspace_min').value)
        self.ws_max = list(gp('workspace_max').value)
        self.plane_dist = gp('plane_distance').value
        self.cluster_eps = gp('cluster_eps').value
        self.cluster_min_points = gp('cluster_min_points').value
        self.max_objects = gp('max_objects').value
        self.min_fitness = gp('min_fitness').value
        self.process_every_n = gp('process_every_n').value

        # ---- 加载 CAD 模型 ----
        cad_path = gp('cad_model_path').value
        if not cad_path:
            self.get_logger().warn('未设置 cad_model_path，节点将无法估计位姿！')
            self.model_cloud = None
        else:
            self.model_cloud = load_cad_as_pointcloud(
                cad_path,
                num_points=gp('model_sample_points').value,
                scale=gp('model_scale').value)
            self.get_logger().info(
                f'已加载 CAD 模型 {cad_path}，{len(self.model_cloud.points)} 点')

        # ---- 通信 ----
        self.pub_objects = self.create_publisher(
            ObjectPoseArray, '/detected_objects', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/detected_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, 5)

        self._frame_count = 0
        self.get_logger().info(f'感知节点已启动，订阅 {self.input_topic}')

    def cloud_callback(self, msg: PointCloud2):
        self._frame_count += 1
        if self._frame_count % self.process_every_n != 0:
            return
        if self.model_cloud is None:
            return

        scene = pointcloud2_to_o3d(msg)
        if len(scene.points) == 0:
            return

        # 预处理：裁剪工作区 -> 去主平面 -> 聚类
        scene = pe.crop_workspace(scene, self.ws_min, self.ws_max)
        scene = pe.remove_dominant_plane(scene, self.plane_dist)
        clusters = pe.cluster_objects(
            scene, eps=self.cluster_eps, min_points=self.cluster_min_points)

        results = []
        for cluster in clusters[:self.max_objects]:
            try:
                res = pe.estimate_pose(self.model_cloud, cluster, self.voxel)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'位姿估计失败: {e}')
                continue
            if res.fitness >= self.min_fitness:
                results.append(res)

        results.sort(key=lambda r: r.fitness, reverse=True)
        self.publish_results(results, msg.header)

    def publish_results(self, results, header):
        arr = ObjectPoseArray()
        arr.header = header
        markers = MarkerArray()

        for i, res in enumerate(results):
            T = res.transform
            tx, ty, tz = T[0, 3], T[1, 3], T[2, 3]
            qx, qy, qz, qw = matrix_to_quaternion(T[:3, :3])

            obj = ObjectPose()
            obj.object_id = self.object_id
            obj.pose.header = header
            obj.pose.pose.position.x = float(tx)
            obj.pose.pose.position.y = float(ty)
            obj.pose.pose.position.z = float(tz)
            obj.pose.pose.orientation.x = float(qx)
            obj.pose.pose.orientation.y = float(qy)
            obj.pose.pose.orientation.z = float(qz)
            obj.pose.pose.orientation.w = float(qw)
            obj.fitness = res.fitness
            obj.inlier_rmse = res.inlier_rmse
            arr.objects.append(obj)

            # TF
            tf = TransformStamped()
            tf.header = header
            tf.child_frame_id = f'object_{i}'
            tf.transform.translation.x = float(tx)
            tf.transform.translation.y = float(ty)
            tf.transform.translation.z = float(tz)
            tf.transform.rotation.x = float(qx)
            tf.transform.rotation.y = float(qy)
            tf.transform.rotation.z = float(qz)
            tf.transform.rotation.w = float(qw)
            self.tf_broadcaster.sendTransform(tf)

            # Marker（坐标轴球，简单可视化）
            m = Marker()
            m.header = header
            m.ns = 'detected'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = obj.pose.pose
            m.scale.x = m.scale.y = m.scale.z = 0.02
            m.color.g = 1.0
            m.color.a = 0.8
            markers.markers.append(m)

        self.pub_objects.publish(arr)
        self.pub_markers.publish(markers)
        if results:
            self.get_logger().info(
                f'识别到 {len(results)} 个零件，最佳 fitness={results[0].fitness:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
