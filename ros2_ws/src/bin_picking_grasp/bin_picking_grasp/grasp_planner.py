"""抓取规划节点。

输入: /detected_objects (零件位姿, 相机系或基座系)
输出: /grasp_candidates (基座系下排序后的夹爪 TCP 目标位姿)

原理:
  在 CAD 模型坐标系下预标注若干抓取位姿 T_object->grasp（见 config/grasp_annotations.yaml）。
  运行时:
    T_base->object = TF(base<-camera) * T_camera->object
    T_base->grasp  = T_base->object * T_object->grasp
  再按"接近方向接近竖直向下、零件置信度、是否在工作区"打分排序。
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener

from bin_picking_interfaces.msg import (
    ObjectPoseArray, GraspCandidate, GraspCandidateArray)

from bin_picking_grasp.geometry_utils import (
    pose_to_matrix, matrix_to_pose, make_grasp_matrix, quaternion_to_matrix)


class GraspPlanner(Node):

    def __init__(self):
        super().__init__('grasp_planner')

        self.declare_parameter('base_frame', 'base_link')
        # 抓取标注: 扁平列表, 每 7 个一组 [x,y,z,roll,pitch,yaw,width]
        self.declare_parameter('grasp_annotations', [0.0, 0.0, 0.0,
                                                     3.14159, 0.0, 0.0, 0.03])
        self.declare_parameter('approach_distance', 0.10)  # 预抓取沿接近轴回退距离 m
        self.declare_parameter('min_grasp_score', 0.0)

        self.base_frame = self.get_parameter('base_frame').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.min_score = self.get_parameter('min_grasp_score').value
        self.annotations = self._parse_annotations(
            list(self.get_parameter('grasp_annotations').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            ObjectPoseArray, '/detected_objects', self.on_objects, 10)
        self.pub = self.create_publisher(
            GraspCandidateArray, '/grasp_candidates', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/grasp_markers', 10)

        self.get_logger().info(
            f'抓取规划已启动，{len(self.annotations)} 个抓取标注，base={self.base_frame}')

    @staticmethod
    def _parse_annotations(flat):
        anns = []
        for i in range(0, len(flat) - 6, 7):
            x, y, z, r, p, yw, width = flat[i:i + 7]
            anns.append((make_grasp_matrix(x, y, z, r, p, yw), float(width)))
        return anns

    def _base_from_camera(self, camera_frame, stamp):
        """查询 TF: base <- camera。失败返回 None。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, camera_frame, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'TF 查询失败 base<-{camera_frame}: {e}', throttle_duration_sec=2.0)
            return None
        T = np.eye(4)
        q = tf.transform.rotation
        T[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        T[0, 3] = tf.transform.translation.x
        T[1, 3] = tf.transform.translation.y
        T[2, 3] = tf.transform.translation.z
        return T

    @staticmethod
    def _score(T_base_grasp, fitness):
        """打分: 接近轴(夹爪 z)越接近竖直向下越好 + 配准置信度。"""
        approach = T_base_grasp[:3, 2]            # 夹爪坐标系 z 轴在基座系下的方向
        downness = float(-approach[2])            # 指向 -Z(向下) 时为 +1
        downness = max(0.0, downness)
        return 0.7 * downness + 0.3 * float(fitness)

    def on_objects(self, msg: ObjectPoseArray):
        if not msg.objects:
            return
        camera_frame = msg.header.frame_id
        # 若位姿已在基座系，TF 为单位阵
        if camera_frame == self.base_frame:
            T_base_cam = np.eye(4)
        else:
            T_base_cam = self._base_from_camera(camera_frame, msg.header.stamp)
            if T_base_cam is None:
                return

        candidates = []
        for obj in msg.objects:
            T_cam_obj = pose_to_matrix(obj.pose.pose)
            T_base_obj = T_base_cam @ T_cam_obj
            for T_obj_grasp, width in self.annotations:
                T_base_grasp = T_base_obj @ T_obj_grasp
                score = self._score(T_base_grasp, obj.fitness)
                if score < self.min_score:
                    continue
                candidates.append((T_base_grasp, width, score, obj.object_id))

        candidates.sort(key=lambda c: c[2], reverse=True)
        self._publish(candidates, msg.header.stamp)

    def _publish(self, candidates, stamp):
        out = GraspCandidateArray()
        out.header.frame_id = self.base_frame
        out.header.stamp = stamp
        markers = MarkerArray()

        for i, (T, width, score, obj_id) in enumerate(candidates):
            gc = GraspCandidate()
            ps = PoseStamped()
            ps.header.frame_id = self.base_frame
            ps.header.stamp = stamp
            ps.pose = matrix_to_pose(T)
            gc.grasp_pose = ps
            gc.width = float(width)
            gc.score = float(score)
            gc.object_id = obj_id
            out.grasps.append(gc)

            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp = stamp
            m.ns = 'grasp'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose = ps.pose
            m.scale.x = 0.06
            m.scale.y = 0.008
            m.scale.z = 0.008
            m.color.r = 1.0 if i > 0 else 0.0
            m.color.g = 1.0 if i == 0 else 0.0   # 最佳抓取绿色
            m.color.a = 1.0
            markers.markers.append(m)

        self.pub.publish(out)
        self.pub_markers.publish(markers)
        if candidates:
            self.get_logger().info(
                f'生成 {len(candidates)} 个抓取候选，最佳 score={candidates[0][2]:.3f}',
                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
