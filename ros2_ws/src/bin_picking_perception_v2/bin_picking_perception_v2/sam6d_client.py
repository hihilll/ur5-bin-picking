"""SAM-6D 位姿估计客户端（阶段二，替代 perception_node 的 PPF+ICP）。

数据流:
  RGB + 对齐深度 + 内参 --ZMQ--> 推理服务(inference/server.py, model='sam6d')
  <-- 每个实例的 4x4 位姿(相机系) + 置信度

输出与阶段一 perception_node **完全一致**，下游 grasp_planner 不用改:
  /detected_objects  (bin_picking_interfaces/ObjectPoseArray, 相机系)
  /detected_markers  (visualization_msgs/MarkerArray)
  TF: object_<i>

CAD 模型与模板在**推理服务端**配置（server 启动参数），本节点只传图像。
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

from bin_picking_interfaces.msg import ObjectPose, ObjectPoseArray
from bin_picking_grasp.geometry_utils import matrix_to_pose, matrix_to_quaternion

from bin_picking_perception_v2.rgbd_client import RGBDClientNode


class Sam6dClient(RGBDClientNode):

    def __init__(self):
        super().__init__('sam6d_client')
        self.declare_parameter('object_id', 'part')
        self.declare_parameter('min_score', 0.3)      # 服务端置信度阈值
        self.declare_parameter('max_objects', 5)

        self.object_id = self.get_parameter('object_id').value
        self.min_score = self.get_parameter('min_score').value
        self.max_objects = self.get_parameter('max_objects').value

        self.pub_objects = self.create_publisher(
            ObjectPoseArray, '/detected_objects', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/detected_markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def on_rgbd(self, rgb, depth_m, K, header):
        res = self.client.call({
            'model': 'sam6d',
            'rgb': rgb,
            'depth': depth_m,
            'K': K,
            'object_id': self.object_id,
        })
        if res is None:
            self.get_logger().warn('推理服务无响应（超时），确认 server.py 已启动',
                                   throttle_duration_sec=5.0)
            return
        if not res.get('ok'):
            self.get_logger().warn(f'sam6d 推理失败: {res.get("msg")}',
                                   throttle_duration_sec=5.0)
            return

        results = [r for r in res.get('results', [])
                   if r.get('score', 0.0) >= self.min_score]
        results.sort(key=lambda r: r['score'], reverse=True)
        self._publish(results[:self.max_objects], header)

    def _publish(self, results, header):
        arr = ObjectPoseArray()
        arr.header = header
        markers = MarkerArray()

        for i, r in enumerate(results):
            T = np.asarray(r['pose'], dtype=np.float64).reshape(4, 4)

            obj = ObjectPose()
            obj.object_id = self.object_id
            obj.pose.header = header
            obj.pose.pose = matrix_to_pose(T)
            obj.fitness = float(r['score'])
            obj.inlier_rmse = 0.0        # 网络方法无此量，置 0
            arr.objects.append(obj)

            tf = TransformStamped()
            tf.header = header
            tf.child_frame_id = f'object_{i}'
            tf.transform.translation.x = float(T[0, 3])
            tf.transform.translation.y = float(T[1, 3])
            tf.transform.translation.z = float(T[2, 3])
            qx, qy, qz, qw = matrix_to_quaternion(T[:3, :3])
            tf.transform.rotation.x = float(qx)
            tf.transform.rotation.y = float(qy)
            tf.transform.rotation.z = float(qz)
            tf.transform.rotation.w = float(qw)
            self.tf_broadcaster.sendTransform(tf)

            m = Marker()
            m.header = header
            m.ns = 'detected_v2'
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
                f'SAM-6D 识别 {len(results)} 个零件，'
                f'最佳 score={results[0]["score"]:.3f}',
                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = Sam6dClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
