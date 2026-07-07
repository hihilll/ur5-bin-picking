"""在手位姿重估计节点（阶段4）。

抓起零件后举到相机前(检视位)，本节点根据感知结果计算
"零件相对夹爪 TCP 的位姿 T_tcp->part"，供放置补偿用。

原理:
  感知节点(对着夹持中的零件)输出 T_camera->part
  TF 提供 T_base->camera(手眼) 与 T_base->tcp(机器人正解)
  =>  T_base->part = T_base->camera * T_camera->part
      T_tcp->part  = inv(T_base->tcp) * T_base->part

对外服务: /estimate_inhand (bin_picking_interfaces/EstimateInHand)
输入: /detected_objects（感知节点，视野内应只剩夹持的那个零件）
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from bin_picking_interfaces.msg import ObjectPoseArray
from bin_picking_interfaces.srv import EstimateInHand
from bin_picking_grasp.geometry_utils import (
    pose_to_matrix, transform_to_matrix, matrix_to_transform, invert)


class InHandEstimator(Node):

    def __init__(self):
        super().__init__('inhand_estimator')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tcp_link', 'gripper_grasp_tcp')  # 夹爪指尖 TCP
        self.declare_parameter('stale_timeout', 1.0)   # 检测结果最大时效(s)

        self.base_frame = self.get_parameter('base_frame').value
        self.tcp_link = self.get_parameter('tcp_link').value
        self.stale_timeout = self.get_parameter('stale_timeout').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._latest = None

        self.sub = self.create_subscription(
            ObjectPoseArray, '/detected_objects', self.on_objects, 10)
        self.srv = self.create_service(
            EstimateInHand, '/estimate_inhand', self.on_estimate)
        self.get_logger().info('在手位姿重估计节点已就绪 (/estimate_inhand)')

    def on_objects(self, msg: ObjectPoseArray):
        if msg.objects:
            self._latest = msg

    def _lookup(self, target, source):
        tf = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        return transform_to_matrix(tf.transform)

    def on_estimate(self, request, response):
        if self._latest is None or not self._latest.objects:
            response.success = False
            response.message = '无检测结果，无法估计在手位姿'
            return response

        # 时效检查
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = (self._latest.header.stamp.sec
                 + self._latest.header.stamp.nanosec * 1e-9)
        if now - stamp > self.stale_timeout:
            response.success = False
            response.message = f'检测结果过期({now - stamp:.2f}s)，请确认零件在视野内'
            return response

        obj = self._latest.objects[0]              # 取置信度最高的
        camera_frame = self._latest.header.frame_id
        T_cam_part = pose_to_matrix(obj.pose.pose)

        try:
            T_base_cam = self._lookup(self.base_frame, camera_frame)
            T_base_tcp = self._lookup(self.base_frame, self.tcp_link)
        except Exception as e:  # noqa: BLE001
            response.success = False
            response.message = f'TF 查询失败: {e}'
            return response

        T_base_part = T_base_cam @ T_cam_part
        T_tcp_part = invert(T_base_tcp) @ T_base_part

        response.success = True
        response.gripper_to_part = matrix_to_transform(T_tcp_part)
        response.fitness = obj.fitness
        response.message = f'在手位姿估计成功 (fitness={obj.fitness:.3f})'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = InHandEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
