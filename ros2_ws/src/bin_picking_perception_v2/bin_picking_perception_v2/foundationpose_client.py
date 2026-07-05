"""FoundationPose 在手位姿客户端（阶段二/四，检视位用）。

用途：抓起零件举到检视位后，对夹持中的零件做位姿估计/跟踪。
FoundationPose 的 track 模式(~30ms) 对夹爪遮挡鲁棒，正适合此场景。

数据流:
  RGB + 对齐深度 + 内参 --ZMQ--> 推理服务(model='foundationpose', mode='auto')
  服务端自动管理 register(首帧/丢跟踪) / track(后续帧)。

输出:
  /detected_objects_inhand (bin_picking_interfaces/ObjectPoseArray, 相机系)

与下游衔接（launch 里已配好，无需改代码）:
  inhand_estimator 订阅的 '/detected_objects' 在 system_v2.launch.py 中
  remap 到 '/detected_objects_inhand'，其余逻辑照旧。

服务:
  /inhand_pose/reset (std_srvs/Trigger)
    强制服务端重新 register。每抓一个新零件后建议调用一次
    （零件在手中的姿态每次都不同，track 的历史无效）。
"""

from __future__ import annotations

import numpy as np
import rclpy
from std_srvs.srv import Trigger

from bin_picking_interfaces.msg import ObjectPose, ObjectPoseArray
from bin_picking_grasp.geometry_utils import matrix_to_pose

from bin_picking_perception_v2.rgbd_client import RGBDClientNode


class FoundationPoseClient(RGBDClientNode):

    def __init__(self):
        super().__init__('foundationpose_client')
        self.declare_parameter('object_id', 'part')
        self.declare_parameter('min_score', 0.2)

        self.object_id = self.get_parameter('object_id').value
        self.min_score = self.get_parameter('min_score').value

        self.pub = self.create_publisher(
            ObjectPoseArray, '/detected_objects_inhand', 10)
        self.srv_reset = self.create_service(
            Trigger, '/inhand_pose/reset', self.on_reset)

    def on_reset(self, request, response):
        res = self.client.call({'model': 'foundationpose', 'cmd': 'reset'})
        ok = res is not None and res.get('ok', False)
        response.success = ok
        response.message = ('已重置，下一帧重新 register' if ok
                            else '重置失败（推理服务无响应？）')
        self.get_logger().info(response.message)
        return response

    def on_rgbd(self, rgb, depth_m, K, header):
        res = self.client.call({
            'model': 'foundationpose',
            'mode': 'auto',          # 服务端决定 register/track
            'rgb': rgb,
            'depth': depth_m,
            'K': K,
            'object_id': self.object_id,
        })
        if res is None:
            self.get_logger().warn('推理服务无响应（超时）',
                                   throttle_duration_sec=5.0)
            return
        if not res.get('ok'):
            self.get_logger().warn(f'foundationpose 推理失败: {res.get("msg")}',
                                   throttle_duration_sec=5.0)
            return

        results = [r for r in res.get('results', [])
                   if r.get('score', 0.0) >= self.min_score]
        if not results:
            return

        r = results[0]
        arr = ObjectPoseArray()
        arr.header = header
        obj = ObjectPose()
        obj.object_id = self.object_id
        obj.pose.header = header
        obj.pose.pose = matrix_to_pose(
            np.asarray(r['pose'], dtype=np.float64).reshape(4, 4))
        obj.fitness = float(r['score'])
        obj.inlier_rmse = 0.0
        arr.objects.append(obj)
        self.pub.publish(arr)
        self.get_logger().info(
            f'在手位姿 score={obj.fitness:.3f} ({r.get("mode", "?")})',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = FoundationPoseClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
