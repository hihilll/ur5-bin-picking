"""阶段二公共基础：ZMQ 推理客户端 + RGB-D 同步订阅节点基类。

三个客户端节点（sam6d_client / foundationpose_client / grasp_client）共用：
  - InferenceClient: 经 ZMQ REQ/REP 调用推理服务（inference/server.py），
    pickle 序列化 numpy 数组，带超时与自动重连。
  - RGBDClientNode:  message_filters 同步 RGB + 对齐深度 + CameraInfo，
    降频后回调 self.on_rgbd(rgb, depth_m, K, header)，子类实现。

⚠️ 序列化用 pickle：仅限本机或可信局域网内使用（推理服务与 ROS 同机/同网段）。
⚠️ 深度话题必须是**对齐到彩色**的深度图（orbbec 驱动开启 align/registration），
    否则 mask 与深度错位、位姿必错。
"""

from __future__ import annotations

import pickle

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters

try:
    import zmq
    _HAS_ZMQ = True
except ImportError:
    _HAS_ZMQ = False


class InferenceClient:
    """ZMQ REQ 客户端。call() 发送 dict 请求，返回 dict 响应；超时返回 None。

    REQ socket 一旦超时状态机即坏，必须关闭重建——已在 _reset() 处理。
    """

    def __init__(self, address: str, timeout_ms: int = 5000):
        if not _HAS_ZMQ:
            raise RuntimeError('未安装 pyzmq: pip install pyzmq')
        self.address = address
        self.timeout_ms = timeout_ms
        self.ctx = zmq.Context.instance()
        self.sock = None
        self._connect()

    def _connect(self):
        self.sock = self.ctx.socket(zmq.REQ)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.connect(self.address)

    def _reset(self):
        try:
            self.sock.close(linger=0)
        except Exception:  # noqa: BLE001
            pass
        self._connect()

    def call(self, request: dict) -> dict | None:
        try:
            self.sock.send(pickle.dumps(request, protocol=4))
            if self.sock.poll(self.timeout_ms) == 0:
                self._reset()          # 超时后 REQ 不可复用
                return None
            return pickle.loads(self.sock.recv())
        except Exception:  # noqa: BLE001
            self._reset()
            return None

    def close(self):
        try:
            self.sock.close(linger=0)
        except Exception:  # noqa: BLE001
            pass


class RGBDClientNode(Node):
    """同步订阅 RGB + 对齐深度 + CameraInfo 的节点基类。

    子类实现 on_rgbd(rgb, depth_m, K, header)：
      rgb:     HxWx3 uint8 (RGB 顺序)
      depth_m: HxW float32, 单位米, 无效点为 0/nan
      K:       3x3 相机内参
      header:  RGB 图像 header（frame_id 为彩色光学系）
    """

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        # ⚠️ 必须是对齐到彩色的深度；以 orbbec 驱动实际话题为准
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('server_address', 'tcp://127.0.0.1:5555')
        self.declare_parameter('request_timeout', 5.0)   # 推理服务超时 s
        self.declare_parameter('process_every_n', 5)     # 每 N 帧处理一次

        gp = self.get_parameter
        self.process_every_n = gp('process_every_n').value
        self._frame_count = 0
        self._busy = False
        self.bridge = CvBridge()

        self.client = InferenceClient(
            gp('server_address').value,
            timeout_ms=int(gp('request_timeout').value * 1000))

        sub_rgb = message_filters.Subscriber(self, Image, gp('rgb_topic').value)
        sub_depth = message_filters.Subscriber(
            self, Image, gp('depth_topic').value)
        sub_info = message_filters.Subscriber(
            self, CameraInfo, gp('camera_info_topic').value)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_rgb, sub_depth, sub_info], queue_size=5, slop=0.1)
        self.sync.registerCallback(self._on_sync)

        self.get_logger().info(
            f'{node_name} 已启动: rgb={gp("rgb_topic").value}, '
            f'depth={gp("depth_topic").value}, '
            f'server={gp("server_address").value}')

    def _on_sync(self, rgb_msg: Image, depth_msg: Image, info: CameraInfo):
        self._frame_count += 1
        if self._frame_count % self.process_every_n != 0:
            return
        if self._busy:      # 推理未返回时丢帧，避免请求堆积
            return
        self._busy = True
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_m = self._to_meters(depth)
            K = np.array(info.k, dtype=np.float64).reshape(3, 3)
            self.on_rgbd(np.asarray(rgb), depth_m, K, rgb_msg.header)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'处理 RGB-D 帧失败: {e}',
                                   throttle_duration_sec=2.0)
        finally:
            self._busy = False

    @staticmethod
    def _to_meters(depth: np.ndarray) -> np.ndarray:
        """深度图 -> float32 米。16UC1 按 mm 计，32FC1 按 m 计。"""
        if depth.dtype == np.uint16:
            return depth.astype(np.float32) / 1000.0
        return depth.astype(np.float32)

    def on_rgbd(self, rgb, depth_m, K, header):
        raise NotImplementedError

    def destroy_node(self):
        self.client.close()
        super().destroy_node()
