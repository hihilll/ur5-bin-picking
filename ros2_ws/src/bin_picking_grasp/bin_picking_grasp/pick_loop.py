"""主状态机（阶段5）：自动循环 识别->抓取->放置，直到料框清空或异常。

状态: IDLE -> RUNNING -> (DONE 料框清空 | ABORTED 连续失败)
鲁棒性:
  - 无抓取候选: 连续 empty_threshold 次判定料框已清空 -> DONE
  - 取放失败: 连续 max_consecutive_failures 次 -> ABORTED（安全停止）
  - 单次失败: 重试（重新识别后再抓）

服务:
  /bin_picking/start (std_srvs/Trigger)  开始循环
  /bin_picking/stop  (std_srvs/Trigger)  停止循环
发布:
  /bin_picking/status (std_msgs/String)
依赖: grasp_planner(出 /grasp_candidates) + grasp_executor(/pick_place/run)
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from std_srvs.srv import Trigger

from bin_picking_interfaces.msg import GraspCandidateArray


class PickLoop(Node):

    def __init__(self):
        super().__init__('pick_loop')
        self.declare_parameter('scan_interval', 1.0)        # 每轮等待识别的间隔 s
        self.declare_parameter('settle_time', 1.0)          # 放置后稳定等待 s
        self.declare_parameter('candidate_timeout', 2.0)    # 候选时效 s
        self.declare_parameter('empty_threshold', 3)        # 连续空判定清空
        self.declare_parameter('max_consecutive_failures', 3)
        self.declare_parameter('max_picks', 0)              # 0=不限
        self.declare_parameter('pick_timeout', 120.0)       # 单次取放最长等待 s

        self.scan_interval = self.get_parameter('scan_interval').value
        self.settle_time = self.get_parameter('settle_time').value
        self.candidate_timeout = self.get_parameter('candidate_timeout').value
        self.empty_threshold = self.get_parameter('empty_threshold').value
        self.max_failures = self.get_parameter('max_consecutive_failures').value
        self.max_picks = self.get_parameter('max_picks').value
        self.pick_timeout = self.get_parameter('pick_timeout').value

        self.cb = ReentrantCallbackGroup()
        self._latest = None
        self._running = False
        self._thread = None

        self.sub = self.create_subscription(
            GraspCandidateArray, '/grasp_candidates', self._on_grasps, 10,
            callback_group=self.cb)
        self.pick_cli = self.create_client(
            Trigger, '/pick_place/run', callback_group=self.cb)
        self.status_pub = self.create_publisher(String, '/bin_picking/status', 10)

        self.create_service(Trigger, '/bin_picking/start', self._on_start,
                            callback_group=self.cb)
        self.create_service(Trigger, '/bin_picking/stop', self._on_stop,
                            callback_group=self.cb)

        self._set_status('IDLE')
        self.get_logger().info('主状态机就绪，调用 /bin_picking/start 开始')

    def _on_grasps(self, msg: GraspCandidateArray):
        self._latest = msg

    def _set_status(self, s):
        self.status_pub.publish(String(data=s))
        self.get_logger().info(f'[状态] {s}')

    def _fresh_candidates(self):
        """返回当前有效(新鲜且非空)的候选数，无则 0。"""
        if self._latest is None or not self._latest.grasps:
            return 0
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = (self._latest.header.stamp.sec
                 + self._latest.header.stamp.nanosec * 1e-9)
        if now - stamp > self.candidate_timeout:
            return 0
        return len(self._latest.grasps)

    # ---------- 服务 ----------
    def _on_start(self, request, response):
        if self._running:
            response.success = False
            response.message = '已在运行'
            return response
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        response.success = True
        response.message = '已启动'
        return response

    def _on_stop(self, request, response):
        self._running = False
        response.success = True
        response.message = '已请求停止'
        return response

    # ---------- 主循环（独立线程）----------
    def _loop(self):
        self._set_status('RUNNING')
        empty_count = 0
        fail_count = 0
        picked = 0

        while self._running and rclpy.ok():
            n = self._fresh_candidates()
            if n == 0:
                empty_count += 1
                self.get_logger().info(
                    f'无候选 ({empty_count}/{self.empty_threshold})')
                if empty_count >= self.empty_threshold:
                    self._set_status('DONE (料框已清空)')
                    break
                time.sleep(self.scan_interval)
                continue
            empty_count = 0

            # 触发一次取放
            self._set_status(f'PICKING (已完成 {picked})')
            ok = self._call_pick()
            if ok:
                picked += 1
                fail_count = 0
                if self.max_picks and picked >= self.max_picks:
                    self._set_status(f'DONE (达到上限 {self.max_picks})')
                    break
            else:
                fail_count += 1
                self.get_logger().warn(
                    f'取放失败 ({fail_count}/{self.max_failures})')
                if fail_count >= self.max_failures:
                    self._set_status('ABORTED (连续失败，安全停止)')
                    break
            time.sleep(self.settle_time)

        self._running = False
        self.get_logger().info(f'循环结束，共成功取放 {picked} 个')

    def _call_pick(self) -> bool:
        if not self.pick_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('/pick_place/run 不可用')
            return False
        # 异步调用 + 非阻塞等待：不占用 executor 线程，且能响应 stop / 超时，
        # 避免同步 .call() 在取放卡住时把整个循环（含停止）一起拖死。
        future = self.pick_cli.call_async(Trigger.Request())
        start = time.time()
        while rclpy.ok() and self._running and not future.done():
            if time.time() - start > self.pick_timeout:
                self.get_logger().error(f'取放调用超时({self.pick_timeout}s)')
                return False
            time.sleep(0.02)
        if not future.done():
            return False        # 被 stop 打断
        res = future.result()
        if res is None:
            return False
        if not res.success:
            self.get_logger().warn(f'取放返回失败: {res.message}')
        return res.success


def main(args=None):
    rclpy.init(args=args)
    node = PickLoop()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
