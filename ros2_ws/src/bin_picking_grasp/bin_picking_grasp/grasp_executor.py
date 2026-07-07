"""取放执行节点：用 MoveIt2(moveit_py) 驱动 UR5 完成 抓取->放置 序列。

运动规划方法（阶段3补全）：
  - 大范围自由移动（去预抓取位、去放置区）: OMPL RRTConnect（MoveIt 默认）
  - 抓取接近 / 抬起 / 放置下压 / 退回: **笛卡尔直线**（/compute_cartesian_path）
  - 避障: 启动时把料框/工作台作为碰撞体加入 Planning Scene

序列:
  开夹爪 -> 自由移到预抓取位 -> 直线下到抓取位 -> 闭夹爪 -> 直线抬起
  -> 自由移到放置预备位 -> 直线下到放置位 -> 开夹爪 -> 直线退回 -> (可回 home)

触发: std_srvs/Trigger 服务 /pick_place/run

⚠️ 按你的 MoveIt 配置调整: PLANNING_GROUP('ur_manipulator')、TCP_LINK('tool0')、料框尺寸/位姿。
阶段四会在"闭夹爪后"插入"在手位姿重估计 + 补偿"。
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger

from bin_picking_interfaces.msg import GraspCandidateArray
from bin_picking_interfaces.srv import SetGripper, EstimateInHand
from bin_picking_grasp.geometry_utils import (
    pose_to_matrix, matrix_to_pose, transform_to_matrix, invert)

try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_trajectory import RobotTrajectory
    from moveit_msgs.srv import GetCartesianPath
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    _HAS_MOVEIT = True
except ImportError:
    _HAS_MOVEIT = False


class GraspExecutor(Node):

    def __init__(self):
        super().__init__('grasp_executor')

        self.declare_parameter('planning_group', 'ur_manipulator')
        self.declare_parameter('tcp_link', 'tool0')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('approach_distance', 0.10)
        self.declare_parameter('lift_height', 0.15)
        self.declare_parameter('gripper_open_width', 0.05)   # EPGC-50 行程50mm
        self.declare_parameter('gripper_grasp_force', 50.0)
        self.declare_parameter('gripper_speed', 50.0)
        self.declare_parameter('cartesian_step', 0.005)       # 笛卡尔插值步长 m
        self.declare_parameter('cartesian_min_fraction', 0.9)  # 直线可行最低比例
        # 放置位姿（基座系）: [x,y,z,qx,qy,qz,qw]
        # enable_inhand=false 时解释为"TCP 放置位姿"；true 时解释为"零件目标位姿"
        self.declare_parameter('place_pose', [0.4, 0.3, 0.2, 1.0, 0.0, 0.0, 0.0])
        # 阶段4 在手位姿重估计 + 放置补偿
        self.declare_parameter('enable_inhand', False)
        # 检视位（基座系 TCP 位姿，把零件举到相机前）
        self.declare_parameter('inspection_pose', [0.3, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0])
        # 料框碰撞盒（基座系）: 中心[x,y,z] 与 尺寸[dx,dy,dz]
        self.declare_parameter('bin_center', [0.5, 0.0, 0.05])
        self.declare_parameter('bin_size', [0.4, 0.3, 0.1])
        self.declare_parameter('bin_wall_thickness', 0.01)
        self.declare_parameter('add_bin_collision', True)

        self.group = self.get_parameter('planning_group').value
        self.tcp_link = self.get_parameter('tcp_link').value
        self.base_frame = self.get_parameter('base_frame').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.lift_height = self.get_parameter('lift_height').value
        self.open_width = self.get_parameter('gripper_open_width').value
        self.grasp_force = self.get_parameter('gripper_grasp_force').value
        self.gripper_speed = self.get_parameter('gripper_speed').value
        self.cart_step = self.get_parameter('cartesian_step').value
        self.cart_min_fraction = self.get_parameter('cartesian_min_fraction').value
        self.place_pose = list(self.get_parameter('place_pose').value)
        self.enable_inhand = self.get_parameter('enable_inhand').value
        self.inspection_pose = list(self.get_parameter('inspection_pose').value)
        self.bin_center = list(self.get_parameter('bin_center').value)
        self.bin_size = list(self.get_parameter('bin_size').value)
        self.bin_wall = self.get_parameter('bin_wall_thickness').value
        self.add_bin_collision = self.get_parameter('add_bin_collision').value

        self.cb_group = ReentrantCallbackGroup()
        self._latest = None

        # MoveIt
        if _HAS_MOVEIT:
            self.moveit = MoveItPy(node_name='grasp_executor_moveit')
            self.arm = self.moveit.get_planning_component(self.group)
            self.robot_model = self.moveit.get_robot_model()
            self.psm = self.moveit.get_planning_scene_monitor()
            self.cart_cli = self.create_client(
                GetCartesianPath, '/compute_cartesian_path',
                callback_group=self.cb_group)
            self.get_logger().info(f'MoveItPy 已初始化, group={self.group}')
            if self.add_bin_collision:
                self.setup_planning_scene()
        else:
            self.moveit = None
            self.arm = None
            self.cart_cli = None
            self.get_logger().warn('未找到 moveit_py，执行器进入仿真(只打印)模式')

        # 夹爪客户端
        self.gripper_cli = self.create_client(
            SetGripper, '/gripper/set_gripper', callback_group=self.cb_group)
        # 在手位姿重估计客户端（阶段4）
        self.inhand_cli = self.create_client(
            EstimateInHand, '/estimate_inhand', callback_group=self.cb_group)

        self.sub = self.create_subscription(
            GraspCandidateArray, '/grasp_candidates', self.on_grasps, 10,
            callback_group=self.cb_group)
        self.srv = self.create_service(
            Trigger, '/pick_place/run', self.on_run,
            callback_group=self.cb_group)

        self.get_logger().info('取放执行器已就绪，调用 /pick_place/run 触发一次')

    def on_grasps(self, msg: GraspCandidateArray):
        self._latest = msg

    # ---------- 碰撞场景 ----------
    def setup_planning_scene(self):
        """把料框近似为四面薄壁 + 底板，加入 Planning Scene 做避障。"""
        try:
            objs = self._make_bin_collision_objects()
            with self.psm.read_write() as scene:
                for obj in objs:
                    scene.apply_collision_object(obj)
                scene.current_state.update()
            self.get_logger().info(f'已加入 {len(objs)} 个料框碰撞体')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'加入碰撞场景失败(可忽略继续): {e}')

    def _box(self, name, cx, cy, cz, dx, dy, dz) -> 'CollisionObject':
        obj = CollisionObject()
        obj.header.frame_id = self.base_frame
        obj.id = name
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [dx, dy, dz]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = cx, cy, cz
        pose.orientation.w = 1.0
        obj.primitives = [prim]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def _make_bin_collision_objects(self):
        cx, cy, cz = self.bin_center
        dx, dy, dz = self.bin_size
        t = self.bin_wall
        half_x, half_y = dx / 2, dy / 2
        objs = [
            self._box('bin_bottom', cx, cy, cz - dz / 2 + t / 2, dx, dy, t),
            self._box('bin_wall_xp', cx + half_x, cy, cz, t, dy, dz),
            self._box('bin_wall_xn', cx - half_x, cy, cz, t, dy, dz),
            self._box('bin_wall_yp', cx, cy + half_y, cz, dx, t, dz),
            self._box('bin_wall_yn', cx, cy - half_y, cz, dx, t, dz),
        ]
        return objs

    # ---------- 基础动作 ----------
    def _call_service(self, client, req, timeout_sec=5.0):
        """在 MultiThreadedExecutor + ReentrantCallbackGroup 下安全地同步调用服务。

        用 call_async + 非阻塞轮询等待，**不在回调里对本节点再 spin**——
        spin_until_future_complete 对已加入 executor 的节点会报错/死锁。
        依赖执行器的其它线程完成 future（本项目 main 里用 MultiThreadedExecutor）。
        """
        future = client.call_async(req)
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > timeout_sec:
                self.get_logger().warn('服务调用超时')
                return None
            time.sleep(0.005)
        return future.result()

    def set_gripper(self, width, force=0.0, speed=None):
        if not self.gripper_cli.service_is_ready():
            self.get_logger().warn('夹爪服务未就绪，跳过')
            return False
        req = SetGripper.Request()
        req.width = float(width)
        req.force = float(force)
        req.speed = float(self.gripper_speed if speed is None else speed)
        res = self._call_service(self.gripper_cli, req, timeout_sec=5.0)
        return res is not None and res.success

    def move_free(self, pose: PoseStamped) -> bool:
        """自由空间规划(RRTConnect)移动到位姿。"""
        if self.arm is None:
            self.get_logger().info(
                f'[模拟] 自由移动 -> ({pose.pose.position.x:.3f}, '
                f'{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})')
            return True
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=pose, pose_link=self.tcp_link)
        result = self.arm.plan()
        if not result:
            self.get_logger().error('自由规划失败')
            return False
        self.moveit.execute(result.trajectory, controllers=[])
        return True

    def move_cartesian(self, target: PoseStamped) -> bool:
        """笛卡尔直线移动到目标位姿（从当前 TCP 到 target 走直线）。"""
        if self.arm is None:
            self.get_logger().info(
                f'[模拟] 直线移动 -> ({target.pose.position.x:.3f}, '
                f'{target.pose.position.y:.3f}, {target.pose.position.z:.3f})')
            return True
        if not self.cart_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('compute_cartesian_path 不可用，退化为自由规划')
            return self.move_free(target)

        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.group_name = self.group
        req.link_name = self.tcp_link
        req.max_step = self.cart_step
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.waypoints = [target.pose]
        # 起始状态用当前状态。此 API 对 MoveIt 版本敏感，失败则留空，
        # move_group 会自动用其当前状态作为起点（兜底不致命）。
        try:
            from moveit.core.robot_state import robotStateToRobotStateMsg
            with self.psm.read_only() as scene:
                req.start_state = robotStateToRobotStateMsg(scene.current_state)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(
                f'设置起始状态失败，改用 move_group 当前状态兜底: {e}')

        res = self._call_service(self.cart_cli, req, timeout_sec=10.0)
        if res is None or res.fraction < self.cart_min_fraction:
            frac = -1 if res is None else res.fraction
            self.get_logger().warn(
                f'笛卡尔直线只完成 {frac:.2f}，退化为自由规划')
            return self.move_free(target)

        # 把 RobotTrajectory msg 转成 moveit core 轨迹并执行
        with self.psm.read_only() as scene:
            rt = RobotTrajectory(self.robot_model)
            rt.set_robot_trajectory_msg(scene.current_state, res.solution)
        self.moveit.execute(rt, controllers=[])
        return True

    @staticmethod
    def _offset_along_approach(pose: PoseStamped, distance: float) -> PoseStamped:
        """沿夹爪接近轴(z)回退 distance（生成预抓取位）。"""
        T = pose_to_matrix(pose.pose)
        T[:3, 3] -= T[:3, 2] * distance
        out = PoseStamped()
        out.header = pose.header
        out.pose = matrix_to_pose(T)
        return out

    def _lift(self, pose: PoseStamped, height: float) -> PoseStamped:
        out = PoseStamped()
        out.header = pose.header
        out.pose = matrix_to_pose(pose_to_matrix(pose.pose))
        out.pose.position.z += height
        return out

    def _pose_from_list(self, lst) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        x, y, z, qx, qy, qz, qw = lst
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    def _make_place_pose(self) -> PoseStamped:
        return self._pose_from_list(self.place_pose)

    # ---------- 阶段4：在手位姿重估计 + 放置补偿 ----------
    def estimate_inhand(self):
        """调用 /estimate_inhand，返回 T_tcp_part(4x4) 或 None。"""
        if not self.inhand_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('在手估计服务不可用')
            return None
        req = EstimateInHand.Request()
        res = self._call_service(self.inhand_cli, req, timeout_sec=5.0)
        if res is None or not res.success:
            msg = '无返回' if res is None else res.message
            self.get_logger().warn(f'在手估计失败: {msg}')
            return None
        self.get_logger().info(f'在手估计: {res.message}')
        return transform_to_matrix(res.gripper_to_part)

    def corrected_place_pose(self, T_tcp_part) -> PoseStamped:
        """由零件目标位姿 + 在手位姿，反算 TCP 放置位姿。

        想让零件落到 place_pose(目标零件位姿):
          T_base_part_target = T_base_tcp_place * T_tcp_part
          => T_base_tcp_place = T_base_part_target * inv(T_tcp_part)
        """
        T_base_part_target = pose_to_matrix(self._make_place_pose().pose)
        T_base_tcp_place = T_base_part_target @ invert(T_tcp_part)
        out = PoseStamped()
        out.header.frame_id = self.base_frame
        out.pose = matrix_to_pose(T_base_tcp_place)
        return out

    # ---------- 主流程 ----------
    def _do(self, name, ok) -> bool:
        self.get_logger().info(f'执行: {name}')
        if not ok:
            self.get_logger().error(f'步骤失败: {name}')
        return ok

    def on_run(self, request, response):
        if self._latest is None or not self._latest.grasps:
            response.success = False
            response.message = '没有可用抓取候选'
            return response

        grasp = self._latest.grasps[0]
        grasp_pose = grasp.grasp_pose
        pre_grasp = self._offset_along_approach(grasp_pose, self.approach_distance)
        lift_pose = self._lift(grasp_pose, self.lift_height)

        def fail(step):
            response.success = False
            response.message = f'步骤失败: {step}'
            return response

        # --- 抓取 ---
        if not self._do('开夹爪', self.set_gripper(self.open_width)):
            return fail('开夹爪')
        if not self._do('自由移到预抓取位', self.move_free(pre_grasp)):
            return fail('自由移到预抓取位')
        if not self._do('直线下到抓取位', self.move_cartesian(grasp_pose)):
            return fail('直线下到抓取位')
        if not self._do('闭合夹爪', self.set_gripper(grasp.width, self.grasp_force)):
            return fail('闭合夹爪')
        if not self._do('直线抬起', self.move_cartesian(lift_pose)):
            return fail('直线抬起')

        # --- 阶段4：在手位姿重估计 + 放置补偿 ---
        if self.enable_inhand:
            insp = self._pose_from_list(self.inspection_pose)
            if not self._do('移到检视位', self.move_free(insp)):
                return fail('移到检视位')
            T_tcp_part = self.estimate_inhand()
            if T_tcp_part is None:
                self.get_logger().warn('在手估计失败，退化为无补偿放置')
                place_pose = self._make_place_pose()
            else:
                place_pose = self.corrected_place_pose(T_tcp_part)
        else:
            place_pose = self._make_place_pose()

        pre_place = self._lift(place_pose, self.lift_height)

        # --- 放置 ---
        if not self._do('自由移到放置预备位', self.move_free(pre_place)):
            return fail('自由移到放置预备位')
        if not self._do('直线下到放置位', self.move_cartesian(place_pose)):
            return fail('直线下到放置位')
        if not self._do('松开夹爪', self.set_gripper(self.open_width)):
            return fail('松开夹爪')
        if not self._do('直线退回', self.move_cartesian(pre_place)):
            return fail('直线退回')

        response.success = True
        response.message = '取放完成'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GraspExecutor()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
