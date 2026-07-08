"""取放执行节点：通过 move_group 的 action/service 接口驱动 UR5 完成 抓取->放置 序列。

⚠️ 2026-07 MoveIt 集成重写：不再使用 moveit_py（MoveItPy）。
   原因：moveit_py 是 MoveIt 2.7(Iron) 才引入的，Ubuntu 22.04 + Humble 的
   apt 二进制 MoveIt(2.5.x) 里没有——真机上 import 必失败、只会静默进模拟模式。
   现改为纯 rclpy 客户端，对 move_group 标准接口编程，任何 MoveIt2 版本都可用：
     自由规划+执行: /move_action           (moveit_msgs/action/MoveGroup)
     笛卡尔直线:    /compute_cartesian_path (moveit_msgs/srv/GetCartesianPath)
                    + /execute_trajectory   (moveit_msgs/action/ExecuteTrajectory)
     碰撞场景:      /apply_planning_scene   (moveit_msgs/srv/ApplyPlanningScene)
   节点本身不再需要 robot_description/SRDF 等 MoveIt 参数（都在 move_group 侧，
   见 bin_picking_bringup/launch/moveit.launch.py）。

运动规划方法（与重写前一致）：
  - 大范围自由移动（去预抓取位、去放置区）: OMPL RRTConnect（MoveIt 默认）
  - 抓取接近 / 抬起 / 放置下压 / 退回: 笛卡尔直线
  - 避障: 启动后把料框作为碰撞体加入 Planning Scene

安全限速：
  - 自由移动: MotionPlanRequest 的 max_velocity/acceleration_scaling_factor
  - 笛卡尔: humble 的 /compute_cartesian_path 无缩放字段（固定按全速做时间
    参数化），本节点对返回轨迹做时间拉伸(_slow_down)达到同样限速效果

序列:
  开夹爪 -> 自由移到预抓取位 -> 直线下到抓取位 -> 闭夹爪 -> 直线抬起
  -> (阶段4: 检视位在手重估计) -> 放置 -> 开夹爪 -> 直线退回

触发: std_srvs/Trigger 服务 /pick_place/run
TCP: 默认 gripper_grasp_tcp（指尖，见 ur5_with_gripper 组合模型）；
     MoveIt 对"固连在规划链末端之后的连杆"可直接求 IK/笛卡尔，无需改规划组。
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from bin_picking_interfaces.msg import GraspCandidateArray
from bin_picking_interfaces.srv import SetGripper, EstimateInHand
from bin_picking_grasp.geometry_utils import (
    pose_to_matrix, matrix_to_pose, transform_to_matrix, invert)

try:
    from rclpy.action import ActionClient
    from moveit_msgs.action import MoveGroup, ExecuteTrajectory
    from moveit_msgs.srv import (
        GetCartesianPath, ApplyPlanningScene, GetPositionIK)
    from moveit_msgs.msg import (
        Constraints, PositionConstraint, OrientationConstraint,
        BoundingVolume, PlanningScene, CollisionObject, MoveItErrorCodes,
        JointConstraint)
    from shape_msgs.msg import SolidPrimitive
    _HAS_MOVEIT_MSGS = True
except ImportError:  # 无 MoveIt 环境（纯逻辑测试）时进模拟模式
    _HAS_MOVEIT_MSGS = False


# 规划组 ur_manipulator 的 6 个关节（构造关节目标用）。
# 若给 UR 加了 tf_prefix，需同步带前缀（见 docs/13 [C4]）。
UR_ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


class GraspExecutor(Node):

    def __init__(self):
        super().__init__('grasp_executor')

        self.declare_parameter('planning_group', 'ur_manipulator')
        self.declare_parameter('tcp_link', 'gripper_grasp_tcp')
        # 规划组 IK 链的真正末端（UR 官方规划组 ur_manipulator 到 tool0 为止）。
        # 抓取以指尖 tcp_link 为 TCP，但 move_group 只能对求解链末端构造目标，
        # 故执行器把指尖目标换算回该 link 再发（见 _tcp_to_tip_pose）。
        self.declare_parameter('planning_tip_link', 'tool0')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('simulate', False)  # true=不连 MoveIt 只打印
        self.declare_parameter('approach_distance', 0.10)
        self.declare_parameter('lift_height', 0.15)
        self.declare_parameter('gripper_open_width', 0.05)   # EPGC-50 行程50mm
        self.declare_parameter('gripper_grasp_force', 50.0)
        self.declare_parameter('gripper_speed', 50.0)
        self.declare_parameter('cartesian_step', 0.005)       # 笛卡尔插值步长 m
        self.declare_parameter('cartesian_min_fraction', 0.9)  # 直线可行最低比例
        # 规划/执行
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('planning_attempts', 5)
        self.declare_parameter('max_velocity_scaling', 0.2)     # 真机先慢！
        self.declare_parameter('max_acceleration_scaling', 0.2)
        self.declare_parameter('goal_position_tolerance', 0.002)   # m
        self.declare_parameter('goal_orientation_tolerance', 0.02)  # rad
        self.declare_parameter('execution_timeout', 60.0)      # 单段运动最长 s
        # 放置位姿（基座系）: [x,y,z,qx,qy,qz,qw]
        # enable_inhand=false 时解释为"TCP 放置位姿"；true 时解释为"零件目标位姿"
        self.declare_parameter('place_pose', [0.4, 0.3, 0.2, 1.0, 0.0, 0.0, 0.0])
        # 阶段4 在手位姿重估计 + 放置补偿
        self.declare_parameter('enable_inhand', False)
        # 检视位（基座系 TCP 位姿，把零件举到相机前）
        self.declare_parameter('inspection_pose', [0.3, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0])
        # 工作区碰撞体（基座系）: 中心[x,y,z] 与 尺寸[dx,dy,dz]
        self.declare_parameter('bin_center', [0.5, 0.0, 0.05])
        self.declare_parameter('bin_size', [0.4, 0.3, 0.1])
        self.declare_parameter('bin_wall_thickness', 0.01)
        self.declare_parameter('add_bin_collision', True)
        # 平台模式：物件平铺在一块平台/桌面上（无料框壁）。
        #   True  -> 只建一块台面板防止夹爪下压怼穿桌子，不建四壁；
        #            此时 bin_center/bin_size 直接描述这块台板本身
        #            （零件放在其顶面 z = bin_center.z + bin_size.z/2）。
        #   False -> 料框模式：建 底 + 四壁 5 个碰撞盒（bin_center/bin_size 描述框腔）。
        self.declare_parameter('flat_platform', False)
        # 桌面/地面碰撞板：机械臂装在桌上，下方是实体桌面。不建模的话
        # move_group 以为下方是自由空间，RRTConnect 会采样出"向下绕到桌面下"
        # 的路径（仿真无害，真机危险）。建一块桌面板，规划器就会避开向下的路径。
        #   ground_z      : 桌面顶面高度(基座系 m)，UR5 装桌面上通常 = 0
        #   ground_center : 板中心 XY——默认偏前避开基座正下方，防与 base 自碰
        #   ground_size   : 板 XY 尺寸，够大以覆盖手臂可能绕下的区域
        self.declare_parameter('add_ground_plane', True)
        self.declare_parameter('ground_z', -0.02)   # 板顶面略低于基座平面防自碰
        self.declare_parameter('ground_center', [0.4, 0.0])
        self.declare_parameter('ground_size', [1.4, 1.4])
        self.declare_parameter('ground_thickness', 0.02)
        # 自由规划偶发失败(RRTConnect 随机性/时间参数化)时的自动重试次数
        self.declare_parameter('planning_retries', 2)
        # 方案C：自由移动前先对目标位姿求 IK(以当前姿态为 seed、避碰)，用"关节目标"
        # 规划，而非把位姿目标丢给 move_group 随机选 IK 解。可根除别扭姿态/大幅绕路。
        # IK 求解失败时自动退回位姿目标（保底）。设 false 则回到纯位姿目标模式。
        self.declare_parameter('use_joint_goal', True)

        gp = self.get_parameter
        self.group = gp('planning_group').value
        self.tcp_link = gp('tcp_link').value
        self.plan_tip = gp('planning_tip_link').value
        self.base_frame = gp('base_frame').value
        self.approach_distance = gp('approach_distance').value
        self.lift_height = gp('lift_height').value
        self.open_width = gp('gripper_open_width').value
        self.grasp_force = gp('gripper_grasp_force').value
        self.gripper_speed = gp('gripper_speed').value
        self.cart_step = gp('cartesian_step').value
        self.cart_min_fraction = gp('cartesian_min_fraction').value
        self.planning_time = gp('planning_time').value
        self.planning_attempts = gp('planning_attempts').value
        self.vel_scale = gp('max_velocity_scaling').value
        self.acc_scale = gp('max_acceleration_scaling').value
        self.pos_tol = gp('goal_position_tolerance').value
        self.ori_tol = gp('goal_orientation_tolerance').value
        self.exec_timeout = gp('execution_timeout').value
        self.place_pose = list(gp('place_pose').value)
        self.enable_inhand = gp('enable_inhand').value
        self.inspection_pose = list(gp('inspection_pose').value)
        self.bin_center = list(gp('bin_center').value)
        self.bin_size = list(gp('bin_size').value)
        self.bin_wall = gp('bin_wall_thickness').value
        self.add_bin_collision = gp('add_bin_collision').value
        self.flat_platform = gp('flat_platform').value
        self.add_ground = gp('add_ground_plane').value
        self.ground_z = gp('ground_z').value
        self.ground_center = list(gp('ground_center').value)
        self.ground_size = list(gp('ground_size').value)
        self.ground_thickness = gp('ground_thickness').value
        self.planning_retries = int(gp('planning_retries').value)
        self.use_joint_goal = gp('use_joint_goal').value

        self.simulate = gp('simulate').value or not _HAS_MOVEIT_MSGS
        if not _HAS_MOVEIT_MSGS:
            self.get_logger().warn('未找到 moveit_msgs，执行器进入模拟(只打印)模式')

        self.cb_group = ReentrantCallbackGroup()
        self._latest = None

        # MoveIt 客户端（全部指向 move_group 节点的标准接口）
        if not self.simulate:
            self.move_cli = ActionClient(
                self, MoveGroup, '/move_action', callback_group=self.cb_group)
            self.exec_cli = ActionClient(
                self, ExecuteTrajectory, '/execute_trajectory',
                callback_group=self.cb_group)
            self.cart_cli = self.create_client(
                GetCartesianPath, '/compute_cartesian_path',
                callback_group=self.cb_group)
            self.scene_cli = self.create_client(
                ApplyPlanningScene, '/apply_planning_scene',
                callback_group=self.cb_group)
            # 方案C：自由移动前对目标位姿求 IK（用关节目标规划，避免别扭姿态）
            self.ik_cli = self.create_client(
                GetPositionIK, '/compute_ik', callback_group=self.cb_group)
            # 指尖 TCP -> 规划链末端 的换算靠 TF（查一次后缓存到 _T_tip_tcp）。
            # spin_thread=False：本节点已在 main() 的 MultiThreadedExecutor 中，
            # /tf 订阅回调由它处理；若 True 会另起 executor 抢 spin 同一节点而报错。
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(
                self.tf_buffer, self, spin_thread=False)
            self._T_tip_tcp = None
            self.get_logger().info(
                f'MoveIt 客户端已就绪, group={self.group}, tcp={self.tcp_link}, '
                f'plan_tip={self.plan_tip}, vel_scale={self.vel_scale}')
            if self.add_bin_collision:
                # move_group 可能晚于本节点启动：定时重试直到场景写入成功
                self._scene_tries = 0
                self._scene_timer = self.create_timer(
                    2.0, self._try_setup_scene, callback_group=self.cb_group)
        else:
            self.move_cli = self.exec_cli = None
            self.cart_cli = self.scene_cli = None

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

    # ---------- 同步等待工具（MultiThreadedExecutor 下非阻塞轮询） ----------
    def _wait_future(self, future, timeout_sec: float):
        """等 future 完成；不在回调里 spin（依赖执行器其它线程），超时返回 None。"""
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > timeout_sec:
                return None
            time.sleep(0.005)
        return future.result()

    def _call_service(self, client, req, timeout_sec=5.0):
        future = client.call_async(req)
        res = self._wait_future(future, timeout_sec)
        if res is None:
            self.get_logger().warn('服务调用超时')
        return res

    def _send_action_goal(self, client, goal, timeout_sec: float, name=''):
        """发送 action 目标并等结果；失败/超时/被拒绝返回 None。"""
        if not client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                f'action 服务器不可用: {name}（move_group 在跑吗？）')
            return None
        handle = self._wait_future(client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            self.get_logger().error('action 目标被拒绝/发送超时')
            return None
        wrapped = self._wait_future(handle.get_result_async(), timeout_sec)
        if wrapped is None:
            self.get_logger().error('action 执行超时，尝试取消')
            handle.cancel_goal_async()
            return None
        return wrapped.result

    # ---------- 碰撞场景 ----------
    def _try_setup_scene(self):
        self._scene_tries += 1
        if self._scene_tries > 30:  # ~60s 放弃
            self.get_logger().warn('料框碰撞场景写入放弃（/apply_planning_scene 一直不可用）')
            self._scene_timer.cancel()
            return
        if not self.scene_cli.service_is_ready():
            return
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = self._make_bin_collision_objects()
        req = ApplyPlanningScene.Request(scene=scene)
        res = self._call_service(self.scene_cli, req, timeout_sec=3.0)
        if res is not None and res.success:
            self.get_logger().info(
                f'已加入 {len(scene.world.collision_objects)} 个料框碰撞体')
            self._scene_timer.cancel()

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
        if self.flat_platform:
            # 平台模式：只建一块台面板（bin_center/bin_size 即这块板本身），
            # 防止夹爪下压怼穿桌面；不建四壁，俯视抓取不受阻。
            objs = [self._box('work_platform', cx, cy, cz, dx, dy, dz)]
        else:
            t = self.bin_wall
            half_x, half_y = dx / 2, dy / 2
            objs = [
                self._box('bin_bottom', cx, cy, cz - dz / 2 + t / 2, dx, dy, t),
                self._box('bin_wall_xp', cx + half_x, cy, cz, t, dy, dz),
                self._box('bin_wall_xn', cx - half_x, cy, cz, t, dy, dz),
                self._box('bin_wall_yp', cx, cy + half_y, cz, dx, t, dz),
                self._box('bin_wall_yn', cx, cy - half_y, cz, dx, t, dz),
            ]
        if self.add_ground:
            # 桌面板：顶面在 ground_z，板体向下延伸，挡住机械臂向下绕的路径。
            gcx, gcy = self.ground_center
            gdx, gdy = self.ground_size
            gt = self.ground_thickness
            objs.append(self._box(
                'ground_plane', gcx, gcy, self.ground_z - gt / 2, gdx, gdy, gt))
        return objs

    # ---------- 基础动作 ----------
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

    def _tip_to_tcp_matrix(self):
        """规划链末端(plan_tip) -> 指尖 TCP(tcp_link) 的固定变换(4x4)，查一次缓存。
        两者都固连在机械臂上，关系恒定；靠 TF 拿到、不硬编码尺寸。"""
        if self._T_tip_tcp is not None:
            return self._T_tip_tcp
        if self.tcp_link == self.plan_tip:
            self._T_tip_tcp = np.eye(4)
            return self._T_tip_tcp
        start = time.time()
        while rclpy.ok() and time.time() - start < 5.0:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.plan_tip, self.tcp_link, Time())
                self._T_tip_tcp = transform_to_matrix(tf.transform)
                self.get_logger().info(
                    f'TCP 换算就绪: {self.plan_tip} -> {self.tcp_link}')
                return self._T_tip_tcp
            except Exception:  # noqa: BLE001  TF 尚未就绪，短暂重试
                time.sleep(0.05)
        self.get_logger().error(
            f'查不到 TF {self.plan_tip}->{self.tcp_link}，无法换算 TCP 目标')
        return None

    def _tcp_to_tip_pose(self, pose: PoseStamped):
        """把"指尖 TCP 目标位姿"换算成"规划链末端(tool0)目标位姿"。
        move_group 只会对规划组 IK 链末端构造目标，直接用指尖会报
        'Unable to construct goal representation'。返回 None 表示换算失败。
          T_base_tip = T_base_tcp * inv(T_tip_tcp)
        """
        if self.tcp_link == self.plan_tip:
            return pose
        T_tip_tcp = self._tip_to_tcp_matrix()
        if T_tip_tcp is None:
            return None
        T_base_tip = pose_to_matrix(pose.pose) @ invert(T_tip_tcp)
        out = PoseStamped()
        out.header = pose.header
        out.pose = matrix_to_pose(T_base_tip)
        return out

    def _compute_ik(self, tip_pose: PoseStamped):
        """对规划链末端(plan_tip)位姿求 IK，以 move_group 当前姿态为 seed
        （数值 IK 倾向收敛到最接近当前的解 → 路径自然、不别扭）、并避开碰撞。
        返回 {joint_name: position} 或 None（失败）。"""
        if not self.ik_cli.service_is_ready() and \
                not self.ik_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('/compute_ik 不可用，退回位姿目标')
            return None
        req = GetPositionIK.Request()
        ikr = req.ik_request
        ikr.group_name = self.group
        ikr.ik_link_name = self.plan_tip
        ikr.pose_stamped = tip_pose
        ikr.robot_state.is_diff = True     # 用当前姿态作 seed（求最近解）
        ikr.avoid_collisions = True        # IK 解本身也避开桌面/料框
        ikr.timeout.sec = 1
        res = self._call_service(self.ik_cli, req, timeout_sec=5.0)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            code = None if res is None else res.error_code.val
            self.get_logger().warn(f'IK 求解失败 (error_code={code})，退回位姿目标')
            return None
        js = res.solution.joint_state
        return dict(zip(js.name, js.position))

    def _joint_goal_constraints(self, joint_map) -> 'Constraints':
        """由 IK 解的关节值构造关节目标（只约束规划组 6 个臂关节）。"""
        c = Constraints()
        for name in UR_ARM_JOINTS:
            if name not in joint_map:
                continue
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(joint_map[name])
            jc.tolerance_above = 1e-3
            jc.tolerance_below = 1e-3
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    def _pose_goal_constraints(self, pose: PoseStamped) -> 'Constraints':
        """把 TCP 位姿目标转成 MoveGroup 的 goal_constraints
        （与 MoveGroupInterface::setPoseTarget 等价的手工构造）。"""
        c = Constraints()

        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = self.plan_tip
        pc.target_point_offset = Vector3()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [max(self.pos_tol, 1e-4)]
        region_pose = Pose()
        region_pose.position = pose.pose.position
        region_pose.orientation.w = 1.0
        bv = BoundingVolume()
        bv.primitives = [sphere]
        bv.primitive_poses = [region_pose]
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints = [pc]

        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = self.plan_tip
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = self.ori_tol
        oc.absolute_y_axis_tolerance = self.ori_tol
        oc.absolute_z_axis_tolerance = self.ori_tol
        oc.weight = 1.0
        c.orientation_constraints = [oc]
        return c

    def move_free(self, pose: PoseStamped) -> bool:
        """自由空间规划(RRTConnect) + 执行（MoveGroup action, plan_and_execute）。"""
        if self.simulate:
            self.get_logger().info(
                f'[模拟] 自由移动 -> ({pose.pose.position.x:.3f}, '
                f'{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})')
            return True
        tip_pose = self._tcp_to_tip_pose(pose)
        if tip_pose is None:
            self.get_logger().error('TCP->规划末端 换算失败，放弃自由规划')
            return False
        # 方案C：先求 IK 用关节目标（避免 move_group 随机选到别扭 IK 解）；
        # IK 失败自动退回位姿目标。
        goal_constraints = None
        if self.use_joint_goal:
            joint_map = self._compute_ik(tip_pose)
            if joint_map:
                goal_constraints = self._joint_goal_constraints(joint_map)
        if goal_constraints is None:
            goal_constraints = self._pose_goal_constraints(tip_pose)
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.group
        req.allowed_planning_time = self.planning_time
        req.num_planning_attempts = int(self.planning_attempts)
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        req.start_state.is_diff = True          # 从当前状态出发
        req.goal_constraints = [goal_constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        # RRTConnect 随机采样 + 时间参数化偶发失败(常见 error_code=-2)，自动重试
        for attempt in range(1, self.planning_retries + 2):
            result = self._send_action_goal(
                self.move_cli, goal, self.planning_time + self.exec_timeout,
                name='/move_action')
            if result is not None and \
                    result.error_code.val == MoveItErrorCodes.SUCCESS:
                return True
            code = None if result is None else result.error_code.val
            self.get_logger().warn(
                f'自由规划/执行失败 (error_code={code})，'
                f'尝试 {attempt}/{self.planning_retries + 1}')
        self.get_logger().error('自由规划/执行重试后仍失败')
        return False

    @staticmethod
    def _slow_down_trajectory(traj, scale: float):
        """时间拉伸限速：t/=scale, v*=scale, a*=scale^2。
        humble 的笛卡尔服务按全速(1.0)做时间参数化，无请求端缩放字段，
        故在客户端等效降速。scale∈(0,1]。"""
        if scale >= 0.999:
            return traj
        for pt in traj.joint_trajectory.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            t /= scale
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            pt.velocities = [v * scale for v in pt.velocities]
            pt.accelerations = [a * scale * scale for a in pt.accelerations]
        return traj

    def _execute_trajectory(self, traj) -> bool:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj
        result = self._send_action_goal(
            self.exec_cli, goal, self.exec_timeout, name='/execute_trajectory')
        if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = None if result is None else result.error_code.val
            self.get_logger().error(f'轨迹执行失败 (error_code={code})')
            return False
        return True

    def move_cartesian(self, target: PoseStamped) -> bool:
        """笛卡尔直线移动到目标位姿（从当前 TCP 到 target 走直线）。"""
        if self.simulate:
            self.get_logger().info(
                f'[模拟] 直线移动 -> ({target.pose.position.x:.3f}, '
                f'{target.pose.position.y:.3f}, {target.pose.position.z:.3f})')
            return True
        if not self.cart_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('compute_cartesian_path 不可用，退化为自由规划')
            return self.move_free(target)

        tip_target = self._tcp_to_tip_pose(target)
        if tip_target is None:
            self.get_logger().warn('TCP->规划末端 换算失败，退化为自由规划')
            return self.move_free(target)

        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.group_name = self.group
        req.link_name = self.plan_tip
        req.start_state.is_diff = True          # 从当前状态出发
        req.max_step = self.cart_step
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.waypoints = [tip_target.pose]

        res = self._call_service(self.cart_cli, req, timeout_sec=10.0)
        if res is None or res.fraction < self.cart_min_fraction:
            frac = -1.0 if res is None else res.fraction
            self.get_logger().warn(
                f'笛卡尔直线只完成 {frac:.2f}，退化为自由规划')
            return self.move_free(target)

        traj = self._slow_down_trajectory(res.solution, self.vel_scale)
        return self._execute_trajectory(traj)

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
