# 夹爪 URDF 集成说明（阶段3.5）

把沃姆 EPGC-50 夹爪装进 UR5 模型，让 MoveIt 知道夹爪几何（自碰撞/避障）并把
**TCP 移到指尖**（抓取规划应以指尖为工具中心点）。

## 文件
- `urdf/worm_epgc50.xacro`：夹爪 xacro 宏（可复用）。**尺寸为占位**，按实物改。
- `urdf/gripper_standalone.xacro`：独立夹爪，用于单独校验。

## 第一步：先单独校验夹爪模型
```bash
# 生成 URDF 并检查结构
xacro $(ros2 pkg prefix bin_picking_description)/share/bin_picking_description/urdf/gripper_standalone.xacro > /tmp/g.urdf
check_urdf /tmp/g.urdf        # 应打印出 base_link -> fingers -> grasp_tcp 树
```
RViz 里加 RobotModel 看夹爪形状是否合理（先把占位尺寸改成实物尺寸）。

## 第二步：填实物尺寸
编辑 `worm_epgc50.xacro` 顶部 `<xacro:property>`：
`body_len/body_w/body_h`（本体）、`finger_len`（指长）、`stroke_half`（单指行程，
EPGC-50 总行程 50mm → 每指 0.025）、`tcp_z`（TCP 距基座距离）。
有沃姆 STL 的话可把 `<box>` 换成 `<mesh>`。

## 第三步：挂到 UR5 法兰
在你的 UR5 总 xacro（基于 ur_description）里：
```xml
<xacro:include filename="$(find bin_picking_description)/urdf/worm_epgc50.xacro"/>
<!-- UR 实例化后，tool0 已存在 -->
<xacro:worm_epgc50 prefix="gripper_" parent="tool0">
  <origin xyz="0 0 0" rpy="0 0 0"/>   <!-- 法兰到夹爪基座的安装偏置，按实物量 -->
</xacro:worm_epgc50>
```

## 第四步：让 MoveIt / 抓取使用新 TCP
- 把 `grasp_params.yaml` 里 `grasp_executor` 和 `inhand_estimator` 的
  `tcp_link` 由 `tool0` 改为 **`gripper_grasp_tcp`**。
- MoveIt 的 SRDF 需把夹爪连杆加入（或重新用 MoveIt Setup Assistant 生成），
  并设置夹爪与末端连杆的 **allow collision**（避免自碰撞误报）。

## 第五步（可选）：ros2_control 控制手指关节
当前夹爪走独立 USB（`gripper_driver` 节点），URDF 里的 prismatic 关节主要用于
**可视化与碰撞**。若想在 MoveIt 里也规划手指开合，可给手指关节加 ros2_control
接口；否则保持由 `gripper_driver` 直接控制即可（推荐，简单）。

> 注意：`right_finger_joint` 用了 `mimic` 跟随左指，URDF 可视化没问题；
> 若加 ros2_control，mimic 关节需对应的 mimic 支持。
