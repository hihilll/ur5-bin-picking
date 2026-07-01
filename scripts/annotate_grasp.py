"""CAD 抓取点标注工具（阶段2.3）。可直接在 Windows 上跑（只需 open3d）。

作用：在零件 CAD 上**可视化点选**若干抓取中心点，自动按表面法线生成抓取位姿，
输出可直接粘贴到 `grasp_params.yaml` 的 `grasp_annotations`（扁平
[x,y,z,roll,pitch,yaw,width] 每 7 个一组，单位与感知一致：米、弧度）。

约定（与本项目一致）:
  - 抓取位姿在 **CAD 模型坐标系** 下（grasp_planner 会左乘零件位姿转到基座系）。
  - 夹爪**接近轴 = 抓取坐标系 z 轴**；这里取为 **-表面法线**（从外向内接近）。
  - 欧拉角为 XYZ 固定轴(Rz@Ry@Rx)，与 geometry_utils.euler_to_matrix 一致。

依赖: pip install open3d numpy

用法:
  python scripts/annotate_grasp.py --model path/to/part.stl --scale 0.001 --width 0.03
操作:
  弹窗后按住 **Shift + 左键** 点选抓取中心点（可点多个），选完**关闭窗口**。
  终端会打印生成的 grasp_annotations 列表。
"""

from __future__ import annotations

import argparse
import math

import numpy as np
import open3d as o3d


def matrix_to_euler_xyz(R: np.ndarray):
    """R = Rz(yaw)@Ry(pitch)@Rx(roll) 的逆解 -> (roll, pitch, yaw)。"""
    sp = -R[2, 0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(sp) < 0.99999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # 万向锁附近
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def rotation_from_approach(approach: np.ndarray) -> np.ndarray:
    """由接近轴(将成为抓取系 z 轴)构造一个合理的正交旋转矩阵。

    绕接近轴的自转(夹爪开合朝向)是任意的，取一个稳定的正交基；
    用户可后续在 yaml 里微调 yaw。
    """
    z = approach / (np.linalg.norm(approach) + 1e-12)
    ref = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(ref, z)) > 0.95:      # 接近轴与世界 z 太平行则换参考
        ref = np.array([1.0, 0.0, 0.0])
    x = np.cross(ref, z)
    x /= (np.linalg.norm(x) + 1e-12)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True, help='CAD 路径 .stl/.obj/.ply')
    ap.add_argument('--scale', type=float, default=0.001,
                    help='单位换算(mm->m=0.001)')
    ap.add_argument('--width', type=float, default=0.03,
                    help='该批抓取的夹爪开口宽度(m)')
    ap.add_argument('--points', type=int, default=30000,
                    help='采样点数（点选用）')
    args = ap.parse_args()

    mesh = o3d.io.read_triangle_mesh(args.model)
    if not mesh.has_vertices():
        raise SystemExit(f'无法读取 CAD: {args.model}')
    mesh.scale(args.scale, center=(0, 0, 0))
    mesh.compute_vertex_normals()

    pcd = mesh.sample_points_poisson_disk(args.points)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=args.scale * 20, max_nn=30))

    # 法线朝外（从质心指向外部），便于取 -法线 作为接近轴
    center = pcd.get_center()
    pts = np.asarray(pcd.points)
    nrm = np.asarray(pcd.normals)
    flip = np.einsum('ij,ij->i', nrm, pts - center) < 0
    nrm[flip] = -nrm[flip]
    pcd.normals = o3d.utility.Vector3dVector(nrm)

    print('=== 操作说明 ===')
    print('  Shift + 左键 点选抓取中心点（可多选）')
    print('  选完直接关闭窗口，终端输出标注结果\n')

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name='点选抓取中心 (Shift+左键)')
    vis.add_geometry(pcd)
    vis.run()        # 阻塞，用户点选后关闭
    vis.destroy_window()
    picked = vis.get_picked_points()

    if not picked:
        print('未选择任何点。（记得用 Shift+左键 点选）')
        return

    flat = []
    print(f'\n已选 {len(picked)} 个抓取点：')
    for i, idx in enumerate(picked):
        p = pts[idx]
        approach = -nrm[idx]                     # 从外向内接近
        R = rotation_from_approach(approach)
        roll, pitch, yaw = matrix_to_euler_xyz(R)
        flat.extend([round(float(p[0]), 5), round(float(p[1]), 5),
                     round(float(p[2]), 5), round(roll, 5), round(pitch, 5),
                     round(yaw, 5), float(args.width)])
        print(f'  [{i}] pos=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f}) '
              f'rpy=({roll:.3f},{pitch:.3f},{yaw:.3f})')

    print('\n=== 粘贴到 grasp_params.yaml 的 grasp_planner.ros__parameters ===')
    print('    grasp_annotations: ' + str(flat))
    print('\n提示：绕接近轴的自转(yaw)是任意取的，若夹爪开合方向不理想可手动调 yaw。')


if __name__ == '__main__':
    main()
