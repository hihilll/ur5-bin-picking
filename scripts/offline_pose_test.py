"""离线感知测试：不依赖 ROS，验证 PPF/ICP 配准算法。可直接在 Windows 上跑。

依赖: pip install open3d numpy

两种用法:

1) 合成测试（只给 CAD，自动造一个带噪声/杂物的场景再配准，已知真值可算误差）:
   python scripts/offline_pose_test.py --model path/to/part.stl --scale 0.001

2) 真实测试（给 CAD + 真实采集的场景点云）:
   python scripts/offline_pose_test.py --model part.stl --scene scene.ply --scale 0.001

加 --no-vis 可关闭可视化窗口（仅打印指标）。
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import open3d as o3d

# 复用感知节点的配准逻辑（该模块只依赖 numpy/open3d，不依赖 ROS）
_PKG = os.path.join(os.path.dirname(__file__), '..', 'ros2_ws', 'src',
                    'bin_picking_perception')
sys.path.insert(0, os.path.abspath(_PKG))
from bin_picking_perception import pose_estimation as pe  # noqa: E402


def load_model(path, scale, num_points=20000):
    mesh = o3d.io.read_triangle_mesh(path)
    if not mesh.has_vertices():
        raise SystemExit(f'无法读取 CAD: {path}')
    mesh.scale(scale, center=(0, 0, 0))
    mesh.compute_vertex_normals()
    return mesh.sample_points_poisson_disk(num_points)


def random_transform(max_t=0.05):
    """随机生成一个 4x4 真值位姿。"""
    R = o3d.geometry.get_rotation_matrix_from_xyz(
        np.random.uniform(-np.pi, np.pi, 3))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.random.uniform(-max_t, max_t, 3)
    return T


def make_synthetic_scene(model, noise_sigma=0.001, clutter=True):
    """把模型变换到随机位姿 + 加噪声 + 加一些杂物点，模拟场景。"""
    gt = random_transform()
    scene = o3d.geometry.PointCloud(model).transform(gt)  # 拷贝后变换，不改原模型
    pts = np.asarray(scene.points)
    pts += np.random.normal(0, noise_sigma, pts.shape)
    if clutter:
        n = len(pts) // 3
        extra = np.random.uniform(pts.min(0) - 0.05, pts.max(0) + 0.05, (n, 3))
        pts = np.vstack([pts, extra])
    scene.points = o3d.utility.Vector3dVector(pts)
    return scene, gt


def pose_error(T_est, T_gt):
    """平移误差(mm) 与 旋转误差(deg)。"""
    dt = np.linalg.norm(T_est[:3, 3] - T_gt[:3, 3]) * 1000.0
    R = T_est[:3, :3] @ T_gt[:3, :3].T
    cos = np.clip((np.trace(R) - 1) / 2, -1, 1)
    dr = np.degrees(np.arccos(cos))
    return dt, dr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True, help='CAD 路径 .stl/.obj/.ply')
    ap.add_argument('--scene', default='', help='真实场景点云 .ply/.pcd（留空则合成）')
    ap.add_argument('--scale', type=float, default=0.001, help='CAD 单位换算(mm->m=0.001)')
    ap.add_argument('--voxel', type=float, default=0.003, help='降采样体素(m)')
    ap.add_argument('--no-vis', action='store_true')
    args = ap.parse_args()

    print(f'加载模型: {args.model} (scale={args.scale})')
    model = load_model(args.model, args.scale)

    gt = None
    if args.scene:
        scene = o3d.io.read_point_cloud(args.scene)
        print(f'加载真实场景: {args.scene}, {len(scene.points)} 点')
    else:
        np.random.seed(0)
        scene, gt = make_synthetic_scene(model)
        print(f'合成场景: {len(scene.points)} 点（含杂物+噪声）')

    print(f'\n开始配准 (voxel={args.voxel}) ...')
    res = pe.estimate_pose(model, scene, args.voxel)
    print(f'  fitness     = {res.fitness:.4f}  (越接近1越好)')
    print(f'  inlier_rmse = {res.inlier_rmse*1000:.3f} mm')

    if gt is not None:
        dt, dr = pose_error(res.transform, gt)
        print(f'  位姿误差: 平移 {dt:.2f} mm, 旋转 {dr:.2f} deg')

    if not args.no_vis:
        model_aligned = o3d.geometry.PointCloud(model).transform(res.transform)
        model_aligned.paint_uniform_color([1, 0, 0])   # 红=配准后的CAD
        scene.paint_uniform_color([0.6, 0.6, 0.6])      # 灰=场景
        print('\n可视化: 红色CAD 应贴合灰色场景中的零件。关闭窗口结束。')
        o3d.visualization.draw_geometries([scene, model_aligned])


if __name__ == '__main__':
    main()
