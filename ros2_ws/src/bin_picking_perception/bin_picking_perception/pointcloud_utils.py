"""sensor_msgs/PointCloud2 <-> Open3D 点云互转工具。"""

from __future__ import annotations

import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def pointcloud2_to_o3d(msg: PointCloud2, skip_nan: bool = True) -> o3d.geometry.PointCloud:
    """把 ROS2 PointCloud2 转成 Open3D 点云（含颜色，若有 rgb 字段）。"""
    field_names = [f.name for f in msg.fields]
    has_rgb = 'rgb' in field_names

    read_fields = ('x', 'y', 'z', 'rgb') if has_rgb else ('x', 'y', 'z')
    # Humble 的 read_points 返回结构化 ndarray（不要再 list()/np.array 包装）
    points = point_cloud2.read_points(
        msg, field_names=read_fields, skip_nans=skip_nan)

    cloud = o3d.geometry.PointCloud()
    if points is None or len(points) == 0:
        return cloud

    xyz = np.column_stack(
        [points['x'], points['y'], points['z']]).astype(np.float64)
    cloud.points = o3d.utility.Vector3dVector(xyz)

    if has_rgb:
        # rgb 字段打包成 float32，按位解包成 r,g,b（ascontiguousarray 保证可 view）
        rgb_float = np.ascontiguousarray(points['rgb'], dtype=np.float32)
        rgb_int = rgb_float.view(np.uint32)
        r = ((rgb_int >> 16) & 0xFF) / 255.0
        g = ((rgb_int >> 8) & 0xFF) / 255.0
        b = (rgb_int & 0xFF) / 255.0
        cloud.colors = o3d.utility.Vector3dVector(
            np.column_stack([r, g, b]).astype(np.float64))

    return cloud


def load_cad_as_pointcloud(mesh_path: str, num_points: int = 20000,
                           scale: float = 1.0) -> o3d.geometry.PointCloud:
    """读取 CAD 网格(.stl/.obj/.ply) 并均匀采样成点云。

    scale: 单位换算系数。CAD 常用 mm，点云用 m -> scale=0.001。
    """
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    if not mesh.has_vertices():
        raise ValueError(f'无法读取 CAD 网格或网格为空: {mesh_path}')
    mesh.scale(scale, center=(0.0, 0.0, 0.0))
    mesh.compute_vertex_normals()
    pcd = mesh.sample_points_poisson_disk(number_of_points=num_points)
    return pcd
