"""6D 位姿估计：CAD 模型 ↔ 场景点云配准。

流程（阶段一，传统方法）：
  场景预处理（裁剪/降采样/去平面/聚类）
    -> 对每个聚类做粗配准（FPFH 特征 + RANSAC，等价于 PPF 的全局匹配作用）
    -> ICP 精配准
    -> 输出 4x4 位姿矩阵 + 配准质量 (fitness / inlier_rmse)

阶段二可把本模块替换为 FoundationPose 等大模型，节点接口保持不变。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import open3d as o3d


@dataclass
class RegistrationResult:
    transform: np.ndarray      # 4x4, CAD -> 场景 的变换（即零件在场景坐标系下的位姿）
    fitness: float             # 重叠度 (0~1)
    inlier_rmse: float         # 内点 RMSE (m)


def preprocess(cloud: o3d.geometry.PointCloud, voxel: float):
    """降采样 + 估计法线 + 计算 FPFH 特征。"""
    down = cloud.voxel_down_sample(voxel)
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.0, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5.0, max_nn=100))
    return down, fpfh


def crop_workspace(cloud: o3d.geometry.PointCloud,
                   min_bound, max_bound) -> o3d.geometry.PointCloud:
    """按工作区 AABB 裁剪点云，去掉料框外的背景。坐标在相机系。"""
    bbox = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=np.asarray(min_bound, dtype=np.float64),
        max_bound=np.asarray(max_bound, dtype=np.float64))
    return cloud.crop(bbox)


def remove_dominant_plane(cloud: o3d.geometry.PointCloud,
                          distance_threshold: float = 0.005,
                          ransac_n: int = 3,
                          num_iterations: int = 1000):
    """分割并移除最大平面（料框底/工作台），返回剩余点云。"""
    if len(cloud.points) < ransac_n:
        return cloud
    _, inliers = cloud.segment_plane(distance_threshold, ransac_n, num_iterations)
    return cloud.select_by_index(inliers, invert=True)


def cluster_objects(cloud: o3d.geometry.PointCloud,
                    eps: float = 0.01,
                    min_points: int = 100):
    """DBSCAN 聚类，把堆叠零件分成若干候选簇。返回点云列表（按大小降序）。"""
    if len(cloud.points) == 0:
        return []
    labels = np.array(cloud.cluster_dbscan(eps=eps, min_points=min_points))
    clusters = []
    for label in range(labels.max() + 1):
        idx = np.where(labels == label)[0]
        if len(idx) >= min_points:
            clusters.append(cloud.select_by_index(idx))
    clusters.sort(key=lambda c: len(c.points), reverse=True)
    return clusters


def global_registration(model_down, model_fpfh, scene_down, scene_fpfh,
                        voxel: float):
    """FPFH + RANSAC 粗配准（PPF 风格全局匹配）。"""
    distance_threshold = voxel * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        model_down, scene_down, model_fpfh, scene_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result


def icp_refine(model_down, scene_down, init_transform, voxel: float):
    """点到面 ICP 精配准。"""
    distance_threshold = voxel * 0.8
    model_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.0, max_nn=30))
    scene_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.0, max_nn=30))
    result = o3d.pipelines.registration.registration_icp(
        model_down, scene_down, distance_threshold, init_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result


def estimate_pose(model_cloud: o3d.geometry.PointCloud,
                  scene_cluster: o3d.geometry.PointCloud,
                  voxel: float) -> RegistrationResult:
    """对单个场景簇估计 CAD 模型的 6D 位姿（粗配准 + ICP）。"""
    model_down, model_fpfh = preprocess(model_cloud, voxel)
    scene_down, scene_fpfh = preprocess(scene_cluster, voxel)

    coarse = global_registration(model_down, model_fpfh,
                                 scene_down, scene_fpfh, voxel)
    fine = icp_refine(model_down, scene_down, coarse.transformation, voxel)

    return RegistrationResult(
        transform=np.asarray(fine.transformation),
        fitness=float(fine.fitness),
        inlier_rmse=float(fine.inlier_rmse),
    )
