"""学习型抓取检测 wrapper —— 完整实现（graspnet-baseline 系 API）。

适配对象（同源代码库，API 基本一致）:
  1. graspnet-baseline  https://github.com/graspnet/graspnet-baseline  （最简，先跑通这个）
  2. EconomicGrasp      https://github.com/iSEE-Laboratory/EconomicGrasp （效果更好，网络构造参数不同）
  备选 AnyGrasp SDK（需 license）/ HGGD（RGBD 输入，API 不同需改造 infer）。

流程: depth+K 反投影成相机系点云 -> 采样 num_points -> 网络 -> GraspGroup
      -> (可选)无模型碰撞过滤 -> NMS -> 按分排序 -> 轴变换 -> 返回

⚠️⚠️ 坐标系契约（本文件已处理，集成后务必在 RViz 复核一次）:
  graspnetAPI 约定: 抓取系 **x 轴 = 接近方向**, y 轴 = 手指闭合方向。
  本项目执行器约定: **z 轴 = 接近方向**, x 轴 = 手指闭合方向。
  转换: R_ours = R_gn @ AXIS_CONV（见下），平移点从"抓取中心"沿接近轴
  推进 g.depth 到指尖抓取点（对应本项目 TCP 语义）。
  RViz 验证: /grasp_markers 箭头(=位姿 x 轴…注意 ARROW 沿 x)——本项目
  grasp_client 发的箭头即 marker pose 的 x 轴，接近方向是 z 轴；
  最直接的核对法是让执行器在仿真里走一遍，预抓取位应在抓取位正上方。

本实现按官方 demo.py 编写，未经真机验证。现场核对要点:
  [V1] GraspNet(...) 构造参数需与所训权重一致（num_view=300 等为官方默认）
  [V2] pred_decode 返回 [B] 个 (N,17) 数组 -> GraspGroup
  [V3] g.translation 是抓取中心、g.depth 是指尖下探量——TCP 点=中心+接近轴*depth
       （若实测偏深/偏浅，调整或去掉 depth 推进）
  [V4] EconomicGrasp 的网络类名/构造参数按其 repo 调整（infer 流程相同）
"""

from __future__ import annotations

import os
import sys

import numpy as np

from wrappers.base import BaseWrapper

# graspnet(x=接近, y=闭合, z=x×y) -> 本项目(z=接近, x=闭合, y=z×x)
# R_ours 各列取自 R_gn: x_ours=y_gn, y_ours=z_gn, z_ours=x_gn（循环置换，det=+1）
AXIS_CONV = np.array([[0.0, 0.0, 1.0],
                      [1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0]])


class GraspWrapper(BaseWrapper):

    name = 'grasp'

    def __init__(self, repo_dir: str = '', checkpoint: str = '',
                 num_points: int = 20000, depth_min: float = 0.2,
                 depth_max: float = 1.2, collision_thresh: float = 0.01,
                 max_return: int = 50):
        self.repo_dir = os.path.expanduser(repo_dir)
        self.checkpoint = checkpoint
        self.num_points = num_points
        self.depth_min = depth_min
        self.depth_max = depth_max
        self.collision_thresh = collision_thresh
        self.max_return = max_return

        self.torch = None
        self.net = None
        self.GraspGroup = None
        self.pred_decode = None
        self.collision_detector = None

    def load(self):
        if not self.repo_dir or not os.path.isdir(self.repo_dir):
            raise SystemExit('grasp: --grasp-repo 未指定或目录不存在')
        if not self.checkpoint or not os.path.isfile(self.checkpoint):
            raise SystemExit('grasp: --grasp-ckpt 未指定或文件不存在')
        sys.path.insert(0, self.repo_dir)
        # graspnet-baseline 内部按 models/ utils/ 相对导入
        for sub in ('models', 'utils', 'dataset'):
            p = os.path.join(self.repo_dir, sub)
            if os.path.isdir(p):
                sys.path.insert(0, p)

        import torch
        from graspnetAPI import GraspGroup
        from models.graspnet import GraspNet, pred_decode

        self.torch = torch
        self.GraspGroup = GraspGroup
        self.pred_decode = pred_decode

        # [V1] 构造参数须与权重训练配置一致（官方 checkpoint 用以下默认值）
        self.net = GraspNet(input_feature_dim=0, num_view=300, num_angle=12,
                            num_depth=4, cylinder_radius=0.05, hmin=-0.02,
                            hmax_list=[0.01, 0.02, 0.03, 0.04],
                            is_training=False)
        self.net.cuda()
        ckpt = torch.load(self.checkpoint)
        self.net.load_state_dict(ckpt['model_state_dict'])
        self.net.eval()

        if self.collision_thresh > 0:
            try:
                from utils.collision_detector import ModelFreeCollisionDetector
                self.collision_detector = ModelFreeCollisionDetector
            except ImportError:
                print('grasp: 无 collision_detector 模块，跳过碰撞过滤')

    # ---------- depth + K -> 相机系点云 ----------
    def _depth_to_cloud(self, depth: np.ndarray, K: np.ndarray) -> np.ndarray:
        h, w = depth.shape
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        us, vs = np.meshgrid(np.arange(w), np.arange(h))
        valid = (depth > self.depth_min) & (depth < self.depth_max)
        z = depth[valid]
        x = (us[valid] - cx) * z / fx
        y = (vs[valid] - cy) * z / fy
        return np.column_stack([x, y, z]).astype(np.float32)

    def infer(self, req: dict) -> dict:
        torch = self.torch
        depth = np.nan_to_num(
            req['depth'].astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        K = np.asarray(req['K'], dtype=np.float64).reshape(3, 3)

        cloud = self._depth_to_cloud(depth, K)
        if len(cloud) < 1000:
            return self.fail(f'有效点太少({len(cloud)})，检查深度图/深度范围参数')

        # 采样到固定点数（不足则有放回补齐）
        if len(cloud) >= self.num_points:
            idx = np.random.choice(len(cloud), self.num_points, replace=False)
        else:
            idx = np.concatenate([
                np.arange(len(cloud)),
                np.random.choice(len(cloud), self.num_points - len(cloud),
                                 replace=True)])
        sampled = cloud[idx]

        end_points = {'point_clouds':
                      torch.from_numpy(sampled[None]).cuda()}
        with torch.no_grad():
            end_points = self.net(end_points)
            grasp_preds = self.pred_decode(end_points)          # [V2]
        gg = self.GraspGroup(grasp_preds[0].detach().cpu().numpy())

        if self.collision_detector is not None:
            mfc = self.collision_detector(cloud, voxel_size=0.01)
            mask = mfc.detect(gg, approach_dist=0.05,
                              collision_thresh=self.collision_thresh)
            gg = gg[~mask]
        gg = gg.nms()
        gg = gg.sort_by_score()

        results = []
        for i in range(min(len(gg), self.max_return)):
            g = gg[i]
            R_gn = np.asarray(g.rotation_matrix, dtype=np.float64).reshape(3, 3)
            T = np.eye(4)
            T[:3, :3] = R_gn @ AXIS_CONV
            # 抓取中心沿接近轴(x_gn)推进 depth 到指尖抓取点 [V3]
            T[:3, 3] = np.asarray(g.translation, dtype=np.float64) \
                + R_gn[:, 0] * float(g.depth)
            results.append({'pose': T.tolist(), 'width': float(g.width),
                            'score': float(g.score)})

        if not results:
            return self.ok([], msg='本帧无有效抓取（碰撞过滤后为空？）')
        return self.ok(results)
