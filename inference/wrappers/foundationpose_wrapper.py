"""FoundationPose wrapper（在手位姿估计/跟踪，检视位用）—— 完整实现。

官方仓库: https://github.com/NVlabs/FoundationPose  (CVPR 2024, NVIDIA)
建议直接用官方 Docker 镜像（依赖 nvdiffrast/kaolin 编译麻烦），
本服务以 --models foundationpose 单独跑在该容器里（端口 5556），
容器内: pip install pyzmq，并把本 inference/ 目录挂载进去。

状态机（mode='auto'）:
  未初始化/丢跟踪 -> register（全局估计, ~1-2s, 需要 mask）
  已初始化        -> track_one（逐帧跟踪, ~30ms, 不需要 mask）
  平移跳变 > jump_thresh -> 判丢失，下一帧自动重新 register
  收到 {'cmd':'reset'} -> 回到未初始化（每抓一个新零件由客户端调用）

register 的 mask 用「检视位深度带」方案：零件被举到相机前 [z_min, z_max]
深度范围内，取该范围内最大连通域。若混入夹爪指尖导致位姿飘，收紧深度带。

本实现按官方 run_demo.py 的 API 编写，未经真机验证。现场核对要点:
  [V1] est.register / est.track_one 的参数名与返回值（4x4 ob_in_cam）
  [V2] --fp-cad 网格必须是**米**单位（mm 模型先 mesh.apply_scale(0.001) 另存）
  [V3] rgb 传 RGB 顺序（官方 demo 用 imageio 读图即 RGB，与本服务一致）
"""

from __future__ import annotations

import os
import sys

import numpy as np

from wrappers.base import BaseWrapper


class FoundationPoseWrapper(BaseWrapper):

    name = 'foundationpose'

    def __init__(self, repo_dir: str = '', cad_path: str = '',
                 z_min: float = 0.25, z_max: float = 0.70,
                 register_iter: int = 5, track_iter: int = 2,
                 jump_thresh: float = 0.08, min_mask_pixels: int = 500):
        self.repo_dir = os.path.expanduser(repo_dir)
        self.cad_path = cad_path
        self.z_min = z_min
        self.z_max = z_max
        self.register_iter = register_iter
        self.track_iter = track_iter
        self.jump_thresh = jump_thresh
        self.min_mask_pixels = min_mask_pixels

        self.est = None
        self.initialized = False
        self.last_t = None
        self.cv2 = None

    def load(self):
        if not self.repo_dir or not os.path.isdir(self.repo_dir):
            raise SystemExit('foundationpose: --fp-repo 未指定或目录不存在')
        if not self.cad_path or not os.path.isfile(self.cad_path):
            raise SystemExit('foundationpose: --fp-cad 未指定或文件不存在'
                             '（注意：米单位网格）')
        sys.path.insert(0, self.repo_dir)

        import cv2
        import trimesh
        import nvdiffrast.torch as dr
        from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor

        self.cv2 = cv2
        mesh = trimesh.load(self.cad_path)
        extent = float(np.max(mesh.extents))
        if extent > 1.0:   # 零件不可能超过 1m，大概率是 mm 单位模型 [V2]
            raise SystemExit(
                f'foundationpose: 网格尺寸 {extent:.1f}（应为米），'
                '疑似 mm 单位 CAD，请先缩放 0.001 另存')

        self.est = FoundationPose(
            model_pts=mesh.vertices, model_normals=mesh.vertex_normals,
            mesh=mesh, scorer=ScorePredictor(), refiner=PoseRefinePredictor(),
            glctx=dr.RasterizeCudaContext(), debug=0)

    # ---------- 检视位 mask：深度带内最大连通域 ----------
    def _make_inspection_mask(self, depth: np.ndarray) -> np.ndarray | None:
        cv2 = self.cv2
        band = ((depth > self.z_min) & (depth < self.z_max)).astype(np.uint8)
        band = cv2.morphologyEx(band, cv2.MORPH_OPEN,
                                np.ones((5, 5), np.uint8))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(band, 8)
        if n <= 1:
            return None
        areas = stats[1:, cv2.CC_STAT_AREA]        # 0 为背景
        idx = int(np.argmax(areas)) + 1
        if stats[idx, cv2.CC_STAT_AREA] < self.min_mask_pixels:
            return None
        return labels == idx

    def infer(self, req: dict) -> dict:
        if req.get('cmd') == 'reset':
            self.initialized = False
            self.last_t = None
            return self.ok([], msg='已重置，下一帧 register')

        rgb = np.ascontiguousarray(req['rgb'])
        depth = np.nan_to_num(
            req['depth'].astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        K = np.asarray(req['K'], dtype=np.float64).reshape(3, 3)

        if not self.initialized:
            mask = self._make_inspection_mask(depth)
            if mask is None:
                return self.fail(
                    f'检视深度带 [{self.z_min},{self.z_max}]m 内未找到物体，'
                    '确认零件已举到检视位/调整深度带参数')
            pose = self.est.register(K=K, rgb=rgb, depth=depth,
                                     ob_mask=mask,
                                     iteration=self.register_iter)   # [V1]
            mode = 'register'
        else:
            pose = self.est.track_one(rgb=rgb, depth=depth, K=K,
                                      iteration=self.track_iter)     # [V1]
            mode = 'track'

        pose = np.asarray(pose, dtype=np.float64).reshape(4, 4)
        t = pose[:3, 3]
        if not np.all(np.isfinite(pose)):
            self.initialized = False
            return self.fail('位姿含非法值，下一帧重新 register')
        if mode == 'track' and self.last_t is not None:
            if float(np.linalg.norm(t - self.last_t)) > self.jump_thresh:
                self.initialized = False
                return self.fail(
                    f'跟踪跳变 >{self.jump_thresh}m，判丢失，下一帧重新 register')

        self.initialized = True
        self.last_t = t
        # FoundationPose 无显式置信度，score 恒 1.0；
        # 质量控制靠上面的跳变/非法值判据 + 客户端 min_score 阈值。
        return self.ok([{'pose': pose.tolist(), 'score': 1.0, 'mode': mode}])
