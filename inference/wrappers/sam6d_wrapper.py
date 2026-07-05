"""SAM-6D wrapper（ISM 分割 + PEM 位姿，零样本 6D）—— 完整实现。

官方仓库: https://github.com/JiehongLin/SAM-6D  (CVPR 2024)
--sam6d-repo 指向仓库中含 Instance_Segmentation_Model / Pose_Estimation_Model
的那一层目录（一般是 SAM-6D/SAM-6D）。

前置（一次性，见 docs/08）:
  1. 按官方 README 装环境、下载 ISM(SAM/FastSAM+DINOv2) 与 PEM 权重
  2. blenderproc 渲染 CAD 模板 -> --templates 目录（rgb_*.png / mask_*.png）
  3. --sam6d-cad 为 **mm 单位** .ply（官方约定，内部 /1000 转米）

实现方式:
  - ISM: 模型常驻内存，逐帧推理（照抄官方 ISM/run_inference_custom.py 流程）
  - PEM: 模型常驻内存；数据准备复用官方 get_test_data()（文件式接口），
    每帧把 rgb/depth/cam/检测结果写入临时目录再喂给它（写文件耗时可忽略）

⚠️ ISM 与 PEM 两个子目录都有各自的顶层 `utils` 包，官方是分进程跑的；
   本 wrapper 在加载完 ISM 后清掉 sys.modules 里的同名缓存再加载 PEM
   （见 _purge_modules）。若现场仍有导入串包问题，退路：把 sam6d 拆成
   两个 server 实例分别只跑 ISM/PEM——不建议，先试本方案。

本实现按官方两个 run_inference_custom.py 编写，未经真机验证。现场核对要点:
  [V1] hydra 配置名（run_inference.yaml / ISM_sam.yaml）与本版本仓库一致
  [V2] 模板 onboarding 段的函数名（compute_features / compute_masked_patch_feature）
  [V3] Detections 保存/npz->json 的工具函数名（save_to_file / convert_npz_to_json）
  [V4] PEM get_test_data 签名与 cfg.test_dataset 字段
  [V5] PEM 输出键名: pred_R / pred_t / pred_pose_score（pred_t 应为米）
"""

from __future__ import annotations

import glob
import json
import os
import sys
import tempfile

import numpy as np

from wrappers.base import BaseWrapper


def _purge_modules(prefixes):
    """把指定前缀的模块从 sys.modules 移除（已加载对象仍持有引用，不受影响）。"""
    for name in list(sys.modules):
        if any(name == p or name.startswith(p + '.') for p in prefixes):
            del sys.modules[name]


class Sam6dWrapper(BaseWrapper):

    name = 'sam6d'

    def __init__(self, repo_dir: str = '', cad_path: str = '',
                 templates_dir: str = '', seg_model: str = 'sam',
                 det_score_thresh: float = 0.2):
        self.repo_dir = os.path.expanduser(repo_dir)
        self.cad_path = cad_path
        self.templates_dir = templates_dir
        self.seg_model = seg_model
        self.det_score_thresh = det_score_thresh

        self.ism = None            # ISM 模型（in-memory 推理）
        self.pem = None            # PEM 模型
        self.pem_mod = None        # PEM run_inference_custom 模块(用其 get_test_data)
        self.pem_cfg = None
        self.ism_utils = {}        # ISM 侧需要的工具函数引用
        self.tmpdir = None

    # ================= 加载 =================
    def load(self):
        if not self.repo_dir or not os.path.isdir(self.repo_dir):
            raise SystemExit('sam6d: --sam6d-repo 未指定或目录不存在')
        if not self.cad_path or not os.path.isfile(self.cad_path):
            raise SystemExit('sam6d: --sam6d-cad 未指定或文件不存在（mm 单位 .ply）')
        if not self.templates_dir or not os.path.isdir(self.templates_dir):
            raise SystemExit('sam6d: --templates 未指定或目录不存在'
                             '（先跑 blenderproc 渲染，见 docs/08）')
        self.ism_dir = os.path.join(self.repo_dir, 'Instance_Segmentation_Model')
        self.pem_dir = os.path.join(self.repo_dir, 'Pose_Estimation_Model')
        for d in (self.ism_dir, self.pem_dir):
            if not os.path.isdir(d):
                raise SystemExit(f'sam6d: 找不到子目录 {d}')

        self.tmpdir = tempfile.mkdtemp(prefix='sam6d_')
        self._load_ism()
        # ISM 与 PEM 同名顶层包(utils/model/...)冲突：清缓存后再加载 PEM
        _purge_modules(['utils', 'model', 'provider', 'segment_anything'])
        self._load_pem()

    def _load_ism(self):
        """照抄官方 ISM/run_inference_custom.py: 建模型 + onboarding 模板。"""
        sys.path.insert(0, self.ism_dir)
        import torch
        import trimesh
        from PIL import Image
        from hydra import initialize_config_dir, compose
        from hydra.utils import instantiate
        from omegaconf import OmegaConf

        cfg_dir = os.path.join(self.ism_dir, 'configs')
        with initialize_config_dir(version_base=None, config_dir=cfg_dir):
            cfg = compose(config_name='run_inference.yaml')            # [V1]
        with initialize_config_dir(version_base=None,
                                   config_dir=os.path.join(cfg_dir, 'model')):
            cfg.model = compose(config_name=f'ISM_{self.seg_model}.yaml')  # [V1]

        model = instantiate(cfg.model)
        device = 'cuda'
        model.descriptor_model.model = model.descriptor_model.model.to(device)
        model.descriptor_model.model.device = device
        if hasattr(model.segmentor_model, 'predictor'):
            model.segmentor_model.predictor.model = \
                model.segmentor_model.predictor.model.to(device)
        else:
            model.segmentor_model.model.setup_model(device=device, verbose=True)

        # ---- 模板 onboarding（官方 "register ref data" 段）[V2] ----
        from utils.bbox_utils import CropResizePad
        num_templates = len(glob.glob(f'{self.templates_dir}/*.npy')) or \
            len(glob.glob(f'{self.templates_dir}/rgb_*.png'))
        boxes, templates, masks = [], [], []
        for idx in range(num_templates):
            image = Image.open(
                os.path.join(self.templates_dir, f'rgb_{idx}.png'))
            mask = Image.open(
                os.path.join(self.templates_dir, f'mask_{idx}.png'))
            boxes.append(mask.getbbox())
            image = torch.from_numpy(
                np.array(image.convert('RGB')) / 255).float()
            mask_t = torch.from_numpy(
                np.array(mask.convert('L')) / 255).float()
            image = image * mask_t[:, :, None]
            templates.append(image)
            masks.append(mask_t.unsqueeze(-1))
        templates = torch.stack(templates).permute(0, 3, 1, 2)
        masks = torch.stack(masks).permute(0, 3, 1, 2)
        boxes = torch.tensor(np.array(boxes))

        processing_config = OmegaConf.create({'image_size': 224})
        proposal_processor = CropResizePad(processing_config.image_size)
        templates = proposal_processor(images=templates, boxes=boxes).to(device)
        masks_cropped = proposal_processor(images=masks, boxes=boxes).to(device)

        model.ref_data = {
            'descriptors': model.descriptor_model.compute_features(
                templates, token_name='x_norm_clstoken').unsqueeze(0).data,
            'appe_descriptors': model.descriptor_model
                .compute_masked_patch_feature(
                    templates, masks_cropped[:, 0, :, :]).unsqueeze(0).data,
        }

        # ---- 几何评分所需: 模板位姿 + CAD 采样点（官方同段）----
        from utils.poses.pose_utils import (
            get_obj_poses_from_template_level, load_index_level_in_level2)
        template_poses = get_obj_poses_from_template_level(
            level=2, pose_distribution='all')
        template_poses[:, :3, 3] *= 0.4
        poses = torch.tensor(template_poses).to(torch.float32).to(device)
        model.ref_data['poses'] = \
            poses[load_index_level_in_level2(0, 'all'), :, :]
        mesh = trimesh.load_mesh(self.cad_path)
        model_points = mesh.sample(2048).astype(np.float32) / 1000.0  # mm->m
        model.ref_data['pointcloud'] = torch.tensor(
            model_points).unsqueeze(0).data.to(device)

        # 推理时用到的类/函数引用 [V3]
        from model.utils import Detections, convert_npz_to_json
        from utils.inout import save_json_bop23
        self.ism_utils = {
            'Detections': Detections,
            'convert_npz_to_json': convert_npz_to_json,
            'save_json_bop23': save_json_bop23,
            'torch': torch,
        }
        self.ism = model

    def _load_pem(self):
        """加载 PEM 模型 + 借用官方 get_test_data 做数据准备。"""
        # 官方脚本自身会 append 这些子路径，先补齐
        for sub in ('', 'provider', 'utils', 'model',
                    os.path.join('model', 'pointnet2')):
            sys.path.insert(0, os.path.join(self.pem_dir, sub))

        import importlib
        import gorilla

        cfg = gorilla.Config.fromfile(
            os.path.join(self.pem_dir, 'config', 'base.yaml'))      # [V4]
        MODEL = importlib.import_module('pose_estimation_model')
        self.pem = MODEL.Net(cfg.model).cuda().eval()
        ckpt = os.path.join(self.pem_dir, 'checkpoints',
                            'sam-6d-pem-base.pth')
        if not os.path.isfile(ckpt):
            raise SystemExit(f'sam6d: 缺 PEM 权重 {ckpt}（官方 README 下载）')
        gorilla.solver.load_checkpoint(model=self.pem, filename=ckpt)

        # 导入 run_inference_custom 拿 get_test_data。该脚本若在模块层 parse_args，
        # 用假 argv 让它无害通过 [V4]
        argv_bak = sys.argv
        sys.argv = ['run_inference_custom.py',
                    '--output_dir', self.tmpdir,
                    '--cad_path', self.cad_path,
                    '--rgb_path', os.path.join(self.tmpdir, 'rgb.png'),
                    '--depth_path', os.path.join(self.tmpdir, 'depth.png'),
                    '--cam_path', os.path.join(self.tmpdir, 'camera.json'),
                    '--seg_path', os.path.join(self.tmpdir, 'seg.json'),
                    '--det_score_thresh', str(self.det_score_thresh)]
        try:
            self.pem_mod = importlib.import_module('run_inference_custom')
        finally:
            sys.argv = argv_bak
        self.pem_cfg = cfg

    # ================= 推理 =================
    def _write_frame(self, rgb, depth_m, K):
        """把当前帧写入临时目录（PEM 的 get_test_data 是文件式接口）。"""
        from PIL import Image
        Image.fromarray(rgb).save(os.path.join(self.tmpdir, 'rgb.png'))
        depth_mm = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)
        Image.fromarray(depth_mm).save(os.path.join(self.tmpdir, 'depth.png'))
        with open(os.path.join(self.tmpdir, 'camera.json'), 'w') as f:
            json.dump({'cam_K': np.asarray(K).flatten().tolist(),
                       'depth_scale': 1.0}, f)

    def _run_ism(self, rgb, depth_m, K):
        """ISM 推理（官方 run_inference_custom.py 主流程），产出 seg.json。"""
        torch = self.ism_utils['torch']
        Detections = self.ism_utils['Detections']
        model = self.ism
        device = 'cuda'

        detections = model.segmentor_model.generate_masks(np.array(rgb))
        detections = Detections(detections)
        query_desc, query_appe_desc = model.descriptor_model.forward(
            np.array(rgb), detections)

        (idx_selected, pred_idx_objects, semantic_score,
         best_template) = model.compute_semantic_score(query_desc)
        detections.filter(idx_selected)
        query_appe_desc = query_appe_desc[idx_selected, :]

        appe_scores, ref_aux_desc = model.compute_appearance_score(
            best_template, pred_idx_objects, query_appe_desc)

        # 几何评分需要深度（官方 batch_input_data 的内存版）
        depth_mm = (depth_m * 1000.0).astype(np.int32)
        batch = {
            'depth': torch.from_numpy(depth_mm).unsqueeze(0).to(device),
            'cam_intrinsic': torch.from_numpy(
                np.asarray(K, dtype=np.float64)).unsqueeze(0).to(device),
            'depth_scale': torch.tensor([1.0]).to(device),
        }
        image_uv = model.project_template_to_image(
            best_template, pred_idx_objects, batch, detections.masks)
        geometric_score, visible_ratio = model.compute_geometric_score(
            image_uv, detections, query_appe_desc, ref_aux_desc,
            visible_thred=model.visible_thred)

        final_score = (semantic_score + appe_scores
                       + geometric_score * visible_ratio) / (2 + visible_ratio)
        detections.add_attribute('scores', final_score)
        detections.add_attribute('object_ids',
                                 torch.zeros_like(final_score))
        detections.to_numpy()

        # 存成 PEM 需要的 json（官方同款 npz->json 转换）[V3]
        prefix = os.path.join(self.tmpdir, 'detection_ism')
        detections.save_to_file(0, 0, 0, prefix, 'Custom',
                                return_results=False)
        dets_json = self.ism_utils['convert_npz_to_json'](
            0, [prefix + '.npz'])
        self.ism_utils['save_json_bop23'](
            os.path.join(self.tmpdir, 'seg.json'), dets_json)
        return len(dets_json) if dets_json is not None else 0

    def _run_pem(self):
        """PEM 推理：官方 get_test_data + 前向。返回 results 列表。[V4][V5]"""
        import torch
        t = self.tmpdir
        input_data, _img, _pts, _model_points, _dets = \
            self.pem_mod.get_test_data(
                os.path.join(t, 'rgb.png'), os.path.join(t, 'depth.png'),
                os.path.join(t, 'camera.json'), self.cad_path,
                os.path.join(t, 'seg.json'), self.det_score_thresh,
                self.pem_cfg.test_dataset)
        with torch.no_grad():
            out = self.pem(input_data)
        if 'pred_pose_score' in out:
            scores = out['pred_pose_score'] * out['score']
        else:
            scores = out['score']
        scores = scores.detach().cpu().numpy()
        pred_R = out['pred_R'].detach().cpu().numpy()
        pred_t = out['pred_t'].detach().cpu().numpy()   # 应为米 [V5]

        results = []
        for i in range(len(scores)):
            T = np.eye(4)
            T[:3, :3] = pred_R[i].reshape(3, 3)
            T[:3, 3] = pred_t[i].reshape(3)
            results.append({'pose': T.tolist(), 'score': float(scores[i])})
        return results

    def infer(self, req: dict) -> dict:
        rgb = np.ascontiguousarray(req['rgb'])
        depth = np.nan_to_num(
            req['depth'].astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        K = np.asarray(req['K'], dtype=np.float64).reshape(3, 3)

        self._write_frame(rgb, depth, K)
        n_det = self._run_ism(rgb, depth, K)
        if n_det == 0:
            return self.ok([], msg='ISM 未检出零件实例')
        results = self._run_pem()
        results.sort(key=lambda r: r['score'], reverse=True)
        return self.ok(results)
