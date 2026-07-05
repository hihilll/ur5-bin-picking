"""阶段二推理服务：在独立 Python 环境(conda/Docker)里跑大模型，与 ROS 解耦。

ROS 侧客户端(bin_picking_perception_v2) 经 ZMQ REQ/REP + pickle 调用本服务。
协议（request dict）:
  {'model': 'sam6d'|'foundationpose'|'grasp',
   'rgb':   HxWx3 uint8 (RGB),
   'depth': HxW float32 (米, 无效=0),
   'K':     3x3 float64 内参,
   ...模型专属字段: object_id / mode / cmd}
响应（response dict）:
  {'ok': bool, 'msg': str,
   'results': [{'pose': 4x4 list(相机系), 'score': float,
                'width': float(仅抓取), 'mode': str(仅fp)}]}

用法:
  # 假模式：不加载任何模型，返回固定假结果，用于先打通 ROS 链路
  python3 server.py --fake

  # 正式：加载全部三个模型（4090 24G 可并存）
  python3 server.py \
      --sam6d-repo ~/SAM-6D/SAM-6D --sam6d-cad /abs/part_mm.ply \
      --templates /abs/templates_dir \
      --fp-repo ~/FoundationPose --fp-cad /abs/part_m.obj \
      --grasp-repo ~/graspnet-baseline --grasp-ckpt /abs/checkpoint.tar

  # 依赖冲突时按模型拆分到不同环境/端口分别起：
  python3 server.py --models sam6d --port 5555 --sam6d-repo ... --sam6d-cad ... --templates ...
  python3 server.py --models foundationpose --port 5556 --fp-repo ... --fp-cad ...  # (FP官方Docker内)
  python3 server.py --models grasp --port 5557 --grasp-repo ... --grasp-ckpt ...

⚠️ CAD 单位注意（两个模型约定不同）:
  --sam6d-cad  : SAM-6D 官方约定 CAD 为 **mm** 单位 .ply
  --fp-cad     : FoundationPose 期望 **米** 单位网格（mm 模型先除 1000 另存）

⚠️ pickle 反序列化不安全，只允许本机/可信内网访问（默认绑定 127.0.0.1）。
"""

from __future__ import annotations

import argparse
import pickle
import traceback

import numpy as np
import zmq

from wrappers.fake_wrapper import FakeWrapper


def build_wrappers(models: list[str], fake: bool, args) -> dict:
    """按需构建各模型 wrapper（权重加载在各 wrapper 的 load() 里做）。"""
    wrappers = {}
    for name in models:
        if fake:
            wrappers[name] = FakeWrapper(name)
            continue
        if name == 'sam6d':
            from wrappers.sam6d_wrapper import Sam6dWrapper
            wrappers[name] = Sam6dWrapper(
                repo_dir=args.sam6d_repo, cad_path=args.sam6d_cad,
                templates_dir=args.templates,
                seg_model=args.sam6d_segmentor,
                det_score_thresh=args.sam6d_det_thresh)
        elif name == 'foundationpose':
            from wrappers.foundationpose_wrapper import FoundationPoseWrapper
            wrappers[name] = FoundationPoseWrapper(
                repo_dir=args.fp_repo, cad_path=args.fp_cad,
                z_min=args.inspect_zmin, z_max=args.inspect_zmax)
        elif name == 'grasp':
            from wrappers.grasp_wrapper import GraspWrapper
            wrappers[name] = GraspWrapper(
                repo_dir=args.grasp_repo, checkpoint=args.grasp_ckpt,
                collision_thresh=args.grasp_collision_thresh)
        else:
            raise SystemExit(f'未知模型: {name}')
    return wrappers


def validate(req: dict) -> str | None:
    """基本请求校验，返回错误信息或 None。纯命令请求(cmd)跳过图像校验。"""
    if 'cmd' in req:
        return None
    for key, nd in (('rgb', 3), ('depth', 2), ('K', 2)):
        v = req.get(key)
        if not isinstance(v, np.ndarray) or v.ndim != nd:
            return f'请求缺少或非法字段: {key}'
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bind', default='tcp://127.0.0.1', help='绑定地址')
    ap.add_argument('--port', type=int, default=5555)
    ap.add_argument('--models', default='sam6d,foundationpose,grasp',
                    help='本实例承载的模型，逗号分隔')
    ap.add_argument('--fake', action='store_true',
                    help='假模式：不加载模型，返回固定结果（链路调试）')
    # --- SAM-6D ---
    ap.add_argument('--sam6d-repo', default='',
                    help='SAM-6D 代码目录（含 Instance_Segmentation_Model/'
                         'Pose_Estimation_Model 的那一层）')
    ap.add_argument('--sam6d-cad', default='', help='零件 CAD .ply（mm 单位）')
    ap.add_argument('--templates', default='',
                    help='blenderproc 预渲染模板目录（含 rgb_*.png/mask_*.png）')
    ap.add_argument('--sam6d-segmentor', default='sam',
                    choices=['sam', 'fastsam'], help='ISM 分割底座')
    ap.add_argument('--sam6d-det-thresh', type=float, default=0.2,
                    help='ISM 检测分数阈值')
    # --- FoundationPose ---
    ap.add_argument('--fp-repo', default='', help='FoundationPose 代码目录')
    ap.add_argument('--fp-cad', default='', help='零件 CAD 网格（米 单位）')
    ap.add_argument('--inspect-zmin', type=float, default=0.25,
                    help='检视位深度带下限 m（register 用 mask）')
    ap.add_argument('--inspect-zmax', type=float, default=0.70,
                    help='检视位深度带上限 m')
    # --- 抓取网络 ---
    ap.add_argument('--grasp-repo', default='',
                    help='graspnet-baseline / EconomicGrasp 代码目录')
    ap.add_argument('--grasp-ckpt', default='', help='抓取网络权重路径')
    ap.add_argument('--grasp-collision-thresh', type=float, default=0.01,
                    help='无模型碰撞过滤阈值 m，0=关闭')
    args = ap.parse_args()

    models = [m.strip() for m in args.models.split(',') if m.strip()]
    wrappers = build_wrappers(models, args.fake, args)

    print(f'加载模型: {models} (fake={args.fake})')
    for name, w in wrappers.items():
        w.load()
        print(f'  [{name}] 就绪')

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    endpoint = f'{args.bind}:{args.port}'
    sock.bind(endpoint)
    print(f'推理服务监听 {endpoint}，等待 ROS 客户端…')

    while True:
        raw = sock.recv()
        try:
            req = pickle.loads(raw)
            model = req.get('model', '')
            if model not in wrappers:
                resp = {'ok': False,
                        'msg': f'本实例未加载模型 {model}（已加载: {models}）'}
            else:
                err = validate(req)
                if err:
                    resp = {'ok': False, 'msg': err}
                else:
                    resp = wrappers[model].infer(req)
        except Exception as e:  # noqa: BLE001
            traceback.print_exc()
            resp = {'ok': False, 'msg': f'服务端异常: {e}'}
        sock.send(pickle.dumps(resp, protocol=4))


if __name__ == '__main__':
    main()
