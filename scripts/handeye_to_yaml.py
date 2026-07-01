"""把 easy_handeye2 的标定结果(.calib) 转成本项目的 handeye_result.yaml。

easy_handeye2 标定完成后（在 GUI 里点 Save）会把结果存到:
  ~/.ros2/easy_handeye2/<name>.calib
本脚本读取它，写出:
  ros2_ws/src/bin_picking_bringup/config/handeye_result.yaml
之后 colcon build 让新外参随 system.launch 的静态 TF 生效。

用法:
  # 默认读 ~/.ros2/easy_handeye2/handeye_ur5_gemini2.calib
  python3 scripts/handeye_to_yaml.py
  # 或指定名字 / 路径
  python3 scripts/handeye_to_yaml.py --name handeye_ur5_gemini2
  python3 scripts/handeye_to_yaml.py --calib /abs/path/to/xxx.calib

依赖: pip install pyyaml
"""

from __future__ import annotations

import argparse
import os

import yaml


def _find(d, keys):
    """在嵌套 dict 里递归找第一个命中 keys 里任意键的值。"""
    if isinstance(d, dict):
        for k in keys:
            if k in d:
                return d[k]
        for v in d.values():
            r = _find(v, keys)
            if r is not None:
                return r
    return None


def parse_calib(calib: dict):
    """从 easy_handeye2 .calib 里抽出平移+四元数，兼容几种字段布局。

    可能布局:
      transformation: {x,y,z,qx,qy,qz,qw}
      result/transform: {translation:{x,y,z}, rotation:{x,y,z,w}}
    """
    # 布局1：扁平 transformation
    tf = _find(calib, ['transformation', 'transform'])
    if isinstance(tf, dict) and 'qx' in tf:
        return (tf['x'], tf['y'], tf['z'],
                tf['qx'], tf['qy'], tf['qz'], tf['qw'])

    # 布局2：translation / rotation 分组
    trans = _find(calib, ['translation'])
    rot = _find(calib, ['rotation'])
    if isinstance(trans, dict) and isinstance(rot, dict):
        return (trans['x'], trans['y'], trans['z'],
                rot['x'], rot['y'], rot['z'], rot['w'])

    raise SystemExit(
        '无法从 .calib 解析出位姿字段，请手动检查文件结构:\n'
        + yaml.safe_dump(calib, allow_unicode=True))


def parse_frames(calib: dict):
    """取父/子坐标系名（eye_on_base 下为 robot_base_frame / tracking_base_frame）。"""
    parent = _find(calib, ['robot_base_frame']) or 'base_link'
    child = _find(calib, ['tracking_base_frame']) or 'camera_link'
    return parent, child


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_out = os.path.join(
        here, '..', 'ros2_ws', 'src', 'bin_picking_bringup',
        'config', 'handeye_result.yaml')
    default_calib_dir = os.path.expanduser('~/.ros2/easy_handeye2')

    ap = argparse.ArgumentParser()
    ap.add_argument('--name', default='handeye_ur5_gemini2',
                    help='标定名（对应 <name>.calib）')
    ap.add_argument('--calib', default='',
                    help='直接指定 .calib 路径（优先于 --name）')
    ap.add_argument('--out', default=default_out,
                    help='输出 handeye_result.yaml 路径')
    args = ap.parse_args()

    calib_path = args.calib or os.path.join(
        default_calib_dir, f'{args.name}.calib')
    if not os.path.isfile(calib_path):
        raise SystemExit(f'找不到标定文件: {calib_path}\n'
                         f'先在 easy_handeye2 GUI 里完成标定并 Save。')

    with open(calib_path) as f:
        calib = yaml.safe_load(f)

    x, y, z, qx, qy, qz, qw = parse_calib(calib)
    parent, child = parse_frames(calib)

    out = {
        'handeye': {
            'parent_frame': parent,
            'child_frame': child,
            'x': float(x), 'y': float(y), 'z': float(z),
            'qx': float(qx), 'qy': float(qy), 'qz': float(qz), 'qw': float(qw),
        }
    }

    out_path = os.path.abspath(args.out)
    header = ('# 手眼标定外参：由 scripts/handeye_to_yaml.py 从 easy_handeye2 结果生成\n'
              f'# 源文件: {calib_path}\n'
              '# 平移单位 m，旋转为四元数。改动后需 colcon build 生效。\n')
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(header)
        yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False)

    print(f'已写出 {out_path}')
    print(f'  {parent} -> {child}')
    print(f'  平移(m):  x={x:.5f} y={y:.5f} z={z:.5f}')
    print(f'  四元数:   qx={qx:.5f} qy={qy:.5f} qz={qz:.5f} qw={qw:.5f}')
    print('下一步: cd ros2_ws && colcon build --packages-select bin_picking_bringup')


if __name__ == '__main__':
    main()
