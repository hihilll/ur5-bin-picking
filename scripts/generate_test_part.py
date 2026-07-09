"""生成一个用于抓取测试的"阶梯型小机械块" STL（零依赖，纯标准库）。

为什么用这个形状：
  - **非对称**：一端高、一端低（阶梯），PPF/ICP 能给出无歧义的 6D 位姿
    （不像立方体/圆柱那样朝向会跳）。
  - **有平行面可夹**：沿宽度方向(Y)两个平面，平行夹爪夹持稳定。
  - **尺寸卡在夹爪范围**：默认夹持宽度(Y)=13mm ≤ 手指宽 16.5mm，整体小、好打印、不反光。

坐标与单位：
  - 单位 mm（perception 用 model_scale=0.001 转成米）。
  - 原点在**包围盒 XY 中心、底面 Z=0**（打印后零件平放在桌面 → 点云原点直观）。
  - 高台段在 -X 侧、矮台段在 +X 侧。

用法：
  python scripts/generate_test_part.py                       # 默认尺寸
  python scripts/generate_test_part.py --length 28 --width 13 --h1 10 --h2 5 --step 16
  python scripts/generate_test_part.py --out some/path/part.stl

生成后：
  1) 打印这块零件。
  2) cad_model_path 指向本 STL（见脚本末尾打印的配置提示）。
  3) grasp_annotations 用脚本打印的默认值，再用 scripts/annotate_grasp.py 微调。
"""

from __future__ import annotations

import argparse
import struct


def build_mesh(length, width, h1, h2, step):
    """返回 (三角形列表)。每个三角形 = 三个 (x,y,z) 顶点，已中心化(XY居中,底Z=0)。

    截面(在 XZ 平面, L 形六边形)按顺时针(CW)排列，使挤出后墙面法线朝外。
    """
    L, W, H1, H2, S = length, width, h1, h2, step
    # L 形截面六个角点（CW：从左上开始）
    poly = [
        (0.0, H1), (S, H1), (S, H2), (L, H2), (L, 0.0), (0.0, 0.0),
    ]
    n = len(poly)
    ox, oy = L / 2.0, W / 2.0        # 中心化偏移（XY 居中，Z 不动）

    def v(x, y, z):
        return (x - ox, y - oy, z)   # Z 保持 0..H1，底面在 Z=0

    tris = []
    # --- 前盖 (y=0)，法线 -Y：从 p0 扇形三角化，反向绕序 ---
    for i in range(1, n - 1):
        a, b = poly[i + 1], poly[i]          # 反向 -> 法线 -Y
        tris.append((v(poly[0][0], 0.0, poly[0][1]),
                     v(a[0], 0.0, a[1]), v(b[0], 0.0, b[1])))
    # --- 后盖 (y=W)，法线 +Y ---
    for i in range(1, n - 1):
        a, b = poly[i], poly[i + 1]
        tris.append((v(poly[0][0], W, poly[0][1]),
                     v(a[0], W, a[1]), v(b[0], W, b[1])))
    # --- 侧墙：每条边挤出成一个四边形(两三角形)，CW 绕序 -> 法线朝外 ---
    for i in range(n):
        ax, az = poly[i]
        bx, bz = poly[(i + 1) % n]
        a0, b0 = v(ax, 0.0, az), v(bx, 0.0, bz)
        b1, a1 = v(bx, W, bz), v(ax, W, az)
        tris.append((a0, b0, b1))
        tris.append((a0, b1, a1))
    return tris


def _normal(t):
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = t
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    m = (nx * nx + ny * ny + nz * nz) ** 0.5 or 1.0
    return nx / m, ny / m, nz / m


def write_ascii_stl(path, tris, name='test_block'):
    with open(path, 'w') as f:
        f.write(f'solid {name}\n')
        for t in tris:
            nx, ny, nz = _normal(t)
            f.write(f'  facet normal {nx:.6e} {ny:.6e} {nz:.6e}\n')
            f.write('    outer loop\n')
            for (x, y, z) in t:
                f.write(f'      vertex {x:.6e} {y:.6e} {z:.6e}\n')
            f.write('    endloop\n  endfacet\n')
        f.write(f'endsolid {name}\n')


def write_binary_stl(path, tris):
    with open(path, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', len(tris)))
        for t in tris:
            nx, ny, nz = _normal(t)
            f.write(struct.pack('<3f', nx, ny, nz))
            for (x, y, z) in t:
                f.write(struct.pack('<3f', x, y, z))
            f.write(struct.pack('<H', 0))


def check_manifold(tris):
    """欧拉检查 V-E+F=2 且每条边恰好被两个三角形共享 -> 水密。"""
    from collections import defaultdict
    edges = defaultdict(int)
    verts = set()
    for t in tris:
        vs = [tuple(round(c, 6) for c in p) for p in t]
        verts.update(vs)
        for i in range(3):
            e = tuple(sorted((vs[i], vs[(i + 1) % 3])))
            edges[e] += 1
    V, E, F = len(verts), len(edges), len(tris)
    non2 = [c for c in edges.values() if c != 2]
    ok = (not non2) and (V - E + F == 2)
    return ok, V, E, F, len(non2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--length', type=float, default=28.0, help='X 总长 mm')
    ap.add_argument('--width', type=float, default=13.0,
                    help='Y 宽(夹持方向) mm，须 ≤ 手指宽')
    ap.add_argument('--h1', type=float, default=10.0, help='高台高 mm')
    ap.add_argument('--h2', type=float, default=5.0, help='矮台高 mm')
    ap.add_argument('--step', type=float, default=16.0, help='阶梯 X 位置 mm')
    ap.add_argument('--ascii', action='store_true', help='输出 ASCII STL(默认二进制)')
    ap.add_argument('--out', default='ros2_ws/src/bin_picking_description/'
                                      'meshes/test_block.stl')
    a = ap.parse_args()

    tris = build_mesh(a.length, a.width, a.h1, a.h2, a.step)
    if a.ascii:
        write_ascii_stl(a.out, tris)
    else:
        write_binary_stl(a.out, tris)

    ok, V, E, F, bad = check_manifold(tris)
    # 高台段 X 中心（中心化后）= (0+step)/2 - length/2
    grasp_x = (0.0 + a.step) / 2.0 - a.length / 2.0
    grasp_z = a.h1 / 2.0
    print(f'已写出 {a.out}  ({"ASCII" if a.ascii else "binary"} STL)')
    print(f'  三角形 F={F}, 顶点 V={V}, 边 E={E}, 水密={ok} (非2共享边={bad})')
    print(f'  外形(mm): 长{a.length} x 宽{a.width} x 高{a.h1}(阶梯到{a.h2})')
    print(f'  夹持宽度(Y) = {a.width}mm')
    print('\n--- perception_params.yaml ---')
    print(f'  cad_model_path: "<绝对路径>/test_block.stl"')
    print(f'  model_scale: 0.001   # STL 是 mm')
    print(f'  object_id: "test_block"')
    print('\n--- grasp_params.yaml (grasp_planner) 默认抓取标注（米/弧度）---')
    print('  # [x,y,z, roll,pitch,yaw, width]，抓高台段、从上方沿Y闭合')
    print(f'  grasp_annotations: [{grasp_x/1000:.4f}, 0.0, {grasp_z/1000:.4f}, '
          f'3.14159, 0.0, 1.5708, {a.width/1000:.4f}]')
    print('  # ↑ 先用它，再用 scripts/annotate_grasp.py 可视化微调')


if __name__ == '__main__':
    main()
