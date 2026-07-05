"""假模型 wrapper：返回固定结果，不需要 GPU/权重。

用途：在模型环境装好之前，先把 ROS 客户端 <-> 推理服务 <-> 下游执行
整条链路打通（类似 gripper_driver 的 simulate 模式）。

返回值:
  sam6d/foundationpose: 相机正前方 0.5m 处一个物体，姿态朝向相机
  grasp:                同一位置一个竖直向下(在典型俯视相机布局下)的抓取
"""

from __future__ import annotations

import numpy as np

from wrappers.base import BaseWrapper


def _fake_pose() -> list:
    T = np.eye(4)
    T[2, 3] = 0.5           # 相机光轴正前方 0.5m
    return T.tolist()


class FakeWrapper(BaseWrapper):

    def __init__(self, name: str):
        self.name = name

    def load(self):
        pass

    def infer(self, req: dict) -> dict:
        if req.get('cmd') == 'reset':
            return self.ok([], msg='fake reset')
        if self.name == 'grasp':
            return self.ok([{'pose': _fake_pose(), 'score': 0.9,
                             'width': 0.02}])
        return self.ok([{'pose': _fake_pose(), 'score': 0.9,
                         'mode': 'fake'}])
