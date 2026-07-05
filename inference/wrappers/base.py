"""模型 wrapper 基类：统一 load()/infer() 接口。

infer(req) 输入见 server.py 协议注释；必须返回:
  {'ok': bool, 'msg': str, 'results': [ {...}, ... ]}
其中每个 result 的 'pose' 为**相机系** 4x4（list 或 ndarray 均可）。
"""

from __future__ import annotations


class BaseWrapper:

    name = 'base'

    def load(self):
        """加载权重/预处理模板。启动时调用一次。"""
        raise NotImplementedError

    def infer(self, req: dict) -> dict:
        raise NotImplementedError

    @staticmethod
    def ok(results, msg='') -> dict:
        return {'ok': True, 'msg': msg, 'results': results}

    @staticmethod
    def fail(msg) -> dict:
        return {'ok': False, 'msg': str(msg), 'results': []}
