"""位姿 <-> 4x4 矩阵 的小工具（避免引入额外依赖）。"""

from __future__ import annotations

import math
import numpy as np
from geometry_msgs.msg import Pose, Transform


def quaternion_to_matrix(x, y, z, w) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(R: np.ndarray):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def euler_to_matrix(roll, pitch, yaw) -> np.ndarray:
    """XYZ 固定轴欧拉角 -> 旋转矩阵。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def pose_to_matrix(pose: Pose) -> np.ndarray:
    T = np.eye(4)
    q = pose.orientation
    T[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    T[0, 3] = pose.position.x
    T[1, 3] = pose.position.y
    T[2, 3] = pose.position.z
    return T


def matrix_to_pose(T: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x = float(T[0, 3])
    pose.position.y = float(T[1, 3])
    pose.position.z = float(T[2, 3])
    qx, qy, qz, qw = matrix_to_quaternion(T[:3, :3])
    pose.orientation.x = float(qx)
    pose.orientation.y = float(qy)
    pose.orientation.z = float(qz)
    pose.orientation.w = float(qw)
    return pose


def make_grasp_matrix(x, y, z, roll, pitch, yaw) -> np.ndarray:
    """由 标注(平移+欧拉) 生成 T_object->grasp。"""
    T = np.eye(4)
    T[:3, :3] = euler_to_matrix(roll, pitch, yaw)
    T[:3, 3] = [x, y, z]
    return T


def transform_to_matrix(tf: Transform) -> np.ndarray:
    """geometry_msgs/Transform -> 4x4 矩阵。"""
    T = np.eye(4)
    q = tf.rotation
    T[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    T[0, 3] = tf.translation.x
    T[1, 3] = tf.translation.y
    T[2, 3] = tf.translation.z
    return T


def matrix_to_transform(T: np.ndarray) -> Transform:
    """4x4 矩阵 -> geometry_msgs/Transform。"""
    tf = Transform()
    tf.translation.x = float(T[0, 3])
    tf.translation.y = float(T[1, 3])
    tf.translation.z = float(T[2, 3])
    qx, qy, qz, qw = matrix_to_quaternion(T[:3, :3])
    tf.rotation.x = float(qx)
    tf.rotation.y = float(qy)
    tf.rotation.z = float(qz)
    tf.rotation.w = float(qw)
    return tf


def invert(T: np.ndarray) -> np.ndarray:
    """刚体变换求逆（比 np.linalg.inv 稳）。"""
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti
