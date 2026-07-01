"""沃姆 EPGC-50-150 二指电动夹爪驱动节点（USB / Modbus-RTU）。

行程 50mm、负载 3kg、USB 独立通信。绝大多数国产电动夹爪走 Modbus-RTU，
这里给出标准骨架：连接串口 -> 写目标位置/力/速 寄存器 -> 读状态。

⚠️ TODO：以下寄存器地址/数值范围为占位，拿到沃姆通信协议手册后按实际填写：
   - REG_TARGET_POS / REG_FORCE / REG_SPEED / REG_INIT / REG_STATUS
   - POS_COUNT_MAX（满行程对应的计数值，常见 0~1000 或 0~255）
   - 是否需要上电初始化(回零)指令

依赖: pip install pymodbus pyserial
对外服务: /gripper/set_gripper  (bin_picking_interfaces/SetGripper)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from bin_picking_interfaces.srv import SetGripper

try:
    from pymodbus.client import ModbusSerialClient
    _HAS_PYMODBUS = True
except ImportError:  # 允许在未装 pymodbus 时也能加载（仿真/占位）
    _HAS_PYMODBUS = False


# ====== TODO: 按沃姆协议手册填写 ======
REG_INIT = 0x0100        # 初始化/回零寄存器（占位）
REG_TARGET_POS = 0x0103  # 目标位置寄存器（占位）
REG_FORCE = 0x0101       # 力寄存器（占位）
REG_SPEED = 0x0104       # 速度寄存器（占位）
REG_STATUS = 0x0200      # 状态寄存器（占位）
POS_COUNT_MAX = 1000     # 满行程(50mm)对应计数（占位）
STROKE_M = 0.050         # 行程 50mm
# =====================================


class GripperDriver(Node):

    def __init__(self):
        super().__init__('gripper_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('simulate', False)  # True 时不连硬件，仅打印

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.slave_id = self.get_parameter('slave_id').value
        self.simulate = self.get_parameter('simulate').value

        self.client = None
        if not self.simulate:
            self._connect()

        self.srv = self.create_service(
            SetGripper, '/gripper/set_gripper', self.on_set_gripper)
        self.get_logger().info(
            f'夹爪驱动已启动 (port={self.port}, simulate={self.simulate})')

    def _connect(self):
        if not _HAS_PYMODBUS:
            self.get_logger().error('未安装 pymodbus，无法连接夹爪。pip install pymodbus')
            return
        self.client = ModbusSerialClient(
            port=self.port, baudrate=self.baudrate,
            bytesize=8, parity='N', stopbits=1, timeout=1.0)
        if self.client.connect():
            self.get_logger().info(f'已连接夹爪串口 {self.port}')
            # TODO: 若夹爪上电需初始化(回零)，在此写 REG_INIT
            # self._write(REG_INIT, 1)
        else:
            self.get_logger().error(f'连接夹爪串口失败: {self.port}')

    def _write(self, address: int, value: int) -> bool:
        if self.simulate or self.client is None:
            self.get_logger().info(f'[模拟] 写寄存器 {hex(address)} = {value}')
            return True
        rr = self.client.write_register(address, value, slave=self.slave_id)
        return not rr.isError()

    def width_to_counts(self, width_m: float) -> int:
        """开口宽度(m) -> 夹爪位置计数。"""
        width_m = max(0.0, min(STROKE_M, width_m))
        return int(round(width_m / STROKE_M * POS_COUNT_MAX))

    def on_set_gripper(self, request: SetGripper.Request,
                       response: SetGripper.Response):
        pos = self.width_to_counts(request.width)
        force = int(max(0.0, min(100.0, request.force)))
        speed = int(max(0.0, min(100.0, request.speed)))

        ok = True
        # 顺序：先设力/速，再设目标位置（具体顺序以手册为准）
        ok &= self._write(REG_FORCE, force)
        ok &= self._write(REG_SPEED, speed)
        ok &= self._write(REG_TARGET_POS, pos)

        response.success = ok
        response.message = (
            f'width={request.width:.3f}m -> pos={pos}, force={force}, speed={speed}'
            if ok else '写夹爪寄存器失败')
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GripperDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.client is not None:
            node.client.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
