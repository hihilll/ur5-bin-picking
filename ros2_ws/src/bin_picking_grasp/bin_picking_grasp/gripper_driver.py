"""沃姆 EPGC-50-150 二指电动夹爪驱动节点（RS485 / Modbus-RTU，行程 50mm）。

接线（2026-07 已确认）：
  供电 = M8 8针航插 → UR5 工具口 24V（示教器 Tool I/O 设 24V，CB3 限流 600mA）
  通讯 = 飞线 485A/B → USB转RS485 转换器 → 主机 /dev/ttyUSB0

寄存器表（摘自《电动夹爪 EPG2 系列》手册"指令总览"，用户拍照核实）：
  反馈(只读):  0x0002 错误标志(bit0未初始化 bit1校准错误 bit2电机失能)
               0x0004 工况(0到位 1运动中 2夹住 3掉落 4卸力/未初始化)
               0x0007 母线电压mV   0x0010 位置反馈U32
  运动控制:    0x0020 速度  0x0021 加速度  0x0022 减速度
               0x0023 电流(力)  0x0024 目标位置(U32,占2寄存器)
  开关量:      0x00E4 bit0清错误 bit1校准开始   0x00E2 bit0关闭卸力
  通讯设置:    0x00F0 站号  0x00F1 波特率(0=9600 ... 6=115200)
  设备定义:    0x1114 满行程mm  0x1115 满速mm/s  0x1117 满电流mA (只读)

单位约定：运动量一律为"满量程的万分值"[0,10000]；满量程实际值从设备定义区读取。
上位机各处的 width 均指**指尖开口**（用户自制二指手指）：
  指尖开口 = width_offset(滑块闭合到底时指尖残隙,卡尺标定) + 滑块行程

⚠️ 上电若未初始化会自动触发校准（auto_calibrate 参数控制）——
   校准会让夹爪**全行程开合一次**，首次上电务必确保手指周围无障碍。

真机核对点（盲写自手册，报错按此排查）：
  [V1] U32 写入字序：默认高字在前 [hi,lo]；若写位置后不动/报非法值，改成 [lo,hi]
  [V2] 若通电后写位置不动且工况=4(卸力)：需先写 0x00E2 bit0(=1) 关闭卸力
  [V3] 默认波特率/站号：用 scripts/test_gripper_modbus.py --scan-slaves 探测

依赖: pip install pymodbus pyserial
对外服务: /gripper/set_gripper  (bin_picking_interfaces/SetGripper)
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from bin_picking_interfaces.srv import SetGripper

try:
    from pymodbus.client import ModbusSerialClient
    _HAS_PYMODBUS = True
except ImportError:  # 允许在未装 pymodbus 时也能加载（仿真/占位）
    _HAS_PYMODBUS = False


def _id_kw(client) -> str:
    """pymodbus>=3.9 把从站参数 slave= 改名 device_id=，运行时探测该用哪个。
    （用旧参数名在新版下会抛 TypeError，曾表现为'夹爪全部无应答'）"""
    import inspect
    params = inspect.signature(client.read_holding_registers).parameters
    return 'device_id' if 'device_id' in params else 'slave'


# ====== 寄存器地址（《电动夹爪EPG2系列》手册 指令总览）======
REG_ERROR = 0x0002        # 错误标志 bit0=未初始化 bit1=校准错误 bit2=电机失能
REG_STATUS = 0x0004       # 工况: 见 STATUS_*
REG_BUS_MV = 0x0007       # 母线电压 mV（诊断 CB3 供电用）
REG_POS_FB = 0x0010       # 夹持位置反馈 U32 万分值
REG_SPEED = 0x0020        # 设置夹持速度 万分值
REG_ACCEL = 0x0021        # 设置夹持加速度 万分值
REG_DECEL = 0x0022        # 设置夹持减速度 万分值
REG_CURRENT = 0x0023      # 设置夹持电流(力) 万分值
REG_TARGET_POS = 0x0024   # 设置夹持位置 U32 万分值（占 0x24/0x25 两个寄存器）
REG_CTRL_AUTO = 0x00E4    # 开关量(自动结束): bit0清错误 bit1校准开始
REG_CTRL_END = 0x00E2     # 开关量(结束接口): bit0关闭卸力 [V2]
REG_FULL_STROKE = 0x1114  # 满行程 mm（只读）
REG_FULL_CURRENT = 0x1117 # 满电流 mA（只读）

COUNT_MAX = 10000         # 万分值满量程

STATUS_REACHED = 0        # 到位
STATUS_MOVING = 1         # 运动中
STATUS_CLAMPED = 2        # 夹住
STATUS_DROPPED = 3        # 掉落
STATUS_UNINIT = 4         # 卸力或未初始化
STATUS_NAMES = {0: '到位', 1: '运动中', 2: '夹住', 3: '掉落', 4: '卸力/未初始化'}
# =====================================


class GripperDriver(Node):

    def __init__(self):
        super().__init__('gripper_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)   # [V3] 以实测扫描结果为准
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('simulate', False)  # True 时不连硬件，仅打印
        # 自制手指标定：滑块完全闭合(计数0)时两指尖夹持面间的实际开口(m)。
        # 上位机各处的 width 均指"指尖开口"，指尖开口 = width_offset + 滑块行程。
        # 标定：发闭合到底指令后用卡尺量指尖间隙即为此值；指尖能贴合则为 0。
        self.declare_parameter('width_offset', 0.0)
        self.declare_parameter('auto_calibrate', True)   # 上电未初始化时自动校准
        self.declare_parameter('calibrate_timeout', 15.0)
        # 请求里 force/speed<=0 时的兜底百分比（写 0 电流电机不出力、动不了）
        self.declare_parameter('default_force_percent', 30.0)
        self.declare_parameter('default_speed_percent', 50.0)
        self.declare_parameter('accel_percent', 50.0)    # 启动时写一次加/减速
        # 动作完成等待：写完位置后轮询工况直到 到位/夹住 或超时；0=不等待
        self.declare_parameter('move_timeout', 3.0)

        gp = self.get_parameter
        self.port = gp('port').value
        self.baudrate = gp('baudrate').value
        self.slave_id = gp('slave_id').value
        self.simulate = gp('simulate').value
        self.width_offset = gp('width_offset').value
        self.auto_calibrate = gp('auto_calibrate').value
        self.calibrate_timeout = gp('calibrate_timeout').value
        self.default_force = gp('default_force_percent').value
        self.default_speed = gp('default_speed_percent').value
        self.accel_percent = gp('accel_percent').value
        self.move_timeout = gp('move_timeout').value

        self.stroke_m = 0.050      # 满行程(EPGC-50)，连上后从 0x1114 读实际值覆盖
        self.client = None
        if not self.simulate:
            self._connect()

        self.srv = self.create_service(
            SetGripper, '/gripper/set_gripper', self.on_set_gripper)
        self.get_logger().info(
            f'夹爪驱动已启动 (port={self.port}, simulate={self.simulate}, '
            f'stroke={self.stroke_m*1000:.0f}mm, width_offset='
            f'{self.width_offset*1000:.1f}mm)')

    # ---------- 底层读写 ----------
    def _connect(self):
        if not _HAS_PYMODBUS:
            self.get_logger().error('未安装 pymodbus，无法连接夹爪。pip install pymodbus')
            return
        self.client = ModbusSerialClient(
            port=self.port, baudrate=self.baudrate,
            bytesize=8, parity='N', stopbits=1, timeout=1.0)
        if not self.client.connect():
            self.get_logger().error(f'连接夹爪串口失败: {self.port}')
            self.client = None
            return
        self._id_kw = _id_kw(self.client)
        self.get_logger().info(f'已连接夹爪串口 {self.port} @ {self.baudrate}')

        # 读设备定义：满行程/满电流（读不到就用默认值继续）
        stroke_mm = self._read(REG_FULL_STROKE)
        if stroke_mm:
            self.stroke_m = stroke_mm[0] / 1000.0
            self.get_logger().info(f'满行程(0x1114) = {stroke_mm[0]} mm')
        full_ma = self._read(REG_FULL_CURRENT)
        if full_ma:
            self.get_logger().info(
                f'满电流(0x1117) = {full_ma[0]} mA'
                + ('  ⚠️ 超过 CB3 工具口 600mA，全力夹持可能掉电，'
                   '建议限制 force 或改外部供电' if full_ma[0] > 600 else ''))
        bus_mv = self._read(REG_BUS_MV)
        if bus_mv:
            self.get_logger().info(f'母线电压(0x0007) = {bus_mv[0]} mV')

        # 未初始化则校准（全行程开合一次！）
        err = self._read(REG_ERROR)
        if err is not None and (err[0] & 0x01):
            if self.auto_calibrate:
                self._calibrate()
            else:
                self.get_logger().warn(
                    '夹爪未初始化(0x0002 bit0=1) 且 auto_calibrate=false，'
                    '写位置将无效；请先手动校准')

        # 写一次加/减速
        acc = self._percent_to_permyriad(self.accel_percent)
        self._write(REG_ACCEL, acc)
        self._write(REG_DECEL, acc)

    def _calibrate(self) -> bool:
        self.get_logger().warn('夹爪未初始化，触发校准(0x00E4 bit1)——将全行程开合一次！')
        if not self._write(REG_CTRL_AUTO, 0x0002):
            return False
        start = time.time()
        while time.time() - start < self.calibrate_timeout:
            time.sleep(0.2)
            err = self._read(REG_ERROR)
            if err is not None and not (err[0] & 0x01):
                self.get_logger().info(f'校准完成({time.time()-start:.1f}s)')
                return True
        self.get_logger().error(
            f'校准超时({self.calibrate_timeout}s)，检查供电/障碍物；'
            '若工况=4(卸力)见 [V2]：写 0x00E2 bit0 关闭卸力')
        return False

    def _read(self, address: int, count: int = 1):
        """读保持寄存器，返回值列表；失败/仿真返回 None。"""
        if self.simulate or self.client is None:
            return None
        try:
            rr = self.client.read_holding_registers(
                address, count=count, **{self._id_kw: self.slave_id})
            if rr.isError():
                return None
            return list(rr.registers)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'读寄存器 {hex(address)} 异常: {e}')
            return None

    def _write(self, address: int, value: int) -> bool:
        if self.simulate or self.client is None:
            self.get_logger().info(f'[模拟] 写寄存器 {hex(address)} = {value}')
            return True
        try:
            rr = self.client.write_register(
                address, value, **{self._id_kw: self.slave_id})
            return not rr.isError()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'写寄存器 {hex(address)} 异常: {e}')
            return False

    def _write_u32(self, address: int, value: int) -> bool:
        """写 U32（占两个寄存器）。[V1] 默认高字在前，若设备拒收改 [lo, hi]。"""
        hi, lo = (value >> 16) & 0xFFFF, value & 0xFFFF
        if self.simulate or self.client is None:
            self.get_logger().info(f'[模拟] 写U32 {hex(address)} = {value}')
            return True
        try:
            rr = self.client.write_registers(
                address, [hi, lo], **{self._id_kw: self.slave_id})
            return not rr.isError()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'写U32 {hex(address)} 异常: {e}')
            return False

    # ---------- 换算 ----------
    @staticmethod
    def _percent_to_permyriad(percent: float) -> int:
        """百分比(0~100) -> 万分值(0~10000)。"""
        return int(round(max(0.0, min(100.0, percent)) * 100.0))

    def width_to_counts(self, width_m: float) -> int:
        """指尖开口宽度(m) -> 位置万分值（扣除自制手指的闭合残余开口）。"""
        travel = width_m - self.width_offset
        travel = max(0.0, min(self.stroke_m, travel))
        return int(round(travel / self.stroke_m * COUNT_MAX))

    def counts_to_width(self, counts: int) -> float:
        return self.width_offset + counts / COUNT_MAX * self.stroke_m

    # ---------- 动作完成等待 ----------
    def _wait_motion_done(self, timeout: float):
        """轮询工况直到离开'运动中'。返回 (状态码 或 None, 反馈开口m 或 None)。"""
        start = time.time()
        status = None
        while time.time() - start < timeout:
            time.sleep(0.05)
            st = self._read(REG_STATUS)
            if st is None:
                return None, None
            status = st[0]
            if status != STATUS_MOVING:
                break
        fb = self._read(REG_POS_FB, count=2)
        width_fb = None
        if fb is not None:
            width_fb = self.counts_to_width((fb[0] << 16) | fb[1])  # [V1] 字序同写
        return status, width_fb

    # ---------- 服务 ----------
    def on_set_gripper(self, request: SetGripper.Request,
                       response: SetGripper.Response):
        pos = self.width_to_counts(request.width)
        force = self._percent_to_permyriad(
            request.force if request.force > 0 else self.default_force)
        speed = self._percent_to_permyriad(
            request.speed if request.speed > 0 else self.default_speed)

        ok = True
        ok &= self._write(REG_SPEED, speed)
        ok &= self._write(REG_CURRENT, force)
        ok &= self._write_u32(REG_TARGET_POS, pos)
        if not ok:
            response.success = False
            response.message = '写夹爪寄存器失败（检查串口/站号/供电）'
            self.get_logger().warn(response.message)
            return response

        msg = (f'width={request.width*1000:.1f}mm -> pos={pos}/{COUNT_MAX}, '
               f'force={force/100:.0f}%, speed={speed/100:.0f}%')

        if self.move_timeout > 0 and not self.simulate and self.client is not None:
            status, width_fb = self._wait_motion_done(self.move_timeout)
            if status is not None:
                msg += f' | 工况: {STATUS_NAMES.get(status, status)}({status})'
                if width_fb is not None:
                    msg += f', 实际开口 {width_fb*1000:.1f}mm'
                # 掉落/未初始化视为失败；到位与夹住均算成功
                # （闭合抓取时"夹住"=夹到零件，"到位"=闭到目标宽度未碰到物
                #  ——是否算抓空由上层结合场景判断）
                ok = status in (STATUS_REACHED, STATUS_CLAMPED)

        response.success = ok
        response.message = msg
        self.get_logger().info(msg)
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
