#!/usr/bin/env python3
"""沃姆夹爪 RS485/Modbus 链路自检（不依赖 ROS，在 Ubuntu 上直接跑）。

用途：接好 USB转RS485 转换器、示教器给工具口上 24V 后，
先用本脚本验证"串口能开 + 从站有应答"，再进 ROS 跑 gripper_driver。
默认**只读**不写，不会让夹爪动作。

用法:
  python3 test_gripper_modbus.py                        # 默认 /dev/ttyUSB0 115200 从站1
  python3 test_gripper_modbus.py --port /dev/ttyUSB1 --baud 9600 --slave 1
  python3 test_gripper_modbus.py --scan-slaves          # 从站号不确定时 1~10 逐个试
  python3 test_gripper_modbus.py --addr 0x1114 --count 4  # 读满行程/满速/满电流
  python3 test_gripper_modbus.py --addr 0x0007            # 读母线电压(mV,查供电)

  # 写测试（会让夹爪动！确保周围无障碍）。EPG2 常用: 0x0020速度 0x0023力(万分值)
  python3 test_gripper_modbus.py --write 0x0020 5000

无应答排查顺序:
  1) ls /dev/ttyUSB* 确认设备在；报 Permission denied 则
     sudo usermod -aG dialout $USER 后重新登录
  2) 示教器 Installation -> Tool I/O 确认已输出 24V（夹爪有电才会应答）
  3) A/B 线试着对调（485 接反不烧，只是收不到）
  4) 波特率试 9600/19200/38400/115200，从站号用 --scan-slaves
"""

from __future__ import annotations

import argparse
import sys

try:
    from pymodbus.client import ModbusSerialClient
except ImportError:
    sys.exit('未安装 pymodbus: pip install pymodbus pyserial')


def make_client(args) -> ModbusSerialClient:
    return ModbusSerialClient(
        port=args.port, baudrate=args.baud,
        bytesize=8, parity=args.parity, stopbits=1, timeout=args.timeout)


def try_read(client, slave: int, addr: int, count: int):
    """读保持寄存器，成功返回寄存器值列表，失败返回 None。"""
    try:
        rr = client.read_holding_registers(addr, count=count, slave=slave)
        if rr.isError():
            return None
        return list(rr.registers)
    except Exception:  # noqa: BLE001
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--parity', default='N', choices=['N', 'E', 'O'])
    ap.add_argument('--slave', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=0.5)
    ap.add_argument('--addr', type=lambda s: int(s, 0), default=None,
                    help='读指定寄存器地址(支持 0x 前缀)')
    ap.add_argument('--count', type=int, default=2, help='连续读几个寄存器')
    ap.add_argument('--scan-slaves', action='store_true',
                    help='从站号 1~10 逐个探测')
    ap.add_argument('--write', nargs=2, type=lambda s: int(s, 0), default=None,
                    metavar=('ADDR', 'VALUE'),
                    help='⚠️ 写单寄存器(会让夹爪动)，仅在寄存器已按手册确认后使用')
    args = ap.parse_args()

    client = make_client(args)
    if not client.connect():
        sys.exit(f'✗ 打不开串口 {args.port}（设备不在/权限不足？见脚本头部排查顺序）')
    print(f'✓ 串口已打开 {args.port} @ {args.baud} 8{args.parity}1')

    try:
        if args.write is not None:
            addr, value = args.write
            rr = client.write_register(addr, value, slave=args.slave)
            if rr.isError():
                sys.exit(f'✗ 写 {hex(addr)}={value} 失败: {rr}')
            print(f'✓ 已写 {hex(addr)} = {value} (从站 {args.slave})')
            return

        if args.scan_slaves:
            # 探测地址按 EPG2 寄存器表: 错误标志/工况/母线电压/满行程
            probe_addrs = [0x0002, 0x0004, 0x0007, 0x1114]
            found = []
            for slave in range(1, 11):
                for addr in probe_addrs:
                    vals = try_read(client, slave, addr, 1)
                    if vals is not None:
                        print(f'✓ 从站 {slave} 有应答 (寄存器 {hex(addr)} = {vals[0]})')
                        found.append(slave)
                        break
            if not found:
                sys.exit('✗ 1~10 号从站均无应答（检查供电/AB线/波特率，见脚本头部）')
            print(f'探测完成，有应答的从站: {found}')
            return

        # 默认：读 EPG2 关键寄存器验证应答（错误标志/工况/母线电压/满行程）
        addrs = ([args.addr] if args.addr is not None
                 else [0x0002, 0x0004, 0x0007, 0x1114])
        ok = False
        for addr in addrs:
            vals = try_read(client, args.slave, addr, args.count)
            if vals is None:
                print(f'  寄存器 {hex(addr)}: 无应答/异常')
            else:
                print(f'✓ 寄存器 {hex(addr)}~: {vals}')
                ok = True
        if ok:
            print('链路正常：串口通、从站应答。下一步按手册填 '
                  'gripper_driver.py 的 REG_* 后即可进 ROS 测试。')
        else:
            sys.exit('✗ 串口通但从站无应答（检查供电24V/AB线对调/波特率/从站号，'
                     '可用 --scan-slaves 探测）')
    finally:
        client.close()


if __name__ == '__main__':
    main()
