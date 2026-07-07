#!/usr/bin/env python3
"""沃姆 W-EPGC 夹爪写寄存器诊断·第二轮。

第一轮结论：FC06 不支持(异常码1)；U32 低字在前(反馈[9999,0]=全开、
写[0,10000]报异常码3)；参数区出厂全 0 所以之前写位置不动；
0x0022 起始的 FC16 无应答(疑似只接受特定块起始地址)。

本轮：① 扫一遍可读寄存器摸清真实地图；② 从 0x0020 连写 4 个参数
(速度/加速度/减速度/电流)；③ 低字在前 写位置 闭合→张开，动作中读工况
确认状态枚举。

⚠️ 会让夹爪 闭合再张开 各一次，手指周围清空！

用法: python3 scripts/diag_gripper_write2.py [--port /dev/ttyUSB0] [--slave 1]
"""

import argparse
import time

from pymodbus.client import ModbusSerialClient


def show(tag, rr):
    print(f'{tag}: isError={rr.isError()}, '
          f'异常码={getattr(rr, "exception_code", None)}')


def read(c, slave, addr, count=1):
    rr = c.read_holding_registers(addr, count=count, slave=slave)
    return None if rr.isError() else list(rr.registers)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--slave', type=int, default=1)
    ap.add_argument('--skip-scan', action='store_true', help='跳过寄存器扫描')
    args = ap.parse_args()

    c = ModbusSerialClient(port=args.port, baudrate=args.baud,
                           bytesize=8, parity='N', stopbits=1, timeout=0.3)
    print('connect:', c.connect())

    if not args.skip_scan:
        print('--- ① 可读寄存器扫描（单读，只列有应答的）---')
        ranges = (list(range(0x0000, 0x0012)) + list(range(0x0020, 0x0028))
                  + list(range(0x00E0, 0x00E8)) + list(range(0x00F0, 0x00F4))
                  + list(range(0x1110, 0x1119)))
        for addr in ranges:
            vals = read(c, args.slave, addr)
            if vals is not None:
                print(f'  {hex(addr)} = {vals[0]}')

    print('--- ② 参数区 4 连写（速度/加速度/减速度/电流 各30%）---')
    show('FC16 0x0020=[3000,3000,3000,3000]',
         c.write_registers(0x0020, [3000, 3000, 3000, 3000],
                           slave=args.slave))
    print('  读回 0x0020~0x0025 =', read(c, args.slave, 0x0020, 6))

    print('--- ③ 位置写入·低字在前 ---')
    show('闭合 0x0024=[0,0]',
         c.write_registers(0x0024, [0, 0], slave=args.slave))
    time.sleep(0.3)
    print('  动作中工况 =', read(c, args.slave, 0x0004))
    time.sleep(2.0)
    print('  完成后工况 =', read(c, args.slave, 0x0004),
          ' 位置反馈 =', read(c, args.slave, 0x0010, 2))

    show('张开 0x0024=[10000,0]',
         c.write_registers(0x0024, [10000, 0], slave=args.slave))
    time.sleep(0.3)
    print('  动作中工况 =', read(c, args.slave, 0x0004))
    time.sleep(2.0)
    print('  完成后工况 =', read(c, args.slave, 0x0004),
          ' 位置反馈 =', read(c, args.slave, 0x0010, 2))
    c.close()


if __name__ == '__main__':
    main()
