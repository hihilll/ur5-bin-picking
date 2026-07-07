#!/usr/bin/env python3
"""沃姆 W-EPGC 夹爪写寄存器诊断（一次性）。

背景：真机实测该夹爪不支持 FC06 写单寄存器（异常码1），FC16 写位置成功；
本脚本进一步确认：① 参数区(速度/加/减/电流)按 2 个一组 FC16 写通不通；
② 目标位置 U32 字序（写 [0,10000] 张开=高字在前正确）；③ 位置反馈读数。

⚠️ 会让夹爪张开到满行程，运行前手指周围清空！

用法: python3 scripts/diag_gripper_write.py [--port /dev/ttyUSB0] [--slave 1]
"""

import argparse
import time

from pymodbus.client import ModbusSerialClient


def show(tag, rr):
    print(f'{tag}: isError={rr.isError()}, '
          f'异常码={getattr(rr, "exception_code", None)}')


def read(c, slave, addr, count, tag):
    rr = c.read_holding_registers(addr, count=count, slave=slave)
    print(f'{tag} =', None if rr.isError() else rr.registers)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--slave', type=int, default=1)
    args = ap.parse_args()

    c = ModbusSerialClient(port=args.port, baudrate=args.baud,
                           bytesize=8, parity='N', stopbits=1, timeout=0.5)
    print('connect:', c.connect())

    read(c, args.slave, 0x0020, 6, '写前读 0x0020~0x0025')

    show('FC16 0x0020=[3000,3000] 速度+加速度',
         c.write_registers(0x0020, [3000, 3000], slave=args.slave))
    show('FC16 0x0022=[3000,3000] 减速+电流',
         c.write_registers(0x0022, [3000, 3000], slave=args.slave))
    show('FC16 0x0024=[0,10000] 张开·高字在前',
         c.write_registers(0x0024, [0, 10000], slave=args.slave))
    time.sleep(2.0)

    read(c, args.slave, 0x0004, 1, '工况0x0004')
    read(c, args.slave, 0x0010, 2, '位置反馈0x0010')
    read(c, args.slave, 0x0020, 6, '写后读 0x0020~0x0025')
    c.close()


if __name__ == '__main__':
    main()
