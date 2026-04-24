#!/usr/bin/env python3
"""
Virtual serial simulator for ne_vision gimbal protocol.

Packet format: [0xAA][0x55][cmd(1)][payload bytes][CRC16_lo][CRC16_hi]
  cmd=0x01 -> GimbalInputProtocol_t  (sim -> ne_vision)
  cmd=0x02 -> GimbalControlProtocol_t (ne_vision -> sim)

Usage:
  python3 serial_sim.py
  # ne_vision connects to /tmp/ttyV1 (default symlink)
  # Or: python3 serial_sim.py --port /dev/ttyUSB0
"""

import argparse
import fcntl
import math
import os
import pty
import struct
import termios
import threading
import time
import serial

_CRC16_TABLE = [
    0x0000,0x1189,0x2312,0x329b,0x4624,0x57ad,0x6536,0x74bf,
    0x8c48,0x9dc1,0xaf5a,0xbed3,0xca6c,0xdbe5,0xe97e,0xf8f7,
    0x1081,0x0108,0x3393,0x221a,0x56a5,0x472c,0x75b7,0x643e,
    0x9cc9,0x8d40,0xbfdb,0xae52,0xdaed,0xcb64,0xf9ff,0xe876,
    0x2102,0x308b,0x0210,0x1399,0x6726,0x76af,0x4434,0x55bd,
    0xad4a,0xbcc3,0x8e58,0x9fd1,0xeb6e,0xfae7,0xc87c,0xd9f5,
    0x3183,0x200a,0x1291,0x0318,0x77a7,0x662e,0x54b5,0x453c,
    0xbdcb,0xac42,0x9ed9,0x8f50,0xfbef,0xea66,0xd8fd,0xc974,
    0x4204,0x538d,0x6116,0x709f,0x0420,0x15a9,0x2732,0x36bb,
    0xce4c,0xdfc5,0xed5e,0xfcd7,0x8868,0x99e1,0xab7a,0xbaf3,
    0x5285,0x430c,0x7197,0x601e,0x14a1,0x0528,0x37b3,0x263a,
    0xdecd,0xcf44,0xfddf,0xec56,0x98e9,0x8960,0xbbfb,0xaa72,
    0x6306,0x728f,0x4014,0x519d,0x2522,0x34ab,0x0630,0x17b9,
    0xef4e,0xfec7,0xcc5c,0xddd5,0xa96a,0xb8e3,0x8a78,0x9bf1,
    0x7387,0x620e,0x5095,0x411c,0x35a3,0x242a,0x16b1,0x0738,
    0xffcf,0xee46,0xdcdd,0xcd54,0xb9eb,0xa862,0x9af9,0x8b70,
    0x8408,0x9581,0xa71a,0xb693,0xc22c,0xd3a5,0xe13e,0xf0b7,
    0x0840,0x19c9,0x2b52,0x3adb,0x4e64,0x5fed,0x6d76,0x7cff,
    0x9489,0x8500,0xb79b,0xa612,0xd2ad,0xc324,0xf1bf,0xe036,
    0x18c1,0x0948,0x3bd3,0x2a5a,0x5ee5,0x4f6c,0x7df7,0x6c7e,
    0xa50a,0xb483,0x8618,0x9791,0xe32e,0xf2a7,0xc03c,0xd1b5,
    0x2942,0x38cb,0x0a50,0x1bd9,0x6f66,0x7eef,0x4c74,0x5dfd,
    0xb58b,0xa402,0x9699,0x8710,0xf3af,0xe226,0xd0bd,0xc134,
    0x39c3,0x284a,0x1ad1,0x0b58,0x7fe7,0x6e6e,0x5cf5,0x4d7c,
    0xc60c,0xd785,0xe51e,0xf497,0x8028,0x91a1,0xa33a,0xb2b3,
    0x4a44,0x5bcd,0x6956,0x78df,0x0c60,0x1de9,0x2f72,0x3efb,
    0xd68d,0xc704,0xf59f,0xe416,0x90a9,0x8120,0xb3bb,0xa232,
    0x5ac5,0x4b4c,0x79d7,0x685e,0x1ce1,0x0d68,0x3ff3,0x2e7a,
    0xe70e,0xf687,0xc41c,0xd595,0xa12a,0xb0a3,0x8238,0x93b1,
    0x6b46,0x7acf,0x4854,0x59dd,0x2d62,0x3ceb,0x0e70,0x1ff9,
    0xf78f,0xe606,0xd49d,0xc514,0xb1ab,0xa022,0x92b9,0x8330,
    0x7bc7,0x6a4e,0x58d5,0x495c,0x3de3,0x2c6a,0x1ef1,0x0f78,
]

HEADER_HI   = 0xAA
HEADER_LO   = 0x55
CMD_INPUT   = 0x01
CMD_CONTROL = 0x02

# GimbalInputProtocol_t: q[4], gyro[3], acc[3], out_color(char), bullet_speed(float)
FMT_INPUT   = '<4f3f3fbf'
# GimbalControlProtocol_t: yaw, pitch, yaw_v, pitch_v, hit_probability
FMT_CONTROL = '<4fB'

_master_fd: int = -1
_slave_fd_keep: int = -1


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ _CRC16_TABLE[(crc ^ b) & 0xFF]
    return crc


def build_packet(cmd: int, payload: bytes) -> bytes:
    frame = bytes([HEADER_HI, HEADER_LO, cmd]) + payload
    return frame + struct.pack('<H', crc16(frame))


def parse_packet(buf: bytearray):
    i = 0
    while i < len(buf) - 1:
        if buf[i] != HEADER_HI or buf[i + 1] != HEADER_LO:
            i += 1
            continue
        if i + 5 > len(buf):
            break
        cmd = buf[i + 2]
        if cmd == CMD_INPUT:
            payload_len = struct.calcsize(FMT_INPUT)
        elif cmd == CMD_CONTROL:
            payload_len = struct.calcsize(FMT_CONTROL)
        else:
            i += 1
            continue
        total = 3 + payload_len + 2
        if i + total > len(buf):
            break
        frame = bytes(buf[i:i + total])
        if crc16(frame[:-2]) != struct.unpack_from('<H', frame, total - 2)[0]:
            i += 1
            continue
        return cmd, frame[3:3 + payload_len], i + total
    return None


def nb_write(data: bytes):
    try:
        os.write(_master_fd, data)
    except BlockingIOError:
        pass


def make_gimbal_input(t: float) -> bytes:
    yaw = t * 0.5
    q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
    payload = struct.pack(FMT_INPUT, *q, 0.0, 0.0, 0.5, 0.0, 0.0, 9.8, 66, 20.0)
    return build_packet(CMD_INPUT, payload)


def make_gimbal_control(t: float) -> bytes:
    yaw, pitch = 0.3 * math.sin(t), 0.1 * math.sin(t * 0.5)
    payload = struct.pack(FMT_CONTROL, yaw, pitch, 0.3 * math.cos(t), 0.05 * math.cos(t * 0.5), 128)
    return build_packet(CMD_CONTROL, payload)


def rx_thread(fd: int):
    buf = bytearray()
    while True:
        try:
            data = os.read(fd, 256)
        except BlockingIOError:
            time.sleep(0.001)
            continue
        buf.extend(data)
        while True:
            result = parse_packet(buf)
            if result is None:
                break
            cmd, payload, consumed = result
            del buf[:consumed]
            if cmd == CMD_INPUT:
                q0,q1,q2,q3,gx,gy,gz,ax,ay,az,color,bspeed = struct.unpack(FMT_INPUT, payload)
                print(f"[RX INPUT ] q=({q0:.3f},{q1:.3f},{q2:.3f},{q3:.3f}) color={color} bspeed={bspeed:.1f}")
            elif cmd == CMD_CONTROL:
                yaw,pitch,yaw_v,pitch_v,hp = struct.unpack(FMT_CONTROL, payload)
                print(f"[RX CTRL  ] yaw={yaw:.3f} pitch={pitch:.3f} hp={hp}")


def make_pty_pair(symlink: str) -> int:
    global _master_fd, _slave_fd_keep
    master_fd, slave_fd = pty.openpty()
    attrs = termios.tcgetattr(master_fd)
    attrs[3] &= ~(termios.ECHO | termios.ICANON)
    termios.tcsetattr(master_fd, termios.TCSANOW, attrs)
    fcntl.fcntl(master_fd, fcntl.F_SETFL,
                fcntl.fcntl(master_fd, fcntl.F_GETFL) | os.O_NONBLOCK)
    slave_path = os.ttyname(slave_fd)
    _slave_fd_keep = slave_fd  # keep open to prevent HUP
    _master_fd = master_fd
    if symlink:
        if os.path.exists(symlink) or os.path.islink(symlink):
            os.unlink(symlink)
        os.symlink(slave_path, symlink)
        print(f"Virtual serial pair created:")
        print(f"  ne_vision  -> {symlink}  (-> {slave_path})")
    else:
        print(f"Virtual serial pair created: ne_vision -> {slave_path}")
    return master_fd


def main():
    global _master_fd
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='', help='Existing port. Empty = create PTY.')
    parser.add_argument('--symlink', default='/tmp/ttyV1')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--mode', choices=['input', 'control', 'both'], default='input')
    parser.add_argument('--hz', type=float, default=100.0)
    args = parser.parse_args()

    if args.port:
        ser = serial.Serial(args.port, args.baud, timeout=0.01)
        _master_fd = ser.fileno()
        print(f"Opened {args.port} @ {args.baud}")
        threading.Thread(target=rx_thread, args=(_master_fd,), daemon=True).start()
    else:
        fd = make_pty_pair(args.symlink)
        threading.Thread(target=rx_thread, args=(fd,), daemon=True).start()

    print(f"mode={args.mode}, hz={args.hz}")
    dt = 1.0 / args.hz
    t = 0.0
    toggle = False
    while True:
        if args.mode == 'input' or (args.mode == 'both' and not toggle):
            nb_write(make_gimbal_input(t))
            print(f"[TX INPUT ] t={t:.2f}")
        if args.mode == 'control' or (args.mode == 'both' and toggle):
            nb_write(make_gimbal_control(t))
            print(f"[TX CTRL  ] t={t:.2f}")
        toggle = not toggle
        t += dt
        time.sleep(dt)


if __name__ == '__main__':
    main()
