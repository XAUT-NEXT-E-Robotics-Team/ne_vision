# 串口性能测试：500Hz发送，验证C++回环，统计丢包率
# TX cmd=0x01: [0xAA][0x55][0x01][float32][int32][crc16]
# RX cmd=0x02: echo from C++ side

import os
import pty
import struct
import time
import threading
import random

HEADER     = b'\xAA\x55'
CMD_TX     = 0x01
CMD_RX     = 0x02
DROP_PROB  = 0.05
TX_HZ      = 500
FRAME_SIZE = 11  # 2+1+4+4 = 11 bytes body, +2 crc = 13... recalc below

# DJI CRC16: poly=0x8408, init=0xFFFF
CRC16_TABLE = [0] * 256
for i in range(256):
    crc = i
    for _ in range(8):
        crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    CRC16_TABLE[i] = crc

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ b) & 0xFF]
    return crc

def make_frame(cmd: int, f: float, i: int) -> bytes:
    payload = struct.pack('<fi', f, i)
    body    = HEADER + bytes([cmd]) + payload
    return body + struct.pack('<H', crc16(body))

FRAME_LEN = len(make_frame(CMD_TX, 0.0, 0))  # 13 bytes

def corrupt(frame: bytes) -> bytes:
    if random.random() < DROP_PROB:
        idx = random.randrange(len(frame))
        return frame[:idx] + frame[idx+1:]
    return frame

# === TX thread ===
sent_count  = 0  # total frames sent (including corrupted)
valid_count = 0  # frames sent intact (C++ should receive these)
sent_lock   = threading.Lock()

def tx_thread(fd):
    global sent_count, valid_count
    global sent_count
    interval = 1.0 / TX_HZ
    counter  = 0
    while True:
        t0    = time.monotonic()
        frame    = make_frame(CMD_TX, float(counter), counter)
        data     = corrupt(frame)
        is_valid = len(data) == len(frame)
        os.write(fd, data)
        with sent_lock:
            sent_count += 1
            if is_valid:
                valid_count += 1
        counter += 1
        elapsed = time.monotonic() - t0
        sleep   = interval - elapsed
        if sleep > 0:
            time.sleep(sleep)

# === RX thread: parse echo frames from C++ ===
recv_count = 0
recv_lock  = threading.Lock()

def rx_thread(fd):
    global recv_count
    global recv_count
    buf = b''
    hdr = bytes([0xAA, 0x55, CMD_RX])
    while True:
        try:
            chunk = os.read(fd, 256)
            buf  += chunk
        except OSError:
            continue
        while len(buf) >= FRAME_LEN:
            idx = buf.find(hdr)
            if idx < 0:
                buf = buf[-2:]
                break
            if idx > 0:
                buf = buf[idx:]
                continue
            if len(buf) < FRAME_LEN:
                break
            frame = buf[:FRAME_LEN]
            buf   = buf[FRAME_LEN:]
            if crc16(frame[:-2]) == struct.unpack('<H', frame[-2:])[0]:
                with recv_lock:
                    recv_count += 1

# === Stats thread ===
def stats_thread():
    prev_sent  = 0
    prev_valid = 0
    prev_recv  = 0
    while True:
        time.sleep(1)
        with sent_lock:
            s = sent_count
            v = valid_count
        with recv_lock:
            r = recv_count
        ds = s - prev_sent
        dv = v - prev_valid
        dr = r - prev_recv
        loss = (1 - dr / dv) * 100 if dv > 0 else 0
        print(f"[STAT] TX={ds}/s  valid={dv}/s  RX={dr}/s  loss={loss:.1f}%  "
              f"total sent={s} valid={v} recv={r}")
        prev_sent  = s
        prev_valid = v
        prev_recv  = r

master, slave = pty.openpty()
slave_name = os.ttyname(slave)
print(f"[SIM] Virtual serial port: {slave_name}")
print(f"[SIM] Frame={FRAME_LEN}B  TX={TX_HZ}Hz  corrupt={DROP_PROB*100:.0f}%")

for fn, args in [(tx_thread, (master,)),
                 (rx_thread, (master,)),
                 (stats_thread, ())]:
    threading.Thread(target=fn, args=args, daemon=True).start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n[SIM] Stopped")
