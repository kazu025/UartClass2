#!/usr/bin/env python3
import serial
import struct
import zlib
import time
import os
import binascii

PORT = "/dev/ttyUSB0"
BAUD = 460800
PAYLOAD = 4096
N = 240000
PRINT_INTERVAL = 5

MAGIC_BYTES = b"\xA5\x5A\x5A\xA5"
FRAME_LEN = 4 + 4 + PAYLOAD + 4

# 保存設定
OUTDIR = "rx_dumps"
AROUND_N = 512         # SYNC時：MAGIC前後に保存するバイト数
PREV_RAW_KEEP = 4096   # 「直前に受け取った生データ」を保持するリング（原因追跡用）
MAX_DUMPS = 200        # 保存しすぎ防止（必要なら増やしてOK）

def hx(b: bytes, maxlen=64) -> str:
    return binascii.hexlify(b[:maxlen]).decode()

def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)

def save_bin(path: str, data: bytes):
    with open(path, "wb") as f:
        f.write(data)

def frame_info(frame: bytes):
    seq = struct.unpack_from("<I", frame, 4)[0]
    recv_crc = struct.unpack_from("<I", frame, 8 + PAYLOAD)[0]
    calc_crc = zlib.crc32(frame[:8] + frame[8:8 + PAYLOAD]) & 0xFFFFFFFF
    return seq, recv_crc, calc_crc

def main():
    ensure_dir(OUTDIR)

    ser = serial.Serial(PORT, BAUD, timeout=0.05)  # 少し短めのtimeoutが無難
    ser.reset_input_buffer()

    buf = bytearray()
    prev_raw = bytearray()  # 直前に受けた生データを保持（リングっぽく使う）

    total = ok = crc_ng = seq_ng = sync = 0
    expected_seq = 0
    locked = False

    dump_count = 0
    t_next = time.time() + PRINT_INTERVAL

    print("=== RX check start ===")

    while total < N:
        chunk = ser.read(65536)
        if chunk:
            buf += chunk

            # 直前のraw保持（PREV_RAW_KEEPまで）
            prev_raw += chunk
            if len(prev_raw) > PREV_RAW_KEEP:
                del prev_raw[:-PREV_RAW_KEEP]

        # ロックしてないなら MAGIC探し
        if not locked:
            pos = buf.find(MAGIC_BYTES)
            if pos < 0:
                # 肥大化防止：末尾だけ残す
                if len(buf) > 2 * 1024 * 1024:
                    del buf[:-4]
                continue

            if pos > 0:
                sync += 1
                if dump_count < MAX_DUMPS:
                    left = max(0, pos - AROUND_N)
                    right = min(len(buf), pos + 4 + AROUND_N)
                    around = bytes(buf[left:right])

                    ts = int(time.time() * 1000)
                    base = f"{OUTDIR}/sync_t{total}_pos{pos}_ts{ts}"
                    save_bin(base + ".bin", around)
                    # 直前rawも保存（欠落の直後に何が来てたかが見える）
                    save_bin(base + "_prevraw.bin", bytes(prev_raw))
                    dump_count += 2

            # MAGIC位置に合わせる
            del buf[:pos]
            locked = True

        # 固定長で切り出し
        if len(buf) < FRAME_LEN:
            continue

        frame = bytes(buf[:FRAME_LEN])
        del buf[:FRAME_LEN]

        # MAGICがズレてたらアンロックして探し直し
        if frame[:4] != MAGIC_BYTES:
            locked = False
            continue

        seq, recv_crc, calc_crc = frame_info(frame)

        # SEQチェック
        if seq != expected_seq:
            seq_ng += 1
            if dump_count < MAX_DUMPS:
                ts = int(time.time() * 1000)
                base = f"{OUTDIR}/seqng_t{total}_exp{expected_seq}_got{seq}_ts{ts}"
                save_bin(base + ".bin", frame)
                save_bin(base + "_prevraw.bin", bytes(prev_raw))
                dump_count += 2
            # 再同期：ここでexpectedを追従させる（続行するため）
            expected_seq = seq + 1
        else:
            expected_seq += 1

        # CRCチェック
        if calc_crc == recv_crc:
            ok += 1
        else:
            crc_ng += 1
            if dump_count < MAX_DUMPS:
                ts = int(time.time() * 1000)
                base = f"{OUTDIR}/crcng_t{total}_seq{seq}_ts{ts}"
                save_bin(base + ".bin", frame)
                save_bin(base + "_prevraw.bin", bytes(prev_raw))
                dump_count += 2
            # CRC NG 時はアンロックして次のMAGICを探す
            locked = False

        total += 1

        now = time.time()
        if now >= t_next:
            # printは最小限
            print(f"T:{total} OK:{ok} CRC_NG:{crc_ng} SEQ_NG:{seq_ng} SYNC:{sync} expected_seq:{expected_seq} dumps:{dump_count}")
            t_next = now + PRINT_INTERVAL

    print(f"=== DONE === T:{total} OK:{ok} CRC_NG:{crc_ng} SEQ_NG:{seq_ng} SYNC:{sync} expected_seq:{expected_seq} dumps:{dump_count}")

if __name__ == "__main__":
    main()
