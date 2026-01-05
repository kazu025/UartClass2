#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include "UartDma.h"

static uint32_t crc32_table[256];
static bool crc32_init_done = false;

static void crc32_init() {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        crc32_table[i] = c;
    }
    crc32_init_done = true;
}

static inline uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint8_t idx = (uint8_t)((crc ^ data[i]) & 0xFFu);
        crc = crc32_table[idx] ^ (crc >> 8);
    }
    return crc;
}

static uint32_t crc32_calc(const uint8_t* header8, const uint8_t* payload, size_t payload_len) {
    if (!crc32_init_done) crc32_init();
    uint32_t crc = 0xFFFFFFFFu;
    crc = crc32_update(crc, header8, 8);
    crc = crc32_update(crc, payload, payload_len);
    return crc ^ 0xFFFFFFFFu;
}

static bool read_exact(UartDma& ud, uint8_t* buf, size_t len) {
    size_t received = 0;
    while (received < len) {
        int n = ud.read_bytes(buf + received, len - received);
        if (n > 0) received += (size_t)n;
        else tight_loop_contents();
    }
    return true;
}

// 1バイトずつ滑らせて MAGIC を探す（再同期）
static bool sync_to_magic(UartDma& ud, uint32_t magic_le) {
    uint8_t b;
    uint32_t win = 0;
    // まず4バイト貯める
    for (int i = 0; i < 4; i++) {
        read_exact(ud, &b, 1);
        win = (win >> 8) | ((uint32_t)b << 24); // little-endian窓に合わせるための窓
    }
    if (win == magic_le) return true;

    // 以降1バイトずつシフト
    while (true) {
        read_exact(ud, &b, 1);
        win = (win >> 8) | ((uint32_t)b << 24);
        if (win == magic_le) return true;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(1000);

    // 必要ならボーレート合わせる（PC側と同じに）
    UartDma uart(uart0, 921600, 0, 1, 1024, 32768); // RXリングは大きめ推奨
    uart.init();
    sleep_ms(1000);

    constexpr uint32_t MAGIC = 0xA55A5AA5u;
    constexpr size_t BLOCK = 4096;
//    constexpr uint32_t N = 2000;
    constexpr uint32_t N = 480000;

    static uint8_t payload[BLOCK];
    static uint8_t buf4[4];

    uint64_t total = 0;
    uint32_t ok = 0, ng_crc = 0, ng_seq = 0, sync_lost = 0;
    uint32_t expected_seq = 0;

    absolute_time_t next_print = make_timeout_time_ms(30 * 1000);

    printf("=== RX SEQ+CRC32 check start ===\n");

    while (total < N) {
        // MAGIC に同期（little-endianで4バイトは A5 5A 5A A5 の順で来る想定）
        if (!sync_to_magic(uart, MAGIC)) {
            sync_lost++;
            continue;
        }

        // SEQ 読む
        read_exact(uart, buf4, 4);
        uint32_t seq =
            (uint32_t)buf4[0] |
            ((uint32_t)buf4[1] << 8) |
            ((uint32_t)buf4[2] << 16) |
            ((uint32_t)buf4[3] << 24);

        // PAYLOAD
        read_exact(uart, payload, BLOCK);

        // CRC
        read_exact(uart, buf4, 4);
        uint32_t recv_crc =
            (uint32_t)buf4[0] |
            ((uint32_t)buf4[1] << 8) |
            ((uint32_t)buf4[2] << 16) |
            ((uint32_t)buf4[3] << 24);

        // header8 を Python と同じ並びで作る（<II: MAGIC, SEQ）
        uint8_t header8[8];
        header8[0] = (uint8_t)(MAGIC & 0xFF);
        header8[1] = (uint8_t)((MAGIC >> 8) & 0xFF);
        header8[2] = (uint8_t)((MAGIC >> 16) & 0xFF);
        header8[3] = (uint8_t)((MAGIC >> 24) & 0xFF);
        header8[4] = (uint8_t)(seq & 0xFF);
        header8[5] = (uint8_t)((seq >> 8) & 0xFF);
        header8[6] = (uint8_t)((seq >> 16) & 0xFF);
        header8[7] = (uint8_t)((seq >> 24) & 0xFF);

        uint32_t calc_crc = crc32_calc(header8, payload, BLOCK);

        // 連番チェック
        if (seq != expected_seq) {
            ng_seq++;
            // 期待値を追従（ここは運用方針で変えられる）
            expected_seq = seq + 1;
        } else {
            expected_seq++;
        }

        // CRCチェック
        if (calc_crc == recv_crc) ok++;
        else ng_crc++;

        total++;

        // 30秒ごと表示
        if (absolute_time_diff_us(get_absolute_time(), next_print) < 0) {
            printf("T:%llu OK:%lu CRC_NG:%lu SEQ_NG:%lu SYNC:%lu last_seq:%lu\n",
                   total, ok, ng_crc, ng_seq, sync_lost, (unsigned long)seq);
            next_print = make_timeout_time_ms(30 * 1000);
        }
    }

    printf("=== DONE === T:%llu OK:%lu CRC_NG:%lu SEQ_NG:%lu SYNC:%lu expected_seq:%lu\n",
           total, ok, ng_crc, ng_seq, sync_lost, (unsigned long)expected_seq);

    while (true) tight_loop_contents();
}
