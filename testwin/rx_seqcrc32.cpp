#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <deque>
#include <filesystem>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

static constexpr uint8_t MAGIC[4] = { 0xA5, 0x5A, 0x5A, 0xA5 };

// ---------------- CRC32 (standard, polynomial 0xEDB88320) ----------------
static uint32_t crc32_table[256];
static void init_crc32_table() {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        crc32_table[i] = c;
    }
}
static uint32_t crc32(const uint8_t* data, size_t len) {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        c = crc32_table[(c ^ data[i]) & 0xFFu] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFu;
}

// ---------------- Little-endian helpers ----------------
static uint32_t le_u32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// ---------------- Serial (Win32 overlapped) ----------------
struct Serial {
    HANDLE h = INVALID_HANDLE_VALUE;
    OVERLAPPED ov = {};
    HANDLE ovEvent = NULL;

    bool open(const std::string& comPort, uint32_t baud) {
        // COM10+ needs "\\\\.\\COM10"
        std::string dev = comPort;
        if (dev.rfind("COM", 0) == 0 && dev.size() > 4) {
            dev = "\\\\.\\" + dev;
        }

        h = CreateFileA(dev.c_str(),
                        GENERIC_READ, 0, nullptr,
                        OPEN_EXISTING,
                        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
                        nullptr);
        if (h == INVALID_HANDLE_VALUE) {
            std::cerr << "CreateFile failed: " << GetLastError() << "\n";
            return false;
        }

        // buffers
        SetupComm(h, 1 << 20, 1 << 20); // 1MB

        // timeouts (non-block-ish, overlapped handles wait)
        COMMTIMEOUTS to{};
        to.ReadIntervalTimeout = MAXDWORD;
        to.ReadTotalTimeoutMultiplier = 0;
        to.ReadTotalTimeoutConstant = 0;
        SetCommTimeouts(h, &to);

        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);
        if (!GetCommState(h, &dcb)) {
            std::cerr << "GetCommState failed: " << GetLastError() << "\n";
            return false;
        }
        dcb.BaudRate = baud;
        dcb.ByteSize = 8;
        dcb.Parity   = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        dcb.fRtsControl = RTS_CONTROL_ENABLE;
        // If you later want HW flow:
        // dcb.fOutxCtsFlow = TRUE; dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;

        if (!SetCommState(h, &dcb)) {
            std::cerr << "SetCommState failed: " << GetLastError() << "\n";
            return false;
        }

        PurgeComm(h, PURGE_RXCLEAR | PURGE_RXABORT);

        ovEvent = CreateEventA(nullptr, TRUE, FALSE, nullptr);
        if (!ovEvent) {
            std::cerr << "CreateEvent failed\n";
            return false;
        }
        ov.hEvent = ovEvent;
        return true;
    }

    int read_some(uint8_t* dst, int maxBytes, int timeoutMs) {
        if (h == INVALID_HANDLE_VALUE) return -1;

        ResetEvent(ov.hEvent);
        DWORD got = 0;

        BOOL ok = ReadFile(h, dst, (DWORD)maxBytes, &got, &ov);
        if (!ok) {
            DWORD err = GetLastError();
            if (err != ERROR_IO_PENDING) {
                std::cerr << "ReadFile failed: " << err << "\n";
                return -1;
            }

            DWORD w = WaitForSingleObject(ov.hEvent, timeoutMs);
            if (w == WAIT_TIMEOUT) {
                // cancel pending read
                CancelIo(h);
                return 0;
            }
            if (w != WAIT_OBJECT_0) {
                std::cerr << "WaitForSingleObject failed\n";
                CancelIo(h);
                return -1;
            }

            if (!GetOverlappedResult(h, &ov, &got, FALSE)) {
                std::cerr << "GetOverlappedResult failed: " << GetLastError() << "\n";
                return -1;
            }
        }
        return (int)got;
    }

    void close() {
        if (h != INVALID_HANDLE_VALUE) {
            CancelIo(h);
            CloseHandle(h);
            h = INVALID_HANDLE_VALUE;
        }
        if (ovEvent) {
            CloseHandle(ovEvent);
            ovEvent = NULL;
        }
    }

    ~Serial() { close(); }
};

// ---------------- Utils ----------------
static std::string now_ms_string() {
    using namespace std::chrono;
    auto ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    return std::to_string(ms);
}

static void save_bin(const std::filesystem::path& path, const std::vector<uint8_t>& data) {
    FILE* f = nullptr;
    fopen_s(&f, path.string().c_str(), "wb");
    if (!f) return;
    fwrite(data.data(), 1, data.size(), f);
    fclose(f);
}

static int find_magic(const std::deque<uint8_t>& q) {
    if (q.size() < 4) return -1;
    // naive scan; q isn't huge in practice because we trim
    for (size_t i = 0; i + 4 <= q.size(); i++) {
        if (q[i] == MAGIC[0] && q[i+1] == MAGIC[1] && q[i+2] == MAGIC[2] && q[i+3] == MAGIC[3]) {
            return (int)i;
        }
    }
    return -1;
}

static void pop_front(std::deque<uint8_t>& q, size_t n) {
    while (n-- && !q.empty()) q.pop_front();
}

static std::vector<uint8_t> take_front(const std::deque<uint8_t>& q, size_t n) {
    std::vector<uint8_t> out;
    out.reserve(n);
    for (size_t i = 0; i < n && i < q.size(); i++) out.push_back(q[i]);
    return out;
}

// ---------------- Main receiver ----------------
int main(int argc, char** argv) {
    init_crc32_table();

    // defaults
    std::string port = "COM4";
    uint32_t baud = 115200;
//   uint32_t baud = 460800;
    uint32_t payload = 4096;
    uint32_t N = 240000;
    int printIntervalSec = 5;

    // args: rx.exe COM4 460800 4096 240000
    if (argc >= 2) port = argv[1];
    if (argc >= 3) baud = (uint32_t)std::strtoul(argv[2], nullptr, 10);
    if (argc >= 4) payload = (uint32_t)std::strtoul(argv[3], nullptr, 10);
    if (argc >= 5) N = (uint32_t)std::strtoul(argv[4], nullptr, 10);

    const uint32_t FRAME_LEN = 4 + 4 + payload + 4;
    const size_t AROUND_N = 512;
    const size_t PREV_RAW_KEEP = 4096;
    const size_t MAX_Q = 2 * 1024 * 1024; // avoid unlimited growth

    std::filesystem::path outdir = "rx_dumps";
    std::filesystem::create_directories(outdir);

    Serial ser;
    if (!ser.open(port, baud)) {
        std::cerr << "Failed to open " << port << "\n";
        return 1;
    }

    std::deque<uint8_t> buf;
    std::deque<uint8_t> prevRaw;

    uint32_t total = 0, ok = 0, crc_ng = 0, seq_ng = 0, sync = 0;
    uint32_t expected_seq = 0;
    bool locked = false;
    size_t dump_count = 0;
    const size_t MAX_DUMPS = 200;

    auto t0 = std::chrono::steady_clock::now();
    auto nextPrint = t0 + std::chrono::seconds(printIntervalSec);

    std::cout << "=== RX check start === "
              << "port=" << port << " baud=" << baud
              << " payload=" << payload << " N=" << N
              << " frame=" << FRAME_LEN << "\n";

    std::vector<uint8_t> rbuf(1 << 16);

    while (total < N) {
        int n = ser.read_some(rbuf.data(), (int)rbuf.size(), 50);
        if (n < 0) break;
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                buf.push_back(rbuf[i]);
                prevRaw.push_back(rbuf[i]);
            }
            while (prevRaw.size() > PREV_RAW_KEEP) prevRaw.pop_front();
            while (buf.size() > MAX_Q) {
                // keep last few bytes to allow magic spanning
                while (buf.size() > 8) buf.pop_front();
            }
        }

        if (!locked) {
            int pos = find_magic(buf);
            if (pos < 0) continue;

            if (pos > 0) {
                sync++;
                if (dump_count < MAX_DUMPS) {
                    size_t left = (pos > (int)AROUND_N) ? (pos - AROUND_N) : 0;
                    size_t right = min(buf.size(), (size_t)pos + 4 + AROUND_N);

                    // extract around-magic from deque
                    std::vector<uint8_t> around;
                    around.reserve(right - left);
                    for (size_t i = left; i < right; i++) around.push_back(buf[i]);

                    std::vector<uint8_t> prev = take_front(prevRaw, prevRaw.size());

                    std::string ts = now_ms_string();
                    auto base = outdir / ("sync_t" + std::to_string(total) + "_pos" + std::to_string(pos) + "_ts" + ts);
                    save_bin(base.string() + ".bin", around);
                    save_bin(base.string() + "_prevraw.bin", prev);
                    dump_count += 2;
                }
            }

            pop_front(buf, (size_t)pos);
            locked = true;
        }

        if (buf.size() < FRAME_LEN) continue;

        // take one frame
        std::vector<uint8_t> frame = take_front(buf, FRAME_LEN);
        pop_front(buf, FRAME_LEN);

        if (std::memcmp(frame.data(), MAGIC, 4) != 0) {
            locked = false;
            continue;
        }

        uint32_t seq = le_u32(frame.data() + 4);
        uint32_t recv_crc = le_u32(frame.data() + 8 + payload);
        uint32_t calc_crc = crc32(frame.data(), 8 + payload); // MAGIC+SEQ+PAYLOAD

        if (seq != expected_seq) {
            seq_ng++;
            if (dump_count < MAX_DUMPS) {
                std::vector<uint8_t> prev = take_front(prevRaw, prevRaw.size());
                std::string ts = now_ms_string();
                auto base = outdir / ("seqng_t" + std::to_string(total) +
                                      "_exp" + std::to_string(expected_seq) +
                                      "_got" + std::to_string(seq) +
                                      "_ts" + ts);
                save_bin(base.string() + ".bin", frame);
                save_bin(base.string() + "_prevraw.bin", prev);
                dump_count += 2;
            }
            expected_seq = seq + 1; // follow to continue
        } else {
            expected_seq++;
        }

        if (calc_crc == recv_crc) {
            ok++;
        } else {
            crc_ng++;
            if (dump_count < MAX_DUMPS) {
                std::vector<uint8_t> prev = take_front(prevRaw, prevRaw.size());
                std::string ts = now_ms_string();
                auto base = outdir / ("crcng_t" + std::to_string(total) +
                                      "_seq" + std::to_string(seq) +
                                      "_ts" + ts);
                save_bin(base.string() + ".bin", frame);
                save_bin(base.string() + "_prevraw.bin", prev);
                dump_count += 2;
            }
            locked = false; // re-lock on next magic
        }

        total++;

        auto now = std::chrono::steady_clock::now();
        if (now >= nextPrint) {
            std::cout << "T:" << total
                      << " OK:" << ok
                      << " CRC_NG:" << crc_ng
                      << " SEQ_NG:" << seq_ng
                      << " SYNC:" << sync
                      << " expected_seq:" << expected_seq
                      << " dumps:" << dump_count
                      << "\n";
            nextPrint = now + std::chrono::seconds(printIntervalSec);
        }
    }

    std::cout << "=== DONE === "
              << "T:" << total
              << " OK:" << ok
              << " CRC_NG:" << crc_ng
              << " SEQ_NG:" << seq_ng
              << " SYNC:" << sync
              << " expected_seq:" << expected_seq
              << " dumps:" << dump_count
              << "\n";

    return 0;
}
