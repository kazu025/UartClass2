#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <string>
#include <vector>
#include <deque>
#include <chrono>
#include <iostream>
#include <algorithm>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <poll.h>

static constexpr uint8_t MAGIC[4] = {0xA5, 0x5A, 0x5A, 0xA5};

// ---------------- CRC32 (poly 0xEDB88320) ----------------
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

static uint32_t crc32_calc(const uint8_t* data, size_t len) {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        c = crc32_table[(c ^ data[i]) & 0xFFu] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFu;
}

static inline uint32_t le_u32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// ---------------- time / filesystem utils ----------------
static std::string now_ms_string() {
    using namespace std::chrono;
    auto ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    return std::to_string(ms);
}

static void ensure_dir(const char* dir) {
    // mkdir is OK if exists
    mkdir(dir, 0775);
}

static bool save_bin(const std::string& path, const std::vector<uint8_t>& data) {
    int fd = ::open(path.c_str(), O_CREAT | O_TRUNC | O_WRONLY, 0664);
    if (fd < 0) return false;
    const uint8_t* p = data.data();
    size_t left = data.size();
    while (left) {
        ssize_t w = ::write(fd, p, left);
        if (w < 0) {
            if (errno == EINTR) continue;
            ::close(fd);
            return false;
        }
        p += (size_t)w;
        left -= (size_t)w;
    }
    ::close(fd);
    return true;
}

// ---------------- serial (termios) ----------------
static speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 576000: return B576000;
        case 921600: return B921600;
#ifdef B1000000
        case 1000000: return B1000000;
#endif
#ifdef B1500000
        case 1500000: return B1500000;
#endif
#ifdef B2000000
        case 2000000: return B2000000;
#endif
        default:
            return (speed_t)0;
    }
}

static int open_serial(const std::string& dev, int baud) {
    int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::perror("open");
        return -1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::perror("tcgetattr");
        ::close(fd);
        return -1;
    }

    // raw mode
    cfmakeraw(&tty);

    // 8N1
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag |= (CLOCAL | CREAD);

    // no flow control by default
    tty.c_cflag &= ~CRTSCTS;     // disable HW flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // disable SW flow control

    // VMIN/VTIME: since we use poll + nonblock reads, keep them 0
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    speed_t sp = baud_to_speed(baud);
    if (sp == 0) {
        std::cerr << "Unsupported baud: " << baud << "\n";
        ::close(fd);
        return -1;
    }
    cfsetispeed(&tty, sp);
    cfsetospeed(&tty, sp);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::perror("tcsetattr");
        ::close(fd);
        return -1;
    }

    // flush
    tcflush(fd, TCIOFLUSH);

    return fd;
}

// ---------------- buffer helpers ----------------
static int find_magic(const std::deque<uint8_t>& q) {
    if (q.size() < 4) return -1;
    for (size_t i = 0; i + 4 <= q.size(); i++) {
        if (q[i] == MAGIC[0] && q[i+1] == MAGIC[1] && q[i+2] == MAGIC[2] && q[i+3] == MAGIC[3])
            return (int)i;
    }
    return -1;
}

static void pop_front(std::deque<uint8_t>& q, size_t n) {
    while (n-- && !q.empty()) q.pop_front();
}

static std::vector<uint8_t> take_range(const std::deque<uint8_t>& q, size_t left, size_t right) {
    std::vector<uint8_t> out;
    if (left > right || left >= q.size()) return out;
    right = std::min(right, q.size());
    out.reserve(right - left);
    for (size_t i = left; i < right; i++) out.push_back(q[i]);
    return out;
}

static std::vector<uint8_t> take_front(const std::deque<uint8_t>& q, size_t n) {
    return take_range(q, 0, std::min(n, q.size()));
}

int main(int argc, char** argv) {
    init_crc32_table();

    // defaults
    std::string dev = "/dev/ttyUSB0";
    int baud = 460800;
    uint32_t payload = 4096;
    uint32_t N = 240000;

    // args: ./rx /dev/ttyUSB0 460800 4096 240000
    if (argc >= 2) dev = argv[1];
    if (argc >= 3) baud = std::atoi(argv[2]);
    if (argc >= 4) payload = (uint32_t)std::strtoul(argv[3], nullptr, 10);
    if (argc >= 5) N = (uint32_t)std::strtoul(argv[4], nullptr, 10);

    const uint32_t FRAME_LEN = 4 + 4 + payload + 4;
    const char* OUTDIR = "rx_dumps";
    const size_t AROUND_N = 512;
    const size_t PREV_RAW_KEEP = 4096;
    const size_t MAX_Q = 2 * 1024 * 1024;
    const size_t MAX_DUMPS = 200;

    ensure_dir(OUTDIR);

    int fd = open_serial(dev, baud);
    if (fd < 0) return 1;

    std::deque<uint8_t> buf;
    std::deque<uint8_t> prevRaw;

    uint32_t total = 0, ok = 0, crc_ng = 0, seq_ng = 0, sync = 0;
    uint32_t expected_seq = 0;
    bool locked = false;
    size_t dump_count = 0;

    auto t0 = std::chrono::steady_clock::now();
    auto nextPrint = t0 + std::chrono::seconds(30);

    std::cout << "=== RX check start === dev=" << dev
              << " baud=" << baud
              << " payload=" << payload
              << " N=" << N
              << " frame=" << FRAME_LEN
              << "\n";

    std::vector<uint8_t> rbuf(1 << 16);

    while (total < N) {
        // wait for readable
        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        int pr = ::poll(&pfd, 1, 50); // 50ms
        if (pr < 0) {
            if (errno == EINTR) continue;
            std::perror("poll");
            break;
        }

        if (pr > 0 && (pfd.revents & POLLIN)) {
            for (;;) {
                ssize_t n = ::read(fd, rbuf.data(), rbuf.size());
                if (n < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                    if (errno == EINTR) continue;
                    std::perror("read");
                    goto done;
                }
                if (n == 0) break;

                for (ssize_t i = 0; i < n; i++) {
                    uint8_t b = rbuf[(size_t)i];
                    buf.push_back(b);
                    prevRaw.push_back(b);
                    if (prevRaw.size() > PREV_RAW_KEEP) prevRaw.pop_front();
                }
                while (buf.size() > MAX_Q) {
                    // keep last few bytes
                    while (buf.size() > 8) buf.pop_front();
                }
            }
        }

        if (!locked) {
            int pos = find_magic(buf);
            if (pos < 0) continue;

            if (pos > 0) {
                sync++;
                if (dump_count + 2 <= MAX_DUMPS) {
                    size_t left = (pos > (int)AROUND_N) ? (pos - AROUND_N) : 0;
                    size_t right = std::min(buf.size(), (size_t)pos + 4 + AROUND_N);

                    std::vector<uint8_t> around = take_range(buf, left, right);
                    std::vector<uint8_t> prev = take_front(prevRaw, prevRaw.size());

                    std::string ts = now_ms_string();
                    std::string base = std::string(OUTDIR) + "/sync_t" + std::to_string(total)
                                     + "_pos" + std::to_string(pos) + "_ts" + ts;
                    save_bin(base + ".bin", around);
                    save_bin(base + "_prevraw.bin", prev);
                    dump_count += 2;
                }
            }

            pop_front(buf, (size_t)pos);
            locked = true;
        }

        if (buf.size() < FRAME_LEN) continue;

        std::vector<uint8_t> frame = take_front(buf, FRAME_LEN);
        pop_front(buf, FRAME_LEN);

        if (std::memcmp(frame.data(), MAGIC, 4) != 0) {
            locked = false;
            continue;
        }

        uint32_t seq = le_u32(frame.data() + 4);
        uint32_t recv_crc = le_u32(frame.data() + 8 + payload);
        uint32_t calc_crc = crc32_calc(frame.data(), 8 + payload); // MAGIC+SEQ+PAYLOAD

        if (seq != expected_seq) {
            seq_ng++;
            if (dump_count + 2 <= MAX_DUMPS) {
                std::vector<uint8_t> prev = take_front(prevRaw, prevRaw.size());
                std::string ts = now_ms_string();
                std::string base = std::string(OUTDIR) + "/seqng_t" + std::to_string(total)
                                 + "_exp" + std::to_string(expected_seq)
                                 + "_got" + std::to_string(seq)
                                 + "_ts" + ts;
                save_bin(base + ".bin", frame);
                save_bin(base + "_prevraw.bin", prev);
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
            if (dump_count + 2 <= MAX_DUMPS) {
                std::vector<uint8_t> prev = take_front(prevRaw, prevRaw.size());
                std::string ts = now_ms_string();
                std::string base = std::string(OUTDIR) + "/crcng_t" + std::to_string(total)
                                 + "_seq" + std::to_string(seq)
                                 + "_ts" + ts;
                save_bin(base + ".bin", frame);
                save_bin(base + "_prevraw.bin", prev);
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
            nextPrint = now + std::chrono::seconds(30);
        }
    }

done:
    std::cout << "=== DONE === T:" << total
              << " OK:" << ok
              << " CRC_NG:" << crc_ng
              << " SEQ_NG:" << seq_ng
              << " SYNC:" << sync
              << " expected_seq:" << expected_seq
              << " dumps:" << dump_count
              << "\n";

    ::close(fd);
    return 0;
}
