#include "uart.hpp"
#include "tool/log/log.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/time.h>
#include <unistd.h>

static const char* const TAG = "Uart";

namespace
{

    /* CRC8 查表 (0x07) */
    const uint8_t CRC8_TBL[256] = {
        0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A,
        0x2D, 0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53,
        0x5A, 0x5D, 0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4,
        0xC3, 0xCA, 0xCD, 0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1,
        0xB4, 0xB3, 0xBA, 0xBD, 0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1,
        0xF6, 0xE3, 0xE4, 0xED, 0xEA, 0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88,
        0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A, 0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F,
        0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A, 0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
        0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A, 0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B,
        0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4, 0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2,
        0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4, 0x69, 0x6E, 0x67, 0x60, 0x75,
        0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44, 0x19, 0x1E, 0x17, 0x10,
        0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34, 0x4E, 0x49, 0x40,
        0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63, 0x3E, 0x39,
        0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13, 0xAE,
        0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
        0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4,
        0xF3,
    };

    /* CRC16 查表 (MODBUS) */
    const uint16_t CRC16_TBL[256] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780,
        0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1,
        0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 0xD801,
        0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40,
        0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680,
        0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0,
        0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501,
        0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1, 0xE981,
        0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1,
        0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200,
        0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141,
        0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480,
        0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0,
        0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01,
        0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381,
        0x7340, 0xB101, 0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0,
        0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01,
        0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40,
        0x9901, 0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81,
        0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 0x4400, 0x84C1,
        0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 0x4100,
        0x81C1, 0x8081, 0x4040,
    };

} // namespace

namespace app::common::uart
{

            /*----------------------------------------------------------------------------
             * COBS编解码
             *----------------------------------------------------------------------------*/

            size_t cobs_encode(const uint8_t* src, size_t len, uint8_t* dst)
            {
                if (!src || !dst || len == 0)
                    return 0;

                const uint8_t* end  = src + len;
                uint8_t*       out  = dst;
                uint8_t*       code = dst++;
                uint8_t        c    = 1;

                while (src < end)
                {
                    if (*src == 0)
                    {
                        *code = c;
                        code  = dst++;
                        c     = 1;
                        src++;
                    }
                    else
                    {
                        *dst++ = *src++;
                        if (++c == 0xFF)
                        {
                            *code = c;
                            code  = dst++;
                            c     = 1;
                        }
                    }
                }
                *code  = c;
                *dst++ = 0x00;
                return (size_t)(dst - out);
            }

            size_t cobs_decode(const uint8_t* src, size_t len, uint8_t* dst, size_t max)
            {
                if (!src || !dst || len == 0 || max == 0)
                    return 0;

                const uint8_t* end = src + len;
                uint8_t*       out = dst;
                uint8_t*       lim = dst + max;

                while (src < end)
                {
                    uint8_t c = *src++;
                    if (c == 0)
                        return 0;
                    for (uint8_t i = 1; i < c; i++)
                    {
                        if (src >= end || dst >= lim)
                            return 0;
                        *dst++ = *src++;
                    }
                    if (c < 0xFF && src < end)
                    {
                        if (dst >= lim)
                            return 0;
                        *dst++ = 0;
                    }
                }
                if (dst > out && *(dst - 1) == 0)
                    dst--;
                return (size_t)(dst - out);
            }

            /*----------------------------------------------------------------------------
             * CRC
             *----------------------------------------------------------------------------*/

            uint8_t UartDrv::calc_crc8(const uint8_t* data, size_t len)
            {
                uint8_t crc = 0;
                for (size_t i = 0; i < len; i++)
                    crc = CRC8_TBL[crc ^ data[i]];
                return crc;
            }

            uint16_t UartDrv::calc_crc16(const uint8_t* data, size_t len)
            {
                uint16_t crc = 0xFFFF;
                for (size_t i = 0; i < len; i++)
                    crc = (crc >> 8) ^ CRC16_TBL[(crc ^ data[i]) & 0xFF];
                return crc;
            }

            /*----------------------------------------------------------------------------
             * 单例
             *----------------------------------------------------------------------------*/

            UartDrv& UartDrv::inst()
            {
                static UartDrv s;
                return s;
            }

            UartDrv::UartDrv()
            {
                frm_buf_.resize(MAX_FRAME_SIZE);
                cobs_buf_.resize(COBS_MAX_LEN);
                dec_buf_.resize(MAX_FRAME_SIZE);
                poll_buf_.resize(4096);
            }

            UartDrv::~UartDrv()
            {
                deinit();
            }

            /*----------------------------------------------------------------------------
             * 初始化
             *----------------------------------------------------------------------------*/

            bool UartDrv::init(const UartCfg& cfg)
            {
                if (init_)
                {
                    LOG_WARN(TAG, "重复初始化");
                    deinit();
                }

                cfg_ = cfg;

                if (!init_uart())
                {
                    cleanup();
                    return false;
                }
                if (!init_poll())
                {
                    cleanup();
                    return false;
                }
                if (!init_buf())
                {
                    cleanup();
                    return false;
                }

                init_ = true;
                LOG_INFO(TAG, "初始化完成 %s %d", cfg_.dev.c_str(), cfg_.baud);
                return true;
            }

            void UartDrv::deinit()
            {
                stop();
                cleanup();
                init_ = false;
            }

            bool UartDrv::init_uart()
            {
                fd_uart_ = open(cfg_.dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
                if (fd_uart_ < 0)
                {
                    LOG_ERROR(TAG, "打开失败 %s: %s", cfg_.dev.c_str(), strerror(errno));
                    return false;
                }
                if (!cfg_uart(fd_uart_))
                {
                    LOG_ERROR(TAG, "配置失败");
                    return false;
                }
                LOG_INFO(TAG, "打开成功 fd=%d", fd_uart_);
                return true;
            }

            bool UartDrv::cfg_uart(int fd)
            {
                struct termios tty = {};
                if (tcgetattr(fd, &tty) != 0)
                    return false;

                speed_t sp = baud_to_speed(cfg_.baud);
                cfsetispeed(&tty, sp);
                cfsetospeed(&tty, sp);

                tty.c_cflag &= ~CSIZE;
                switch (cfg_.bits)
                {
                case 5:
                    tty.c_cflag |= CS5;
                    break;
                case 6:
                    tty.c_cflag |= CS6;
                    break;
                case 7:
                    tty.c_cflag |= CS7;
                    break;
                default:
                    tty.c_cflag |= CS8;
                    break;
                }

                if (cfg_.stop == 2)
                    tty.c_cflag |= CSTOPB;
                else
                    tty.c_cflag &= ~CSTOPB;

                tty.c_cflag &= ~PARENB;
                if (cfg_.parity == 'O' || cfg_.parity == 'o')
                    tty.c_cflag |= PARENB | PARODD;
                else if (cfg_.parity == 'E' || cfg_.parity == 'e')
                    tty.c_cflag |= PARENB;

                tty.c_cflag |= CLOCAL | CREAD;
                tty.c_cflag &= ~CRTSCTS;

                tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR |
                                 IGNCR | ICRNL);
                tty.c_oflag &= ~OPOST;
                tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

                tty.c_cc[VMIN]  = 0;
                tty.c_cc[VTIME] = 0;

                if (tcsetattr(fd, TCSANOW, &tty) != 0)
                    return false;
                tcflush(fd, TCIOFLUSH);
                return true;
            }

            speed_t UartDrv::baud_to_speed(int baud)
            {
                switch (baud)
                {
                case 9600:
                    return B9600;
                case 19200:
                    return B19200;
                case 38400:
                    return B38400;
                case 57600:
                    return B57600;
                case 115200:
                    return B115200;
                case 230400:
                    return B230400;
                case 460800:
                    return B460800;
                case 500000:
                    return B500000;
                case 921600:
                    return B921600;
                case 1000000:
                    return B1000000;
                case 1500000:
                    return B1500000;
                case 2000000:
                    return B2000000;
                case 3000000:
                    return B3000000;
                default:
                    return B115200;
                }
            }

            bool UartDrv::init_poll()
            {
                fd_poll_ = epoll_create1(0);
                if (fd_poll_ < 0)
                {
                    LOG_ERROR(TAG, "epoll创建失败");
                    return false;
                }
                struct epoll_event ev = {};
                ev.events             = EPOLLIN | EPOLLET;
                ev.data.fd            = fd_uart_;
                if (epoll_ctl(fd_poll_, EPOLL_CTL_ADD, fd_uart_, &ev) < 0)
                {
                    LOG_ERROR(TAG, "epoll添加失败");
                    return false;
                }
                return true;
            }

            bool UartDrv::init_buf()
            {
                rx_ring_ =
                    std::make_unique<tool::memory::ring_buffer::RingBufferWithStats>(cfg_.rx_size);
                if (!rx_ring_ || !rx_ring_->valid())
                {
                    LOG_ERROR(TAG, "缓冲区创建失败");
                    return false;
                }
                LOG_INFO(TAG, "缓冲区就绪 %luKB", (unsigned long)(cfg_.rx_size / 1024));
                return true;
            }

            void UartDrv::cleanup()
            {
                if (fd_poll_ >= 0)
                {
                    close(fd_poll_);
                    fd_poll_ = -1;
                }
                if (fd_uart_ >= 0)
                {
                    close(fd_uart_);
                    fd_uart_ = -1;
                }
                rx_ring_.reset();
                frag_.reset();
                cobs_.reset();
            }

            /*----------------------------------------------------------------------------
             * 启停
             *----------------------------------------------------------------------------*/

            bool UartDrv::start()
            {
                if (!init_)
                {
                    LOG_ERROR(TAG, "未初始化");
                    return false;
                }
                if (running_)
                    return true;
                running_ = true;
                rx_thd_  = std::thread(&UartDrv::rx_loop, this);
                LOG_INFO(TAG, "启动");
                return true;
            }

            void UartDrv::stop()
            {
                if (!running_)
                    return;
                running_ = false;
                if (rx_thd_.joinable())
                    rx_thd_.join();
                LOG_INFO(TAG, "停止");
            }

            /*----------------------------------------------------------------------------
             * 接收线程
             *----------------------------------------------------------------------------*/

            void UartDrv::rx_loop()
            {
                LOG_INFO(TAG, "RX线程启动");
                struct epoll_event evs[1];

                while (running_)
                {
                    int n = epoll_wait(fd_poll_, evs, 1, cfg_.poll_ms);
                    if (n < 0)
                    {
                        if (errno == EINTR)
                            continue;
                        LOG_ERROR(TAG, "epoll错误");
                        break;
                    }
                    if (n > 0 && (evs[0].events & EPOLLIN))
                    {
                        while (true)
                        {
                            ssize_t len = read(fd_uart_, poll_buf_.data(), poll_buf_.size());
                            if (len > 0)
                            {
                                stats_.rx_bytes.fetch_add(len);
                                proc_rx(poll_buf_.data(), (size_t)len);
                            }
                            else
                            {
                                if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
                                    LOG_ERROR(TAG, "读取错误");
                                break;
                            }
                        }
                    }
                    check_flow();
                    check_frag_timeout();
                }
                LOG_INFO(TAG, "RX线程退出");
            }

            /*----------------------------------------------------------------------------
             * 接收处理
             *----------------------------------------------------------------------------*/

            void UartDrv::proc_rx(const uint8_t* data, size_t len)
            {
                for (size_t i = 0; i < len; i++)
                {
                    uint8_t b = data[i];
                    if (b == 0)
                    {
                        if (!cobs_.buf.empty())
                        {
                            handle_frame(cobs_.buf.data(), cobs_.buf.size());
                            cobs_.buf.clear();
                        }
                        cobs_.in_frame = false;
                    }
                    else
                    {
                        if (cobs_.buf.size() < COBS_MAX_LEN)
                        {
                            cobs_.buf.push_back(b);
                            cobs_.in_frame = true;
                        }
                        else
                        {
                            LOG_WARN(TAG, "缓冲溢出");
                            cobs_.buf.clear();
                            cobs_.in_frame = false;
                            stats_.overflow.fetch_add(1);
                        }
                    }
                }
            }

            void UartDrv::handle_frame(const uint8_t* data, size_t len)
            {
                size_t dec_len = cobs_decode(data, len, dec_buf_.data(), dec_buf_.size());
                if (dec_len == 0)
                {
                    stats_.cobs_err.fetch_add(1);
                    return;
                }

                FrameHeader    hdr;
                const uint8_t* payload = nullptr;
                size_t         plen    = 0;

                if (!parse_frame(dec_buf_.data(), dec_len, hdr, payload, plen))
                {
                    stats_.invalid.fetch_add(1);
                    return;
                }

                stats_.rx_frm.fetch_add(1);
                FrameType type = (FrameType)hdr.type;

                switch (type)
                {
                case FrameType::NOP:
                    break;
                case FrameType::XON:
                    on_xon();
                    break;
                case FrameType::XOFF:
                    on_xoff();
                    break;
                case FrameType::DATA:
                    dispatch(payload, plen);
                    break;
                case FrameType::FRAG_FIRST:
                case FrameType::FRAG_MID:
                case FrameType::FRAG_LAST:
                    handle_frag(hdr, payload, plen);
                    break;
                default:
                    break;
                }
            }

            bool UartDrv::parse_frame(const uint8_t* data, size_t len, FrameHeader& hdr,
                                      const uint8_t*& payload, size_t& plen)
            {
                if (len < FRAME_HEADER_SIZE + FRAME_CRC_SIZE)
                    return false;
                memcpy(&hdr, data, sizeof(hdr));
                if (hdr.magic != FRAME_MAGIC)
                    return false;

                if (calc_crc8(data, FRAME_HEADER_SIZE - 1) != hdr.hdr_crc)
                {
                    stats_.crc_err.fetch_add(1);
                    return false;
                }

                size_t exp = FRAME_HEADER_SIZE + hdr.len + FRAME_CRC_SIZE;
                if (len != exp)
                    return false;

                const uint8_t* crc_ptr = data + FRAME_HEADER_SIZE + hdr.len;
                uint16_t       rx_crc  = crc_ptr[0] | (crc_ptr[1] << 8);
                if (calc_crc16(data + FRAME_HEADER_SIZE, hdr.len) != rx_crc)
                {
                    stats_.crc_err.fetch_add(1);
                    return false;
                }

                payload = data + FRAME_HEADER_SIZE;
                plen    = hdr.len;
                return true;
            }

            void UartDrv::handle_frag(const FrameHeader& hdr, const uint8_t* payload, size_t len)
            {
                std::lock_guard<std::mutex> lk(frag_mtx_);
                FrameType                   type = (FrameType)hdr.type;

                if (type == FrameType::FRAG_FIRST)
                {
                    if (len < FRAGMENT_META_SIZE)
                        return;
                    FragMeta m;
                    memcpy(&m, payload, sizeof(m));
                    if (m.total > MAX_DATA_SIZE)
                        return;

                    frag_.reset();
                    frag_.active = true;
                    frag_.mid    = hdr.msg_id;
                    frag_.cnt    = hdr.frag_cnt;
                    frag_.total  = m.total;
                    frag_.ts     = now_ms();
                    frag_.buf.resize(m.total);
                    frag_.map.resize(hdr.frag_cnt, false);

                    size_t dlen = len - FRAGMENT_META_SIZE;
                    if (dlen > 0 && dlen <= frag_.total)
                        memcpy(frag_.buf.data(), payload + FRAGMENT_META_SIZE, dlen);
                    frag_.map[0] = true;
                    frag_.got    = 1;
                }
                else if (frag_.active && hdr.msg_id == frag_.mid)
                {
                    if (hdr.frag_idx >= frag_.cnt)
                        return;
                    if (frag_.map[hdr.frag_idx])
                        return;

                    size_t first_len = MAX_PAYLOAD_SIZE - FRAGMENT_META_SIZE;
                    size_t pos =
                        (hdr.frag_idx == 0) ? 0 : first_len + (hdr.frag_idx - 1) * MAX_PAYLOAD_SIZE;
                    if (pos + len > frag_.total)
                        return;

                    memcpy(frag_.buf.data() + pos, payload, len);
                    frag_.map[hdr.frag_idx] = true;
                    frag_.got++;

                    if (frag_.complete())
                    {
                        LOG_INFO(TAG, "分片完成 %u字节", frag_.total);
                        dispatch(frag_.buf.data(), frag_.total);
                        frag_.reset();
                    }
                }
            }

            void UartDrv::dispatch(const uint8_t* data, size_t len)
            {
                std::lock_guard<std::mutex> lk(cb_mtx_);
                if (rx_cb_)
                    rx_cb_(FrameType::DATA, data, len);
            }

            void UartDrv::check_frag_timeout()
            {
                std::lock_guard<std::mutex> lk(frag_mtx_);
                if (frag_.active && now_ms() - frag_.ts > cfg_.frag_ms)
                {
                    LOG_WARN(TAG, "分片超时 %u/%u", frag_.got, frag_.cnt);
                    stats_.timeout.fetch_add(1);
                    frag_.reset();
                }
            }

            /*----------------------------------------------------------------------------
             * 流控
             *----------------------------------------------------------------------------*/

            void UartDrv::check_flow()
            {
                if (!rx_ring_)
                    return;
                size_t  used = rx_ring_->available_fast();
                size_t  cap  = rx_ring_->capacity();
                uint8_t pct  = (uint8_t)((used * 100) / cap);

                if (pct >= cfg_.xoff_th && !xoff_tx_)
                {
                    send_xoff();
                    xoff_tx_ = true;
                    stats_.xoff_cnt.fetch_add(1);
                }
                else if (pct <= cfg_.xon_th && xoff_tx_)
                {
                    send_xon();
                    xoff_tx_ = false;
                    stats_.xon_cnt.fetch_add(1);
                }
            }

            void UartDrv::on_xon()
            {
                xoff_rx_      = false;
                stats_.paused = false;
            }
            void UartDrv::on_xoff()
            {
                xoff_rx_      = true;
                stats_.paused = true;
            }

            bool UartDrv::send_xon()
            {
                return send_frame(FrameType::XON, nullptr, 0, FLAG_NONE);
            }
            bool UartDrv::send_xoff()
            {
                return send_frame(FrameType::XOFF, nullptr, 0, FLAG_NONE);
            }

            /*----------------------------------------------------------------------------
             * 发送
             *----------------------------------------------------------------------------*/

            bool UartDrv::send(const uint8_t* data, size_t len)
            {
                if (!running_)
                    return false;
                if (!data || len == 0)
                    return true;
                if (len > MAX_DATA_SIZE)
                {
                    LOG_ERROR(TAG, "数据过大");
                    return false;
                }

                for (int i = 0; i < 1000 && xoff_rx_ && running_; i++)
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                if (xoff_rx_)
                {
                    LOG_WARN(TAG, "流控超时");
                    return false;
                }

                if (len <= MAX_PAYLOAD_SIZE)
                    return send_frame(FrameType::DATA, data, len, FLAG_NONE);
                return send_frags(data, len);
            }

            bool UartDrv::send_frags(const uint8_t* data, size_t len)
            {
                size_t first = MAX_PAYLOAD_SIZE - FRAGMENT_META_SIZE;
                size_t rem   = len > first ? len - first : 0;
                size_t cnt   = 1 + (rem + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;
                if (cnt > 255)
                    return false;

                uint8_t mid = next_mid();
                size_t  off = 0;

                for (size_t i = 0; i < cnt; i++)
                {
                    FrameType            type;
                    uint8_t              flags = FLAG_FRAG;
                    std::vector<uint8_t> pl;

                    if (i == 0)
                    {
                        type          = FrameType::FRAG_FIRST;
                        FragMeta m    = {(uint8_t)cnt, 0, (uint32_t)len};
                        size_t   dlen = std::min(first, len);
                        pl.resize(FRAGMENT_META_SIZE + dlen);
                        memcpy(pl.data(), &m, FRAGMENT_META_SIZE);
                        memcpy(pl.data() + FRAGMENT_META_SIZE, data, dlen);
                        off = dlen;
                    }
                    else if (i == cnt - 1)
                    {
                        type = FrameType::FRAG_LAST;
                        flags |= FLAG_LAST;
                        size_t dlen = len - off;
                        pl.resize(dlen);
                        memcpy(pl.data(), data + off, dlen);
                    }
                    else
                    {
                        type        = FrameType::FRAG_MID;
                        size_t dlen = std::min(MAX_PAYLOAD_SIZE, len - off);
                        pl.resize(dlen);
                        memcpy(pl.data(), data + off, dlen);
                        off += dlen;
                    }

                    std::lock_guard<std::mutex> lk(tx_mtx_);
                    size_t flen = build_frame(frm_buf_.data(), type, pl.data(), pl.size(), flags,
                                              mid, (uint8_t)i, (uint8_t)cnt);
                    if (flen == 0)
                        return false;

                    size_t clen = cobs_encode(frm_buf_.data(), flen, cobs_buf_.data());
                    if (clen == 0)
                        return false;

                    if (!uart_write(cobs_buf_.data(), clen))
                        return false;

                    stats_.tx_frm.fetch_add(1);
                    stats_.tx_bytes.fetch_add(clen);

                    if (i < cnt - 1)
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                return true;
            }

            bool UartDrv::send_frame(FrameType type, const uint8_t* data, size_t len, uint8_t flags)
            {
                std::lock_guard<std::mutex> lk(tx_mtx_);

                size_t flen = build_frame(frm_buf_.data(), type, data, len, flags, 0, 0, 0);
                if (flen == 0)
                    return false;

                size_t clen = cobs_encode(frm_buf_.data(), flen, cobs_buf_.data());
                if (clen == 0)
                    return false;

                if (!uart_write(cobs_buf_.data(), clen))
                    return false;

                stats_.tx_frm.fetch_add(1);
                stats_.tx_bytes.fetch_add(clen);
                return true;
            }

            size_t UartDrv::build_frame(uint8_t* buf, FrameType type, const uint8_t* data,
                                        size_t len, uint8_t flags, uint8_t mid, uint8_t idx,
                                        uint8_t cnt)
            {
                if (len > MAX_PAYLOAD_SIZE)
                    return 0;

                FrameHeader* h = reinterpret_cast<FrameHeader*>(buf);
                h->magic       = FRAME_MAGIC;
                h->type        = (uint8_t)type;
                h->flags       = flags;
                h->len         = (uint16_t)len;
                h->seq         = next_seq();
                h->msg_id      = mid;
                h->frag_idx    = idx;
                h->frag_cnt    = cnt;
                h->hdr_crc     = calc_crc8(buf, FRAME_HEADER_SIZE - 1);

                if (data && len > 0)
                    memcpy(buf + FRAME_HEADER_SIZE, data, len);

                uint16_t crc                     = calc_crc16(buf + FRAME_HEADER_SIZE, len);
                buf[FRAME_HEADER_SIZE + len]     = crc & 0xFF;
                buf[FRAME_HEADER_SIZE + len + 1] = crc >> 8;

                return FRAME_HEADER_SIZE + len + FRAME_CRC_SIZE;
            }

            bool UartDrv::uart_write(const uint8_t* data, size_t len)
            {
                size_t sent = 0;
                while (sent < len)
                {
                    ssize_t n = write(fd_uart_, data + sent, len - sent);
                    if (n > 0)
                    {
                        sent += n;
                    }
                    else if (n == 0)
                    {
                        break;
                    }
                    else
                    {
                        if (errno == EAGAIN || errno == EWOULDBLOCK)
                        {
                            std::this_thread::sleep_for(std::chrono::microseconds(100));
                            continue;
                        }
                        LOG_ERROR(TAG, "写入失败");
                        return false;
                    }
                }
                return sent == len;
            }

            /*----------------------------------------------------------------------------
             * 工具
             *----------------------------------------------------------------------------*/

            uint8_t UartDrv::next_mid()
            {
                uint8_t id = mid_.fetch_add(1);
                if (id == 0)
                    id = mid_.fetch_add(1);
                return id;
            }

            uint32_t UartDrv::now_ms()
            {
                struct timeval tv;
                gettimeofday(&tv, nullptr);
                return (uint32_t)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
            }

            void UartDrv::set_cb(RxCb cb)
            {
                std::lock_guard<std::mutex> lk(cb_mtx_);
                rx_cb_ = std::move(cb);
    }

} // namespace app::common::uart
