#include "spi_master.hpp"

#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <poll.h>
#include <errno.h>
#include <chrono>
#include <fstream>
#include <sstream>

#include "tool/log/log.hpp"

#define TAG "Spi"

namespace
{

    const char* GPIO_EXPORT = "/sys/class/gpio/export";
    const char* GPIO_BASE   = "/sys/class/gpio/gpio";

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

namespace app::common::spi
{

            /*----------------------------------------------------------------------------
             * CRC
             *----------------------------------------------------------------------------*/

            uint8_t SpiDrv::calc_crc8(const uint8_t* data, size_t len)
            {
                uint8_t crc = 0;
                for (size_t i = 0; i < len; i++)
                    crc = CRC8_TBL[crc ^ data[i]];
                return crc;
            }

            uint16_t SpiDrv::calc_crc16(const uint8_t* data, size_t len)
            {
                uint16_t crc = 0xFFFF;
                for (size_t i = 0; i < len; i++)
                    crc = (crc >> 8) ^ CRC16_TBL[(crc ^ data[i]) & 0xFF];
                return crc;
            }

            /*----------------------------------------------------------------------------
             * 单例
             *----------------------------------------------------------------------------*/

            SpiDrv& SpiDrv::inst()
            {
                static SpiDrv s;
                return s;
            }

            SpiDrv::SpiDrv()
            {
                memset(nop_tmpl_, 0, sizeof(nop_tmpl_));
            }

            SpiDrv::~SpiDrv()
            {
                deinit();
            }

            /*----------------------------------------------------------------------------
             * 初始化
             *----------------------------------------------------------------------------*/

            bool SpiDrv::init(const SpiCfg& cfg)
            {
                if (init_)
                {
                    LOG_WARN(TAG, "重复初始化");
                    return true;
                }
                cfg_ = cfg;

                if (!init_buf() || !init_gpio() || !init_spi())
                {
                    cleanup();
                    LOG_ERROR(TAG, "初始化失败");
                    return false;
                }

                init_ = true;
                LOG_INFO(TAG, "初始化完成");
                return true;
            }

            void SpiDrv::deinit()
            {
                if (!init_)
                    return;
                stop();
                cleanup();
                init_ = false;
                LOG_INFO(TAG, "已关闭");
            }

            bool SpiDrv::start()
            {
                if (!init_ || running_)
                    return false;
                running_  = true;
                comm_thd_ = std::thread(&SpiDrv::comm_loop, this);
                hb_thd_   = std::thread(&SpiDrv::hb_loop, this);
                LOG_INFO(TAG, "启动");
                return true;
            }

            void SpiDrv::stop()
            {
                if (!running_)
                    return;
                running_ = false;
                tx_cv_.notify_all();
                if (comm_thd_.joinable())
                    comm_thd_.join();
                if (hb_thd_.joinable())
                    hb_thd_.join();
                update_conn(false);
                LOG_INFO(TAG, "停止");
            }

            /*----------------------------------------------------------------------------
             * 缓冲区初始化
             *----------------------------------------------------------------------------*/

            bool SpiDrv::init_buf()
            {
                void* ptr;
                if (posix_memalign(&ptr, DMA_ALIGN, MAX_FRAME_SIZE) != 0)
                    return false;
                dma_tx_ = (uint8_t*)ptr;

                if (posix_memalign(&ptr, DMA_ALIGN, MAX_FRAME_SIZE) != 0)
                    return false;
                dma_rx_ = (uint8_t*)ptr;

                if (posix_memalign(&ptr, DMA_ALIGN, MAX_FRAME_SIZE) != 0)
                    return false;
                frm_buf_ = (uint8_t*)ptr;

                memset(dma_tx_, 0, MAX_FRAME_SIZE);
                memset(dma_rx_, 0, MAX_FRAME_SIZE);
                memset(frm_buf_, 0, MAX_FRAME_SIZE);

                tx_ring_ = std::make_unique<tool::memory::ring_buffer::RingBufferWithStats>(
                    cfg_.tx_size, DMA_ALIGN);
                rx_ring_ = std::make_unique<tool::memory::ring_buffer::RingBufferWithStats>(
                    cfg_.rx_size, DMA_ALIGN);

                if (!tx_ring_ || !tx_ring_->valid() || !rx_ring_ || !rx_ring_->valid())
                    return false;

                FrameHeader* h                   = (FrameHeader*)nop_tmpl_;
                h->magic                         = FRAME_MAGIC;
                h->version                       = PROTOCOL_VER;
                h->type                          = (uint8_t)FrameType::NOP;
                h->flags                         = 0;
                h->seq                           = 0;
                h->len                           = 0;
                h->ts                            = 0;
                h->msg_id                        = 0;
                h->frag_idx                      = 0;
                h->hdr_crc                       = calc_crc8(nop_tmpl_, FRAME_HEADER_SIZE - 1);
                uint16_t crc                     = calc_crc16(nop_tmpl_ + FRAME_HEADER_SIZE, 0);
                nop_tmpl_[FRAME_HEADER_SIZE]     = crc & 0xFF;
                nop_tmpl_[FRAME_HEADER_SIZE + 1] = crc >> 8;

                LOG_INFO(TAG, "缓冲 TX=%luKB RX=%luKB", (unsigned long)(cfg_.tx_size / 1024),
                         (unsigned long)(cfg_.rx_size / 1024));
                return true;
            }

            /*----------------------------------------------------------------------------
             * GPIO初始化
             *----------------------------------------------------------------------------*/

            bool SpiDrv::init_gpio()
            {
                if (cfg_.gpio_req >= 0)
                {
                    gpio_export(cfg_.gpio_req);
                    if (!gpio_set_dir(cfg_.gpio_req, true))
                        return false;
                    gpio_write(cfg_.gpio_req, 0);

                    std::stringstream path;
                    path << GPIO_BASE << cfg_.gpio_req << "/value";
                    gpio_req_fd_ = open(path.str().c_str(), O_WRONLY);
                }

                if (cfg_.gpio_rdy >= 0)
                {
                    gpio_export(cfg_.gpio_rdy);
                    if (!gpio_set_dir(cfg_.gpio_rdy, false))
                    {
                        LOG_ERROR(TAG, "GPIO%d方向失败", cfg_.gpio_rdy);
                        return false;
                    }
                    if (!gpio_set_edge(cfg_.gpio_rdy, "rising"))
                    {
                        LOG_ERROR(TAG, "GPIO%d中断失败", cfg_.gpio_rdy);
                        return false;
                    }

                    std::stringstream path;
                    path << GPIO_BASE << cfg_.gpio_rdy << "/value";
                    gpio_rdy_fd_ = open(path.str().c_str(), O_RDWR);
                    if (gpio_rdy_fd_ < 0)
                    {
                        LOG_ERROR(TAG, "GPIO%d打开失败", cfg_.gpio_rdy);
                        return false;
                    }
                    LOG_INFO(TAG, "GPIO中断 RDY=%d REQ=%d", cfg_.gpio_rdy, cfg_.gpio_req);
                }
                return true;
            }

            /*----------------------------------------------------------------------------
             * SPI初始化
             *----------------------------------------------------------------------------*/

            bool SpiDrv::init_spi()
            {
                spi_fd_ = open(cfg_.dev.c_str(), O_RDWR);
                if (spi_fd_ < 0)
                {
                    LOG_ERROR(TAG, "SPI打开失败 %s", cfg_.dev.c_str());
                    return false;
                }

                uint8_t  mode  = cfg_.mode;
                uint8_t  bits  = cfg_.bits;
                uint32_t speed = cfg_.speed;

                if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
                    ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
                    ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
                {
                    LOG_ERROR(TAG, "SPI配置失败 mode=%d bits=%d speed=%u", mode, bits, speed);
                    close(spi_fd_);
                    spi_fd_ = -1;
                    return false;
                }

                LOG_INFO(TAG, "SPI %s %uHz", cfg_.dev.c_str(), speed);
                return true;
            }

            void SpiDrv::cleanup()
            {
                if (gpio_req_fd_ >= 0)
                {
                    close(gpio_req_fd_);
                    gpio_req_fd_ = -1;
                }
                if (gpio_rdy_fd_ >= 0)
                {
                    close(gpio_rdy_fd_);
                    gpio_rdy_fd_ = -1;
                }
                if (spi_fd_ >= 0)
                {
                    close(spi_fd_);
                    spi_fd_ = -1;
                }

                if (dma_tx_)
                {
                    free(dma_tx_);
                    dma_tx_ = nullptr;
                }
                if (dma_rx_)
                {
                    free(dma_rx_);
                    dma_rx_ = nullptr;
                }
                if (frm_buf_)
                {
                    free(frm_buf_);
                    frm_buf_ = nullptr;
                }

                tx_ring_.reset();
                rx_ring_.reset();

                std::lock_guard<std::mutex> lk(cb_mtx_);
                rx_cb_ = nullptr;
            }

            /*----------------------------------------------------------------------------
             * GPIO操作
             *----------------------------------------------------------------------------*/

            bool SpiDrv::gpio_export(int pin)
            {
                if (pin < 0)
                    return true;
                std::ofstream fs(GPIO_EXPORT);
                if (!fs)
                    return false;
                fs << pin;
                fs.close();
                usleep(100000);
                return true;
            }

            bool SpiDrv::gpio_set_dir(int pin, bool out)
            {
                if (pin < 0)
                    return true;
                std::stringstream path;
                path << GPIO_BASE << pin << "/direction";
                std::ofstream fs(path.str());
                if (!fs)
                    return false;
                fs << (out ? "out" : "in");
                return true;
            }

            bool SpiDrv::gpio_set_edge(int pin, const char* edge)
            {
                if (pin < 0)
                    return true;
                std::stringstream path;
                path << GPIO_BASE << pin << "/edge";
                std::ofstream fs(path.str());
                if (!fs)
                {
                    LOG_ERROR(TAG, "无法打开edge文件");
                    return false;
                }
                fs << edge;
                fs.close();
                usleep(10000);
                return true;
            }

            bool SpiDrv::gpio_write(int pin, int val)
            {
                if (pin < 0)
                    return true;
                if (pin == cfg_.gpio_req && gpio_req_fd_ >= 0)
                {
                    char buf[2] = {(char)('0' + val), '\0'};
                    lseek(gpio_req_fd_, 0, SEEK_SET);
                    return write(gpio_req_fd_, buf, 1) == 1;
                }
                return false;
            }

            /*----------------------------------------------------------------------------
             * SPI传输
             *----------------------------------------------------------------------------*/

            bool SpiDrv::spi_xfer(const uint8_t* tx, uint8_t* rx, size_t len)
            {
                if (spi_fd_ < 0)
                    return false;
                struct spi_ioc_transfer xfer = {};
                xfer.tx_buf                  = (unsigned long)tx;
                xfer.rx_buf                  = (unsigned long)rx;
                xfer.len                     = len;
                xfer.speed_hz                = cfg_.speed;
                xfer.bits_per_word           = cfg_.bits;
                return ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &xfer) >= 0;
            }

            void SpiDrv::set_req(bool active)
            {
                gpio_write(cfg_.gpio_req, active ? 1 : 0);
            }

            bool SpiDrv::wait_rdy(uint32_t timeout_ms)
            {
                if (gpio_rdy_fd_ < 0)
                    return false;

                char buf[8];
                lseek(gpio_rdy_fd_, 0, SEEK_SET);
                if (read(gpio_rdy_fd_, buf, sizeof(buf)) > 0 && buf[0] == '1')
                    return true;

                struct pollfd pfd = {};
                pfd.fd            = gpio_rdy_fd_;
                pfd.events        = POLLPRI | POLLERR;

                int ret = poll(&pfd, 1, (int)timeout_ms);
                if (ret < 0)
                {
                    LOG_ERROR(TAG, "poll错误");
                    stats_.timeout.fetch_add(1);
                    return false;
                }
                if (ret == 0)
                {
                    stats_.timeout.fetch_add(1);
                    return false;
                }

                lseek(gpio_rdy_fd_, 0, SEEK_SET);
                if (read(gpio_rdy_fd_, buf, sizeof(buf)) > 0)
                    return (buf[0] == '1');
                return false;
            }

            /*----------------------------------------------------------------------------
             * 帧构建与解析
             *----------------------------------------------------------------------------*/

            size_t SpiDrv::build_frame(uint8_t* buf, FrameType type, const uint8_t* data,
                                       size_t len, uint8_t flags, uint8_t mid, uint8_t idx)
            {
                if (!buf || len > MAX_PAYLOAD_SIZE)
                    return 0;

                FrameHeader* h = (FrameHeader*)buf;
                h->magic       = FRAME_MAGIC;
                h->version     = PROTOCOL_VER;
                h->type        = (uint8_t)type;
                h->flags       = flags;
                h->seq         = next_seq();
                h->len         = (uint16_t)len;
                h->ts          = now_ms();
                h->msg_id      = mid;
                h->frag_idx    = idx;
                h->hdr_crc     = calc_crc8(buf, FRAME_HEADER_SIZE - 1);

                if (data && len > 0)
                    memcpy(buf + FRAME_HEADER_SIZE, data, len);

                uint16_t crc                     = calc_crc16(buf + FRAME_HEADER_SIZE, len);
                buf[FRAME_HEADER_SIZE + len]     = crc & 0xFF;
                buf[FRAME_HEADER_SIZE + len + 1] = crc >> 8;

                return FRAME_HEADER_SIZE + len + FRAME_CRC_SIZE;
            }

            bool SpiDrv::parse_frame(const uint8_t* data, size_t len, FrameHeader& hdr,
                                     const uint8_t*& payload, size_t& plen)
            {
                if (!data || len < FRAME_HEADER_SIZE)
                    return false;
                memcpy(&hdr, data, FRAME_HEADER_SIZE);

                if (hdr.magic != FRAME_MAGIC || hdr.version != PROTOCOL_VER)
                    return false;
                if (hdr.hdr_crc != calc_crc8(data, FRAME_HEADER_SIZE - 1))
                {
                    stats_.crc_err.fetch_add(1);
                    return false;
                }
                if (hdr.len > MAX_PAYLOAD_SIZE)
                {
                    stats_.invalid.fetch_add(1);
                    return false;
                }

                size_t total = FRAME_HEADER_SIZE + hdr.len + FRAME_CRC_SIZE;
                if (len < total)
                    return false;

                const uint8_t* p        = data + FRAME_HEADER_SIZE;
                size_t         crc_off  = FRAME_HEADER_SIZE + hdr.len;
                uint16_t       rx_crc   = data[crc_off] | (data[crc_off + 1] << 8);
                uint16_t       calc_crc = calc_crc16(p, hdr.len);

                if (rx_crc != calc_crc)
                {
                    stats_.crc_err.fetch_add(1);
                    return false;
                }

                payload = p;
                plen    = hdr.len;
                return true;
            }

            /*----------------------------------------------------------------------------
             * 发送
             *----------------------------------------------------------------------------*/

            bool SpiDrv::send(const uint8_t* data, size_t len, bool need_ack)
            {
                if (!running_ || !data || len == 0 || len > MAX_DATA_SIZE)
                    return false;
                if (len <= MAX_PAYLOAD_SIZE)
                    return send_frame(FrameType::DATA, data, len, need_ack ? FLAG_ACK : FLAG_NONE);
                return send_frags(data, len, need_ack);
            }

            bool SpiDrv::send_hb()
            {
                return send_frame(FrameType::HEARTBEAT, nullptr, 0, FLAG_NONE);
            }

            bool SpiDrv::send_ack(uint16_t seq)
            {
                uint8_t buf[2] = {(uint8_t)seq, (uint8_t)(seq >> 8)};
                return send_frame(FrameType::ACK, buf, 2, FLAG_NONE);
            }

            bool SpiDrv::send_nack(uint16_t seq, ErrCode err)
            {
                uint8_t buf[3] = {(uint8_t)seq, (uint8_t)(seq >> 8), (uint8_t)err};
                return send_frame(FrameType::NACK, buf, 3, FLAG_NONE);
            }

            bool SpiDrv::send_frame(FrameType type, const uint8_t* data, size_t len, uint8_t flags)
            {
                size_t flen = build_frame(frm_buf_, type, data, len, flags);
                if (flen == 0)
                    return false;

                if (tx_ring_->write(frm_buf_, flen, 100) != flen)
                    return false;

                stats_.tx_frm.fetch_add(1);
                stats_.tx_bytes.fetch_add(flen);
                stats_.tx_pending.fetch_add(1);

                tx_cv_.notify_one();
                return true;
            }

            bool SpiDrv::send_frags(const uint8_t* data, size_t len, bool need_ack)
            {
                size_t first = MAX_PAYLOAD_SIZE - FRAGMENT_META_SIZE;
                size_t cnt =
                    1 + (len > first ? (len - first + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE : 0);

                if (cnt > 255)
                {
                    LOG_ERROR(TAG, "分片超限");
                    return false;
                }

                uint8_t mid   = next_mid();
                uint8_t flags = FLAG_FRAG | (need_ack ? FLAG_ACK : 0);

                LOG_INFO(TAG, "TX分片 msg=%u len=%lu frags=%lu", mid, (unsigned long)len,
                         (unsigned long)cnt);

                size_t off = 0;
                for (uint8_t i = 0; i < cnt; i++)
                {
                    FrameType type;
                    uint8_t   frag_flags = flags;
                    size_t    chunk;
                    size_t    plen = 0;
                    uint8_t   pl_buf[MAX_PAYLOAD_SIZE];

                    if (i == 0)
                    {
                        type       = FrameType::FRAG_FIRST;
                        FragMeta m = {(uint8_t)cnt, 0, (uint32_t)len};
                        memcpy(pl_buf, &m, FRAGMENT_META_SIZE);
                        chunk = (len < first) ? len : first;
                        memcpy(pl_buf + FRAGMENT_META_SIZE, data, chunk);
                        plen = FRAGMENT_META_SIZE + chunk;
                    }
                    else if (i == cnt - 1)
                    {
                        type = FrameType::FRAG_LAST;
                        frag_flags |= FLAG_LAST;
                        chunk = len - off;
                        memcpy(pl_buf, data + off, chunk);
                        plen = chunk;
                    }
                    else
                    {
                        type  = FrameType::FRAG_MID;
                        chunk = (len - off < MAX_PAYLOAD_SIZE) ? (len - off) : MAX_PAYLOAD_SIZE;
                        memcpy(pl_buf, data + off, chunk);
                        plen = chunk;
                    }

                    size_t flen = build_frame(frm_buf_, type, pl_buf, plen, frag_flags, mid, i);
                    if (flen == 0 || tx_ring_->write(frm_buf_, flen, 100) != flen)
                    {
                        LOG_ERROR(TAG, "TX失败 msg=%u frag=%u", mid, i);
                        return false;
                    }

                    stats_.tx_frm.fetch_add(1);
                    stats_.tx_bytes.fetch_add(flen);
                    stats_.tx_pending.fetch_add(1);

                    tx_cv_.notify_one();

                    if (i < cnt - 1)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(FRAG_INTERVAL_MS));
                    }
                    off += chunk;
                }
                return true;
            }

            /*----------------------------------------------------------------------------
             * 接收处理
             *----------------------------------------------------------------------------*/

            void SpiDrv::proc_rx(const uint8_t* data, size_t len)
            {
                FrameHeader    hdr;
                const uint8_t* payload;
                size_t         plen;

                if (!parse_frame(data, len, hdr, payload, plen))
                    return;

                stats_.rx_frm.fetch_add(1);
                stats_.rx_bytes.fetch_add(FRAME_HEADER_SIZE + plen + FRAME_CRC_SIZE);

                FrameType type = (FrameType)hdr.type;
                if (type == FrameType::HEARTBEAT || type == FrameType::ACK)
                {
                    stats_.last_hb_ms.store(now_ms());
                    update_conn(true);
                    if (cfg_.auto_ack && type == FrameType::HEARTBEAT)
                        send_ack(hdr.seq);
                    return;
                }

                if (type == FrameType::NOP)
                    return;

                if (type == FrameType::FRAG_FIRST || type == FrameType::FRAG_MID ||
                    type == FrameType::FRAG_LAST)
                {
                    handle_frag(hdr, payload, plen);
                    return;
                }

                if (type == FrameType::DATA)
                {
                    dispatch(payload, plen);
                    if ((hdr.flags & FLAG_ACK) && cfg_.auto_ack)
                        send_ack(hdr.seq);
                    return;
                }
            }

            void SpiDrv::handle_frag(const FrameHeader& hdr, const uint8_t* payload, size_t len)
            {
                std::lock_guard<std::mutex> lk(frag_mtx_);
                FrameType                   type = (FrameType)hdr.type;

                if (type == FrameType::FRAG_FIRST)
                {
                    if (len < FRAGMENT_META_SIZE)
                    {
                        send_nack(hdr.seq, ErrCode::INVALID);
                        return;
                    }

                    FragMeta m;
                    memcpy(&m, payload, FRAGMENT_META_SIZE);

                    if (m.cnt == 0 || m.total == 0 || m.total > MAX_DATA_SIZE)
                    {
                        send_nack(hdr.seq, ErrCode::INVALID);
                        return;
                    }

                    if (frag_.active && frag_.mid != hdr.msg_id)
                    {
                        LOG_WARN(TAG, "重组中断 %u->%u", frag_.mid, hdr.msg_id);
                        stats_.invalid.fetch_add(1);
                    }

                    frag_.reset();
                    frag_.active = true;
                    frag_.mid    = hdr.msg_id;
                    frag_.cnt    = m.cnt;
                    frag_.total  = m.total;
                    frag_.ts     = now_ms();
                    frag_.buf.resize(m.total, 0);
                    frag_.map.resize(m.cnt, false);

                    size_t dlen = len - FRAGMENT_META_SIZE;
                    if (dlen > 0)
                        memcpy(frag_.buf.data(), payload + FRAGMENT_META_SIZE, dlen);

                    frag_.map[0] = true;
                    frag_.got    = 1;

                    LOG_INFO(TAG, "RX分片 msg=%u %u片/%uB", hdr.msg_id, m.cnt, m.total);

                    if (hdr.flags & FLAG_ACK)
                        send_ack(hdr.seq);
                    return;
                }

                if (!frag_.active || hdr.msg_id != frag_.mid)
                {
                    send_nack(hdr.seq, ErrCode::INVALID);
                    return;
                }

                if (hdr.frag_idx >= frag_.cnt || frag_.map[hdr.frag_idx])
                {
                    if (hdr.flags & FLAG_ACK)
                        send_ack(hdr.seq);
                    return;
                }

                size_t off =
                    (MAX_PAYLOAD_SIZE - FRAGMENT_META_SIZE) + (hdr.frag_idx - 1) * MAX_PAYLOAD_SIZE;
                if (off + len > frag_.total)
                    len = frag_.total - off;

                memcpy(frag_.buf.data() + off, payload, len);
                frag_.map[hdr.frag_idx] = true;
                frag_.got++;

                if (frag_.complete())
                {
                    LOG_INFO(TAG, "重组完成 msg=%u %uB", hdr.msg_id, frag_.total);
                    dispatch(frag_.buf.data(), frag_.total);
                    frag_.reset();
                }

                if (hdr.flags & FLAG_ACK)
                    send_ack(hdr.seq);
            }

            void SpiDrv::dispatch(const uint8_t* data, size_t len)
            {
                std::lock_guard<std::mutex> lk(cb_mtx_);
                if (rx_cb_)
                    rx_cb_(FrameType::DATA, data, len);
            }

            void SpiDrv::set_cb(RxCb cb)
            {
                std::lock_guard<std::mutex> lk(cb_mtx_);
                rx_cb_ = std::move(cb);
            }

            /*----------------------------------------------------------------------------
             * 线程
             *----------------------------------------------------------------------------*/

            void SpiDrv::comm_loop()
            {
                while (running_)
                {
                    {
                        std::unique_lock<std::mutex> lk(tx_mtx_);
                        tx_cv_.wait_for(lk, std::chrono::milliseconds(10),
                                        [this]
                                        { return tx_ring_->available_fast() > 0 || !running_; });
                    }
                    if (!running_)
                        break;
                    do_xfer();
                }
            }

            void SpiDrv::hb_loop()
            {
                while (running_)
                {
                    send_hb();
                    check_timeout();
                    std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.hb_ms));
                }
            }

            bool SpiDrv::do_xfer()
            {
                set_req(true);
                if (!wait_rdy(cfg_.rdy_ms))
                {
                    set_req(false);
                    return false;
                }

                if (tx_ring_->available_fast() >= FRAME_HEADER_SIZE)
                {
                    size_t len = tx_ring_->read(dma_tx_, MAX_FRAME_SIZE, 0);
                    if (len > 0)
                    {
                        stats_.tx_pending.fetch_sub(1);
                        if (len < MAX_FRAME_SIZE)
                            memset(dma_tx_ + len, 0, MAX_FRAME_SIZE - len);
                    }
                }
                else
                {
                    memcpy(dma_tx_, nop_tmpl_, FRAME_HEADER_SIZE + FRAME_CRC_SIZE);
                    FrameHeader* h = (FrameHeader*)dma_tx_;
                    h->ts          = now_ms();
                    h->hdr_crc     = calc_crc8(dma_tx_, FRAME_HEADER_SIZE - 1);
                    memset(dma_tx_ + FRAME_HEADER_SIZE + FRAME_CRC_SIZE, 0,
                           MAX_FRAME_SIZE - FRAME_HEADER_SIZE - FRAME_CRC_SIZE);
                }

                memset(dma_rx_, 0, MAX_FRAME_SIZE);
                bool ok = spi_xfer(dma_tx_, dma_rx_, MAX_FRAME_SIZE);
                set_req(false);

                if (ok)
                    proc_rx(dma_rx_, MAX_FRAME_SIZE);
                return ok;
            }

            /*----------------------------------------------------------------------------
             * 工具
             *----------------------------------------------------------------------------*/

            uint8_t SpiDrv::next_mid()
            {
                uint8_t id = mid_.fetch_add(1);
                if (id == 0)
                    id = mid_.fetch_add(1);
                return id;
            }

            uint32_t SpiDrv::now_ms()
            {
                auto now = std::chrono::steady_clock::now();
                return (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                           now.time_since_epoch())
                    .count();
            }

            void SpiDrv::update_conn(bool connected)
            {
                bool prev = stats_.connected.exchange(connected);
                if (prev != connected)
                    LOG_INFO(TAG, "%s", connected ? "已连接" : "已断开");
            }

            void SpiDrv::check_timeout()
            {
                uint32_t last = stats_.last_hb_ms.load();
                uint32_t now  = now_ms();

                if (last > 0 && (now - last) > cfg_.timeout_ms)
                    update_conn(false);

                std::lock_guard<std::mutex> lk(frag_mtx_);
                if (frag_.active && (now - frag_.ts) > FRAG_TIMEOUT_MS)
                {
                    LOG_WARN(TAG, "分片超时 msg=%u %u/%u", frag_.mid, frag_.got, frag_.cnt);
                    stats_.timeout.fetch_add(1);
                    frag_.reset();
                }
    }

} // namespace app::common::spi
