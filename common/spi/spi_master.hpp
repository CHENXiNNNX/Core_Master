#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <string>
#include <vector>

#include "tool/memory/ring_buffer/ring_buffer.hpp"

namespace app::common::spi
{

            /*============================================================================
             * 协议定义
             *============================================================================*/

            constexpr uint16_t FRAME_MAGIC        = 0xAA55;
            constexpr uint8_t  PROTOCOL_VER       = 0x01;
            constexpr size_t   FRAME_HEADER_SIZE  = 16;
            constexpr size_t   FRAME_CRC_SIZE     = 2;
            constexpr size_t   MAX_PAYLOAD_SIZE   = 4078;
            constexpr size_t   MAX_FRAME_SIZE     = 4096;
            constexpr size_t   DMA_ALIGN          = 4;
            constexpr size_t   FRAGMENT_META_SIZE = 6;

            static_assert(FRAME_HEADER_SIZE + MAX_PAYLOAD_SIZE + FRAME_CRC_SIZE == MAX_FRAME_SIZE,
                          "帧大小不匹配");

            constexpr uint16_t HEARTBEAT_MS     = 1000;
            constexpr uint16_t TIMEOUT_MS       = 3000;
            constexpr size_t   RX_RING_SIZE     = 32 * 1024;
            constexpr uint16_t FRAG_TIMEOUT_MS  = 5000;
            constexpr size_t   MAX_DATA_SIZE    = 256 * 1024;
            constexpr uint32_t FRAG_INTERVAL_MS = 10;

            /*============================================================================
             * 帧类型
             *============================================================================*/

            enum class FrameType : uint8_t
            {
                NOP        = 0x00,
                HEARTBEAT  = 0x01,
                ACK        = 0x02,
                NACK       = 0x03,
                DATA       = 0x40,
                FRAG_FIRST = 0x41,
                FRAG_MID   = 0x42,
                FRAG_LAST  = 0x43,
            };

            enum FrameFlags : uint8_t
            {
                FLAG_NONE = 0x00,
                FLAG_ACK  = 0x01,
                FLAG_FRAG = 0x10,
                FLAG_LAST = 0x20,
            };

            enum class ErrCode : uint8_t
            {
                OK      = 0x00,
                CRC_ERR = 0x01,
                TIMEOUT = 0x02,
                INVALID = 0x05,
            };

            /*============================================================================
             * 帧头 16字节
             *============================================================================*/

            struct __attribute__((packed)) FrameHeader
            {
                uint16_t magic;
                uint8_t  version;
                uint8_t  type;
                uint8_t  flags;
                uint16_t seq;
                uint16_t len;
                uint32_t ts;
                uint8_t  msg_id;
                uint8_t  frag_idx;
                uint8_t  hdr_crc;
            };
            static_assert(sizeof(FrameHeader) == FRAME_HEADER_SIZE, "帧头大小错误");

            struct __attribute__((packed)) FragMeta
            {
                uint8_t  cnt;
                uint8_t  rsv;
                uint32_t total;
            };
            static_assert(sizeof(FragMeta) == FRAGMENT_META_SIZE, "分片元数据大小错误");

            /*============================================================================
             * 配置
             *============================================================================*/

            struct SpiCfg
            {
                std::string dev        = "/dev/spidev0.0";
                uint32_t    speed      = 10000000;
                uint8_t     mode       = 0;
                uint8_t     bits       = 8;
                int         gpio_req   = 58;
                int         gpio_rdy   = 59;
                uint16_t    hb_ms      = HEARTBEAT_MS;
                uint16_t    timeout_ms = TIMEOUT_MS;
                uint16_t    rdy_ms     = 100;
                bool        auto_ack   = true;
                size_t      tx_size    = RX_RING_SIZE;
                size_t      rx_size    = RX_RING_SIZE;
            };

            /*============================================================================
             * 统计
             *============================================================================*/

            struct Stats
            {
                std::atomic<uint32_t> tx_frm{0};
                std::atomic<uint32_t> rx_frm{0};
                std::atomic<uint64_t> tx_bytes{0};
                std::atomic<uint64_t> rx_bytes{0};
                std::atomic<uint32_t> tx_pending{0};
                std::atomic<uint32_t> crc_err{0};
                std::atomic<uint32_t> timeout{0};
                std::atomic<uint32_t> invalid{0};
                std::atomic<uint32_t> last_hb_ms{0};
                std::atomic<bool>     connected{false};

                void reset()
                {
                    tx_frm = rx_frm = 0;
                    tx_bytes = rx_bytes = 0;
                    tx_pending          = 0;
                    crc_err = timeout = invalid = 0;
                    last_hb_ms                  = 0;
                    connected                   = false;
                }
            };

            /*============================================================================
             * SPI驱动
             *============================================================================*/

            class SpiDrv
            {
            public:
                using RxCb = std::function<void(FrameType, const uint8_t*, size_t)>;

                static SpiDrv& inst();

                bool init(const SpiCfg& cfg);
                void deinit();
                bool start();
                void stop();

                bool send(const uint8_t* data, size_t len, bool need_ack = false);
                bool send_hb();
                bool send_ack(uint16_t seq);
                bool send_nack(uint16_t seq, ErrCode err);

                void set_cb(RxCb cb);

                bool connected() const
                {
                    return stats_.connected.load();
                }
                bool running() const
                {
                    return running_.load();
                }
                const Stats& stats() const
                {
                    return stats_;
                }
                void reset_stats()
                {
                    stats_.reset();
                }

            private:
                SpiDrv();
                ~SpiDrv();
                SpiDrv(const SpiDrv&)            = delete;
                SpiDrv& operator=(const SpiDrv&) = delete;

                bool init_buf();
                bool init_gpio();
                bool init_spi();
                void cleanup();

                bool gpio_export(int pin);
                bool gpio_set_dir(int pin, bool out);
                bool gpio_set_edge(int pin, const char* edge);
                bool gpio_write(int pin, int val);

                bool spi_xfer(const uint8_t* tx, uint8_t* rx, size_t len);

                void set_req(bool active);
                bool wait_rdy(uint32_t timeout_ms);

                size_t build_frame(uint8_t* buf, FrameType type, const uint8_t* data, size_t len,
                                   uint8_t flags, uint8_t mid = 0, uint8_t idx = 0);
                bool   parse_frame(const uint8_t* data, size_t len, FrameHeader& hdr,
                                   const uint8_t*& payload, size_t& plen);

                bool send_frame(FrameType type, const uint8_t* data, size_t len, uint8_t flags);
                bool send_frags(const uint8_t* data, size_t len, bool need_ack);

                void proc_rx(const uint8_t* data, size_t len);
                void handle_frag(const FrameHeader& hdr, const uint8_t* payload, size_t len);
                void dispatch(const uint8_t* data, size_t len);

                void comm_loop();
                void hb_loop();
                bool do_xfer();

                uint16_t next_seq()
                {
                    return seq_.fetch_add(1);
                }
                uint8_t  next_mid();
                uint32_t now_ms();
                void     update_conn(bool connected);
                void     check_timeout();

                static uint8_t  calc_crc8(const uint8_t* data, size_t len);
                static uint16_t calc_crc16(const uint8_t* data, size_t len);

                /* 分片上下文 */
                struct FragCtx
                {
                    std::vector<uint8_t> buf;
                    std::vector<bool>    map;
                    uint32_t             ts     = 0;
                    uint32_t             total  = 0;
                    uint8_t              mid    = 0;
                    uint8_t              cnt    = 0;
                    uint8_t              got    = 0;
                    bool                 active = false;

                    void reset()
                    {
                        buf.clear();
                        map.clear();
                        ts = total = mid = cnt = got = 0;
                        active                       = false;
                    }
                    bool complete() const
                    {
                        return active && got == cnt;
                    }
                };

                SpiCfg  cfg_;
                Stats   stats_;
                RxCb    rx_cb_;
                FragCtx frag_;

                int spi_fd_      = -1;
                int gpio_req_fd_ = -1;
                int gpio_rdy_fd_ = -1;

                std::unique_ptr<tool::memory::ring_buffer::RingBufferWithStats> tx_ring_;
                std::unique_ptr<tool::memory::ring_buffer::RingBufferWithStats> rx_ring_;

                uint8_t* frm_buf_ = nullptr;
                uint8_t* dma_tx_  = nullptr;
                uint8_t* dma_rx_  = nullptr;
                uint8_t  nop_tmpl_[FRAME_HEADER_SIZE + FRAME_CRC_SIZE];

                std::thread             comm_thd_;
                std::thread             hb_thd_;
                std::condition_variable tx_cv_;
                std::mutex              tx_mtx_;
                std::mutex              cb_mtx_;
                std::mutex              frag_mtx_;

                std::atomic<uint16_t> seq_{0};
                std::atomic<uint8_t>  mid_{1};
                std::atomic<bool>     running_{false};
                bool                  init_ = false;
    };

} // namespace app::common::spi
