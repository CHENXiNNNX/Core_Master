#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <termios.h>

#include "tool/memory/ring_buffer/ring_buffer.hpp"

namespace app::common::uart
{

    /*============================================================================
     * 协议定义
     *============================================================================*/

            constexpr uint16_t FRAME_MAGIC        = 0xAA55;
            constexpr size_t   FRAME_HEADER_SIZE  = 12;
            constexpr size_t   FRAME_CRC_SIZE     = 2;
            constexpr size_t   MAX_PAYLOAD_SIZE   = 4070;
            constexpr size_t   MAX_FRAME_SIZE     = 4084;
            constexpr size_t   FRAGMENT_META_SIZE = 6;

            constexpr size_t COBS_OVERHEAD = (MAX_FRAME_SIZE / 254) + 2;
            constexpr size_t COBS_MAX_LEN  = MAX_FRAME_SIZE + COBS_OVERHEAD;

            constexpr size_t   RX_RING_SIZE    = 64 * 1024;
            constexpr uint16_t FRAG_TIMEOUT_MS = 5000;
            constexpr size_t   MAX_DATA_SIZE   = 256 * 1024;

            constexpr uint8_t XON_THRESH  = 20;
            constexpr uint8_t XOFF_THRESH = 80;

            /*============================================================================
             * 帧类型
             *============================================================================*/

            enum class FrameType : uint8_t
            {
                NOP        = 0x00,
                XON        = 0x01,
                XOFF       = 0x02,
                DATA       = 0x40,
                FRAG_FIRST = 0x41,
                FRAG_MID   = 0x42,
                FRAG_LAST  = 0x43,
            };

            enum FrameFlags : uint8_t
            {
                FLAG_NONE = 0x00,
                FLAG_FRAG = 0x10,
                FLAG_LAST = 0x20,
            };

            enum class ErrCode : uint8_t
            {
                OK       = 0x00,
                CRC_ERR  = 0x01,
                TIMEOUT  = 0x02,
                COBS_ERR = 0x03,
                OVERFLOW = 0x04,
                INVALID  = 0x05,
            };

            /*============================================================================
             * 帧头 12字节
             *============================================================================*/

            struct __attribute__((packed)) FrameHeader
            {
                uint16_t magic;
                uint8_t  type;
                uint8_t  flags;
                uint16_t len;
                uint16_t seq;
                uint8_t  msg_id;
                uint8_t  frag_idx;
                uint8_t  frag_cnt;
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

            struct UartCfg
            {
                std::string dev     = "/dev/ttyS3";
                int         baud    = 115200;
                int         bits    = 8;
                int         stop    = 1;
                char        parity  = 'N';
                size_t      rx_size = RX_RING_SIZE;
                size_t      tx_size = RX_RING_SIZE;
                uint8_t     xon_th  = XON_THRESH;
                uint8_t     xoff_th = XOFF_THRESH;
                int         poll_ms = 10;
                uint16_t    frag_ms = FRAG_TIMEOUT_MS;
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
                std::atomic<uint32_t> crc_err{0};
                std::atomic<uint32_t> cobs_err{0};
                std::atomic<uint32_t> timeout{0};
                std::atomic<uint32_t> invalid{0};
                std::atomic<uint32_t> overflow{0};
                std::atomic<bool>     paused{false};
                std::atomic<uint32_t> xon_cnt{0};
                std::atomic<uint32_t> xoff_cnt{0};

                void reset()
                {
                    tx_frm = rx_frm = 0;
                    tx_bytes = rx_bytes = 0;
                    crc_err = cobs_err = timeout = invalid = overflow = 0;
                    paused                                            = false;
                    xon_cnt = xoff_cnt = 0;
                }
            };

            /*============================================================================
             * COBS编解码
             *============================================================================*/

            size_t cobs_encode(const uint8_t* src, size_t len, uint8_t* dst);
            size_t cobs_decode(const uint8_t* src, size_t len, uint8_t* dst, size_t max);

            /*============================================================================
             * UART驱动
             *============================================================================*/

            class UartDrv
            {
            public:
                using RxCb = std::function<void(FrameType, const uint8_t*, size_t)>;

                static UartDrv& inst();

                bool init(const UartCfg& cfg);
                void deinit();
                bool start();
                void stop();

                bool send(const uint8_t* data, size_t len);
                void set_cb(RxCb cb);

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

                bool send_xon();
                bool send_xoff();

            private:
                UartDrv();
                ~UartDrv();
                UartDrv(const UartDrv&)            = delete;
                UartDrv& operator=(const UartDrv&) = delete;

                bool    init_uart();
                bool    init_poll();
                bool    init_buf();
                void    cleanup();
                bool    cfg_uart(int fd);
                speed_t baud_to_speed(int baud);

                size_t build_frame(uint8_t* buf, FrameType type, const uint8_t* data, size_t len,
                                   uint8_t flags, uint8_t mid = 0, uint8_t idx = 0,
                                   uint8_t cnt = 0);
                bool   parse_frame(const uint8_t* data, size_t len, FrameHeader& hdr,
                                   const uint8_t*& payload, size_t& plen);

                bool send_frame(FrameType type, const uint8_t* data, size_t len, uint8_t flags);
                bool send_frags(const uint8_t* data, size_t len);
                bool uart_write(const uint8_t* data, size_t len);

                void proc_rx(const uint8_t* data, size_t len);
                void handle_frame(const uint8_t* data, size_t len);
                void handle_frag(const FrameHeader& hdr, const uint8_t* payload, size_t len);
                void dispatch(const uint8_t* data, size_t len);

                void check_flow();
                void on_xon();
                void on_xoff();

                void rx_loop();

                uint16_t next_seq()
                {
                    return seq_.fetch_add(1);
                }
                uint8_t  next_mid();
                uint32_t now_ms();
                void     check_frag_timeout();

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

                /* COBS接收状态 */
                struct CobsRx
                {
                    std::vector<uint8_t> buf;
                    bool                 in_frame = false;
                    CobsRx()
                    {
                        buf.reserve(COBS_MAX_LEN);
                    }
                    void reset()
                    {
                        buf.clear();
                        in_frame = false;
                    }
                };

                UartCfg cfg_;
                Stats   stats_;
                RxCb    rx_cb_;
                FragCtx frag_;
                CobsRx  cobs_;

                int fd_uart_ = -1;
                int fd_poll_ = -1;

                std::unique_ptr<tool::memory::ring_buffer::RingBufferWithStats> rx_ring_;

                std::vector<uint8_t> frm_buf_;
                std::vector<uint8_t> cobs_buf_;
                std::vector<uint8_t> dec_buf_;
                std::vector<uint8_t> poll_buf_;

                std::thread rx_thd_;
                std::mutex  tx_mtx_;
                std::mutex  cb_mtx_;
                std::mutex  frag_mtx_;

                std::atomic<uint16_t> seq_{0};
                std::atomic<uint8_t>  mid_{1};
                std::atomic<bool>     running_{false};
                std::atomic<bool>     xoff_rx_{false};
                bool                  init_    = false;
                bool                  xoff_tx_ = false;
    };

} // namespace app::common::uart
