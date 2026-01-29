/**
 * @file ring_buffer.hpp
 * @brief Thread-safe ring buffer with optional statistics (Linux version)
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>

namespace app::tool::memory::ring_buffer
{

    struct Stats
    {
        std::atomic<uint32_t> write_count{0};
        std::atomic<uint32_t> read_count{0};
        std::atomic<uint32_t> write_bytes{0};
        std::atomic<uint32_t> read_bytes{0};
        std::atomic<uint32_t> overflow_count{0};
        std::atomic<uint32_t> underflow_count{0};

        void reset()
        {
            write_count.store(0, std::memory_order_relaxed);
            read_count.store(0, std::memory_order_relaxed);
            write_bytes.store(0, std::memory_order_relaxed);
            read_bytes.store(0, std::memory_order_relaxed);
            overflow_count.store(0, std::memory_order_relaxed);
            underflow_count.store(0, std::memory_order_relaxed);
        }
    };

    /**
     * @brief Thread-safe ring buffer
     * @tparam EnableStats Enable statistics tracking (default: disabled)
     */
    template <bool EnableStats = false> class RingBuffer
    {
    public:
        /**
         * @brief Construct with dynamic allocation
         * @param capacity Buffer capacity in bytes
         * @param alignment Memory alignment (default: 4)
         */
        explicit RingBuffer(size_t capacity, size_t alignment = 4);

        /**
         * @brief Construct with static buffer (zero allocation)
         * @param static_buffer External buffer pointer
         * @param capacity Buffer capacity
         */
        RingBuffer(uint8_t* static_buffer, size_t capacity);

        ~RingBuffer();

        RingBuffer(const RingBuffer&)            = delete;
        RingBuffer& operator=(const RingBuffer&) = delete;
        RingBuffer(RingBuffer&&)                 = delete;
        RingBuffer& operator=(RingBuffer&&)      = delete;

        // Basic operations
        size_t write(const uint8_t* data, size_t len, int timeout_ms = 0);
        size_t read(uint8_t* data, size_t len, int timeout_ms = 0);
        size_t peek(uint8_t* data, size_t len) const;
        size_t skip(size_t len);
        void   clear();

        // Single byte operations
        bool write_byte(uint8_t byte);
        bool read_byte(uint8_t* byte);

        template <size_t N> size_t write_fixed(const uint8_t (&data)[N])
        {
            return write(data, N, 0);
        }

        template <size_t N> size_t read_fixed(uint8_t (&data)[N])
        {
            return read(data, N, 0);
        }

        // Lock-free queries
        size_t available_fast() const
        {
            return data_size_.load(std::memory_order_acquire);
        }

        size_t free_space_fast() const
        {
            return capacity_ - data_size_.load(std::memory_order_acquire);
        }

        // Locked queries
        size_t available() const;
        size_t free_space() const;

        // State
        size_t capacity() const
        {
            return capacity_;
        }
        bool empty() const
        {
            return available_fast() == 0;
        }
        bool full() const
        {
            return free_space_fast() == 0;
        }
        bool valid() const
        {
            return buffer_ != nullptr;
        }
        bool is_static_buffer() const
        {
            return is_static_;
        }

        uint8_t* get_buffer()
        {
            return buffer_;
        }
        const uint8_t* get_buffer() const
        {
            return buffer_;
        }

        // Statistics (returns nullptr if EnableStats == false)
        const Stats* get_stats() const
        {
            if constexpr (EnableStats)
                return &stats_;
            return nullptr;
        }

        void reset_stats()
        {
            if constexpr (EnableStats)
                stats_.reset();
        }

    private:
        void update_write_stats(size_t bytes);
        void update_read_stats(size_t bytes);
        void update_overflow();
        void update_underflow();

        uint8_t*                    buffer_{nullptr};
        size_t                      capacity_{0};
        std::atomic<size_t>         read_pos_{0};
        std::atomic<size_t>         write_pos_{0};
        std::atomic<size_t>         data_size_{0};
        mutable std::timed_mutex    mutex_;
        std::condition_variable_any read_cv_;
        std::condition_variable_any write_cv_;
        bool                        is_static_{false};
        Stats                       stats_;
    };

    using RingBufferBasic     = RingBuffer<false>;
    using RingBufferWithStats = RingBuffer<true>;

} // namespace app::tool::memory::ring_buffer

#include "ring_buffer_impl.hpp"
