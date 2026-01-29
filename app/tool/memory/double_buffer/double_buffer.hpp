/**
 * @file double_buffer.hpp
 * @brief Zero-copy double buffer with optional statistics (Linux version)
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>

namespace app::tool::memory::double_buffer
{

    enum class BufferState : uint8_t
    {
        EMPTY,
        WRITING,
        READY,
        READING
    };

    struct Stats
    {
        std::atomic<uint32_t> write_count{0};
        std::atomic<uint32_t> read_count{0};
        std::atomic<uint32_t> swap_count{0};
        std::atomic<uint32_t> write_bytes{0};
        std::atomic<uint32_t> read_bytes{0};
        std::atomic<uint32_t> overflow_count{0};
        std::atomic<uint32_t> underflow_count{0};

        void reset()
        {
            write_count.store(0, std::memory_order_relaxed);
            read_count.store(0, std::memory_order_relaxed);
            swap_count.store(0, std::memory_order_relaxed);
            write_bytes.store(0, std::memory_order_relaxed);
            read_bytes.store(0, std::memory_order_relaxed);
            overflow_count.store(0, std::memory_order_relaxed);
            underflow_count.store(0, std::memory_order_relaxed);
        }
    };

    /**
     * @brief Zero-copy double buffer
     * @tparam EnableStats Enable statistics tracking (default: disabled)
     */
    template <bool EnableStats = false> class DoubleBuffer
    {
    public:
        /**
         * @brief Construct with dynamic allocation
         * @param buffer_size Single buffer size in bytes
         * @param alignment Memory alignment (default: 4)
         */
        explicit DoubleBuffer(size_t buffer_size, size_t alignment = 4);

        /**
         * @brief Construct with static buffers (zero allocation)
         */
        DoubleBuffer(uint8_t* buffer0, uint8_t* buffer1, size_t buffer_size);

        ~DoubleBuffer();

        DoubleBuffer(const DoubleBuffer&)            = delete;
        DoubleBuffer& operator=(const DoubleBuffer&) = delete;
        DoubleBuffer(DoubleBuffer&&)                 = delete;
        DoubleBuffer& operator=(DoubleBuffer&&)      = delete;

        // Zero-copy interface
        uint8_t* acquire_write_buf(int timeout_ms = 100);
        void     release_write_buf(size_t data_size);
        uint8_t* acquire_read_buf(size_t* data_size, int timeout_ms = 100);
        void     release_read_buf();

        // Convenience interface (with copy)
        bool   write(const uint8_t* data, size_t len);
        size_t read(uint8_t* data, size_t max_len);

        // Lock-free queries
        bool has_data_fast() const
        {
            return states_[0].load(std::memory_order_acquire) == BufferState::READY ||
                   states_[1].load(std::memory_order_acquire) == BufferState::READY;
        }

        bool can_write_fast() const
        {
            return states_[0].load(std::memory_order_acquire) == BufferState::EMPTY ||
                   states_[1].load(std::memory_order_acquire) == BufferState::EMPTY;
        }

        // Locked queries
        bool has_data() const;
        bool can_write() const;

        // State
        size_t get_buffer_size() const
        {
            return buffer_size_;
        }
        bool valid() const
        {
            return buffers_[0] && buffers_[1];
        }
        bool is_static_buffer() const
        {
            return is_static_;
        }

        BufferState get_buffer_state(int i) const
        {
            return (i < 0 || i > 1) ? BufferState::EMPTY
                                    : states_[i].load(std::memory_order_acquire);
        }

        size_t get_data_size(int i) const
        {
            return (i < 0 || i > 1) ? 0 : data_sizes_[i].load(std::memory_order_acquire);
        }

        uint8_t* get_buffer(int i)
        {
            return (i < 0 || i > 1) ? nullptr : buffers_[i];
        }

        const uint8_t* get_buffer(int i) const
        {
            return (i < 0 || i > 1) ? nullptr : buffers_[i];
        }

        void reset();

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
        int  find_buffer(BufferState state) const;
        void update_write_stats(size_t bytes);
        void update_read_stats(size_t bytes);
        void update_overflow();
        void update_underflow();

        uint8_t*                 buffers_[2]{nullptr, nullptr};
        size_t                   buffer_size_{0};
        std::atomic<BufferState> states_[2];
        std::atomic<size_t>      data_sizes_[2];
        std::atomic<int>         write_index_{0};
        std::atomic<int>         read_index_{0};
        mutable std::timed_mutex mutex_;
        bool                     is_static_{false};
        Stats                    stats_;
    };

    using DoubleBufferBasic     = DoubleBuffer<false>;
    using DoubleBufferWithStats = DoubleBuffer<true>;

} // namespace app::tool::memory::double_buffer

#include "double_buffer_impl.hpp"
