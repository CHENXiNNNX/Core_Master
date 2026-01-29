/**
 * @file double_buffer_impl.hpp
 * @brief DoubleBuffer template implementation (Linux version)
 */

#pragma once

#include <cstring>
#include <cstdlib>
#include "tool/log/log.hpp"

namespace app::tool::memory::double_buffer
{

    namespace
    {
        constexpr const char* TAG = "DblBuf";
    }

    template <bool EnableStats>
    DoubleBuffer<EnableStats>::DoubleBuffer(size_t buffer_size, size_t alignment)
        : buffer_size_(buffer_size)
        , is_static_(false)
    {
        states_[0].store(BufferState::EMPTY, std::memory_order_relaxed);
        states_[1].store(BufferState::EMPTY, std::memory_order_relaxed);
        data_sizes_[0].store(0, std::memory_order_relaxed);
        data_sizes_[1].store(0, std::memory_order_relaxed);

        if (buffer_size_ == 0)
            return;

        if (alignment == 0 || (alignment & (alignment - 1)) != 0)
            alignment = 4;

        buffer_size_ = (buffer_size_ + alignment - 1) & ~(alignment - 1);

        for (int i = 0; i < 2; i++)
        {
            void* ptr = nullptr;
            if (posix_memalign(&ptr, alignment, buffer_size_) != 0 || ptr == nullptr)
            {
                LOG_ERROR(TAG, "alloc buf[%d] failed: %zu bytes", i, buffer_size_);
                if (i == 1 && buffers_[0])
                {
                    std::free(buffers_[0]);
                    buffers_[0] = nullptr;
                }
                return;
            }
            buffers_[i] = static_cast<uint8_t*>(ptr);
        }
    }

    template <bool EnableStats>
    DoubleBuffer<EnableStats>::DoubleBuffer(uint8_t* buffer0, uint8_t* buffer1, size_t buffer_size)
        : buffer_size_(buffer_size)
        , is_static_(true)
    {
        buffers_[0] = buffer0;
        buffers_[1] = buffer1;
        states_[0].store(BufferState::EMPTY, std::memory_order_relaxed);
        states_[1].store(BufferState::EMPTY, std::memory_order_relaxed);
        data_sizes_[0].store(0, std::memory_order_relaxed);
        data_sizes_[1].store(0, std::memory_order_relaxed);

        if (!buffers_[0] || !buffers_[1] || buffer_size_ == 0)
        {
            buffers_[0] = nullptr;
            buffers_[1] = nullptr;
        }
    }

    template <bool EnableStats> DoubleBuffer<EnableStats>::~DoubleBuffer()
    {
        if (!is_static_)
        {
            if (buffers_[0])
                std::free(buffers_[0]);
            if (buffers_[1])
                std::free(buffers_[1]);
        }
    }

    // Statistics helpers
    template <bool EnableStats>
    inline void DoubleBuffer<EnableStats>::update_write_stats(size_t bytes)
    {
        if constexpr (EnableStats)
        {
            stats_.write_count.fetch_add(1, std::memory_order_relaxed);
            stats_.write_bytes.fetch_add(bytes, std::memory_order_relaxed);
        }
    }

    template <bool EnableStats>
    inline void DoubleBuffer<EnableStats>::update_read_stats(size_t bytes)
    {
        if constexpr (EnableStats)
        {
            stats_.read_count.fetch_add(1, std::memory_order_relaxed);
            stats_.read_bytes.fetch_add(bytes, std::memory_order_relaxed);
        }
    }

    template <bool EnableStats> inline void DoubleBuffer<EnableStats>::update_overflow()
    {
        if constexpr (EnableStats)
            stats_.overflow_count.fetch_add(1, std::memory_order_relaxed);
    }

    template <bool EnableStats> inline void DoubleBuffer<EnableStats>::update_underflow()
    {
        if constexpr (EnableStats)
            stats_.underflow_count.fetch_add(1, std::memory_order_relaxed);
    }

    template <bool EnableStats> int DoubleBuffer<EnableStats>::find_buffer(BufferState state) const
    {
        int idx = write_index_.load(std::memory_order_relaxed);
        if (states_[idx].load(std::memory_order_acquire) == state)
            return idx;

        int other = 1 - idx;
        if (states_[other].load(std::memory_order_acquire) == state)
            return other;

        return -1;
    }

    // Acquire write buffer
    template <bool EnableStats>
    uint8_t* DoubleBuffer<EnableStats>::acquire_write_buf(int timeout_ms)
    {
        if (!valid())
            return nullptr;

        std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
        if (timeout_ms > 0)
        {
            if (!lock.try_lock_for(std::chrono::milliseconds(timeout_ms)))
                return nullptr;
        }
        else
        {
            lock.lock();
        }

        int idx = find_buffer(BufferState::EMPTY);
        if (idx < 0)
        {
            update_overflow();
            return nullptr;
        }

        states_[idx].store(BufferState::WRITING, std::memory_order_release);
        write_index_.store(idx, std::memory_order_relaxed);

        return buffers_[idx];
    }

    // Release write buffer
    template <bool EnableStats> void DoubleBuffer<EnableStats>::release_write_buf(size_t data_size)
    {
        if (!valid())
            return;

        std::lock_guard<std::timed_mutex> lock(mutex_);

        int idx = write_index_.load(std::memory_order_relaxed);
        if (states_[idx].load(std::memory_order_acquire) == BufferState::WRITING)
        {
            data_sizes_[idx].store(data_size, std::memory_order_release);
            states_[idx].store(BufferState::READY, std::memory_order_release);
            update_write_stats(data_size);

            if constexpr (EnableStats)
                stats_.swap_count.fetch_add(1, std::memory_order_relaxed);
        }
    }

    // Acquire read buffer
    template <bool EnableStats>
    uint8_t* DoubleBuffer<EnableStats>::acquire_read_buf(size_t* data_size, int timeout_ms)
    {
        if (!valid())
            return nullptr;

        std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
        if (timeout_ms > 0)
        {
            if (!lock.try_lock_for(std::chrono::milliseconds(timeout_ms)))
                return nullptr;
        }
        else
        {
            lock.lock();
        }

        int idx = find_buffer(BufferState::READY);
        if (idx < 0)
        {
            update_underflow();
            return nullptr;
        }

        states_[idx].store(BufferState::READING, std::memory_order_release);
        read_index_.store(idx, std::memory_order_relaxed);

        if (data_size)
            *data_size = data_sizes_[idx].load(std::memory_order_acquire);

        return buffers_[idx];
    }

    // Release read buffer
    template <bool EnableStats> void DoubleBuffer<EnableStats>::release_read_buf()
    {
        if (!valid())
            return;

        std::lock_guard<std::timed_mutex> lock(mutex_);

        int idx = read_index_.load(std::memory_order_relaxed);
        if (states_[idx].load(std::memory_order_acquire) == BufferState::READING)
        {
            size_t bytes = data_sizes_[idx].load(std::memory_order_acquire);
            data_sizes_[idx].store(0, std::memory_order_release);
            states_[idx].store(BufferState::EMPTY, std::memory_order_release);
            update_read_stats(bytes);
        }
    }

    // Convenience write (with copy)
    template <bool EnableStats>
    bool DoubleBuffer<EnableStats>::write(const uint8_t* data, size_t len)
    {
        if (!data || len == 0 || len > buffer_size_)
            return false;

        uint8_t* buf = acquire_write_buf();
        if (!buf)
            return false;

        std::memcpy(buf, data, len);
        release_write_buf(len);
        return true;
    }

    // Convenience read (with copy)
    template <bool EnableStats>
    size_t DoubleBuffer<EnableStats>::read(uint8_t* data, size_t max_len)
    {
        if (!data || max_len == 0)
            return 0;

        size_t   size = 0;
        uint8_t* buf  = acquire_read_buf(&size);
        if (!buf)
            return 0;

        size_t to_read = (size < max_len) ? size : max_len;
        std::memcpy(data, buf, to_read);
        release_read_buf();
        return to_read;
    }

    // Has data (locked)
    template <bool EnableStats> bool DoubleBuffer<EnableStats>::has_data() const
    {
        if (!valid())
            return false;

        std::lock_guard<std::timed_mutex> lock(mutex_);

        return states_[0].load(std::memory_order_acquire) == BufferState::READY ||
               states_[1].load(std::memory_order_acquire) == BufferState::READY;
    }

    // Can write (locked)
    template <bool EnableStats> bool DoubleBuffer<EnableStats>::can_write() const
    {
        if (!valid())
            return false;

        std::lock_guard<std::timed_mutex> lock(mutex_);

        return states_[0].load(std::memory_order_acquire) == BufferState::EMPTY ||
               states_[1].load(std::memory_order_acquire) == BufferState::EMPTY;
    }

    // Reset
    template <bool EnableStats> void DoubleBuffer<EnableStats>::reset()
    {
        if (!valid())
            return;

        std::lock_guard<std::timed_mutex> lock(mutex_);

        states_[0].store(BufferState::EMPTY, std::memory_order_release);
        states_[1].store(BufferState::EMPTY, std::memory_order_release);
        data_sizes_[0].store(0, std::memory_order_release);
        data_sizes_[1].store(0, std::memory_order_release);
        write_index_.store(0, std::memory_order_release);
        read_index_.store(0, std::memory_order_release);
    }

} // namespace app::tool::memory::double_buffer
