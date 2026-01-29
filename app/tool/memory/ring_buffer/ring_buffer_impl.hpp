/**
 * @file ring_buffer_impl.hpp
 * @brief RingBuffer template implementation (Linux version)
 */

#pragma once

#include <cstring>
#include <cstdlib>
#include "tool/log/log.hpp"

namespace app::tool::memory::ring_buffer
{

    namespace
    {
        constexpr const char* TAG = "RingBuf";
    }

    template <bool EnableStats>
    RingBuffer<EnableStats>::RingBuffer(size_t capacity, size_t alignment)
        : capacity_(capacity)
        , is_static_(false)
    {
        if (capacity_ == 0)
            return;

        if (alignment == 0 || (alignment & (alignment - 1)) != 0)
            alignment = 4;

        capacity_ = (capacity_ + alignment - 1) & ~(alignment - 1);

        void* ptr = nullptr;
        if (posix_memalign(&ptr, alignment, capacity_) != 0 || ptr == nullptr)
        {
            LOG_ERROR(TAG, "alloc failed: %zu bytes", capacity_);
            return;
        }
        buffer_ = static_cast<uint8_t*>(ptr);
    }

    template <bool EnableStats>
    RingBuffer<EnableStats>::RingBuffer(uint8_t* static_buffer, size_t capacity)
        : buffer_(static_buffer)
        , capacity_(capacity)
        , is_static_(true)
    {
        if (!buffer_ || capacity_ == 0)
        {
            buffer_ = nullptr;
        }
    }

    template <bool EnableStats> RingBuffer<EnableStats>::~RingBuffer()
    {
        if (!is_static_ && buffer_)
            std::free(buffer_);
    }

    // Statistics helpers
    template <bool EnableStats>
    inline void RingBuffer<EnableStats>::update_write_stats(size_t bytes)
    {
        if constexpr (EnableStats)
        {
            stats_.write_count.fetch_add(1, std::memory_order_relaxed);
            stats_.write_bytes.fetch_add(bytes, std::memory_order_relaxed);
        }
    }

    template <bool EnableStats> inline void RingBuffer<EnableStats>::update_read_stats(size_t bytes)
    {
        if constexpr (EnableStats)
        {
            stats_.read_count.fetch_add(1, std::memory_order_relaxed);
            stats_.read_bytes.fetch_add(bytes, std::memory_order_relaxed);
        }
    }

    template <bool EnableStats> inline void RingBuffer<EnableStats>::update_overflow()
    {
        if constexpr (EnableStats)
            stats_.overflow_count.fetch_add(1, std::memory_order_relaxed);
    }

    template <bool EnableStats> inline void RingBuffer<EnableStats>::update_underflow()
    {
        if constexpr (EnableStats)
            stats_.underflow_count.fetch_add(1, std::memory_order_relaxed);
    }

    // Write
    template <bool EnableStats>
    size_t RingBuffer<EnableStats>::write(const uint8_t* data, size_t len, int timeout_ms)
    {
        if (!valid() || !data || len == 0)
            return 0;

        // Fast path: check if there's space without locking
        if (timeout_ms == 0 && free_space_fast() == 0)
        {
            update_overflow();
            return 0;
        }

        std::unique_lock<std::timed_mutex> lock(mutex_);

        // Wait for space if timeout specified
        if (timeout_ms != 0)
        {
            auto predicate = [this]()
            {
                return data_size_.load(std::memory_order_relaxed) < capacity_;
            };

            if (timeout_ms < 0)
            {
                write_cv_.wait(lock, predicate);
            }
            else
            {
                if (!write_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), predicate))
                {
                    update_overflow();
                    return 0;
                }
            }
        }

        size_t current  = data_size_.load(std::memory_order_relaxed);
        size_t free     = capacity_ - current;
        size_t to_write = (len < free) ? len : free;

        if (to_write > 0)
        {
            size_t wp    = write_pos_.load(std::memory_order_relaxed);
            size_t first = capacity_ - wp;

            if (to_write <= first)
            {
                std::memcpy(buffer_ + wp, data, to_write);
            }
            else
            {
                std::memcpy(buffer_ + wp, data, first);
                std::memcpy(buffer_, data + first, to_write - first);
            }

            write_pos_.store((wp + to_write) % capacity_, std::memory_order_release);
            data_size_.fetch_add(to_write, std::memory_order_release);

            update_write_stats(to_write);
        }

        lock.unlock();
        read_cv_.notify_one();

        return to_write;
    }

    // Read
    template <bool EnableStats>
    size_t RingBuffer<EnableStats>::read(uint8_t* data, size_t len, int timeout_ms)
    {
        if (!valid() || !data || len == 0)
            return 0;

        // Fast path: check if there's data without locking
        if (timeout_ms == 0 && available_fast() == 0)
        {
            update_underflow();
            return 0;
        }

        std::unique_lock<std::timed_mutex> lock(mutex_);

        // Wait for data if timeout specified
        if (timeout_ms != 0)
        {
            auto predicate = [this]()
            {
                return data_size_.load(std::memory_order_relaxed) > 0;
            };

            if (timeout_ms < 0)
            {
                read_cv_.wait(lock, predicate);
            }
            else
            {
                if (!read_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), predicate))
                {
                    update_underflow();
                    return 0;
                }
            }
        }

        size_t current = data_size_.load(std::memory_order_relaxed);
        size_t to_read = (len < current) ? len : current;

        if (to_read > 0)
        {
            size_t rp    = read_pos_.load(std::memory_order_relaxed);
            size_t first = capacity_ - rp;

            if (to_read <= first)
            {
                std::memcpy(data, buffer_ + rp, to_read);
            }
            else
            {
                std::memcpy(data, buffer_ + rp, first);
                std::memcpy(data + first, buffer_, to_read - first);
            }

            read_pos_.store((rp + to_read) % capacity_, std::memory_order_release);
            data_size_.fetch_sub(to_read, std::memory_order_release);

            update_read_stats(to_read);
        }

        lock.unlock();
        write_cv_.notify_one();

        return to_read;
    }

    // Single byte write
    template <bool EnableStats> bool RingBuffer<EnableStats>::write_byte(uint8_t byte)
    {
        if (!valid() || free_space_fast() == 0)
        {
            update_overflow();
            return false;
        }

        std::lock_guard<std::timed_mutex> lock(mutex_);

        if (data_size_.load(std::memory_order_relaxed) >= capacity_)
        {
            update_overflow();
            return false;
        }

        size_t wp   = write_pos_.load(std::memory_order_relaxed);
        buffer_[wp] = byte;
        write_pos_.store((wp + 1) % capacity_, std::memory_order_release);
        data_size_.fetch_add(1, std::memory_order_release);

        update_write_stats(1);
        read_cv_.notify_one();

        return true;
    }

    // Single byte read
    template <bool EnableStats> bool RingBuffer<EnableStats>::read_byte(uint8_t* byte)
    {
        if (!valid() || !byte || available_fast() == 0)
        {
            update_underflow();
            return false;
        }

        std::lock_guard<std::timed_mutex> lock(mutex_);

        if (data_size_.load(std::memory_order_relaxed) == 0)
        {
            update_underflow();
            return false;
        }

        size_t rp = read_pos_.load(std::memory_order_relaxed);
        *byte     = buffer_[rp];
        read_pos_.store((rp + 1) % capacity_, std::memory_order_release);
        data_size_.fetch_sub(1, std::memory_order_release);

        update_read_stats(1);
        write_cv_.notify_one();

        return true;
    }

    // Peek
    template <bool EnableStats>
    size_t RingBuffer<EnableStats>::peek(uint8_t* data, size_t len) const
    {
        if (!valid() || !data || len == 0)
            return 0;

        std::lock_guard<std::timed_mutex> lock(mutex_);

        size_t current = data_size_.load(std::memory_order_relaxed);
        size_t to_peek = (len < current) ? len : current;

        if (to_peek > 0)
        {
            size_t rp    = read_pos_.load(std::memory_order_relaxed);
            size_t first = capacity_ - rp;

            if (to_peek <= first)
                std::memcpy(data, buffer_ + rp, to_peek);
            else
            {
                std::memcpy(data, buffer_ + rp, first);
                std::memcpy(data + first, buffer_, to_peek - first);
            }
        }

        return to_peek;
    }

    // Skip
    template <bool EnableStats> size_t RingBuffer<EnableStats>::skip(size_t len)
    {
        if (!valid() || len == 0)
            return 0;

        std::lock_guard<std::timed_mutex> lock(mutex_);

        size_t current = data_size_.load(std::memory_order_relaxed);
        size_t to_skip = (len < current) ? len : current;

        if (to_skip > 0)
        {
            size_t rp = read_pos_.load(std::memory_order_relaxed);
            read_pos_.store((rp + to_skip) % capacity_, std::memory_order_release);
            data_size_.fetch_sub(to_skip, std::memory_order_release);
        }

        write_cv_.notify_one();
        return to_skip;
    }

    // Clear
    template <bool EnableStats> void RingBuffer<EnableStats>::clear()
    {
        if (!valid())
            return;

        std::lock_guard<std::timed_mutex> lock(mutex_);

        read_pos_.store(0, std::memory_order_release);
        write_pos_.store(0, std::memory_order_release);
        data_size_.store(0, std::memory_order_release);

        write_cv_.notify_all();
    }

    // Available (locked)
    template <bool EnableStats> size_t RingBuffer<EnableStats>::available() const
    {
        if (!valid())
            return 0;

        std::lock_guard<std::timed_mutex> lock(mutex_);
        return data_size_.load(std::memory_order_acquire);
    }

    // Free space (locked)
    template <bool EnableStats> size_t RingBuffer<EnableStats>::free_space() const
    {
        if (!valid())
            return 0;

        std::lock_guard<std::timed_mutex> lock(mutex_);
        return capacity_ - data_size_.load(std::memory_order_acquire);
    }

} // namespace app::tool::memory::ring_buffer
