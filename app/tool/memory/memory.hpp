#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <unordered_map>
#include <mutex>
#include <vector>
#include <memory>
#include <atomic>

namespace app::tool::memory
{

            constexpr size_t BYTES_PER_KILOBYTE        = 1024;
            constexpr size_t BYTES_PER_MEGABYTE        = BYTES_PER_KILOBYTE * 1024;
            constexpr size_t DEFAULT_INITIAL_POOL_SIZE = BYTES_PER_MEGABYTE; // 1 MB
            constexpr double DEFAULT_EXPANSION_FACTOR  = 2.0;

            /**
             * @brief 自定义内存删除器（支持对齐内存释放）
             */
            struct AlignedDeleter
            {
                void operator()(uint8_t* ptr) const noexcept
                {
                    if (ptr)
                    {
                        std::free(ptr);
                    }
                }
            };

            /**
             * @brief 高性能内存池类
             *
             * 特性:
             * - O(log n) 空闲块查找（使用 multimap 按大小排序）
             * - 自动合并相邻空闲块
             * - 支持自定义内存对齐
             * - 线程安全
             * - 增量统计信息维护
             */
            class MemoryPool
            {
            public:
                /**
                 * @brief 内存池统计信息
                 */
                struct Stats
                {
                    size_t total_memory{0};     ///< 总内存大小
                    size_t used_memory{0};      ///< 已使用内存大小
                    size_t free_memory{0};      ///< 空闲内存大小
                    size_t allocated_blocks{0}; ///< 已分配块数量
                    size_t free_blocks{0};      ///< 空闲块数量
                    size_t peak_usage{0};       ///< 峰值使用量
                    size_t allocation_count{0}; ///< 总分配次数
                    size_t dealloc_count{0};    ///< 总释放次数
                };

                /**
                 * @brief 构造函数
                 * @param initial_size 初始内存池大小（字节）
                 * @param alignment 内存对齐要求（字节，必须是2的幂）
                 * @param expansion_factor 扩展因子（当内存不足时的扩展倍数）
                 */
                explicit MemoryPool(size_t initial_size     = DEFAULT_INITIAL_POOL_SIZE,
                                    size_t alignment        = alignof(std::max_align_t),
                                    double expansion_factor = DEFAULT_EXPANSION_FACTOR);

                ~MemoryPool();

                // 禁止拷贝
                MemoryPool(const MemoryPool&)            = delete;
                MemoryPool& operator=(const MemoryPool&) = delete;

                // 允许移动
                MemoryPool(MemoryPool&&) noexcept;
                MemoryPool& operator=(MemoryPool&&) noexcept;

                /**
                 * @brief 分配内存
                 * @param size 请求的内存大小（字节）
                 * @return 分配的内存指针，失败返回 nullptr
                 */
                [[nodiscard]] void* allocate(size_t size);

                /**
                 * @brief 释放内存
                 * @param ptr 要释放的内存指针
                 */
                void deallocate(void* ptr) noexcept;

                /**
                 * @brief 重置内存池（释放所有内存块）
                 */
                void reset();

                /**
                 * @brief 获取内存池统计信息
                 * @return 统计信息结构体
                 */
                [[nodiscard]] Stats get_stats() const;

                /**
                 * @brief 快速获取可用内存大小（无锁）
                 */
                [[nodiscard]] size_t get_free_memory_fast() const noexcept
                {
                    return cached_free_memory_.load(std::memory_order_acquire);
                }

                /**
                 * @brief 快速获取已用内存大小（无锁）
                 */
                [[nodiscard]] size_t get_used_memory_fast() const noexcept
                {
                    return cached_used_memory_.load(std::memory_order_acquire);
                }

                /**
                 * @brief 检查内存池是否有效
                 */
                [[nodiscard]] bool valid() const noexcept
                {
                    return !pool_blocks_.empty();
                }

                /**
                 * @brief 获取对齐要求
                 */
                [[nodiscard]] size_t get_alignment() const noexcept
                {
                    return alignment_;
                }

            private:
                /**
                 * @brief 内存块头部信息
                 */
                struct BlockHeader
                {
                    size_t       size;       ///< 用户可用大小
                    bool         is_free;    ///< 是否空闲
                    BlockHeader* next;       ///< 链表下一个块
                    BlockHeader* prev;       ///< 链表上一个块
                    size_t       pool_index; ///< 所属池块索引
                };

                /**
                 * @brief 内存池块结构
                 */
                struct PoolBlock
                {
                    std::unique_ptr<uint8_t[], AlignedDeleter> memory;      ///< 内存块
                    size_t                                     size;        ///< 总大小
                    BlockHeader*                               first_block; ///< 第一个块
                };

                // 成员变量
                mutable std::mutex     mutex_;            ///< 线程安全互斥锁
                size_t                 alignment_;        ///< 内存对齐要求
                double                 expansion_factor_; ///< 扩展因子
                std::vector<PoolBlock> pool_blocks_;      ///< 内存池块列表

                // 指针 → 池块索引映射
                std::unordered_map<void*, size_t> pointer_map_;

                // 空闲块按大小排序（O(log n) 查找）
                std::multimap<size_t, void*> free_blocks_by_size_;

                // 空闲块指针 → multimap 迭代器（O(1) 删除）
                std::unordered_map<void*, std::multimap<size_t, void*>::iterator>
                    free_blocks_iterators_;

                // 缓存的统计信息（无锁快速访问）
                std::atomic<size_t> cached_free_memory_{0};
                std::atomic<size_t> cached_used_memory_{0};
                std::atomic<size_t> cached_peak_usage_{0};
                std::atomic<size_t> allocation_count_{0};
                std::atomic<size_t> dealloc_count_{0};

                // 私有方法
                void         init_pool(size_t size);
                void         expand_pool(size_t required_size);
                BlockHeader* find_free_block(size_t size, size_t& out_pool_index);
                void         split_block(BlockHeader* block, size_t size);
                void         coalesce_blocks(BlockHeader* block);
                void         add_to_free_list(BlockHeader* block);
                void         remove_from_free_list(BlockHeader* block);
                void         update_cached_stats();

                [[nodiscard]] size_t       aligned_size(size_t size) const noexcept;
                [[nodiscard]] size_t       get_header_size() const noexcept;
                [[nodiscard]] void*        get_user_data_ptr(BlockHeader* block) const noexcept;
                [[nodiscard]] BlockHeader* get_block_header(void* user_ptr) const noexcept;
            };

} // namespace app::tool::memory
