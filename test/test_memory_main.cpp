/**
 * @file main.cpp
 * @brief 内存工具模块综合测试
 */

#include "tool/memory/memory.hpp"
#include "tool/memory/double_buffer/double_buffer.hpp"
#include "tool/memory/ring_buffer/ring_buffer.hpp"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cstring>

using namespace app::tool::memory;
using namespace app::tool::memory::double_buffer;
using namespace app::tool::memory::ring_buffer;

// ===================== 内存池测试 =====================
void testMemoryPool()
{
    std::cout << "\n========== 内存池测试 ==========\n" << std::endl;

    MemoryPool pool(1024, 8, 2.0);

    void* ptr1 = pool.allocate(100);
    void* ptr2 = pool.allocate(200);
    void* ptr3 = pool.allocate(300);

    std::cout << "[分配] 3块内存: " << ptr1 << ", " << ptr2 << ", " << ptr3 << std::endl;

    pool.deallocate(ptr2);
    std::cout << "[释放] 中间块内存" << std::endl;

    void* ptr4 = pool.allocate(150);
    std::cout << "[重分配] " << ptr4 << std::endl;

    auto stats = pool.get_stats();
    std::cout << "[统计] 总内存: " << stats.total_memory << " 字节, "
              << "已使用: " << stats.used_memory << " 字节, "
              << "已分配块: " << stats.allocated_blocks << std::endl;

    pool.deallocate(ptr1);
    pool.deallocate(ptr3);
    pool.deallocate(ptr4);

    std::cout << "[完成] 内存池测试通过 ✓" << std::endl;
}

// ===================== 双缓冲测试 =====================
void testDoubleBuffer()
{
    std::cout << "\n========== 双缓冲测试 ==========\n" << std::endl;

    // 基本测试
    DoubleBufferWithStats db(1024, 8);

    if (!db.valid())
    {
        std::cerr << "[错误] 双缓冲创建失败!" << std::endl;
        return;
    }

    std::cout << "[创建] 双缓冲大小: " << db.getBufferSize() << " 字节" << std::endl;

    // 写入测试
    uint8_t writeData[256];
    for (int i = 0; i < 256; i++)
        writeData[i] = static_cast<uint8_t>(i);

    if (db.write(writeData, 256))
    {
        std::cout << "[写入] 256 字节成功" << std::endl;
    }

    // 读取测试
    uint8_t readData[256];
    size_t  readLen = db.read(readData, 256);
    std::cout << "[读取] " << readLen << " 字节" << std::endl;

    // 验证数据
    bool dataValid = true;
    for (int i = 0; i < 256; i++)
    {
        if (readData[i] != static_cast<uint8_t>(i))
        {
            dataValid = false;
            break;
        }
    }
    std::cout << "[验证] 数据完整性: " << (dataValid ? "通过 ✓" : "失败 ✗") << std::endl;

    // Zero-copy 测试
    uint8_t* writeBuf = db.acquire_write_buf();
    if (writeBuf)
    {
        std::memset(writeBuf, 0xAA, 512);
        db.release_write_buf(512);
        std::cout << "[Zero-copy写入] 512 字节成功" << std::endl;
    }

    size_t   dataSize = 0;
    uint8_t* readBuf  = db.acquire_read_buf(&dataSize);
    if (readBuf)
    {
        std::cout << "[Zero-copy读取] " << dataSize << " 字节" << std::endl;
        db.release_read_buf();
    }

    // 统计信息
    const auto* stats = db.get_stats();
    if (stats)
    {
        std::cout << "[统计] 写入次数: " << stats->write_count.load()
                  << ", 读取次数: " << stats->read_count.load()
                  << ", 写入字节: " << stats->write_bytes.load()
                  << ", 读取字节: " << stats->read_bytes.load() << std::endl;
    }

    std::cout << "[完成] 双缓冲测试通过 ✓" << std::endl;
}

// ===================== 环形缓冲测试 =====================
void testRingBuffer()
{
    std::cout << "\n========== 环形缓冲测试 ==========\n" << std::endl;

    RingBufferWithStats rb(1024, 8);

    if (!rb.valid())
    {
        std::cerr << "[错误] 环形缓冲创建失败!" << std::endl;
        return;
    }

    std::cout << "[创建] 环形缓冲容量: " << rb.capacity() << " 字节" << std::endl;

    // 写入测试
    uint8_t writeData[100];
    for (int i = 0; i < 100; i++)
        writeData[i] = static_cast<uint8_t>(i);

    size_t written = rb.write(writeData, 100);
    std::cout << "[写入] " << written << " 字节, 可用: " << rb.available()
              << ", 空闲: " << rb.freeSpace() << std::endl;

    // 再写入一些
    written = rb.write(writeData, 100);
    std::cout << "[写入] " << written << " 字节, 可用: " << rb.available() << std::endl;

    // 读取测试
    uint8_t readData[50];
    size_t  readLen = rb.read(readData, 50);
    std::cout << "[读取] " << readLen << " 字节, 可用: " << rb.available() << std::endl;

    // Peek 测试
    uint8_t peekData[20];
    size_t  peekLen = rb.peek(peekData, 20);
    std::cout << "[Peek] " << peekLen << " 字节 (不消耗数据)" << std::endl;

    // Skip 测试
    size_t skipped = rb.skip(30);
    std::cout << "[Skip] " << skipped << " 字节, 可用: " << rb.available() << std::endl;

    // 单字节操作
    rb.writeByte(0xFF);
    uint8_t byte;
    if (rb.readByte(&byte))
    {
        std::cout << "[单字节] 写入/读取: 0x" << std::hex << static_cast<int>(byte) << std::dec
                  << std::endl;
    }

    // 统计信息
    const auto* stats = rb.get_stats();
    if (stats)
    {
        std::cout << "[统计] 写入次数: " << stats->write_count.load()
                  << ", 读取次数: " << stats->read_count.load()
                  << ", 写入字节: " << stats->write_bytes.load()
                  << ", 读取字节: " << stats->read_bytes.load() << std::endl;
    }

    // 清空测试
    rb.clear();
    std::cout << "[清空] 可用: " << rb.available() << ", 空闲: " << rb.freeSpace() << std::endl;

    std::cout << "[完成] 环形缓冲测试通过 ✓" << std::endl;
}

// ===================== 多线程测试 =====================
void testMultiThread()
{
    std::cout << "\n========== 多线程测试 ==========\n" << std::endl;

    // 环形缓冲生产者-消费者测试
    RingBufferBasic     rb(4096);
    std::atomic<bool>   running{true};
    std::atomic<size_t> totalWritten{0};
    std::atomic<size_t> totalRead{0};

    auto startTime = std::chrono::high_resolution_clock::now();

    // 生产者线程
    std::thread producer(
        [&]()
        {
            uint8_t data[64];
            std::memset(data, 0xAA, sizeof(data));

            for (int i = 0; i < 1000 && running; i++)
            {
                size_t written = rb.write(data, sizeof(data), 10);
                totalWritten += written;
                if (written == 0)
                    std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        });

    // 消费者线程
    std::thread consumer(
        [&]()
        {
            uint8_t data[64];

            for (int i = 0; i < 1000 && running; i++)
            {
                size_t readLen = rb.read(data, sizeof(data), 10);
                totalRead += readLen;
                if (readLen == 0)
                    std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        });

    producer.join();
    running = false;
    consumer.join();

    auto endTime  = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "[生产者-消费者] 耗时: " << duration.count() << " ms" << std::endl;
    std::cout << "  写入: " << totalWritten.load() << " 字节" << std::endl;
    std::cout << "  读取: " << totalRead.load() << " 字节" << std::endl;

    // 双缓冲多线程测试
    DoubleBufferBasic db(1024);
    std::atomic<int>  writeCount{0};
    std::atomic<int>  readCount{0};

    std::thread writer(
        [&]()
        {
            uint8_t data[512];
            std::memset(data, 0xBB, sizeof(data));

            for (int i = 0; i < 100; i++)
            {
                if (db.write(data, sizeof(data)))
                    writeCount++;
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }
        });

    std::thread reader(
        [&]()
        {
            uint8_t data[512];

            for (int i = 0; i < 100; i++)
            {
                if (db.read(data, sizeof(data)) > 0)
                    readCount++;
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }
        });

    writer.join();
    reader.join();

    std::cout << "[双缓冲多线程] 写入: " << writeCount.load() << " 次, 读取: " << readCount.load()
              << " 次" << std::endl;

    std::cout << "[完成] 多线程测试通过 ✓" << std::endl;
}

// ===================== 主函数 =====================
int main()
{
    std::cout << "╔════════════════════════════════════════╗" << std::endl;
    std::cout << "║     内存工具模块综合测试 (Linux)        ║" << std::endl;
    std::cout << "╚════════════════════════════════════════╝" << std::endl;

    try
    {
        testMemoryPool();
        testDoubleBuffer();
        testRingBuffer();
        testMultiThread();

        std::cout << "\n╔════════════════════════════════════════╗" << std::endl;
        std::cout << "║         所有测试完成! ✓                 ║" << std::endl;
        std::cout << "╚════════════════════════════════════════╝" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\n[异常] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
