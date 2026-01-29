/**
 * @file main.cpp
 * @brief SPI 主机侧功能测试入口
 *
 * 职责：
 * - 建立与从机的 SPI 通道
 * - 发送小包/大包测试数据
 * - 输出收发与错误统计
 */

#include "common/spi/spi_master.hpp"
#include "tool/log/log.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace app::common::spi;

namespace
{
    const char* TAG = "TEST";

    std::atomic<bool>     g_running{true};
    std::atomic<uint32_t> g_tx_count{0};
    std::atomic<uint32_t> g_rx_count{0};
    std::atomic<uint32_t> g_tx_success{0};
    std::atomic<uint32_t> g_tx_failed{0};
    std::atomic<uint32_t> g_large_rx_count{0};
    std::atomic<uint64_t> g_total_tx_bytes{0};
    std::atomic<uint64_t> g_total_rx_bytes{0};

    // 测试模式配置
    enum class TestMode
    {
        NORMAL,     // 小包发送
        LARGE,      // 大包分片发送
        MIXED,      // 大包/小包交替
        STRESS,     // 小包高压测试
        PERFORMANCE // 性能统计优先
    };

    TestMode             g_test_mode        = TestMode::NORMAL;
    size_t               g_large_data_size  = 0;
    size_t               g_small_data_size  = 128;
    uint32_t             g_send_interval_ms = 500;
    std::vector<uint8_t> g_large_tx_buffer;
    std::vector<uint8_t> g_small_tx_buffer;

    void signalHandler(int)
    {
        g_running = false;
        LOG_INFO(TAG, "收到退出信号，准备退出");
    }

    std::string formatHex(const uint8_t* data, size_t len, size_t max_show = 16)
    {
        if (!data || len == 0)
            return "";

        std::string result;
        result.reserve(max_show * 3 + 16);

        size_t show = std::min(len, max_show);
        for (size_t i = 0; i < show; i++)
        {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02x ", data[i]);
            result += buf;
        }

        if (len > show)
            result += "...(+" + std::to_string(len - show) + "B)";

        return result;
    }

    const char* getTypeName(FrameType type)
    {
        switch (type)
        {
        case FrameType::SYS_NOP:
            return "NOP";
        case FrameType::SYS_HEARTBEAT:
            return "HB";
        case FrameType::SYS_ACK:
            return "ACK";
        case FrameType::SYS_NACK:
            return "NACK";
        case FrameType::DATA_SINGLE:
            return "DATA";
        case FrameType::DATA_FIRST:
            return "DATA_F";
        case FrameType::DATA_MIDDLE:
            return "DATA_M";
        case FrameType::DATA_LAST:
            return "DATA_L";
        default:
            return "?";
        }
    }

    // 从机数据接收回调
    void onDataReceived(FrameType type, const uint8_t* data, size_t len)
    {
        g_rx_count.fetch_add(1);
        g_total_rx_bytes.fetch_add(len);

        if (len > 256)
        {
            // 大数据（分片重组后）
            g_large_rx_count.fetch_add(1);

            if (g_test_mode == TestMode::PERFORMANCE || g_test_mode == TestMode::STRESS)
            {
                // 性能/压力模式：降低日志频率
                if (g_rx_count.load() % 100 == 0)
                    LOG_INFO(TAG, "RX: #%u, %zuB (重组)", g_rx_count.load(), len);
            }
            else
            {
                LOG_INFO(TAG, "RX: #%u, %zuB (重组)", g_rx_count.load(), len);

                if (len >= 16)
                {
                    LOG_INFO(TAG, "  数据头: %s", formatHex(data, 16).c_str());

                    // 校验数据完整性
                    if (len >= 64)
                    {
                        bool ok = true;
                        for (size_t i = 8; i < 64 && ok; i++)
                        {
                            if (data[i] != (uint8_t)(i))
                            {
                                LOG_WARN(TAG, "数据错误: [%zu]=%02x (期望%02x)", i, data[i],
                                         (uint8_t)i);
                                ok = false;
                            }
                        }
                        if (ok)
                            LOG_INFO(TAG, "  数据校验通过");
                    }
                }
            }
        }
        else
        {
            // 小数据
            if (g_test_mode != TestMode::PERFORMANCE && g_test_mode != TestMode::STRESS)
            {
                LOG_INFO(TAG, "RX: #%u, type=%s, %zuB", g_rx_count.load(), getTypeName(type), len);
                if (len > 0 && data)
                    LOG_INFO(TAG, "  %s", formatHex(data, len, 8).c_str());
            }
        }
    }

    void printStats(const SpiDrv& spi)
    {
        const auto&     s             = spi.stats();
        auto            now           = std::chrono::steady_clock::now();
        static auto     last_time     = now;
        static uint64_t last_tx_bytes = 0;
        static uint64_t last_rx_bytes = 0;

        auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
        uint64_t tx_bytes_diff = s.tx_bytes.load() - last_tx_bytes;
        uint64_t rx_bytes_diff = s.rx_bytes.load() - last_rx_bytes;

        double tx_rate = (elapsed > 0) ? (tx_bytes_diff * 1000.0 / elapsed / 1024.0) : 0.0;
        double rx_rate = (elapsed > 0) ? (rx_bytes_diff * 1000.0 / elapsed / 1024.0) : 0.0;

        last_time     = now;
        last_tx_bytes = s.tx_bytes.load();
        last_rx_bytes = s.rx_bytes.load();

        LOG_INFO(TAG, "");
        LOG_INFO(TAG, "统计信息:");
        LOG_INFO(TAG, "  连接状态: %s", s.connected.load() ? "已连接" : "未连接");
        LOG_INFO(TAG, "  发送:");
        LOG_INFO(TAG, "  总帧数: %u", s.tx_frm.load());
        LOG_INFO(TAG, "    总字节: %llu", (unsigned long long)s.tx_bytes.load());
        LOG_INFO(TAG, "    待发送: %u", s.tx_pending.load());
        LOG_INFO(TAG, "    发送速率: %.2f KB/s", tx_rate);
        LOG_INFO(TAG, "    本地计数: %u (成功:%u, 失败:%u)", g_tx_count.load(), g_tx_success.load(),
                 g_tx_failed.load());
        LOG_INFO(TAG, "  接收:");
        LOG_INFO(TAG, "    总帧数: %u", s.rx_frm.load());
        LOG_INFO(TAG, "    总字节: %llu", (unsigned long long)s.rx_bytes.load());
        LOG_INFO(TAG, "    接收速率: %.2f KB/s", rx_rate);
        LOG_INFO(TAG, "    本地计数: %u (大包:%u)", g_rx_count.load(), g_large_rx_count.load());
        LOG_INFO(TAG, "  错误:");
        LOG_INFO(TAG, "    CRC错误: %u", s.crc_err.load());
        LOG_INFO(TAG, "    超时错误: %u", s.timeout.load());
        LOG_INFO(TAG, "    无效帧: %u", s.invalid.load());

        // 计算错误率
        uint32_t total_frames = s.tx_frm.load() + s.rx_frm.load();
        uint32_t total_errors = s.crc_err.load() + s.timeout.load() + s.invalid.load();
        if (total_frames > 0)
        {
            double error_rate = (total_errors * 100.0) / total_frames;
            LOG_INFO(TAG, "    错误率: %.2f%%", error_rate);
        }

        LOG_INFO(TAG, "  性能:");
        if (g_tx_count.load() > 0)
        {
            double success_rate = (g_tx_success.load() * 100.0) / g_tx_count.load();
            LOG_INFO(TAG, "  发送成功率: %.2f%%", success_rate);
        }

        LOG_INFO(TAG, "===============================");
        LOG_INFO(TAG, "");
    }

    void sendThread(SpiDrv& spi)
    {
        LOG_INFO(TAG, "发送线程启动 (模式: %d)", (int)g_test_mode);

        std::vector<uint8_t> buf;
        bool                 use_large  = false;
        uint32_t             send_count = 0;

        while (g_running)
        {
            if (!spi.connected())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            bool   ok        = false;
            size_t data_size = 0;

            switch (g_test_mode)
            {
            case TestMode::NORMAL:
                // 正常模式：仅发送小包
                data_size = g_small_data_size;
                buf.resize(data_size);
                {
                    uint32_t count = g_tx_count.load();
                    memcpy(buf.data(), &count, sizeof(count));
                    auto ts = std::chrono::steady_clock::now().time_since_epoch().count();
                    memcpy(buf.data() + 4, &ts, sizeof(ts));
                    for (size_t i = 8; i < data_size; i++)
                        buf[i] = static_cast<uint8_t>(i);
                }
                ok = spi.send(buf.data(), data_size);
                break;

            case TestMode::LARGE:
                // 大包模式：仅发送分片数据
                if (g_large_data_size > 0)
                {
                    data_size = g_large_data_size;
                    ok        = spi.send(g_large_tx_buffer.data(), g_large_data_size);
                }
                break;

            case TestMode::MIXED:
                // 混合模式：大包/小包交替
                use_large = !use_large;
                if (use_large && g_large_data_size > 0)
                {
                    data_size = g_large_data_size;
                    ok        = spi.send(g_large_tx_buffer.data(), g_large_data_size);
                }
                else
                {
                    data_size = g_small_data_size;
                    buf.resize(data_size);
                    uint32_t count = g_tx_count.load();
                    memcpy(buf.data(), &count, sizeof(count));
                    auto ts = std::chrono::steady_clock::now().time_since_epoch().count();
                    memcpy(buf.data() + 4, &ts, sizeof(ts));
                    for (size_t i = 8; i < data_size; i++)
                        buf[i] = static_cast<uint8_t>(i);
                    ok = spi.send(buf.data(), data_size);
                }
                break;

            case TestMode::STRESS:
            case TestMode::PERFORMANCE:
                // 压力/性能模式：高频小包发送
                data_size = g_small_data_size;
                buf.resize(data_size);
                {
                    uint32_t count = g_tx_count.load();
                    memcpy(buf.data(), &count, sizeof(count));
                    auto ts = std::chrono::steady_clock::now().time_since_epoch().count();
                    memcpy(buf.data() + 4, &ts, sizeof(ts));
                    for (size_t i = 8; i < data_size; i++)
                        buf[i] = static_cast<uint8_t>(i);
                }
                ok = spi.send(buf.data(), data_size);
                // 压力模式：进一步缩短发送间隔
                if (g_test_mode == TestMode::STRESS)
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                break;
            }

            if (ok)
            {
                g_tx_count.fetch_add(1);
                g_tx_success.fetch_add(1);
                g_total_tx_bytes.fetch_add(data_size);

                send_count++;
                if (g_test_mode == TestMode::PERFORMANCE || g_test_mode == TestMode::STRESS)
                {
                    // 性能/压力测试：每100次打印一次
                    if (send_count % 100 == 0)
                        LOG_INFO(TAG, "TX: #%u, %zuB", g_tx_count.load(), data_size);
                }
                else
                {
                    if (data_size > 256)
                        LOG_INFO(TAG, "TX: #%u, %zuB (大数据)", g_tx_count.load(), data_size);
                    else
                        LOG_INFO(TAG, "TX: #%u, %zuB", g_tx_count.load(), data_size);
                }
            }
            else
            {
                g_tx_failed.fetch_add(1);
                LOG_WARN(TAG, "TX失败: #%u", g_tx_count.load() + 1);
            }

            // 检查tx_pending计数（验证修复效果）
            if (send_count % 50 == 0)
            {
                uint32_t pending = spi.stats().tx_pending.load();
                if (pending > 100)
                {
                    LOG_WARN(TAG, "tx_pending较高: %u (可能缓冲区积压)", pending);
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(g_send_interval_ms));
        }

        LOG_INFO(TAG, "发送线程退出");
    }

    void statsThread(SpiDrv& spi)
    {
        LOG_INFO(TAG, "统计线程启动");

        while (g_running)
        {
            std::this_thread::sleep_for(std::chrono::seconds(10));
            printStats(spi);
        }

        LOG_INFO(TAG, "统计线程退出");
    }

    void printUsage(const char* prog)
    {
        printf("用法: %s [选项]\n", prog);
        printf("选项:\n");
        printf("  -d <device>      SPI 设备节点 (默认: /dev/spidev0.0)\n");
        printf("  -s <speed>       SPI 时钟(Hz) (默认: 10000000)\n");
        printf("  -r <gpio>        MASTER_REQ GPIO (默认: 58)\n");
        printf("  -y <gpio>        SLAVE_READY GPIO (默认: 59)\n");
        printf(
            "  -m <mode>        测试模式: normal/large/mixed/stress/performance (默认: normal)\n");
        printf("  -l <size>        大包大小(字节) (默认: 0=不发送)\n");
        printf("  -i <ms>          发送间隔(毫秒) (默认: 500)\n");
        printf("  -h               显示帮助\n");
        printf("\n");
        printf("测试模式:\n");
        printf("  normal      - 小包发送\n");
        printf("  large       - 大包分片发送\n");
        printf("  mixed       - 大包/小包交替\n");
        printf("  stress      - 小包高压测试\n");
        printf("  performance - 高速发送并关注统计\n");
    }

} // namespace

int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // 初始化日志系统
    app::tool::log::Logger::inst().init(app::tool::log::LogConfig());

    // 解析参数
    SpiCfg cfg;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "-d") == 0 && i + 1 < argc)
        {
            cfg.dev = argv[++i];
        }
        else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc)
        {
            cfg.speed = (uint32_t)atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc)
        {
            cfg.gpio_req = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "-y") == 0 && i + 1 < argc)
        {
            cfg.gpio_rdy = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "-m") == 0 && i + 1 < argc)
        {
            std::string mode = argv[++i];
            if (mode == "normal")
                g_test_mode = TestMode::NORMAL;
            else if (mode == "large")
                g_test_mode = TestMode::LARGE;
            else if (mode == "mixed")
                g_test_mode = TestMode::MIXED;
            else if (mode == "stress")
                g_test_mode = TestMode::STRESS;
            else if (mode == "performance")
                g_test_mode = TestMode::PERFORMANCE;
            else
            {
                LOG_ERROR(TAG, "未知测试模式: %s", mode.c_str());
                return 1;
            }
        }
        else if (strcmp(argv[i], "-l") == 0 && i + 1 < argc)
        {
            g_large_data_size = (size_t)atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "-i") == 0 && i + 1 < argc)
        {
            g_send_interval_ms = (uint32_t)atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "-h") == 0)
        {
            printUsage(argv[0]);
            return 0;
        }
    }

    // 根据测试模式调整参数
    if (g_test_mode == TestMode::STRESS || g_test_mode == TestMode::PERFORMANCE)
    {
        g_send_interval_ms = 10; // 压力/性能测试：快速发送
    }

    // 准备大数据缓冲区
    if (g_large_data_size > 0)
    {
        if (g_large_data_size > MAX_DATA_SIZE)
        {
            LOG_ERROR(TAG, "大数据尺寸过大: %zu > %zu", g_large_data_size, MAX_DATA_SIZE);
            return 1;
        }

        g_large_tx_buffer.resize(g_large_data_size);
        uint32_t count = 0;
        memcpy(g_large_tx_buffer.data(), &count, sizeof(count));
        auto ts = std::chrono::steady_clock::now().time_since_epoch().count();
        memcpy(g_large_tx_buffer.data() + 4, &ts, sizeof(ts));

        for (size_t i = 8; i < g_large_data_size; i++)
            g_large_tx_buffer[i] = static_cast<uint8_t>(i);

        LOG_INFO(TAG, "大数据缓冲区准备完成: %zuB", g_large_data_size);
    }

    LOG_INFO(TAG, "SPI 主机启动");
    LOG_INFO(TAG, "设备: %s", cfg.dev.c_str());
    LOG_INFO(TAG, "速度: %u MHz", cfg.speed / 1000000);
    LOG_INFO(TAG, "MASTER_REQ GPIO: %d", cfg.gpio_req);
    LOG_INFO(TAG, "SLAVE_READY GPIO: %d", cfg.gpio_rdy);
    LOG_INFO(TAG, "测试模式: %d", (int)g_test_mode);
    if (g_large_data_size > 0)
        LOG_INFO(TAG, "大包大小: %zuB", g_large_data_size);
    LOG_INFO(TAG, "发送间隔: %u ms", g_send_interval_ms);

    // 初始化SPI
    auto& spi = SpiDrv::inst();

    if (!spi.init(cfg))
    {
        LOG_ERROR(TAG, "SPI初始化失败");
        return 1;
    }

    // 注册回调
    spi.set_cb(onDataReceived);

    // 启动
    if (!spi.start())
    {
        LOG_ERROR(TAG, "SPI启动失败");
        spi.deinit();
        return 1;
    }

    LOG_INFO(TAG, "开始发送，按 Ctrl+C 终止");
    LOG_INFO(TAG, "统计周期 10 秒");

    // 启动工作线程
    std::thread send_thread(sendThread, std::ref(spi));
    std::thread stats_thread(statsThread, std::ref(spi));

    // 等待退出信号
    while (g_running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    LOG_INFO(TAG, "退出中...");

    // 等待线程结束
    if (send_thread.joinable())
        send_thread.join();
    if (stats_thread.joinable())
        stats_thread.join();

    // 打印最终统计
    LOG_INFO(TAG, "最终统计:");
    printStats(spi);
    LOG_INFO(TAG, "  发送: %u次 (成功:%u, 失败:%u)", g_tx_count.load(), g_tx_success.load(),
             g_tx_failed.load());
    LOG_INFO(TAG, "  接收: %u次 (大包:%u)", g_rx_count.load(), g_large_rx_count.load());
    LOG_INFO(TAG, "  发送字节: %llu", (unsigned long long)g_total_tx_bytes.load());
    LOG_INFO(TAG, "  接收字节: %llu", (unsigned long long)g_total_rx_bytes.load());

    // 清理
    spi.stop();
    spi.deinit();

    LOG_INFO(TAG, "程序结束");
    return 0;
}
