/**
 * @file main.cpp
 * @brief 时间模块测试
 */

#include <iostream>
#include <thread>
#include <chrono>
#include "tool/time/time.hpp"

int main()
{
    std::cout << "========== 时间模块测试 ==========" << std::endl;

    // 测试 Unix 时间戳
    std::cout << "\n[Unix 时间戳]" << std::endl;
    std::cout << "  秒:   " << app::tool::time::unix_timestamp_sec() << std::endl;
    std::cout << "  毫秒: " << app::tool::time::unix_timestamp_ms() << std::endl;
    std::cout << "  微秒: " << app::tool::time::unix_timestamp_us() << std::endl;

    // 测试系统启动时间
    std::cout << "\n[系统启动时间 (Uptime)]" << std::endl;
    std::cout << "  秒:   " << app::tool::time::uptime_sec() << std::endl;
    std::cout << "  毫秒: " << app::tool::time::uptime_ms() << std::endl;
    std::cout << "  微秒: " << app::tool::time::uptime_us() << std::endl;

    // 测试时间字符串
    std::cout << "\n[时间字符串]" << std::endl;
    std::cout << "  ISO 8601 (UTC): " << app::tool::time::iso8601Timestamp() << std::endl;
    std::cout << "  本地时间:       " << app::tool::time::local_time_string() << std::endl;

    // 测试时间精度
    std::cout << "\n[时间精度测试 - 休眠 100ms]" << std::endl;
    int64_t start_ms = app::tool::time::uptime_ms();
    int64_t start_us = app::tool::time::uptime_us();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int64_t end_ms = app::tool::time::uptime_ms();
    int64_t end_us = app::tool::time::uptime_us();

    std::cout << "  毫秒差值: " << (end_ms - start_ms) << " ms" << std::endl;
    std::cout << "  微秒差值: " << (end_us - start_us) << " us" << std::endl;

    std::cout << "\n========== 测试完成 ==========" << std::endl;

    return 0;
}
