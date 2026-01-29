/**
 * @file time.hpp
 * @brief 时间工具模块
 */

#pragma once

#include <cstdint>
#include <string>

namespace app::tool::time
{

    /**
     * @brief 获取 Unix 时间戳（秒）
     * @return Unix 时间戳，从 1970-01-01 00:00:00 UTC 开始的秒数
     * @note 需要先通过 NTP 同步系统时间，否则返回的是未同步的时间
     */
    int64_t unix_timestamp_sec();

    /**
     * @brief 获取 Unix 时间戳（毫秒）
     * @return Unix 时间戳，从 1970-01-01 00:00:00 UTC 开始的毫秒数
     * @note 需要先通过 NTP 同步系统时间，否则返回的是未同步的时间
     */
    int64_t unix_timestamp_ms();

    /**
     * @brief 获取 Unix 时间戳（微秒）
     * @return Unix 时间戳，从 1970-01-01 00:00:00 UTC 开始的微秒数
     * @note 需要先通过 NTP 同步系统时间，否则返回的是未同步的时间
     */
    int64_t unix_timestamp_us();

    /**
     * @brief 获取系统启动时间（毫秒）
     * @return 从系统启动开始的毫秒数
     * @note 系统重启后会重置为 0，适合用于相对时间测量
     */
    int64_t uptime_ms();

    /**
     * @brief 获取系统启动时间（微秒）
     * @return 从系统启动开始的微秒数
     * @note 系统重启后会重置为 0，适合用于高精度相对时间测量
     */
    int64_t uptime_us();

    /**
     * @brief 获取系统启动时间（秒）
     * @return 从系统启动开始的秒数
     * @note 系统重启后会重置为 0，适合用于相对时间测量
     */
    int64_t uptime_sec();

    /**
     * @brief 获取 ISO 8601 格式的时间戳字符串（UTC）
     * @return ISO 8601 格式的时间戳，如 "2025-03-12T19:00:00Z"
     * @note 需要先通过 NTP 同步系统时间，否则返回的是未同步的时间
     */
    std::string iso8601_timestamp();

    /**
     * @brief 获取本地时间字符串
     * @return 本地时间字符串，如 "2025-03-12 19:00:00"
     */
    std::string local_time_string();

} // namespace app::tool::time
