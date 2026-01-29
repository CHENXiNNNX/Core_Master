/**
 * @file    system.hpp
 * @brief   系统模块 - 重启/休眠/信息等
 */

#pragma once

#include "../base.hpp"
#include "../../message/message.hpp"

#include <functional>
#include <string>

namespace app::common::uart::module
{

                /*============================================================================
                 * 系统模块命令
                 *============================================================================*/

                namespace SysCmd
                {
                    constexpr const char* REBOOT  = "reboot";
                    constexpr const char* SLEEP   = "sleep";
                    constexpr const char* WAKEUP  = "wakeup";
                    constexpr const char* INFO    = "info";
                    constexpr const char* VERSION = "version";
                    constexpr const char* PING    = "ping";
                } // namespace SysCmd

                /*============================================================================
                 * 数据结构
                 *============================================================================*/

                struct SystemInfo
                {
                    std::string chip;
                    uint32_t    heap_free  = 0;
                    uint32_t    heap_total = 0;
                    uint32_t    uptime_sec = 0;
                };

                struct VersionInfo
                {
                    std::string fw_version;
                    std::string hw_version;
                    std::string build_date;
                };

                /*============================================================================
                 * 回调类型
                 *============================================================================*/

                using SysInfoCb = std::function<void(bool ok, const SystemInfo& info)>;
                using VersionCb = std::function<void(bool ok, const VersionInfo& ver)>;
                using PingCb    = std::function<void(bool ok, uint32_t latency_ms)>;

                /*============================================================================
                 * 系统模块
                 *============================================================================*/

                class SystemModule : public ModuleBase
                {
                public:
                    static SystemModule& inst();

                    const char* name() const override
                    {
                        return message::ModName::SYSTEM;
                    }

                    /* 初始化 */
                    void init();

                    /*--------------------------------------------------------------------
                     * 系统操作
                     *--------------------------------------------------------------------*/

                    /* 重启从机 */
                    void reboot();

                    /* 进入休眠 */
                    void sleep();

                    /* 唤醒 */
                    void wakeup();

                    /* 获取系统信息 */
                    void get_info(SysInfoCb cb);

                    /* 获取版本信息 */
                    void get_version(VersionCb cb);

                    /* Ping 测试连通性 */
                    void ping(PingCb cb);

                private:
                    SystemModule();
                    ~SystemModule()                              = default;
                    SystemModule(const SystemModule&)            = delete;
                    SystemModule& operator=(const SystemModule&) = delete;

                    void init_handlers();

                    static SystemInfo  parse_info(const std::string& data);
                    static VersionInfo parse_version(const std::string& data);
        };

} // namespace app::common::uart::module
