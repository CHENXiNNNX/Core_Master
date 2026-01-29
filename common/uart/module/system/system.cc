/**
 * @file    system.cc
 * @brief   系统模块实现
 */

#include "system.hpp"
#include "tool/time/time.hpp"
#include "tool/log/log.hpp"

static const char* const TAG = "SysMod";

namespace app::common::uart::module
{

                SystemModule& SystemModule::inst()
                {
                    static SystemModule s;
                    return s;
                }

                SystemModule::SystemModule()
                {
                    init_handlers();
                }

                void SystemModule::init()
                {
                    Dispatcher::inst().reg(this);
                    LOG_INFO(TAG, "初始化");
                }

                void SystemModule::init_handlers()
                {
                    // 处理从机主动上报（如需要）
                }

                /*============================================================================
                 * 系统操作
                 *============================================================================*/

                void SystemModule::reboot()
                {
                    Message msg = make_notify(SysCmd::REBOOT);
                    send(msg);
                }

                void SystemModule::sleep()
                {
                    Message msg = make_notify(SysCmd::SLEEP);
                    send(msg);
                }

                void SystemModule::wakeup()
                {
                    Message msg = make_notify(SysCmd::WAKEUP);
                    send(msg);
                }

                void SystemModule::get_info(SysInfoCb cb)
                {
                    Message req = make_req(SysCmd::INFO);

                    Dispatcher::inst().request(req,
                                               [cb](const Message& reply)
                                               {
                                                   if (reply.ok)
                                                   {
                                                       auto info = parse_info(reply.data);
                                                       if (cb)
                                                           cb(true, info);
                                                   }
                                                   else
                                                   {
                                                       if (cb)
                                                           cb(false, {});
                                                   }
                                               });
                }

                void SystemModule::get_version(VersionCb cb)
                {
                    Message req = make_req(SysCmd::VERSION);

                    Dispatcher::inst().request(req,
                                               [cb](const Message& reply)
                                               {
                                                   if (reply.ok)
                                                   {
                                                       auto ver = parse_version(reply.data);
                                                       if (cb)
                                                           cb(true, ver);
                                                   }
                                                   else
                                                   {
                                                       if (cb)
                                                           cb(false, {});
                                                   }
                                               });
                }

                void SystemModule::ping(PingCb cb)
                {
                    uint64_t start = tool::time::uptime_ms();
                    Message  req   = make_req(SysCmd::PING);

                    Dispatcher::inst().request(
                        req,
                        [cb, start](const Message& reply)
                        {
                            if (reply.ok)
                            {
                                uint32_t latency =
                                    static_cast<uint32_t>(tool::time::uptime_ms() - start);
                                if (cb)
                                    cb(true, latency);
                            }
                            else
                            {
                                if (cb)
                                    cb(false, 0);
                            }
                        },
                        3000);
                }

                /*============================================================================
                 * 解析函数
                 *============================================================================*/

                SystemInfo SystemModule::parse_info(const std::string& data)
                {
                    SystemInfo info;
                    if (data.empty())
                        return info;

                    message::json_get_str(data, "chip", info.chip);

                    int32_t val = 0;
                    if (message::json_get_int(data, "heap_free", val))
                        info.heap_free = static_cast<uint32_t>(val);
                    if (message::json_get_int(data, "heap_total", val))
                        info.heap_total = static_cast<uint32_t>(val);
                    if (message::json_get_int(data, "uptime", val))
                        info.uptime_sec = static_cast<uint32_t>(val);

                    return info;
                }

                VersionInfo SystemModule::parse_version(const std::string& data)
                {
                    VersionInfo ver;
                    if (data.empty())
                        return ver;

                    message::json_get_str(data, "fw", ver.fw_version);
                    message::json_get_str(data, "hw", ver.hw_version);
                    message::json_get_str(data, "build", ver.build_date);

                    return ver;
        }

} // namespace app::common::uart::module
