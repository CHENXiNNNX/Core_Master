/**
 * @file    config.cc
 * @brief   配置模块实现
 */

#include "config.hpp"
#include "tool/log/log.hpp"

static const char* const TAG = "CfgMod";

namespace app::common::uart::module
{

                ConfigModule& ConfigModule::inst()
                {
                    static ConfigModule s;
                    return s;
                }

                ConfigModule::ConfigModule()
                {
                    init_handlers();
                }

                void ConfigModule::init()
                {
                    Dispatcher::inst().reg(this);
                    LOG_INFO(TAG, "初始化");
                }

                void ConfigModule::init_handlers()
                {
                    // 待实现
        }

} // namespace app::common::uart::module
