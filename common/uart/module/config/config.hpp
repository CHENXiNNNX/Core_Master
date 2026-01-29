/**
 * @file    config.hpp
 * @brief   配置模块 - 参数读写
 */

#pragma once

#include "../base.hpp"
#include "../../message/message.hpp"

#include <functional>
#include <string>

namespace app::common::uart::module
{

                /*============================================================================
                 * 配置模块命令（待定义）
                 *============================================================================*/

                namespace CfgCmd
                {
                    // 预留
                } // namespace CfgCmd

                /*============================================================================
                 * 配置模块
                 *============================================================================*/

                class ConfigModule : public ModuleBase
                {
                public:
                    static ConfigModule& inst();

                    const char* name() const override
                    {
                        return message::ModName::CONFIG;
                    }

                    void init();

                private:
                    ConfigModule();
                    ~ConfigModule()                              = default;
                    ConfigModule(const ConfigModule&)            = delete;
                    ConfigModule& operator=(const ConfigModule&) = delete;

                    void init_handlers();
        };

} // namespace app::common::uart::module
