/**
 * @file    sensor.hpp
 * @brief   传感器模块 - 传感器数据读取/上报
 */

#pragma once

#include "../base.hpp"
#include "../../message/message.hpp"

#include <functional>
#include <string>

namespace app::common::uart::module
{

                /*============================================================================
                 * 传感器模块命令（待定义）
                 *============================================================================*/

                namespace SensorCmd
                {
                    // 预留
                } // namespace SensorCmd

                /*============================================================================
                 * 传感器模块
                 *============================================================================*/

                class SensorModule : public ModuleBase
                {
                public:
                    static SensorModule& inst();

                    const char* name() const override
                    {
                        return message::ModName::SENSOR;
                    }

                    void init();

                private:
                    SensorModule();
                    ~SensorModule()                              = default;
                    SensorModule(const SensorModule&)            = delete;
                    SensorModule& operator=(const SensorModule&) = delete;

                    void init_handlers();
        };

} // namespace app::common::uart::module
