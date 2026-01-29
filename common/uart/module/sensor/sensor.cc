/**
 * @file    sensor.cc
 * @brief   传感器模块实现
 */

#include "sensor.hpp"
#include "tool/log/log.hpp"

static const char* const TAG = "SensorMod";

namespace app::common::uart::module
{

                SensorModule& SensorModule::inst()
                {
                    static SensorModule s;
                    return s;
                }

                SensorModule::SensorModule()
                {
                    init_handlers();
                }

                void SensorModule::init()
                {
                    Dispatcher::inst().reg(this);
                    LOG_INFO(TAG, "初始化");
                }

                void SensorModule::init_handlers()
                {
                    // 待实现
        }

} // namespace app::common::uart::module
