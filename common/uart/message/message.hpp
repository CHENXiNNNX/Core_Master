/**
 * @file    message.hpp
 * @brief   UART消息结构定义
 */

#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace app::common::uart::message
{

                /*============================================================================
                 * 模块名称常量
                 *============================================================================*/

                namespace ModName
                {
                    constexpr const char* NETWORK = "net";
                    constexpr const char* SYSTEM  = "sys";
                    constexpr const char* SENSOR  = "sensor";
                    constexpr const char* CONFIG  = "cfg";
                } // namespace ModName

                /*============================================================================
                 * 消息结构
                 *============================================================================*/

                struct Message
                {
                    std::string mod;       // 模块: net/sys/sensor/cfg
                    std::string cmd;       // 命令
                    int32_t     id = 0;    // 请求ID（0=无需应答）
                    bool        ok = true; // 应答状态
                    std::string err;       // 错误信息
                    std::string data;      // 业务数据（JSON子对象字符串）

                    Message() = default;

                    Message(const std::string& m, const std::string& c, int32_t i = 0)
                        : mod(m)
                        , cmd(c)
                        , id(i)
                    {
                    }

                    /* 序列化为JSON */
                    std::string to_json() const;

                    /* 从JSON反序列化 */
                    static bool from_json(const std::string& json, Message& msg);

                    /* 构建成功应答 */
                    Message reply_ok(const std::string& resp_data = "") const;

                    /* 构建错误应答 */
                    Message reply_err(const std::string& error) const;

                    /* 是否需要应答 */
                    bool need_reply() const
                    {
                        return id != 0;
                    }

                    /* 是否为应答消息 */
                    bool is_reply() const
                    {
                        return !err.empty() || (id != 0 && !ok);
                    }
                };

                /*============================================================================
                 * JSON 工具函数
                 *============================================================================*/

                /* 从JSON提取字符串字段 */
                bool json_get_str(const std::string& json, const char* key, std::string& out);

                /* 从JSON提取整数字段 */
                bool json_get_int(const std::string& json, const char* key, int32_t& out);

                /* 从JSON提取布尔字段 */
                bool json_get_bool(const std::string& json, const char* key, bool& out);

                /* 从JSON提取对象字段（返回原始JSON子串） */
                bool json_get_obj(const std::string& json, const char* key, std::string& out);

                /* 转义JSON字符串 */
        std::string json_escape(const std::string& s);

} // namespace app::common::uart::message
