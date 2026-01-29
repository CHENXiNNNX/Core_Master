/**
 * @file    dispatcher.hpp
 * @brief   UART消息分发器
 */

#pragma once

#include "message.hpp"
#include "../uart.hpp"

#include <functional>
#include <map>
#include <mutex>
#include <string>

namespace app::common::uart::message
{

                /*============================================================================
                 * 模块处理器接口
                 *============================================================================*/

                class IHandler
                {
                public:
                    virtual ~IHandler() = default;

                    /* 模块名称 */
                    virtual const char* name() const = 0;

                    /* 处理消息 */
                    virtual void handle(const Message& msg) = 0;
                };

                /*============================================================================
                 * 请求回调
                 *============================================================================*/

                using ReplyCb = std::function<void(const Message& reply)>;

                /*============================================================================
                 * 分发器
                 *============================================================================*/

                class Dispatcher
                {
                public:
                    static Dispatcher& inst();

                    /* 初始化/反初始化 */
                    bool init();
                    void deinit();

                    /* 注册模块处理器 */
                    void reg(IHandler* handler);

                    /* 发送消息（无需应答） */
                    bool send(const Message& msg);

                    /* 发送请求（需要应答） */
                    bool request(const Message& msg, ReplyCb cb, uint32_t timeout_ms = 5000);

                    /* 手动分发（内部使用） */
                    void dispatch(const std::string& json);

                    /* 生成请求ID */
                    int32_t next_id();

                private:
                    Dispatcher();
                    ~Dispatcher();
                    Dispatcher(const Dispatcher&)            = delete;
                    Dispatcher& operator=(const Dispatcher&) = delete;

                    void on_uart_rx(FrameType type, const uint8_t* data, size_t len);
                    void handle_reply(const Message& msg);
                    void check_timeout();

                    struct PendingReq
                    {
                        ReplyCb  cb;
                        uint64_t expire_ms;
                    };

                    std::map<std::string, IHandler*> handlers_;
                    std::map<int32_t, PendingReq>    pending_;
                    std::mutex                       mtx_;
                    std::atomic<int32_t>             id_gen_{1};
                    bool                             init_ = false;
        };

} // namespace app::common::uart::message
