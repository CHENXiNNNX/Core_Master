/**
 * @file    base.hpp
 * @brief   UART模块基类
 */

#pragma once

#include "../message/dispatcher.hpp"
#include "../message/message.hpp"

#include <functional>
#include <map>
#include <string>

namespace app::common::uart::module
{

                using Message    = message::Message;
                using Dispatcher = message::Dispatcher;

                /*============================================================================
                 * 命令处理函数类型
                 *============================================================================*/

                using CmdHandler = std::function<void(const Message& msg)>;

                /*============================================================================
                 * 模块基类
                 *============================================================================*/

                class ModuleBase : public message::IHandler
                {
                public:
                    virtual ~ModuleBase() = default;

                    /* 发送消息 */
                    void send(const Message& msg)
                    {
                        Dispatcher::inst().send(msg);
                    }

                    /* 发送成功应答 */
                    void reply_ok(const Message& req, const std::string& data = "")
                    {
                        if (req.need_reply())
                            send(req.reply_ok(data));
                    }

                    /* 发送错误应答 */
                    void reply_err(const Message& req, const std::string& error)
                    {
                        if (req.need_reply())
                            send(req.reply_err(error));
                    }

                    /* 构建请求消息 */
                    Message make_req(const std::string& cmd)
                    {
                        return Message(name(), cmd, Dispatcher::inst().next_id());
                    }

                    /* 构建通知消息（无需应答） */
                    Message make_notify(const std::string& cmd)
                    {
                        return Message(name(), cmd, 0);
                    }

                protected:
                    /* 注册命令处理器 */
                    void reg_cmd(const std::string& cmd, CmdHandler handler)
                    {
                        cmd_handlers_[cmd] = handler;
                    }

                    /* IHandler 实现 */
                    void handle(const Message& msg) override
                    {
                        auto it = cmd_handlers_.find(msg.cmd);
                        if (it != cmd_handlers_.end())
                        {
                            it->second(msg);
                        }
                        else
                        {
                            on_unknown_cmd(msg);
                        }
                    }

                    /* 未知命令处理（可重写） */
                    virtual void on_unknown_cmd(const Message& msg)
                    {
                        reply_err(msg, "unknown cmd");
                    }

                private:
                    std::map<std::string, CmdHandler> cmd_handlers_;
        };

} // namespace app::common::uart::module
