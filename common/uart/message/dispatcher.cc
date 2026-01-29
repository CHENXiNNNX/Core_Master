/**
 * @file    dispatcher.cc
 * @brief   UART消息分发器实现
 */

#include "dispatcher.hpp"
#include "tool/time/time.hpp"
#include "tool/log/log.hpp"

static const char* const TAG = "MsgDisp";

namespace app::common::uart::message
{

                Dispatcher& Dispatcher::inst()
                {
                    static Dispatcher s;
                    return s;
                }

                Dispatcher::Dispatcher() {}

                Dispatcher::~Dispatcher()
                {
                    deinit();
                }

                bool Dispatcher::init()
                {
                    if (init_)
                        return true;

                    auto& drv = UartDrv::inst();
                    drv.set_cb([this](FrameType type, const uint8_t* data, size_t len)
                               { on_uart_rx(type, data, len); });

                    init_ = true;
                    LOG_INFO(TAG, "初始化完成");
                    return true;
                }

                void Dispatcher::deinit()
                {
                    if (!init_)
                        return;

                    auto& drv = UartDrv::inst();
                    drv.set_cb(nullptr);

                    std::lock_guard<std::mutex> lk(mtx_);
                    pending_.clear();
                    init_ = false;
                }

                void Dispatcher::reg(IHandler* handler)
                {
                    if (!handler)
                        return;

                    std::lock_guard<std::mutex> lk(mtx_);
                    handlers_[handler->name()] = handler;
                    LOG_INFO(TAG, "注册模块: %s", handler->name());
                }

                bool Dispatcher::send(const Message& msg)
                {
                    if (!init_)
                        return false;

                    std::string json = msg.to_json();
                    LOG_INFO(TAG, ">>> TX: %s", json.c_str());

                    auto& drv = UartDrv::inst();
                    return drv.send(reinterpret_cast<const uint8_t*>(json.c_str()), json.size());
                }

                bool Dispatcher::request(const Message& msg, ReplyCb cb, uint32_t timeout_ms)
                {
                    if (!init_ || !cb)
                        return false;

                    Message req = msg;
                    req.id      = next_id();

                    {
                        std::lock_guard<std::mutex> lk(mtx_);
                        PendingReq                  pr;
                        pr.cb            = cb;
                        pr.expire_ms     = tool::time::uptime_ms() + timeout_ms;
                        pending_[req.id] = pr;
                    }

                    if (!send(req))
                    {
                        std::lock_guard<std::mutex> lk(mtx_);
                        pending_.erase(req.id);
                        return false;
                    }

                    return true;
                }

                int32_t Dispatcher::next_id()
                {
                    int32_t id = id_gen_.fetch_add(1);
                    if (id <= 0)
                    {
                        id_gen_ = 1;
                        id      = id_gen_.fetch_add(1);
                    }
                    return id;
                }

                void Dispatcher::on_uart_rx(FrameType type, const uint8_t* data, size_t len)
                {
                    if (type != FrameType::DATA || !data || len == 0)
                        return;

                    std::string json(reinterpret_cast<const char*>(data), len);
                    dispatch(json);
                }

                void Dispatcher::dispatch(const std::string& json)
                {
                    LOG_INFO(TAG, "<<< RX: %s", json.c_str());

                    Message msg;
                    if (!Message::from_json(json, msg))
                    {
                        LOG_WARN(TAG, "JSON解析失败");
                        return;
                    }

                    // 检查是否为应答消息
                    if (msg.id != 0)
                    {
                        std::lock_guard<std::mutex> lk(mtx_);
                        auto                        it = pending_.find(msg.id);
                        if (it != pending_.end())
                        {
                            // 是我们发出请求的应答
                            ReplyCb cb = it->second.cb;
                            pending_.erase(it);
                            if (cb)
                                cb(msg);
                            return;
                        }
                    }

                    // 分发给对应模块
                    IHandler* handler = nullptr;
                    {
                        std::lock_guard<std::mutex> lk(mtx_);
                        auto                        it = handlers_.find(msg.mod);
                        if (it != handlers_.end())
                            handler = it->second;
                    }

                    if (handler)
                    {
                        handler->handle(msg);
                    }
                    else
                    {
                        LOG_WARN(TAG, "未知模块: %s", msg.mod.c_str());
                        if (msg.need_reply())
                        {
                            send(msg.reply_err("unknown module"));
                        }
                    }

                    // 清理超时请求
                    check_timeout();
                }

                void Dispatcher::check_timeout()
                {
                    uint64_t                                 now = tool::time::uptime_ms();
                    std::vector<std::pair<int32_t, ReplyCb>> expired;

                    {
                        std::lock_guard<std::mutex> lk(mtx_);
                        for (auto it = pending_.begin(); it != pending_.end();)
                        {
                            if (now >= it->second.expire_ms)
                            {
                                expired.emplace_back(it->first, it->second.cb);
                                it = pending_.erase(it);
                            }
                            else
                            {
                                ++it;
                            }
                        }
                    }

                    for (auto& [id, cb] : expired)
                    {
                        LOG_WARN(TAG, "请求超时: id=%d", id);
                        if (cb)
                        {
                            Message timeout_msg;
                            timeout_msg.id  = id;
                            timeout_msg.ok  = false;
                            timeout_msg.err = "timeout";
                            cb(timeout_msg);
                        }
                    }
        }

} // namespace app::common::uart::message
