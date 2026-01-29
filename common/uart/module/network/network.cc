/**
 * @file    network.cc
 * @brief   网络模块实现
 */

#include "network.hpp"
#include "tool/log/log.hpp"

static const char* const TAG = "NetMod";

namespace app::common::uart::module
{

                NetworkModule& NetworkModule::inst()
                {
                    static NetworkModule s;
                    return s;
                }

                NetworkModule::NetworkModule()
                {
                    init_handlers();
                }

                void NetworkModule::init()
                {
                    Dispatcher::inst().reg(this);
                    LOG_INFO(TAG, "初始化");
                }

                void NetworkModule::init_handlers()
                {
                    // 主机端主要发送请求，如需处理从机主动上报可在此注册
                }

                /*============================================================================
                 * WiFi 操作
                 *============================================================================*/

                void NetworkModule::wifi_scan(WifiScanCb cb)
                {
                    Message req = make_req(NetCmd::WIFI_SCAN);

                    Dispatcher::inst().request(
                        req,
                        [cb](const Message& reply)
                        {
                            if (reply.ok)
                            {
                                auto aps = parse_scan_result(reply.data);
                                if (cb)
                                    cb(true, aps);
                            }
                            else
                            {
                                LOG_ERROR(TAG, "WiFi扫描失败: %s", reply.err.c_str());
                                if (cb)
                                    cb(false, {});
                            }
                        },
                        15000);
                }

                void NetworkModule::wifi_connect(const std::string& ssid, const std::string& pwd,
                                                 WifiConnCb cb)
                {
                    Message req = make_req(NetCmd::WIFI_CONNECT);
                    req.data    = "{\"ssid\":\"" + message::json_escape(ssid) + "\",\"pwd\":\"" +
                               message::json_escape(pwd) + "\"}";

                    Dispatcher::inst().request(
                        req,
                        [cb](const Message& reply)
                        {
                            if (reply.ok)
                            {
                                std::string ip;
                                message::json_get_str(reply.data, "ip", ip);
                                if (cb)
                                    cb(true, ip);
                            }
                            else
                            {
                                LOG_ERROR(TAG, "WiFi连接失败: %s", reply.err.c_str());
                                if (cb)
                                    cb(false, "");
                            }
                        },
                        30000);
                }

                void NetworkModule::wifi_disconnect()
                {
                    Message msg = make_notify(NetCmd::WIFI_DISCONNECT);
                    send(msg);
                }

                void NetworkModule::wifi_status(WifiStatusCb cb)
                {
                    Message req = make_req(NetCmd::WIFI_STATUS);

                    Dispatcher::inst().request(req,
                                               [cb](const Message& reply)
                                               {
                                                   if (reply.ok)
                                                   {
                                                       auto status = parse_wifi_status(reply.data);
                                                       if (cb)
                                                           cb(status);
                                                   }
                                                   else
                                                   {
                                                       WifiStatus empty;
                                                       empty.state = "error";
                                                       if (cb)
                                                           cb(empty);
                                                   }
                                               });
                }

                void
                NetworkModule::wifi_list(std::function<void(const std::vector<std::string>&)> cb)
                {
                    Message req = make_req(NetCmd::WIFI_LIST);

                    Dispatcher::inst().request(
                        req,
                        [cb](const Message& reply)
                        {
                            std::vector<std::string> ssids;
                            if (reply.ok && !reply.data.empty())
                            {
                                std::string arr;
                                if (message::json_get_obj(reply.data, "ssids", arr) ||
                                    reply.data.front() == '[')
                                {
                                    if (arr.empty())
                                        arr = reply.data;
                                    size_t pos = 0;
                                    while ((pos = arr.find('"', pos)) != std::string::npos)
                                    {
                                        pos++;
                                        size_t end = arr.find('"', pos);
                                        if (end != std::string::npos)
                                        {
                                            ssids.push_back(arr.substr(pos, end - pos));
                                            pos = end + 1;
                                        }
                                    }
                                }
                            }
                            if (cb)
                                cb(ssids);
                        });
                }

                void NetworkModule::wifi_forget(const std::string& ssid)
                {
                    Message msg = make_notify(NetCmd::WIFI_FORGET);
                    msg.data    = "{\"ssid\":\"" + message::json_escape(ssid) + "\"}";
                    send(msg);
                }

                void NetworkModule::wifi_forget_all()
                {
                    Message msg = make_notify(NetCmd::WIFI_FORGET_ALL);
                    send(msg);
                }

                /*============================================================================
                 * BLE 配网操作
                 *============================================================================*/

                void NetworkModule::ble_adv_start(const std::string& name, uint32_t duration_ms,
                                                  BleResultCb cb)
                {
                    Message req = make_req(NetCmd::BLE_ADV_START);

                    std::string data = "{";
                    if (!name.empty())
                    {
                        data += "\"name\":\"" + message::json_escape(name) + "\"";
                    }
                    if (duration_ms > 0)
                    {
                        if (!name.empty())
                            data += ",";
                        char buf[32];
                        snprintf(buf, sizeof(buf), "\"duration\":%u", duration_ms);
                        data += buf;
                    }
                    data += "}";
                    if (data != "{}")
                        req.data = data;

                    Dispatcher::inst().request(
                        req,
                        [cb](const Message& reply)
                        {
                            if (cb)
                                cb(reply.ok);
                            if (!reply.ok)
                            {
                                LOG_ERROR(TAG, "BLE广播启动失败: %s", reply.err.c_str());
                            }
                        },
                        5000);
                }

                void NetworkModule::ble_adv_stop(BleResultCb cb)
                {
                    Message req = make_req(NetCmd::BLE_ADV_STOP);

                    Dispatcher::inst().request(
                        req,
                        [cb](const Message& reply)
                        {
                            if (cb)
                                cb(reply.ok);
                            if (!reply.ok)
                            {
                                LOG_ERROR(TAG, "BLE广播停止失败: %s", reply.err.c_str());
                            }
                        },
                        5000);
                }

                void NetworkModule::ble_status(BleStatusCb cb)
                {
                    Message req = make_req(NetCmd::BLE_STATUS);

                    Dispatcher::inst().request(req,
                                               [cb](const Message& reply)
                                               {
                                                   if (reply.ok)
                                                   {
                                                       auto status = parse_ble_status(reply.data);
                                                       if (cb)
                                                           cb(status);
                                                   }
                                                   else
                                                   {
                                                       BleStatus empty;
                                                       empty.state = "error";
                                                       if (cb)
                                                           cb(empty);
                                                   }
                                               });
                }

                void NetworkModule::ble_disconnect()
                {
                    Message msg = make_notify(NetCmd::BLE_DISCONNECT);
                    send(msg);
                }

                /*============================================================================
                 * 解析函数
                 *============================================================================*/

                std::vector<WifiApInfo> NetworkModule::parse_scan_result(const std::string& data)
                {
                    std::vector<WifiApInfo> aps;
                    if (data.empty())
                        return aps;

                    std::string arr;
                    if (!message::json_get_obj(data, "aps", arr))
                    {
                        if (data.front() == '[')
                            arr = data;
                        else
                            return aps;
                    }

                    size_t pos = 0;
                    while ((pos = arr.find('{', pos)) != std::string::npos)
                    {
                        size_t end = arr.find('}', pos);
                        if (end == std::string::npos)
                            break;

                        std::string obj = arr.substr(pos, end - pos + 1);
                        WifiApInfo  ap;

                        message::json_get_str(obj, "ssid", ap.ssid);

                        int32_t rssi = 0;
                        if (message::json_get_int(obj, "rssi", rssi))
                            ap.rssi = static_cast<int8_t>(rssi);

                        int32_t auth = 0;
                        if (message::json_get_int(obj, "auth", auth))
                            ap.authmode = static_cast<uint8_t>(auth);

                        message::json_get_bool(obj, "enc", ap.encrypted);

                        if (!ap.ssid.empty())
                            aps.push_back(ap);

                        pos = end + 1;
                    }

                    return aps;
                }

                WifiStatus NetworkModule::parse_wifi_status(const std::string& data)
                {
                    WifiStatus status;
                    if (data.empty())
                        return status;

                    message::json_get_str(data, "state", status.state);
                    message::json_get_str(data, "ssid", status.ssid);
                    message::json_get_str(data, "ip", status.ip);

                    int32_t rssi = 0;
                    if (message::json_get_int(data, "rssi", rssi))
                        status.rssi = static_cast<int8_t>(rssi);

                    return status;
                }

                BleStatus NetworkModule::parse_ble_status(const std::string& data)
                {
                    BleStatus status;
                    if (data.empty())
                        return status;

                    message::json_get_str(data, "state", status.state);
                    message::json_get_str(data, "name", status.name);
                    message::json_get_str(data, "mac", status.mac);

                    int32_t cnt = 0;
                    if (message::json_get_int(data, "conn_count", cnt))
                        status.conn_count = static_cast<uint8_t>(cnt);

                    return status;
        }

} // namespace app::common::uart::module
