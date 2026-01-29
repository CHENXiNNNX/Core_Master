/**
 * @file    network.hpp
 * @brief   网络模块 - WiFi/BLE控制
 */

#pragma once

#include "../base.hpp"
#include "../../message/message.hpp"

#include <functional>
#include <string>
#include <vector>

namespace app::common::uart::module
{

                /*============================================================================
                 * 网络模块命令
                 *============================================================================*/

                namespace NetCmd
                {
                    // WiFi
                    constexpr const char* WIFI_SCAN       = "wifi_scan";
                    constexpr const char* WIFI_CONNECT    = "wifi_connect";
                    constexpr const char* WIFI_DISCONNECT = "wifi_disconnect";
                    constexpr const char* WIFI_STATUS     = "wifi_status";
                    constexpr const char* WIFI_LIST       = "wifi_list";
                    constexpr const char* WIFI_FORGET     = "wifi_forget";
                    constexpr const char* WIFI_FORGET_ALL = "wifi_forget_all";

                    // BLE 配网
                    constexpr const char* BLE_ADV_START  = "ble_adv_start";
                    constexpr const char* BLE_ADV_STOP   = "ble_adv_stop";
                    constexpr const char* BLE_STATUS     = "ble_status";
                    constexpr const char* BLE_DISCONNECT = "ble_disconnect";

                    // BLE 扫描连接（预留）
                    constexpr const char* BLE_SCAN    = "ble_scan";
                    constexpr const char* BLE_CONNECT = "ble_connect";
                } // namespace NetCmd

                /*============================================================================
                 * 数据结构
                 *============================================================================*/

                struct WifiApInfo
                {
                    std::string ssid;
                    int8_t      rssi      = 0;
                    uint8_t     authmode  = 0;
                    bool        encrypted = false;
                };

                struct WifiStatus
                {
                    std::string state; // connected/connecting/disconnected/failed
                    std::string ssid;
                    std::string ip;
                    int8_t      rssi = 0;
                };

                struct BleStatus
                {
                    std::string state; // idle/advertising/connected
                    std::string name;
                    std::string mac;
                    uint8_t     conn_count = 0;
                };

                /*============================================================================
                 * 回调类型
                 *============================================================================*/

                using WifiScanCb = std::function<void(bool ok, const std::vector<WifiApInfo>& aps)>;
                using WifiConnCb = std::function<void(bool ok, const std::string& ip)>;
                using WifiStatusCb = std::function<void(const WifiStatus& status)>;
                using BleStatusCb  = std::function<void(const BleStatus& status)>;
                using BleResultCb  = std::function<void(bool ok)>;

                /*============================================================================
                 * 网络模块
                 *============================================================================*/

                class NetworkModule : public ModuleBase
                {
                public:
                    static NetworkModule& inst();

                    const char* name() const override
                    {
                        return message::ModName::NETWORK;
                    }

                    void init();

                    /*--------------------------------------------------------------------
                     * WiFi 操作
                     *--------------------------------------------------------------------*/

                    void wifi_scan(WifiScanCb cb);
                    void wifi_connect(const std::string& ssid, const std::string& pwd,
                                      WifiConnCb cb);
                    void wifi_disconnect();
                    void wifi_status(WifiStatusCb cb);
                    void wifi_list(std::function<void(const std::vector<std::string>&)> cb);
                    void wifi_forget(const std::string& ssid);
                    void wifi_forget_all();

                    /*--------------------------------------------------------------------
                     * BLE 配网操作
                     *--------------------------------------------------------------------*/

                    /* 开始BLE广播 */
                    void ble_adv_start(const std::string& name = "", uint32_t duration_ms = 0,
                                       BleResultCb cb = nullptr);

                    /* 停止BLE广播 */
                    void ble_adv_stop(BleResultCb cb = nullptr);

                    /* 查询BLE状态 */
                    void ble_status(BleStatusCb cb);

                    /* 断开BLE连接 */
                    void ble_disconnect();

                private:
                    NetworkModule();
                    ~NetworkModule()                               = default;
                    NetworkModule(const NetworkModule&)            = delete;
                    NetworkModule& operator=(const NetworkModule&) = delete;

                    void init_handlers();

                    static std::vector<WifiApInfo> parse_scan_result(const std::string& data);
                    static WifiStatus              parse_wifi_status(const std::string& data);
                    static BleStatus               parse_ble_status(const std::string& data);
        };

} // namespace app::common::uart::module
