/**
 * @file    main.cpp
 * @brief   RV1106主机 - UART通信控制台
 */

 #include <chrono>
 #include <csignal>
 #include <cstdio>
 #include <cstring>
 #include <iostream>
 #include <sstream>
 #include <string>
 #include <thread>
 #include <unistd.h>
 #include <vector>
 
 #include "common/uart/uart.hpp"
 #include "common/uart/message/dispatcher.hpp"
 #include "common/uart/module/network/network.hpp"
 #include "common/uart/module/system/system.hpp"
 
 #define TAG "Main"
 
 using namespace app::common::uart;
 using namespace app::common::uart::message;
 using namespace app::common::uart::module;
 
 static volatile bool g_running = true;
 
 static void sig_handler(int sig)
 {
     (void)sig;
     g_running = false;
     printf("\n");
 }
 
 /*============================================================================
  * 辅助函数
  *============================================================================*/
 
 static std::vector<std::string> split(const std::string& s, char delim = ' ')
 {
     std::vector<std::string> tokens;
     std::istringstream       iss(s);
     std::string              token;
     while (std::getline(iss, token, delim))
     {
         if (!token.empty())
             tokens.push_back(token);
     }
     return tokens;
 }
 
 static void print_help()
 {
     printf("\n");
     printf("========== 命令列表 ==========\n");
     printf("\n");
     printf("--- 系统 ---\n");
     printf("  ping              测试连通性\n");
     printf("  info              获取系统信息\n");
     printf("  version           获取版本信息\n");
     printf("  reboot            重启从机\n");
     printf("  sleep             进入休眠\n");
     printf("\n");
     printf("--- WiFi ---\n");
     printf("  wifi scan         扫描WiFi\n");
     printf("  wifi status       查询WiFi状态\n");
     printf("  wifi list         已保存WiFi列表\n");
     printf("  wifi connect <ssid> [pwd]   连接WiFi\n");
     printf("  wifi disconnect   断开WiFi\n");
     printf("  wifi forget <ssid>          忘记WiFi\n");
     printf("  wifi forget_all   忘记所有WiFi\n");
     printf("\n");
     printf("--- BLE ---\n");
     printf("  ble start [name]  开始BLE广播\n");
     printf("  ble stop          停止BLE广播\n");
     printf("  ble status        查询BLE状态\n");
     printf("  ble disconnect    断开BLE连接\n");
     printf("\n");
     printf("--- 其他 ---\n");
     printf("  help              显示帮助\n");
     printf("  quit/exit         退出程序\n");
     printf("\n");
     printf("==============================\n");
 }
 
 /*============================================================================
  * 系统命令
  *============================================================================*/
 
 static void cmd_ping()
 {
     printf("[执行] ping...\n");
     SystemModule::inst().ping(
         [](bool ok, uint32_t latency_ms)
         {
             if (ok)
                 printf("[结果] Ping成功: %ums\n", latency_ms);
             else
                 printf("[结果] Ping失败\n");
         });
 }
 
 static void cmd_info()
 {
     printf("[执行] 获取系统信息...\n");
     SystemModule::inst().get_info(
         [](bool ok, const SystemInfo& info)
         {
             if (ok)
             {
                 printf("[结果] 芯片: %s\n", info.chip.c_str());
                 printf("       堆空闲: %u / %u\n", info.heap_free, info.heap_total);
                 printf("       运行时间: %us\n", info.uptime_sec);
             }
             else
             {
                 printf("[结果] 获取失败\n");
             }
         });
 }
 
 static void cmd_version()
 {
     printf("[执行] 获取版本信息...\n");
     SystemModule::inst().get_version(
         [](bool ok, const VersionInfo& ver)
         {
             if (ok)
             {
                 printf("[结果] 固件: %s\n", ver.fw_version.c_str());
                 printf("       编译: %s\n", ver.build_date.c_str());
             }
             else
             {
                 printf("[结果] 获取失败\n");
             }
         });
 }
 
 static void cmd_reboot()
 {
     printf("[执行] 重启从机...\n");
     SystemModule::inst().reboot();
     printf("[结果] 重启命令已发送\n");
 }
 
 static void cmd_sleep()
 {
     printf("[执行] 进入休眠...\n");
     SystemModule::inst().sleep();
     printf("[结果] 休眠命令已发送\n");
 }
 
 /*============================================================================
  * WiFi命令
  *============================================================================*/
 
 static void cmd_wifi_scan()
 {
     printf("[执行] WiFi扫描...\n");
     NetworkModule::inst().wifi_scan(
         [](bool ok, const std::vector<WifiApInfo>& aps)
         {
             if (ok)
             {
                 printf("[结果] 扫描到 %zu 个AP:\n", aps.size());
                 for (size_t i = 0; i < aps.size(); ++i)
                 {
                     printf("  %2zu. %-24s %4ddBm  auth:%d  %s\n", i + 1, aps[i].ssid.c_str(),
                            aps[i].rssi, aps[i].authmode, aps[i].encrypted ? "加密" : "开放");
                 }
             }
             else
             {
                 printf("[结果] 扫描失败\n");
             }
         });
 }
 
 static void cmd_wifi_status()
 {
     printf("[执行] 查询WiFi状态...\n");
     NetworkModule::inst().wifi_status(
         [](const WifiStatus& status)
         {
             printf("[结果] 状态: %s\n", status.state.c_str());
             if (!status.ssid.empty())
             {
                 printf("       SSID: %s\n", status.ssid.c_str());
                 printf("       IP: %s\n", status.ip.c_str());
                 printf("       RSSI: %ddBm\n", status.rssi);
             }
         });
 }
 
 static void cmd_wifi_list()
 {
     printf("[执行] 获取已保存WiFi列表...\n");
     NetworkModule::inst().wifi_list(
         [](const std::vector<std::string>& ssids)
         {
             printf("[结果] 已保存 %zu 个WiFi:\n", ssids.size());
             for (size_t i = 0; i < ssids.size(); ++i)
             {
                 printf("  %zu. %s\n", i + 1, ssids[i].c_str());
             }
         });
 }
 
 static void cmd_wifi_connect(const std::string& ssid, const std::string& pwd)
 {
     printf("[执行] 连接WiFi: %s...\n", ssid.c_str());
     NetworkModule::inst().wifi_connect(ssid, pwd,
                                        [](bool ok, const std::string& ip)
                                        {
                                            if (ok)
                                                printf("[结果] 连接成功, IP: %s\n", ip.c_str());
                                            else
                                                printf("[结果] 连接失败\n");
                                        });
 }
 
 static void cmd_wifi_disconnect()
 {
     printf("[执行] 断开WiFi...\n");
     NetworkModule::inst().wifi_disconnect();
     printf("[结果] 断开命令已发送\n");
 }
 
 static void cmd_wifi_forget(const std::string& ssid)
 {
     printf("[执行] 忘记WiFi: %s...\n", ssid.c_str());
     NetworkModule::inst().wifi_forget(ssid);
     printf("[结果] 忘记命令已发送\n");
 }
 
 static void cmd_wifi_forget_all()
 {
     printf("[执行] 忘记所有WiFi...\n");
     NetworkModule::inst().wifi_forget_all();
     printf("[结果] 命令已发送\n");
 }
 
 /*============================================================================
  * BLE命令
  *============================================================================*/
 
 static void cmd_ble_start(const std::string& name)
 {
     printf("[执行] 开始BLE广播: %s...\n", name.empty() ? "(默认)" : name.c_str());
     NetworkModule::inst().ble_adv_start(name, 0,
                                         [](bool ok)
                                         {
                                             if (ok)
                                                 printf("[结果] 广播已启动\n");
                                             else
                                                 printf("[结果] 广播启动失败\n");
                                         });
 }
 
 static void cmd_ble_stop()
 {
     printf("[执行] 停止BLE广播...\n");
     NetworkModule::inst().ble_adv_stop(
         [](bool ok)
         {
             if (ok)
                 printf("[结果] 广播已停止\n");
             else
                 printf("[结果] 广播停止失败\n");
         });
 }
 
 static void cmd_ble_status()
 {
     printf("[执行] 查询BLE状态...\n");
     NetworkModule::inst().ble_status(
         [](const BleStatus& status)
         {
             printf("[结果] 状态: %s\n", status.state.c_str());
             printf("       MAC: %s\n", status.mac.c_str());
             printf("       连接数: %d\n", status.conn_count);
         });
 }
 
 static void cmd_ble_disconnect()
 {
     printf("[执行] 断开BLE连接...\n");
     NetworkModule::inst().ble_disconnect();
     printf("[结果] 断开命令已发送\n");
 }
 
 /*============================================================================
  * 命令处理
  *============================================================================*/
 
 static void process_cmd(const std::string& line)
 {
     auto args = split(line);
     if (args.empty())
         return;
 
     const std::string& cmd = args[0];
 
     // 系统命令
     if (cmd == "ping")
     {
         cmd_ping();
     }
     else if (cmd == "info")
     {
         cmd_info();
     }
     else if (cmd == "version")
     {
         cmd_version();
     }
     else if (cmd == "reboot")
     {
         cmd_reboot();
     }
     else if (cmd == "sleep")
     {
         cmd_sleep();
     }
     // WiFi命令
     else if (cmd == "wifi" && args.size() >= 2)
     {
         const std::string& sub = args[1];
         if (sub == "scan")
         {
             cmd_wifi_scan();
         }
         else if (sub == "status")
         {
             cmd_wifi_status();
         }
         else if (sub == "list")
         {
             cmd_wifi_list();
         }
         else if (sub == "connect" && args.size() >= 3)
         {
             std::string pwd = args.size() >= 4 ? args[3] : "";
             cmd_wifi_connect(args[2], pwd);
         }
         else if (sub == "disconnect")
         {
             cmd_wifi_disconnect();
         }
         else if (sub == "forget" && args.size() >= 3)
         {
             cmd_wifi_forget(args[2]);
         }
         else if (sub == "forget_all")
         {
             cmd_wifi_forget_all();
         }
         else
         {
             printf("[错误] 未知WiFi命令: %s\n", sub.c_str());
         }
     }
     // BLE命令
     else if (cmd == "ble" && args.size() >= 2)
     {
         const std::string& sub = args[1];
         if (sub == "start")
         {
             std::string name = args.size() >= 3 ? args[2] : "";
             cmd_ble_start(name);
         }
         else if (sub == "stop")
         {
             cmd_ble_stop();
         }
         else if (sub == "status")
         {
             cmd_ble_status();
         }
         else if (sub == "disconnect")
         {
             cmd_ble_disconnect();
         }
         else
         {
             printf("[错误] 未知BLE命令: %s\n", sub.c_str());
         }
     }
     // 其他
     else if (cmd == "help" || cmd == "?")
     {
         print_help();
     }
     else if (cmd == "quit" || cmd == "exit" || cmd == "q")
     {
         g_running = false;
     }
     else
     {
         printf("[错误] 未知命令: %s (输入help查看帮助)\n", cmd.c_str());
     }
 }
 
 /*============================================================================
  * 主函数
  *============================================================================*/
 
 int main(int argc, char* argv[])
 {
     (void)argc;
     (void)argv;
 
     signal(SIGINT, sig_handler);
     signal(SIGTERM, sig_handler);
 
     printf("[%s] ===== RV1106 UART控制台 =====\n", TAG);
 
     /* UART初始化 */
     UartCfg cfg;
     cfg.dev  = "/dev/ttyS3";
     cfg.baud = 115200;
 
     auto& uart = UartDrv::inst();
     if (!uart.init(cfg))
     {
         printf("[%s] UART初始化失败\n", TAG);
         return -1;
     }
     if (!uart.start())
     {
         printf("[%s] UART启动失败\n", TAG);
         return -1;
     }
     printf("[%s] UART就绪: %s @ %d\n", TAG, cfg.dev.c_str(), cfg.baud);
 
     /* 消息分发器初始化 */
     if (!Dispatcher::inst().init())
     {
         printf("[%s] 分发器初始化失败\n", TAG);
         return -1;
     }
     printf("[%s] 分发器就绪\n", TAG);
 
     /* 模块初始化 */
     NetworkModule::inst().init();
     SystemModule::inst().init();
     printf("[%s] 模块就绪\n", TAG);
 
     printf("[%s] ================================\n", TAG);
     printf("[%s] 输入help查看命令列表\n", TAG);
     printf("\n");
 
     /* 命令行循环 */
     std::string line;
     while (g_running)
     {
         printf("> ");
         fflush(stdout);
 
         if (!std::getline(std::cin, line))
         {
             break;
         }
 
         if (!line.empty())
         {
             process_cmd(line);
             std::this_thread::sleep_for(std::chrono::milliseconds(100));
         }
     }
 
     /* 清理 */
     printf("\n[%s] 正在退出...\n", TAG);
     Dispatcher::inst().deinit();
     uart.stop();
     uart.deinit();
 
     printf("[%s] 退出完成\n", TAG);
     return 0;
 }
 