/**
 * @file main.cpp
 * @brief VideoSystem 测试程序
 * @details 测试内容：
 *          1. 视频流启动/停止
 *          2. 拍照测试
 *          3. 录像测试
 *          4. RTSP推流测试
 *          5. ISP参数调节测试
 *          6. 编码参数测试
 */

#include "media/camera/camera.hpp"
#include "media/sync/sync.hpp"
#include "tool/log/log.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <sys/stat.h>
#include <unistd.h>

using namespace app::media::camera;
using namespace app::tool::log;

namespace
{
    constexpr const char* LOG_TAG    = "TEST_CAMERA";
    constexpr const char* OUTPUT_DIR = "/tmp/camera_test";
} // namespace

// 全局标志：控制程序退出
std::atomic<bool> g_quit{false};

void signalHandler(int signal)
{
    LOG_INFO(LOG_TAG, "收到信号 %d，正在退出...", signal);
    g_quit.store(true);
}

// 工具函数：确保目录存在
bool ensureDirectory(const std::string& path)
{
    struct stat st;
    if (stat(path.c_str(), &st) != 0)
    {
        std::string cmd = "mkdir -p " + path;
        if (system(cmd.c_str()) != 0)
        {
            LOG_ERROR(LOG_TAG, "无法创建目录: %s", path.c_str());
            return false;
        }
    }
    return true;
}

// 获取时间戳字符串
std::string getTimestamp()
{
    auto now  = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&time));

    char result[80];
    snprintf(result, sizeof(result), "%s_%03d", buf, (int)ms.count());
    return std::string(result);
}

// ==================== 测试函数 ====================

// 测试1: 视频流启动/停止
bool testVideoStream(VideoSystem& video)
{
    LOG_INFO(LOG_TAG, "");
    LOG_INFO(LOG_TAG, "========== 测试1: 视频流启动/停止 ==========");

    // 启动视频流
    LOG_INFO(LOG_TAG, "启动视频流...");
    VideoError err = video.start_stream();
    if (err != VideoError::NONE)
    {
        LOG_ERROR(LOG_TAG, "启动视频流失败: %d", static_cast<int>(err));
        return false;
    }

    LOG_INFO(LOG_TAG, "视频流已启动，等待3秒...");
    for (int i = 0; i < 30 && !g_quit; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (i % 10 == 0)
        {
            LOG_INFO(LOG_TAG, "  当前帧率: %.2f FPS, 状态: %d", video.getCurrentFPS(),
                     static_cast<int>(video.get_state()));
        }
    }

    // 停止视频流
    LOG_INFO(LOG_TAG, "停止视频流...");
    err = video.stop_stream();
    if (err != VideoError::NONE)
    {
        LOG_ERROR(LOG_TAG, "停止视频流失败: %d", static_cast<int>(err));
        return false;
    }

    LOG_INFO(LOG_TAG, "[通过] 视频流测试完成 ✓");
    return true;
}

// 测试2: 拍照测试
bool testPhoto(VideoSystem& video)
{
    LOG_INFO(LOG_TAG, "");
    LOG_INFO(LOG_TAG, "========== 测试2: 拍照测试 ==========");

    // 确保视频流已启动
    if (!video.is_streaming())
    {
        LOG_INFO(LOG_TAG, "启动视频流...");
        VideoError err = video.start_stream();
        if (err != VideoError::NONE)
        {
            LOG_ERROR(LOG_TAG, "启动视频流失败");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 单张拍照
    LOG_INFO(LOG_TAG, "--- 2.1 单张拍照测试 ---");
    std::string filename = std::string(OUTPUT_DIR) + "/photo_" + getTimestamp() + ".jpg";
    LOG_INFO(LOG_TAG, "拍照保存到: %s", filename.c_str());

    VideoError err = video.take_photo(filename);
    if (err != VideoError::NONE)
    {
        LOG_ERROR(LOG_TAG, "拍照失败: %d", static_cast<int>(err));
        return false;
    }
    LOG_INFO(LOG_TAG, "拍照成功!");

    // 连续拍照测试
    LOG_INFO(LOG_TAG, "--- 2.2 连续拍照测试 (5张) ---");
    for (int i = 0; i < 5 && !g_quit; i++)
    {
        filename = std::string(OUTPUT_DIR) + "/photo_burst_" + std::to_string(i) + "_" +
                   getTimestamp() + ".jpg";
        err = video.take_photo(filename);
        if (err != VideoError::NONE)
        {
            LOG_WARN(LOG_TAG, "第 %d 张拍照失败", i + 1);
        }
        else
        {
            LOG_INFO(LOG_TAG, "第 %d 张拍照成功: %s", i + 1, filename.c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // 不同JPEG质量测试
    LOG_INFO(LOG_TAG, "--- 2.3 不同JPEG质量测试 ---");
    int qualities[] = {30, 60, 90};
    for (int q : qualities)
    {
        if (g_quit)
            break;

        video.setJPEGQuality(q);
        filename = std::string(OUTPUT_DIR) + "/photo_q" + std::to_string(q) + "_" + getTimestamp() +
                   ".jpg";
        err = video.take_photo(filename);
        if (err == VideoError::NONE)
        {
            LOG_INFO(LOG_TAG, "质量 %d 拍照成功: %s", q, filename.c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    video.setJPEGQuality(80); // 恢复默认

    LOG_INFO(LOG_TAG, "[通过] 拍照测试完成 ✓");
    return true;
}

// 测试3: 录像测试
bool testRecord(VideoSystem& video)
{
    LOG_INFO(LOG_TAG, "");
    LOG_INFO(LOG_TAG, "========== 测试3: 录像测试 ==========");

    // 确保视频流已启动
    if (!video.is_streaming())
    {
        LOG_INFO(LOG_TAG, "启动视频流...");
        VideoError err = video.start_stream();
        if (err != VideoError::NONE)
        {
            LOG_ERROR(LOG_TAG, "启动视频流失败");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 录制5秒视频
    std::string filename = std::string(OUTPUT_DIR) + "/record_" + getTimestamp() + ".h264";
    LOG_INFO(LOG_TAG, "开始录像 (5秒): %s", filename.c_str());

    VideoError err = video.start_record(filename, 5);
    if (err != VideoError::NONE)
    {
        LOG_ERROR(LOG_TAG, "开始录像失败: %d", static_cast<int>(err));
        return false;
    }

    // 等待录像完成
    int wait_count = 0;
    while (video.is_recording() && !g_quit && wait_count < 70)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_count++;
        if (wait_count % 10 == 0)
        {
            LOG_INFO(LOG_TAG, "录像中... %d秒", wait_count / 10);
        }
    }

    // 如果还在录像，手动停止
    if (video.is_recording())
    {
        LOG_INFO(LOG_TAG, "手动停止录像...");
        video.stop_record();
    }

    LOG_INFO(LOG_TAG, "录像完成!");

    // 检查文件是否存在
    struct stat st;
    if (stat(filename.c_str(), &st) == 0)
    {
        LOG_INFO(LOG_TAG, "录像文件大小: %ld 字节", st.st_size);
    }

    LOG_INFO(LOG_TAG, "[通过] 录像测试完成 ✓");
    return true;
}

// 测试4: RTSP推流测试
bool testRTSP(VideoSystem& video)
{
    LOG_INFO(LOG_TAG, "");
    LOG_INFO(LOG_TAG, "========== 测试4: RTSP推流测试 ==========");

    // 确保视频流已启动
    if (!video.is_streaming())
    {
        LOG_INFO(LOG_TAG, "启动视频流...");
        VideoError err = video.start_stream();
        if (err != VideoError::NONE)
        {
            LOG_ERROR(LOG_TAG, "启动视频流失败");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 启动RTSP推流
    LOG_INFO(LOG_TAG, "启动RTSP推流 (端口554, 路径/live/0)...");
    VideoError err = video.startRTSP(554, "/live/0");
    if (err != VideoError::NONE)
    {
        LOG_ERROR(LOG_TAG, "启动RTSP推流失败: %d", static_cast<int>(err));
        return false;
    }

    LOG_INFO(LOG_TAG, "RTSP推流已启动!");
    LOG_INFO(LOG_TAG, "可以使用 VLC 播放: rtsp://<设备IP>:554/live/0");
    LOG_INFO(LOG_TAG, "推流10秒后自动停止...");

    for (int i = 0; i < 100 && !g_quit && video.isRTSPStreaming(); i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (i % 20 == 0)
        {
            LOG_INFO(LOG_TAG, "RTSP推流中... %d秒", i / 10);
        }
    }

    // 停止RTSP推流
    LOG_INFO(LOG_TAG, "停止RTSP推流...");
    err = video.stopRTSP();
    if (err != VideoError::NONE)
    {
        LOG_WARN(LOG_TAG, "停止RTSP推流失败: %d", static_cast<int>(err));
    }

    LOG_INFO(LOG_TAG, "[通过] RTSP推流测试完成 ✓");
    return true;
}

// 测试5: ISP参数调节测试
bool testISP(VideoSystem& video)
{
    LOG_INFO(LOG_TAG, "");
    LOG_INFO(LOG_TAG, "========== 测试5: ISP参数调节测试 ==========");

    // 确保视频流已启动
    if (!video.is_streaming())
    {
        LOG_INFO(LOG_TAG, "启动视频流...");
        VideoError err = video.start_stream();
        if (err != VideoError::NONE)
        {
            LOG_ERROR(LOG_TAG, "启动视频流失败");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 亮度测试
    LOG_INFO(LOG_TAG, "--- 5.1 亮度测试 ---");
    unsigned int brightness_levels[] = {30, 50, 70};
    for (unsigned int level : brightness_levels)
    {
        if (g_quit)
            break;
        video.set_brightness(level);
        LOG_INFO(LOG_TAG, "亮度设置为: %u", level);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    video.set_brightness(50); // 恢复默认

    // 对比度测试
    LOG_INFO(LOG_TAG, "--- 5.2 对比度测试 ---");
    unsigned int contrast_levels[] = {30, 50, 70};
    for (unsigned int level : contrast_levels)
    {
        if (g_quit)
            break;
        video.set_contrast(level);
        LOG_INFO(LOG_TAG, "对比度设置为: %u", level);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    video.set_contrast(50);

    // 饱和度测试
    LOG_INFO(LOG_TAG, "--- 5.3 饱和度测试 ---");
    unsigned int saturation_levels[] = {30, 50, 70};
    for (unsigned int level : saturation_levels)
    {
        if (g_quit)
            break;
        video.set_saturation(level);
        LOG_INFO(LOG_TAG, "饱和度设置为: %u", level);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    video.set_saturation(50);

    // 锐度测试
    LOG_INFO(LOG_TAG, "--- 5.4 锐度测试 ---");
    unsigned int sharpness_levels[] = {30, 50, 70};
    for (unsigned int level : sharpness_levels)
    {
        if (g_quit)
            break;
        video.set_sharpness(level);
        LOG_INFO(LOG_TAG, "锐度设置为: %u", level);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    video.set_sharpness(50);

    LOG_INFO(LOG_TAG, "[通过] ISP参数测试完成 ✓");
    return true;
}

// 测试6: 编码参数测试
bool testEncoding(VideoSystem& video)
{
    LOG_INFO(LOG_TAG, "");
    LOG_INFO(LOG_TAG, "========== 测试6: 编码参数测试 ==========");

    // 确保视频流已启动
    if (!video.is_streaming())
    {
        LOG_INFO(LOG_TAG, "启动视频流...");
        VideoError err = video.start_stream();
        if (err != VideoError::NONE)
        {
            LOG_ERROR(LOG_TAG, "启动视频流失败");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 码率测试
    LOG_INFO(LOG_TAG, "--- 6.1 码率测试 ---");
    int bitrates[] = {1000, 2000, 4000, 6000};
    for (int br : bitrates)
    {
        if (g_quit)
            break;
        video.set_bitrate(br);
        LOG_INFO(LOG_TAG, "码率设置为: %d kbps", br);

        // 录制一小段视频
        std::string filename = std::string(OUTPUT_DIR) + "/bitrate_" + std::to_string(br) + "k_" +
                               getTimestamp() + ".h264";
        video.start_record(filename, 2);
        std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        video.stop_record();

        struct stat st;
        if (stat(filename.c_str(), &st) == 0)
        {
            LOG_INFO(LOG_TAG, "  文件大小: %ld 字节", st.st_size);
        }
    }
    video.set_bitrate(4000); // 恢复默认

    // GOP测试
    LOG_INFO(LOG_TAG, "--- 6.2 GOP测试 ---");
    int gops[] = {15, 30, 60};
    for (int gop : gops)
    {
        if (g_quit)
            break;
        video.setGOP(gop);
        LOG_INFO(LOG_TAG, "GOP设置为: %d", gop);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    video.setGOP(30);

    LOG_INFO(LOG_TAG, "[通过] 编码参数测试完成 ✓");
    return true;
}

// 打印统计信息
void printStats(VideoSystem& video)
{
    LOG_INFO(LOG_TAG, "");
    LOG_INFO(LOG_TAG, "========== 统计信息 ==========");

    VideoSystem::Stats stats;
    video.get_stats(stats);

    LOG_INFO(LOG_TAG, "已捕获帧数:  %llu", stats.frames_captured.load());
    LOG_INFO(LOG_TAG, "已拍照次数:  %llu", stats.photos_taken.load());
    LOG_INFO(LOG_TAG, "录像时长:    %llu ms", stats.record_duration_ms.load());

    LOG_INFO(LOG_TAG, "内存池统计:");
    LOG_INFO(LOG_TAG, "  固定池命中: %zu", stats.mem_stats.fixed_pool_hits.load());
    LOG_INFO(LOG_TAG, "  总分配次数: %zu", stats.mem_stats.total_allocations.load());
}

// ==================== 主函数 ====================

int main(int argc, char* argv[])
{
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // 初始化日志系统
    LogConfig log_config;
    Logger::inst().init(log_config);

    LOG_INFO(LOG_TAG, "╔════════════════════════════════════════╗");
    LOG_INFO(LOG_TAG, "║      VideoSystem 测试程序 v1.0          ║");
    LOG_INFO(LOG_TAG, "╚════════════════════════════════════════╝");

    // 创建输出目录
    if (!ensureDirectory(OUTPUT_DIR))
    {
        return 1;
    }
    LOG_INFO(LOG_TAG, "输出目录: %s", OUTPUT_DIR);

    // 创建同步上下文
    auto sync_ctx = std::make_shared<sync_context_t>();
    sync_init(sync_ctx.get());

    // 创建视频系统配置
    VideoConfig config;
    config.width   = 1920;
    config.height  = 1080;
    config.fps     = 30;
    config.bitrate = 4000;
    config.gop     = 30;
    config.quality = 80;

    LOG_INFO(LOG_TAG, "视频配置: %dx%d @ %d fps, 码率 %d kbps", config.width, config.height,
             config.fps, config.bitrate);

    // 创建视频系统
    VideoSystem video(config);

    // 初始化
    LOG_INFO(LOG_TAG, "初始化视频系统...");
    VideoError err = video.init(sync_ctx);
    if (err != VideoError::NONE)
    {
        LOG_ERROR(LOG_TAG, "视频系统初始化失败: %d", static_cast<int>(err));
        return 1;
    }
    LOG_INFO(LOG_TAG, "视频系统初始化成功!");

    // 运行测试
    int passed = 0;
    int failed = 0;

    // 测试1: 视频流
    if (!g_quit && testVideoStream(video))
        passed++;
    else
        failed++;

    // 测试2: 拍照
    if (!g_quit && testPhoto(video))
        passed++;
    else
        failed++;

    // 测试3: 录像
    if (!g_quit && testRecord(video))
        passed++;
    else
        failed++;

    // 测试4: RTSP (可选，如果不需要可以注释掉)
    // if (!g_quit && testRTSP(video)) passed++; else failed++;

    // 测试5: ISP参数
    if (!g_quit && testISP(video))
        passed++;
    else
        failed++;

    // 测试6: 编码参数
    if (!g_quit && testEncoding(video))
        passed++;
    else
        failed++;

    // 停止视频流
    if (video.is_streaming())
    {
        video.stop_stream();
    }

    // 打印统计信息
    printStats(video);

    // 反初始化
    LOG_INFO(LOG_TAG, "反初始化视频系统...");
    video.deinit();
    sync_deinit(sync_ctx.get());

    // 打印测试结果
    LOG_INFO(LOG_TAG, "");
    LOG_INFO(LOG_TAG, "╔════════════════════════════════════════╗");
    LOG_INFO(LOG_TAG, "║            测试结果汇总                 ║");
    LOG_INFO(LOG_TAG, "╠════════════════════════════════════════╣");
    LOG_INFO(LOG_TAG, "║  通过: %d                               ║", passed);
    LOG_INFO(LOG_TAG, "║  失败: %d                               ║", failed);
    LOG_INFO(LOG_TAG, "╚════════════════════════════════════════╝");

    // 关闭日志系统
    Logger::inst().deinit();

    return failed > 0 ? 1 : 0;
}
