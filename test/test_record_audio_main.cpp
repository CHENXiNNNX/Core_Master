/**
 * @file    test_record_audio_main.cpp
 * @brief   录音播放测试
 */

#include "app/media/audio/audio.hpp"
#include "app/tool/file/file.hpp"
#include "app/tool/log/log.hpp"
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

using namespace app::media::audio;
using namespace app::tool::file;
using namespace app::tool::log;

namespace
{
constexpr const char* TAG = "MAIN";
}

int main()
{
    LogConfig lc;
    lc.min_level      = LogLevel::INFO;
    lc.enable_console = true;
    lc.enable_file    = false;

    Logger& log = Logger::inst();
    if (!log.init(lc))
    {
        std::cerr << "日志初始化失败" << std::endl;
        return -1;
    }

    AudioConfig cfg;
    cfg.sample_rate = 48000;
    cfg.channels    = 1;
    cfg.frame_ms    = 20;
    cfg.wav_path    = "/tmp/audio/";

    AudioSystem audio(cfg);
    if (audio.init() != AudioError::NONE)
    {
        LOG_ERROR(TAG, "初始化失败");
        return -1;
    }

    /* 启动录音流 */
    if (audio.start_stream(StreamDirection::INPUT) != AudioError::NONE)
    {
        LOG_ERROR(TAG, "启动录音失败");
        audio.deinit();
        return -1;
    }

    /* 启动WAV录制(5秒) */
    if (audio.start_wav("", 5) != AudioError::NONE)
    {
        LOG_ERROR(TAG, "启动WAV录制失败");
        audio.stop_stream(StreamDirection::INPUT);
        audio.deinit();
        return -1;
    }

    LOG_INFO(TAG, "录音中(5秒)...");
    std::this_thread::sleep_for(std::chrono::seconds(6));

    if (audio.wav_running()) audio.stop_wav();
    audio.stop_stream(StreamDirection::INPUT);

    /* 检查录音文件 */
    std::string fn = cfg.wav_path + "rec_0.wav";
    if (!exists(fn))
    {
        LOG_ERROR(TAG, "文件不存在: %s", fn.c_str());
        audio.deinit();
        return -1;
    }

    /* 读取WAV */
    std::vector<uint8_t> wav;
    if (!read_all(fn, wav))
    {
        LOG_ERROR(TAG, "读取失败");
        audio.deinit();
        return -1;
    }

    constexpr size_t HDR = 44;
    if (wav.size() < HDR)
    {
        LOG_ERROR(TAG, "WAV格式错误");
        audio.deinit();
        return -1;
    }

    std::vector<uint8_t> pcm(wav.begin() + HDR, wav.end());
    LOG_INFO(TAG, "PCM %zu字节", pcm.size());

    int    fsize = cfg.sample_rate * cfg.frame_ms / 1000;
    size_t fbytes = static_cast<size_t>(fsize) * cfg.channels * sizeof(int16_t);

    /* 启动播放 */
    if (audio.start_stream(StreamDirection::OUTPUT) != AudioError::NONE)
    {
        LOG_ERROR(TAG, "启动播放失败");
        audio.deinit();
        return -1;
    }

    LOG_INFO(TAG, "播放中...");

    MemoryPoolConfig pc;
    AudioMemoryPool  pool(pc);

    size_t cnt = 0, off = 0;
    while (off < pcm.size())
    {
        size_t rem = pcm.size() - off;
        size_t len = rem < fbytes ? rem : fbytes;

        auto f = pool.allocate(len);
        if (!f) { LOG_ERROR(TAG, "分配失败"); break; }

        std::memcpy(f->data, pcm.data() + off, len);
        f->size      = len;
        f->timestamp = 0;

        audio.push_frame(f);
        cnt++;
        off += len;

        std::this_thread::sleep_for(std::chrono::milliseconds(cfg.frame_ms));
    }

    LOG_INFO(TAG, "推送 %zu帧", cnt);

    double dur = static_cast<double>(cnt) * cfg.frame_ms / 1000.0;
    int    sec = static_cast<int>(dur) + 1;

    LOG_INFO(TAG, "等待播放完成(约%d秒)...", sec);
    std::this_thread::sleep_for(std::chrono::seconds(sec));

    audio.stop_stream(StreamDirection::OUTPUT);
    audio.deinit();

    LOG_INFO(TAG, "完成");
    return 0;
}
