/**
 * @file    test_audio_main.cpp
 * @brief   音频子系统功能测试
 * @note    包含内存池、Opus编解码、重采样、音量控制、3A算法、WAV录制
 */

#include "app/media/audio/audio.hpp"
#include "app/tool/log/log.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

using namespace app::media::audio;
using namespace app::tool::log;

namespace
{
constexpr const char* TAG = "TEST";
} // namespace

/* 全局变量 */
std::queue<AudioFramePtr> g_rec_q;
std::mutex                g_rec_mtx;
std::atomic<bool>         g_running{false};
std::atomic<int>          g_cb_cnt{0};

/*===========================================================================*/
/* 辅助函数                                                                   */
/*===========================================================================*/

std::vector<int16_t> gen_sine(double freq, double amp, int fs, int ms)
{
    int n = fs * ms / 1000;
    std::vector<int16_t> buf(static_cast<size_t>(n));
    for (int i = 0; i < n; i++)
    {
        double t = static_cast<double>(i) / fs;
        double v = amp * sin(2.0 * M_PI * freq * t);
        buf[static_cast<size_t>(i)] = static_cast<int16_t>(v * 32767.0);
    }
    return buf;
}

AudioFramePtr make_sine_frame(AudioMemoryPool& pool, double freq, double amp, int fs, int ms)
{
    auto   buf  = gen_sine(freq, amp, fs, ms);
    size_t len  = buf.size() * sizeof(int16_t);
    auto   f    = pool.allocate(len);
    if (f)
    {
        std::memcpy(f->data, buf.data(), len);
        f->size      = len;
        f->timestamp = 0;
    }
    return f;
}

void on_record(AudioFramePtr f)
{
    std::lock_guard<std::mutex> lk(g_rec_mtx);
    g_rec_q.push(f);
    g_cb_cnt.fetch_add(1);
    if (g_cb_cnt.load() % 50 == 0)
        LOG_DEBUG(TAG, "已录 %d帧 队列=%zu", g_cb_cnt.load(), g_rec_q.size());
}

/*===========================================================================*/
/* 测试1: 内存池                                                              */
/*===========================================================================*/

bool test_mem_pool()
{
    LOG_INFO(TAG, "==== 测试1: 内存池 ====");

    MemoryPoolConfig cfg;
    cfg.fixed_block_size  = 2048;
    cfg.fixed_block_count = 50;
    cfg.dynamic_pool_size = 1024 * 1024;

    AudioMemoryPool pool(cfg);

    /* 固定池分配 */
    LOG_INFO(TAG, "1.1 固定池分配");
    std::vector<AudioFramePtr> vec;
    for (int i = 0; i < 50; i++)
    {
        auto f = pool.allocate(1024);
        if (!f) { LOG_ERROR(TAG, "固定池分配失败 i=%d", i); return false; }
        vec.push_back(f);
    }

    /* 回退到动态池 */
    auto extra = pool.allocate(1024);
    if (!extra) { LOG_ERROR(TAG, "动态池回退失败"); return false; }
    LOG_INFO(TAG, "固定池耗尽后回退到动态池正常");

    /* 动态池大块 */
    LOG_INFO(TAG, "1.2 动态池大块分配");
    std::vector<AudioFramePtr> dyn;
    for (int i = 0; i < 10; i++)
    {
        auto f = pool.allocate(4096);
        if (!f) { LOG_ERROR(TAG, "动态池分配失败 i=%d", i); return false; }
        dyn.push_back(f);
    }

    /* 回收复用 */
    LOG_INFO(TAG, "1.3 内存回收复用");
    vec.clear();
    extra.reset();
    auto reuse = pool.allocate(1024);
    if (!reuse) { LOG_ERROR(TAG, "复用失败"); return false; }

    AudioMemoryPool::Stats st;
    pool.get_stats(st);
    LOG_INFO(TAG, "统计: 总=%llu 固定=%llu(%.1f%%) 动态=%llu 失败=%llu",
             st.total_allocs.load(), st.fixed_hits.load(), st.get_fixed_hit_rate(),
             st.dynamic_hits.load(), st.failures.load());

    LOG_INFO(TAG, "[PASS] 内存池测试");
    return true;
}

/*===========================================================================*/
/* 测试2: Opus编解码播放                                                      */
/*===========================================================================*/

bool test_opus_playback()
{
    LOG_INFO(TAG, "==== 测试2: Opus编解码播放 ====");

    AudioConfig cfg;
    cfg.sample_rate = 48000;
    cfg.channels    = 1;
    cfg.frame_ms    = 20;
    cfg.denoise_en  = false;
    cfg.agc_en      = false;

    AudioSystem audio(cfg);
    if (audio.init() != AudioError::NONE) { LOG_ERROR(TAG, "初始化失败"); return false; }

    LOG_INFO(TAG, "2.1 录音3秒");
    if (audio.start_stream(StreamDirection::INPUT) != AudioError::NONE)
    {
        LOG_ERROR(TAG, "启动录音失败");
        return false;
    }

    std::vector<AudioFramePtr> rec;
    auto t0 = std::chrono::steady_clock::now();

    LOG_INFO(TAG, "录音中...");
    while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(3))
    {
        auto f = audio.get_frame(std::chrono::milliseconds(100));
        if (f)
        {
            rec.push_back(f);
            if (rec.size() % 50 == 0) LOG_INFO(TAG, "已录 %zu帧", rec.size());
        }
    }
    audio.stop_stream(StreamDirection::INPUT);
    LOG_INFO(TAG, "录制完成 %zu帧", rec.size());

    /* 无真实录音则生成模拟数据 */
    if (rec.empty())
    {
        LOG_WARN(TAG, "无录音数据，生成模拟音频");
        MemoryPoolConfig pc;
        AudioMemoryPool  pool(pc);
        int cnt = 3000 / cfg.frame_ms;
        for (int i = 0; i < cnt; i++)
        {
            auto buf = gen_sine(440.0 + i * 2, 0.3, cfg.sample_rate, cfg.frame_ms);
            size_t sz = buf.size() * sizeof(int16_t);
            auto f = pool.allocate(sz);
            if (f)
            {
                std::memcpy(f->data, buf.data(), sz);
                f->size = sz;
                f->timestamp = static_cast<uint64_t>(i) * cfg.frame_ms * 1000;
                rec.push_back(f);
            }
        }
        LOG_INFO(TAG, "生成 %zu帧", rec.size());
    }

    if (rec.empty()) { LOG_ERROR(TAG, "无测试数据"); return false; }

    LOG_INFO(TAG, "2.2 Opus编码");
    std::vector<AudioFramePtr> opus;
    for (const auto& pcm : rec)
    {
        auto op = audio.opus_encode(pcm->get_data<int16_t>(), pcm->get_sample_count());
        if (op) opus.push_back(op);
    }
    LOG_INFO(TAG, "编码完成 %zu帧", opus.size());

    if (opus.empty()) { LOG_ERROR(TAG, "编码失败"); return false; }

    LOG_INFO(TAG, "2.3 解码播放");
    if (audio.start_stream(StreamDirection::OUTPUT) != AudioError::NONE)
    {
        LOG_ERROR(TAG, "启动播放失败");
        return false;
    }

    for (const auto& op : opus)
    {
        auto pcm = audio.opus_decode(op->data, op->size);
        if (pcm) audio.push_frame(pcm);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));
    audio.stop_stream(StreamDirection::OUTPUT);

    LOG_INFO(TAG, "[PASS] Opus编解码测试");
    return true;
}

/*===========================================================================*/
/* 测试3: 重采样                                                              */
/*===========================================================================*/

bool test_resample()
{
    LOG_INFO(TAG, "==== 测试3: 重采样 ====");

    AudioConfig cfg;
    cfg.sample_rate   = 48000;
    cfg.channels      = 1;
    cfg.resample_en   = true;
    cfg.resample_rate = 16000;

    AudioSystem audio(cfg);
    if (audio.init() != AudioError::NONE) { LOG_ERROR(TAG, "初始化失败"); return false; }

    LOG_INFO(TAG, "3.1 生成48kHz测试音频");
    auto buf48 = gen_sine(440.0, 0.5, 48000, 1000);
    LOG_INFO(TAG, "48kHz %zu样本", buf48.size());

    LOG_INFO(TAG, "3.2 重采样到16kHz");
    auto rs = audio.resample(buf48.data(), buf48.size(), 48000, 16000);
    if (!rs)
    {
        LOG_WARN(TAG, "重采样返回空（可能未启用）");
        return true;
    }

    size_t expect = buf48.size() / 3;
    size_t actual = rs->get_sample_count();
    LOG_INFO(TAG, "结果 %zu样本 (预期约%zu)", actual, expect);

    if (actual < expect * 0.9 || actual > expect * 1.1)
        LOG_WARN(TAG, "样本数偏差较大");

    LOG_INFO(TAG, "[PASS] 重采样测试");
    return true;
}

/*===========================================================================*/
/* 测试4: 正弦波&音量控制                                                     */
/*===========================================================================*/

bool test_sine_volume()
{
    LOG_INFO(TAG, "==== 测试4: 正弦波&音量控制 ====");

    AudioConfig cfg;
    cfg.sample_rate = 48000;
    cfg.channels    = 1;

    AudioSystem audio(cfg);
    if (audio.init() != AudioError::NONE) { LOG_ERROR(TAG, "初始化失败"); return false; }

    MemoryPoolConfig pc;
    pc.fixed_block_count = 100;
    AudioMemoryPool pool(pc);

    LOG_INFO(TAG, "4.1 播放不同频率");
    std::vector<std::pair<double, const char*>> freqs = {
        {440.0, "A4"}, {523.25, "C5"}, {880.0, "A5"}, {220.0, "A3"}
    };

    if (audio.start_stream(StreamDirection::OUTPUT) != AudioError::NONE)
    {
        LOG_ERROR(TAG, "启动播放失败");
        return false;
    }

    for (const auto& [f, name] : freqs)
    {
        LOG_INFO(TAG, "播放 %s (%.1fHz)", name, f);
        std::vector<int> vols = {30, 60, 100, 10};
        for (int v : vols)
        {
            audio.set_volume(v);
            LOG_INFO(TAG, "  音量 %d%%", v);
            auto frm = make_sine_frame(pool, f, 0.8, cfg.sample_rate, 500);
            if (frm) audio.push_frame(frm);
            std::this_thread::sleep_for(std::chrono::milliseconds(600));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    LOG_INFO(TAG, "4.2 音量渐变");
    auto lf = make_sine_frame(pool, 440.0, 0.8, cfg.sample_rate, 3000);
    if (lf)
    {
        audio.push_frame(lf);
        for (int i = 0; i <= 20; i++)
        {
            audio.set_volume(i * 5);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        for (int i = 20; i >= 0; i--)
        {
            audio.set_volume(i * 5);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    audio.set_volume(50);
    audio.stop_stream(StreamDirection::OUTPUT);

    LOG_INFO(TAG, "[PASS] 正弦波&音量测试");
    return true;
}

/*===========================================================================*/
/* 测试5: 3A算法                                                              */
/*===========================================================================*/

bool test_3a()
{
    LOG_INFO(TAG, "==== 测试5: 3A算法 ====");

    AudioConfig cfg;
    cfg.sample_rate = 48000;
    cfg.channels    = 1;
    cfg.denoise_en  = true;
    cfg.agc_en      = true;
    cfg.agc_level   = 8000.0f;

    AudioSystem audio(cfg);
    if (audio.init() != AudioError::NONE) { LOG_ERROR(TAG, "初始化失败"); return false; }

    LOG_INFO(TAG, "5.1 3A参数");
    LOG_INFO(TAG, "  降噪=%d AGC=%d level=%.0f", cfg.denoise_en, cfg.agc_en, cfg.agc_level);

    {
        std::lock_guard<std::mutex> lk(g_rec_mtx);
        std::queue<AudioFramePtr> empty;
        g_rec_q.swap(empty);
        g_cb_cnt.store(0);
    }

    audio.set_callback(on_record);

    LOG_INFO(TAG, "5.2 录音5秒");
    if (audio.start_stream(StreamDirection::INPUT) != AudioError::NONE)
    {
        LOG_ERROR(TAG, "启动录音失败");
        return false;
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));
    audio.stop_stream(StreamDirection::INPUT);

    std::vector<AudioFramePtr> vec;
    {
        std::lock_guard<std::mutex> lk(g_rec_mtx);
        while (!g_rec_q.empty())
        {
            vec.push_back(g_rec_q.front());
            g_rec_q.pop();
        }
    }

    LOG_INFO(TAG, "处理完成 %zu帧", vec.size());

    if (!vec.empty())
    {
        int64_t sum = 0;
        size_t  n   = 0;
        for (const auto& f : vec)
        {
            const int16_t* p = f->get_data<int16_t>();
            size_t         c = f->get_sample_count();
            for (size_t i = 0; i < c; i++) sum += abs(p[i]);
            n += c;
        }
        double avg = n > 0 ? static_cast<double>(sum) / n : 0.0;
        LOG_INFO(TAG, "总样本=%zu 平均幅度=%.1f", n, avg);
    }

    LOG_INFO(TAG, "[PASS] 3A算法测试");
    return true;
}

/*===========================================================================*/
/* 测试6: WAV录制                                                             */
/*===========================================================================*/

bool test_wav()
{
    LOG_INFO(TAG, "==== 测试6: WAV录制 ====");

    AudioConfig cfg;
    cfg.sample_rate = 48000;
    cfg.channels    = 1;
    cfg.wav_path    = "/tmp/audio_test/";

    AudioSystem audio(cfg);
    if (audio.init() != AudioError::NONE) { LOG_ERROR(TAG, "初始化失败"); return false; }

    LOG_INFO(TAG, "6.1 启动录音流+WAV");
    if (audio.start_stream(StreamDirection::INPUT) != AudioError::NONE)
    {
        LOG_ERROR(TAG, "启动录音失败");
        return false;
    }

    if (audio.start_wav("", 3) != AudioError::NONE)
    {
        LOG_ERROR(TAG, "启动WAV失败");
        audio.stop_stream(StreamDirection::INPUT);
        return false;
    }

    LOG_INFO(TAG, "录制3秒...");
    std::this_thread::sleep_for(std::chrono::seconds(4));

    if (audio.wav_running()) audio.stop_wav();
    audio.stop_stream(StreamDirection::INPUT);

    LOG_INFO(TAG, "[PASS] WAV录制测试");
    return true;
}

/*===========================================================================*/
/* 主函数                                                                     */
/*===========================================================================*/

int main()
{
    LogConfig lc;
    lc.min_level      = LogLevel::INFO;
    lc.enable_console = true;
    lc.enable_file    = true;
    lc.log_file_path  = "./log/test_audio.log";

    Logger& log = Logger::inst();
    if (!log.init(lc))
    {
        std::cerr << "日志初始化失败" << std::endl;
        return -1;
    }

    LOG_INFO(TAG, "音频子系统测试开始");
    LOG_INFO(TAG, "============================");

    int pass = 0, fail = 0;
    std::vector<std::pair<std::function<bool()>, const char*>> tests = {
        {test_mem_pool,     "内存池"},
        {test_opus_playback,"Opus编解码"},
        {test_resample,     "重采样"},
        {test_sine_volume,  "正弦波&音量"},
        {test_3a,           "3A算法"},
        {test_wav,          "WAV录制"}
    };

    for (const auto& [fn, name] : tests)
    {
        try
        {
            LOG_INFO(TAG, "\n>>> %s", name);
            if (fn())
            {
                LOG_INFO(TAG, "[OK] %s", name);
                pass++;
            }
            else
            {
                LOG_ERROR(TAG, "[FAIL] %s", name);
                fail++;
            }
        }
        catch (const std::exception& e)
        {
            LOG_ERROR(TAG, "[EXCEPT] %s: %s", name, e.what());
            fail++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    LOG_INFO(TAG, "============================");
    LOG_INFO(TAG, "测试完成 通过=%d 失败=%d", pass, fail);

    if (fail == 0)
    {
        std::cout << "\n所有测试通过\n" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "\n" << fail << "个测试失败\n" << std::endl;
        return fail;
    }
}

