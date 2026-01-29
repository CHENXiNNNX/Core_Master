/**
 * @file    audio.cc
 * @brief   音频子系统实现
 */

#include "audio.hpp"
#include "../../tool/file/file.hpp"
#include "../../tool/log/log.hpp"

#include <algorithm>
#include <cstring>

namespace app::media::audio
{

using namespace tool::log;
using namespace tool::file;

/*===========================================================================*/
/* 内部常量                                                                   */
/*===========================================================================*/

namespace
{
constexpr const char* TAG               = "AUDIO";
constexpr double      KB                = 1024.0;
constexpr double      MB                = 1024.0 * 1024.0;
constexpr int         MS_PER_S          = 1000;
constexpr float       PCM_NORM          = 32768.0f;
constexpr float       VOL_GAIN_K        = 0.02f;
constexpr size_t      ALIGN_SIZE        = 64;
constexpr double      POOL_EXPAND       = 2.0;
constexpr size_t      FLOAT_BUF_LEN     = 4096;
constexpr size_t      FIXED_MAX_BLOCKS  = 1024;
constexpr size_t      BITS_PER_WORD     = 64;

/* WAV文件头 */
struct WavHeader
{
    char     riff[4]         = {'R', 'I', 'F', 'F'};
    uint32_t file_size       = 0;
    char     wave[4]         = {'W', 'A', 'V', 'E'};
    char     fmt[4]          = {'f', 'm', 't', ' '};
    uint32_t fmt_size        = 16;
    uint16_t audio_fmt       = 1;
    uint16_t channels        = 1;
    uint32_t sample_rate     = 48000;
    uint32_t byte_rate       = 96000;
    uint16_t block_align     = 2;
    uint16_t bits_per_sample = 16;
    char     data[4]         = {'d', 'a', 't', 'a'};
    uint32_t data_size       = 0;
};

bool wav_write_header(FileWrapper& f, int fs, int ch, size_t len)
{
    WavHeader h{};
    h.channels        = static_cast<uint16_t>(ch);
    h.sample_rate     = static_cast<uint32_t>(fs);
    h.bits_per_sample = 16;
    h.block_align     = static_cast<uint16_t>(ch * h.bits_per_sample / 8);
    h.byte_rate       = h.sample_rate * h.block_align;
    h.data_size       = static_cast<uint32_t>(len);
    h.file_size       = static_cast<uint32_t>(sizeof(WavHeader) - 8 + len);
    return f.write(&h, sizeof(h));
}

bool wav_update_header(FileWrapper& f, size_t len)
{
    long pos = f.get_position();
    if (pos < 0) return false;

    if (!f.seek(4, SEEK_SET)) return false;
    uint32_t fsz = static_cast<uint32_t>(sizeof(WavHeader) - 8 + len);
    if (!f.write(&fsz, sizeof(fsz))) return false;

    if (!f.seek(40, SEEK_SET)) return false;
    uint32_t dsz = static_cast<uint32_t>(len);
    if (!f.write(&dsz, sizeof(dsz))) return false;

    f.seek(pos, SEEK_SET);
    return true;
}

uint64_t now_us()
{
    auto t = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count());
}
} // namespace

/*===========================================================================*/
/* AudioMemoryPool::FixedPool                                                */
/*===========================================================================*/

struct AudioMemoryPool::FixedPool
{
    static constexpr size_t BLK_SIZE    = 4096;
    static constexpr size_t BITMAP_LEN  = 16;

    size_t blk_cnt;

    alignas(64) std::array<std::atomic<uint64_t>, BITMAP_LEN> bitmap{};
    std::vector<std::array<uint8_t, BLK_SIZE>> blocks;
    std::vector<AudioFrame>                    frames;

    explicit FixedPool(size_t cnt)
        : blk_cnt(std::min(cnt, FIXED_MAX_BLOCKS))
    {
        for (auto& w : bitmap) w.store(0, std::memory_order_relaxed);
        blocks.resize(blk_cnt);
        frames.resize(blk_cnt);
        LOG_INFO(TAG, "固定池 %zu块 × %zuB = %.1fKB", blk_cnt, BLK_SIZE, (blk_cnt * BLK_SIZE) / KB);
    }

    int alloc()
    {
        size_t words = (blk_cnt + BITS_PER_WORD - 1) / BITS_PER_WORD;
        for (size_t w = 0; w < words; ++w)
        {
            uint64_t val  = bitmap[w].load(std::memory_order_acquire);
            int      base = static_cast<int>(w * BITS_PER_WORD);
            int      max  = static_cast<int>(std::min(BITS_PER_WORD, blk_cnt - w * BITS_PER_WORD));

            while (val != UINT64_MAX)
            {
                uint64_t inv = ~val;
                if (inv == 0) break;
                int bit = __builtin_ctzll(inv);
                if (bit >= max) break;

                uint64_t nv = val | (1ULL << bit);
                if (bitmap[w].compare_exchange_weak(val, nv,
                                                    std::memory_order_acq_rel,
                                                    std::memory_order_acquire))
                    return base + bit;
            }
        }
        return -1;
    }

    void free(int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(blk_cnt)) return;
        size_t   w   = static_cast<size_t>(idx) / BITS_PER_WORD;
        int      b   = idx % static_cast<int>(BITS_PER_WORD);
        uint64_t m   = ~(1ULL << b);
        bitmap[w].fetch_and(m, std::memory_order_release);
    }

    uint8_t* ptr(int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(blk_cnt)) return nullptr;
        return blocks[static_cast<size_t>(idx)].data();
    }
};

/*===========================================================================*/
/* AudioMemoryPool::Stats                                                    */
/*===========================================================================*/

double AudioMemoryPool::Stats::get_fixed_hit_rate() const
{
    uint64_t t = total_allocs.load();
    return t > 0 ? static_cast<double>(fixed_hits.load()) / t * 100.0 : 0.0;
}

/*===========================================================================*/
/* AudioMemoryPool                                                           */
/*===========================================================================*/

AudioMemoryPool::AudioMemoryPool(const MemoryPoolConfig& cfg)
    : cfg_(cfg)
{
    LOG_INFO(TAG, "内存池初始化");
    fixed_ = std::make_unique<FixedPool>(cfg_.fixed_block_count);
    if (cfg_.dynamic_pool_size > 0)
    {
        dynamic_ = std::make_unique<tool::memory::MemoryPool>(
            cfg_.dynamic_pool_size, ALIGN_SIZE, POOL_EXPAND);
        LOG_INFO(TAG, "动态池 %.1fMB", cfg_.dynamic_pool_size / MB);
    }
}

AudioMemoryPool::~AudioMemoryPool()
{
    log_stats();
}

AudioFramePtr AudioMemoryPool::allocate(size_t size)
{
    stats_.total_allocs.fetch_add(1, std::memory_order_relaxed);

    if (size <= cfg_.fixed_block_size)
    {
        auto f = alloc_fixed(size);
        if (f) { stats_.fixed_hits.fetch_add(1, std::memory_order_relaxed); return f; }
    }
    auto f = alloc_dynamic(size);
    if (f) { stats_.dynamic_hits.fetch_add(1, std::memory_order_relaxed); return f; }

    stats_.failures.fetch_add(1, std::memory_order_relaxed);
    LOG_ERROR(TAG, "分配失败 %zu字节", size);
    return nullptr;
}

AudioFramePtr AudioMemoryPool::alloc_fixed(size_t size)
{
    if (size > FixedPool::BLK_SIZE) return nullptr;
    int idx = fixed_->alloc();
    if (idx < 0) return nullptr;

    AudioFrame* f = &fixed_->frames[static_cast<size_t>(idx)];
    f->data      = fixed_->ptr(idx);
    f->capacity  = FixedPool::BLK_SIZE;
    f->size      = size;
    f->timestamp = now_us();

    auto* pool = fixed_.get();
    int   i    = idx;
    return std::shared_ptr<AudioFrame>(f, [pool, i](AudioFrame*) { pool->free(i); });
}

AudioFramePtr AudioMemoryPool::alloc_dynamic(size_t size)
{
    if (!dynamic_) return nullptr;
    void* buf = dynamic_->allocate(size);
    if (!buf) return nullptr;

    auto f       = std::make_shared<AudioFrame>();
    f->data      = static_cast<uint8_t*>(buf);
    f->capacity  = size;
    f->size      = size;
    f->timestamp = now_us();

    auto* pool = dynamic_.get();
    return std::shared_ptr<AudioFrame>(f.get(),
        [pool, f](AudioFrame* p) { if (p->data) pool->deallocate(p->data); });
}

void AudioMemoryPool::get_stats(Stats& out) const
{
    out.fixed_hits.store(stats_.fixed_hits.load());
    out.dynamic_hits.store(stats_.dynamic_hits.load());
    out.total_allocs.store(stats_.total_allocs.load());
    out.failures.store(stats_.failures.load());
}

void AudioMemoryPool::reset_stats()
{
    stats_.fixed_hits.store(0);
    stats_.dynamic_hits.store(0);
    stats_.total_allocs.store(0);
    stats_.failures.store(0);
}

void AudioMemoryPool::log_stats() const
{
    uint64_t t = stats_.total_allocs.load();
    if (t == 0) return;
    LOG_INFO(TAG, "内存池 总=%llu 固定=%llu(%.1f%%) 动态=%llu 失败=%llu",
             t, stats_.fixed_hits.load(), stats_.get_fixed_hit_rate(),
             stats_.dynamic_hits.load(), stats_.failures.load());
}

/*===========================================================================*/
/* AudioSystem::Impl                                                         */
/*===========================================================================*/

class AudioSystem::Impl
{
public:
    AudioConfig cfg;

    std::atomic<bool> inited{false};
    std::atomic<bool> rec_run{false};
    std::atomic<bool> play_run{false};
    std::atomic<int>  volume{VOLUME_DEFAULT};

    std::unique_ptr<AudioMemoryPool> mem;

    OpusEncoderPtr encoder;
    OpusDecoderPtr decoder;
    SrcStatePtr    resampler;
    SpeexStatePtr  speex;

    PaStreamPtr rec_stream;
    PaStreamPtr play_stream;

    std::queue<AudioFramePtr> rec_q;
    std::queue<AudioFramePtr> play_q;
    std::mutex                rec_mtx;
    std::mutex                play_mtx;
    std::condition_variable   rec_cv;

    mutable std::mutex   cb_mtx;
    AudioFrameCallback   rec_cb;

    /* WAV */
    std::unique_ptr<FileWrapper> wav_f;
    std::atomic<bool>            wav_run{false};
    size_t                       wav_len = 0;
    int                          wav_dur = 0;
    std::chrono::steady_clock::time_point wav_t0;
    std::mutex                   wav_mtx;
    int                          wav_id = 0;

    Stats stats;

    alignas(64) std::array<uint8_t, OPUS_MAX_PACKET_SIZE> opus_buf;
    alignas(64) std::array<float, FLOAT_BUF_LEN> f_in;
    alignas(64) std::array<float, FLOAT_BUF_LEN> f_out;

    explicit Impl(const AudioConfig& c) : cfg(c) {}

    static int rec_cb_fn(const void*, void*, unsigned long,
                         const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags, void*);
    static int play_cb_fn(const void*, void*, unsigned long,
                          const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags, void*);
};

/*===========================================================================*/
/* PortAudio回调                                                              */
/*===========================================================================*/

int AudioSystem::Impl::rec_cb_fn(const void* in, void*, unsigned long frames,
                                 const PaStreamCallbackTimeInfo*,
                                 PaStreamCallbackFlags, void* ud)
{
    auto*       impl = static_cast<Impl*>(ud);
    const auto* pcm  = static_cast<const int16_t*>(in);
    size_t      len  = frames * impl->cfg.channels * sizeof(int16_t);

    auto frame = impl->mem->allocate(len);
    if (!frame) return paContinue;

    std::memcpy(frame->data, pcm, len);
    frame->size      = len;
    frame->timestamp = now_us();

    /* 3A处理 */
    if (impl->speex) speex_preprocess_run(impl->speex.get(), frame->get_data<int16_t>());

    /* 入队 */
    {
        std::lock_guard<std::mutex> lk(impl->rec_mtx);
        if (impl->rec_q.size() >= impl->cfg.rec_queue_max)
        {
            impl->rec_q.pop();
            impl->stats.drop_frames.fetch_add(1);
        }
        impl->rec_q.push(frame);
    }
    impl->rec_cv.notify_one();
    impl->stats.rec_frames.fetch_add(1);

    /* WAV写入 */
    if (impl->wav_run.load())
    {
        std::lock_guard<std::mutex> lk(impl->wav_mtx);
        if (impl->wav_f && impl->wav_f->valid())
        {
            if (impl->wav_f->write(frame->data, frame->size))
            {
                impl->wav_len += frame->size;
                impl->wav_f->flush();
                if (impl->wav_dur > 0)
                {
                    auto sec = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::steady_clock::now() - impl->wav_t0).count();
                    if (sec >= impl->wav_dur) impl->wav_run.store(false);
                }
            }
        }
    }

    /* 用户回调 */
    {
        std::lock_guard<std::mutex> lk(impl->cb_mtx);
        if (impl->rec_cb)
        {
            try { impl->rec_cb(frame); }
            catch (const std::exception& e) { LOG_ERROR(TAG, "回调异常 %s", e.what()); }
        }
    }
    return paContinue;
}

int AudioSystem::Impl::play_cb_fn(const void*, void* out, unsigned long frames,
                                  const PaStreamCallbackTimeInfo*,
                                  PaStreamCallbackFlags, void* ud)
{
    auto*   impl = static_cast<Impl*>(ud);
    auto*   dst  = static_cast<int16_t*>(out);
    size_t  need = frames * impl->cfg.channels;

    std::lock_guard<std::mutex> lk(impl->play_mtx);
    if (impl->play_q.empty())
    {
        std::fill_n(dst, need, static_cast<int16_t>(0));
        return paContinue;
    }

    auto&  f    = impl->play_q.front();
    size_t have = f->size / sizeof(int16_t);
    size_t copy = std::min(need, have);

    int   v = impl->volume.load(std::memory_order_relaxed);
    float g = static_cast<float>(v) * VOL_GAIN_K;
    const int16_t* src = f->get_data<int16_t>();
    for (size_t i = 0; i < copy; ++i) dst[i] = static_cast<int16_t>(src[i] * g);
    if (copy < need) std::fill_n(dst + copy, need - copy, static_cast<int16_t>(0));

    impl->play_q.pop();
    impl->stats.play_frames.fetch_add(1);
    return paContinue;
}

/*===========================================================================*/
/* AudioSystem                                                               */
/*===========================================================================*/

AudioSystem::AudioSystem(const AudioConfig& cfg)
    : impl_(std::make_unique<Impl>(cfg))
{
    LOG_DEBUG(TAG, "创建");
}

AudioSystem::~AudioSystem()
{
    deinit();
    LOG_DEBUG(TAG, "销毁");
}

AudioError AudioSystem::init()
{
    if (impl_->inited.load())
    {
        LOG_WARN(TAG, "重复初始化");
        return AudioError::ALREADY_INIT;
    }
    LOG_INFO(TAG, "初始化 fs=%dHz ch=%d", impl_->cfg.sample_rate, impl_->cfg.channels);

    /* 内存池 */
    impl_->mem = std::make_unique<AudioMemoryPool>(impl_->cfg.mem_cfg);
    create_directory(impl_->cfg.wav_path);

    /* PortAudio */
    PaError err = Pa_Initialize();
    if (err != paNoError)
    {
        LOG_ERROR(TAG, "PA初始化失败 %s", Pa_GetErrorText(err));
        return AudioError::STREAM_OPEN_ERR;
    }
    LOG_INFO(TAG, "PA %s", Pa_GetVersionText());

    int oe = 0;

    /* Opus编码器 */
    OpusEncoder* enc = opus_encoder_create(impl_->cfg.sample_rate,
                                           impl_->cfg.channels,
                                           OPUS_APPLICATION_VOIP, &oe);
    if (oe == OPUS_OK && enc)
    {
        impl_->encoder.reset(enc);
        opus_encoder_ctl(enc, OPUS_SET_BITRATE(impl_->cfg.opus_bitrate));
        opus_encoder_ctl(enc, OPUS_SET_VBR(1));
        opus_encoder_ctl(enc, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
        LOG_INFO(TAG, "Opus编码 %dkbps", impl_->cfg.opus_bitrate / 1000);
    }
    else
    {
        LOG_WARN(TAG, "Opus编码器创建失败");
    }

    /* Opus解码器 */
    OpusDecoder* dec = opus_decoder_create(impl_->cfg.sample_rate, impl_->cfg.channels, &oe);
    if (oe == OPUS_OK && dec)
    {
        impl_->decoder.reset(dec);
        LOG_INFO(TAG, "Opus解码 %dHz", impl_->cfg.sample_rate);
    }
    else
    {
        LOG_WARN(TAG, "Opus解码器创建失败");
    }

    /* 重采样 */
    if (impl_->cfg.resample_en)
    {
        int se = 0;
        SRC_STATE* src = src_new(SRC_SINC_FASTEST, impl_->cfg.channels, &se);
        if (src)
        {
            impl_->resampler.reset(src);
            LOG_INFO(TAG, "重采样 %d->%dHz", impl_->cfg.sample_rate, impl_->cfg.resample_rate);
        }
    }

    /* Speex 3A */
    if (impl_->cfg.denoise_en || impl_->cfg.agc_en)
    {
        int fsize = impl_->cfg.sample_rate * impl_->cfg.frame_ms / MS_PER_S;
        SpeexPreprocessState* sp = speex_preprocess_state_init(fsize, impl_->cfg.sample_rate);
        if (sp)
        {
            impl_->speex.reset(sp);
            int dns = impl_->cfg.denoise_en ? 1 : 0;
            int agc = impl_->cfg.agc_en ? 1 : 0;
            speex_preprocess_ctl(sp, SPEEX_PREPROCESS_SET_DENOISE, &dns);
            speex_preprocess_ctl(sp, SPEEX_PREPROCESS_SET_AGC, &agc);
            if (agc)
            {
                float lv = impl_->cfg.agc_level;
                speex_preprocess_ctl(sp, SPEEX_PREPROCESS_SET_AGC_LEVEL, &lv);
            }
            LOG_INFO(TAG, "3A DNS=%d AGC=%d", dns, agc);
        }
    }

    impl_->volume.store(impl_->cfg.volume);
    impl_->inited.store(true);
    LOG_INFO(TAG, "初始化完成");
    return AudioError::NONE;
}

void AudioSystem::deinit()
{
    if (!impl_->inited.load()) return;
    LOG_INFO(TAG, "关闭");

    stop_wav();
    stop_stream(StreamDirection::INPUT);
    stop_stream(StreamDirection::OUTPUT);
    clear_rec_queue();
    clear_play_queue();

    Pa_Terminate();
    if (impl_->mem) impl_->mem->log_stats();

    impl_->inited.store(false);
    LOG_INFO(TAG, "已关闭");
}

bool AudioSystem::is_init() const { return impl_->inited.load(); }

AudioError AudioSystem::start_stream(StreamDirection dir)
{
    if (!impl_->inited.load()) return AudioError::NOT_INIT;

    PaStreamParameters p{};
    PaStream* s = nullptr;
    PaError   e;
    int fpp = impl_->cfg.sample_rate * impl_->cfg.frame_ms / MS_PER_S;

    if (dir == StreamDirection::INPUT)
    {
        if (impl_->rec_run.load()) return AudioError::ALREADY_RUNNING;
        p.device = Pa_GetDefaultInputDevice();
        if (p.device == paNoDevice) { LOG_ERROR(TAG, "无输入设备"); return AudioError::NO_DEVICE; }
        p.channelCount = impl_->cfg.channels;
        p.sampleFormat = paInt16;
        p.suggestedLatency = Pa_GetDeviceInfo(p.device)->defaultLowInputLatency;
        p.hostApiSpecificStreamInfo = nullptr;
        e = Pa_OpenStream(&s, &p, nullptr, impl_->cfg.sample_rate, fpp,
                          paClipOff, Impl::rec_cb_fn, impl_.get());
    }
    else
    {
        if (impl_->play_run.load()) return AudioError::ALREADY_RUNNING;
        p.device = Pa_GetDefaultOutputDevice();
        if (p.device == paNoDevice) { LOG_ERROR(TAG, "无输出设备"); return AudioError::NO_DEVICE; }
        p.channelCount = impl_->cfg.channels;
        p.sampleFormat = paInt16;
        p.suggestedLatency = Pa_GetDeviceInfo(p.device)->defaultLowOutputLatency;
        p.hostApiSpecificStreamInfo = nullptr;
        e = Pa_OpenStream(&s, nullptr, &p, impl_->cfg.sample_rate, fpp,
                          paClipOff, Impl::play_cb_fn, impl_.get());
    }

    if (e != paNoError)
    {
        LOG_ERROR(TAG, "打开流失败 %s", Pa_GetErrorText(e));
        return AudioError::STREAM_OPEN_ERR;
    }
    e = Pa_StartStream(s);
    if (e != paNoError)
    {
        LOG_ERROR(TAG, "启动流失败 %s", Pa_GetErrorText(e));
        Pa_CloseStream(s);
        return AudioError::STREAM_START_ERR;
    }

    if (dir == StreamDirection::INPUT)
    {
        impl_->rec_stream.reset(s);
        impl_->rec_run.store(true);
        LOG_INFO(TAG, "采集启动");
    }
    else
    {
        impl_->play_stream.reset(s);
        impl_->play_run.store(true);
        LOG_INFO(TAG, "播放启动");
    }
    return AudioError::NONE;
}

AudioError AudioSystem::stop_stream(StreamDirection dir)
{
    if (dir == StreamDirection::INPUT)
    {
        if (!impl_->rec_run.load()) return AudioError::NONE;
        impl_->rec_run.store(false);
        impl_->rec_stream.reset();
        LOG_INFO(TAG, "采集停止");
    }
    else
    {
        if (!impl_->play_run.load()) return AudioError::NONE;
        impl_->play_run.store(false);
        impl_->play_stream.reset();
        LOG_INFO(TAG, "播放停止");
    }
    return AudioError::NONE;
}

bool AudioSystem::stream_running(StreamDirection dir) const
{
    return dir == StreamDirection::INPUT ? impl_->rec_run.load() : impl_->play_run.load();
}

AudioFramePtr AudioSystem::get_frame(std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lk(impl_->rec_mtx);
    if (impl_->rec_q.empty())
    {
        impl_->rec_cv.wait_for(lk, timeout, [this]() {
            return !impl_->rec_q.empty() || !impl_->rec_run.load();
        });
    }
    if (impl_->rec_q.empty()) return nullptr;
    auto f = std::move(impl_->rec_q.front());
    impl_->rec_q.pop();
    return f;
}

void AudioSystem::clear_rec_queue()
{
    std::lock_guard<std::mutex> lk(impl_->rec_mtx);
    std::queue<AudioFramePtr> q;
    std::swap(impl_->rec_q, q);
}

void AudioSystem::push_frame(AudioFramePtr f)
{
    if (!f) return;
    std::lock_guard<std::mutex> lk(impl_->play_mtx);
    if (impl_->play_q.size() >= impl_->cfg.play_queue_max)
    {
        impl_->play_q.pop();
        impl_->stats.drop_frames.fetch_add(1);
    }
    impl_->play_q.push(std::move(f));
}

void AudioSystem::clear_play_queue()
{
    std::lock_guard<std::mutex> lk(impl_->play_mtx);
    std::queue<AudioFramePtr> q;
    std::swap(impl_->play_q, q);
}

AudioError AudioSystem::start_wav(const std::string& path, int sec)
{
    if (!impl_->inited.load()) return AudioError::NOT_INIT;

    std::lock_guard<std::mutex> lk(impl_->wav_mtx);
    if (impl_->wav_run.load()) return AudioError::ALREADY_RUNNING;

    std::string fn = path.empty()
        ? impl_->cfg.wav_path + "rec_" + std::to_string(impl_->wav_id++) + ".wav"
        : path;
    if (fn.size() < 4 || fn.substr(fn.size() - 4) != ".wav") fn += ".wav";

    impl_->wav_f = std::make_unique<FileWrapper>(fn, FileMode::WRITE);
    if (!impl_->wav_f->valid())
    {
        LOG_ERROR(TAG, "WAV创建失败 %s", fn.c_str());
        impl_->wav_f.reset();
        return AudioError::FILE_ERR;
    }

    if (!wav_write_header(*impl_->wav_f, impl_->cfg.sample_rate, impl_->cfg.channels, 0))
    {
        impl_->wav_f.reset();
        return AudioError::FILE_ERR;
    }

    impl_->wav_dur = sec;
    impl_->wav_t0  = std::chrono::steady_clock::now();
    impl_->wav_len = 0;
    impl_->wav_run.store(true);
    LOG_INFO(TAG, "WAV录制开始 %s", fn.c_str());
    return AudioError::NONE;
}

AudioError AudioSystem::stop_wav()
{
    std::lock_guard<std::mutex> lk(impl_->wav_mtx);
    if (!impl_->wav_run.load()) return AudioError::NONE;

    impl_->wav_run.store(false);
    if (impl_->wav_f)
    {
        impl_->wav_f->flush();
        if (impl_->wav_len > 0)
        {
            wav_update_header(*impl_->wav_f, impl_->wav_len);
            impl_->wav_f->flush();
        }
        LOG_INFO(TAG, "WAV录制停止 %zu字节", impl_->wav_len);
        impl_->wav_f.reset();
        impl_->wav_len = 0;
    }
    return AudioError::NONE;
}

bool AudioSystem::wav_running() const { return impl_->wav_run.load(); }

AudioFramePtr AudioSystem::opus_encode(const int16_t* pcm, size_t n)
{
    if (!impl_->encoder) return nullptr;

    auto f = impl_->mem->allocate(OPUS_MAX_PACKET_SIZE);
    if (!f) return nullptr;

    int enc = ::opus_encode(impl_->encoder.get(), pcm, static_cast<int>(n),
                            f->data, static_cast<int>(f->capacity));
    if (enc < 0) return nullptr;

    f->size = static_cast<size_t>(enc);
    impl_->stats.enc_cnt.fetch_add(1);
    return f;
}

AudioFramePtr AudioSystem::opus_decode(const uint8_t* data, size_t len)
{
    if (!impl_->decoder) return nullptr;

    int    fsize = impl_->cfg.sample_rate * impl_->cfg.frame_ms / MS_PER_S;
    size_t bytes = static_cast<size_t>(fsize) * impl_->cfg.channels * sizeof(int16_t);

    auto f = impl_->mem->allocate(bytes);
    if (!f) return nullptr;

    int dec = ::opus_decode(impl_->decoder.get(), data, static_cast<int32_t>(len),
                            f->get_data<int16_t>(), fsize, 0);
    if (dec < 0) return nullptr;

    f->size = static_cast<size_t>(dec) * impl_->cfg.channels * sizeof(int16_t);
    impl_->stats.dec_cnt.fetch_add(1);
    return f;
}

AudioFramePtr AudioSystem::resample(const int16_t* in, size_t n, int src_fs, int dst_fs)
{
    if (!impl_->resampler || src_fs == dst_fs) return nullptr;

    double ratio = static_cast<double>(dst_fs) / static_cast<double>(src_fs);
    size_t out_n = static_cast<size_t>(n * ratio) + 16;

    std::vector<float> fi(n), fo(out_n);
    for (size_t i = 0; i < n; ++i) fi[i] = static_cast<float>(in[i]) / PCM_NORM;

    SRC_DATA d{};
    d.data_in       = fi.data();
    d.input_frames  = static_cast<long>(n);
    d.data_out      = fo.data();
    d.output_frames = static_cast<long>(out_n);
    d.src_ratio     = ratio;
    d.end_of_input  = 0;

    if (src_process(impl_->resampler.get(), &d) != 0) return nullptr;

    size_t bytes = static_cast<size_t>(d.output_frames_gen) * sizeof(int16_t);
    auto   f     = impl_->mem->allocate(bytes);
    if (!f) return nullptr;

    int16_t* po = f->get_data<int16_t>();
    for (long i = 0; i < d.output_frames_gen; ++i)
    {
        float v = std::clamp(fo[static_cast<size_t>(i)], -1.0f, 1.0f);
        po[i]   = static_cast<int16_t>(v * 32767.0f);
    }
    f->size = bytes;
    return f;
}

void AudioSystem::set_volume(int v)
{
    v = std::clamp(v, VOLUME_MIN, VOLUME_MAX);
    impl_->volume.store(v, std::memory_order_relaxed);
    LOG_DEBUG(TAG, "音量 %d%%", v);
}

int AudioSystem::get_volume() const { return impl_->volume.load(std::memory_order_relaxed); }

void AudioSystem::set_callback(AudioFrameCallback cb)
{
    std::lock_guard<std::mutex> lk(impl_->cb_mtx);
    impl_->rec_cb = std::move(cb);
}

void AudioSystem::get_stats(Stats& out) const
{
    if (impl_->mem) impl_->mem->get_stats(out.mem);
    out.rec_frames.store(impl_->stats.rec_frames.load());
    out.play_frames.store(impl_->stats.play_frames.load());
    out.drop_frames.store(impl_->stats.drop_frames.load());
    out.enc_cnt.store(impl_->stats.enc_cnt.load());
    out.dec_cnt.store(impl_->stats.dec_cnt.load());
}

void AudioSystem::reset_stats()
{
    impl_->stats.rec_frames.store(0);
    impl_->stats.play_frames.store(0);
    impl_->stats.drop_frames.store(0);
    impl_->stats.enc_cnt.store(0);
    impl_->stats.dec_cnt.store(0);
    if (impl_->mem) impl_->mem->reset_stats();
}

void AudioSystem::log_stats() const
{
    LOG_INFO(TAG, "录制=%llu 播放=%llu 丢弃=%llu 编码=%llu 解码=%llu",
             impl_->stats.rec_frames.load(), impl_->stats.play_frames.load(),
             impl_->stats.drop_frames.load(), impl_->stats.enc_cnt.load(),
             impl_->stats.dec_cnt.load());
    if (impl_->mem) impl_->mem->log_stats();
}

} // namespace app::media::audio
