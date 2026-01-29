/**
 * @file    audio.hpp
 * @brief   音频子系统
 * @author  Core_Master
 * @note    提供PCM采集/播放、Opus编解码、重采样、3A处理
 */

#ifndef AUDIO_HPP
#define AUDIO_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

/*===========================================================================*/
/* 外部库声明                                                                 */
/*===========================================================================*/

#if __has_include(<samplerate.h>)
#include <samplerate.h>
#else
extern "C"
{
    struct SRC_STATE;
    struct SRC_DATA
    {
        const float* data_in;
        float*       data_out;
        long         input_frames;
        long         output_frames;
        long         input_frames_used;
        long         output_frames_gen;
        int          end_of_input;
        double       src_ratio;
    };
#define SRC_SINC_BEST_QUALITY 0
#define SRC_SINC_FASTEST      2
    SRC_STATE*  src_new(int type, int ch, int* err);
    int         src_process(SRC_STATE* st, SRC_DATA* d);
    SRC_STATE*  src_delete(SRC_STATE* st);
    const char* src_strerror(int err);
}
#endif

#if __has_include(<opus/opus.h>)
#include <opus/opus.h>
#else
extern "C"
{
    struct OpusEncoder;
    struct OpusDecoder;
#define OPUS_OK               0
#define OPUS_APPLICATION_VOIP 2048
#define OPUS_SET_BITRATE(x)   4002, (x)
#define OPUS_SET_VBR(x)       4006, (x)
#define OPUS_SET_SIGNAL(x)    4024, (x)
#define OPUS_SIGNAL_VOICE     3001
    OpusEncoder* opus_encoder_create(int32_t fs, int ch, int app, int* err);
    int          opus_encode(OpusEncoder* st, const int16_t* pcm, int n, uint8_t* out, int32_t max);
    int          opus_encoder_ctl(OpusEncoder* st, int req, ...);
    void         opus_encoder_destroy(OpusEncoder* st);
    OpusDecoder* opus_decoder_create(int32_t fs, int ch, int* err);
    int          opus_decode(OpusDecoder* st, const uint8_t* in, int32_t len, int16_t* pcm, int n, int fec);
    void         opus_decoder_destroy(OpusDecoder* st);
}
#endif

#if __has_include(<portaudio.h>)
#include <portaudio.h>
#else
extern "C"
{
    typedef void          PaStream;
    typedef int           PaError;
    typedef int           PaDeviceIndex;
    typedef unsigned long PaStreamCallbackFlags;
    typedef double        PaTime;
    typedef unsigned long PaSampleFormat;

    constexpr PaSampleFormat        paInt16    = 0x00000008;
    constexpr PaStreamCallbackFlags paClipOff  = 0x00000001;
    constexpr PaDeviceIndex         paNoDevice = -1;
    constexpr PaError               paNoError  = 0;
    constexpr int                   paContinue = 0;

    struct PaStreamParameters
    {
        PaDeviceIndex  device;
        int            channelCount;
        PaSampleFormat sampleFormat;
        PaTime         suggestedLatency;
        void*          hostApiSpecificStreamInfo;
    };
    struct PaStreamCallbackTimeInfo
    {
        PaTime inputBufferAdcTime;
        PaTime currentTime;
        PaTime outputBufferDacTime;
    };
    struct PaDeviceInfo
    {
        const char* name;
        PaTime      defaultLowInputLatency;
        PaTime      defaultLowOutputLatency;
    };

    typedef int (*PaStreamCallback)(const void*, void*, unsigned long,
                                    const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags, void*);

    PaError             Pa_Initialize();
    PaError             Pa_Terminate();
    const char*         Pa_GetVersionText();
    const char*         Pa_GetErrorText(PaError err);
    PaDeviceIndex       Pa_GetDefaultInputDevice();
    PaDeviceIndex       Pa_GetDefaultOutputDevice();
    const PaDeviceInfo* Pa_GetDeviceInfo(PaDeviceIndex idx);
    PaError             Pa_OpenStream(PaStream**, const PaStreamParameters*, const PaStreamParameters*,
                                      double, unsigned long, PaStreamCallbackFlags, PaStreamCallback, void*);
    PaError             Pa_StartStream(PaStream*);
    PaError             Pa_StopStream(PaStream*);
    PaError             Pa_CloseStream(PaStream*);
}
#endif

#if __has_include(<speex/speex_preprocess.h>)
#include <speex/speex_preprocess.h>
#else
extern "C"
{
    struct SpeexPreprocessState;
    typedef int16_t spx_int16_t;
#define SPEEX_PREPROCESS_SET_DENOISE   0
#define SPEEX_PREPROCESS_SET_AGC       2
#define SPEEX_PREPROCESS_SET_AGC_LEVEL 6
    SpeexPreprocessState* speex_preprocess_state_init(int frame, int fs);
    int                   speex_preprocess_ctl(SpeexPreprocessState* st, int req, void* ptr);
    int                   speex_preprocess_run(SpeexPreprocessState* st, spx_int16_t* x);
    void                  speex_preprocess_state_destroy(SpeexPreprocessState* st);
}
#endif

#include "../../tool/memory/memory.hpp"

namespace app::media::audio
{

/*===========================================================================*/
/* 常量定义                                                                   */
/*===========================================================================*/

constexpr int    AUDIO_SAMPLE_RATE_48K  = 48000;
constexpr int    AUDIO_SAMPLE_RATE_16K  = 16000;
constexpr int    AUDIO_CHANNELS_MONO    = 1;
constexpr int    AUDIO_FRAME_MS_20      = 20;
constexpr int    OPUS_BITRATE_64K       = 64000;
constexpr size_t OPUS_MAX_PACKET_SIZE   = 4096;
constexpr size_t MEM_FIXED_BLOCK_SIZE   = 4096;
constexpr size_t MEM_FIXED_BLOCK_COUNT  = 256;
constexpr size_t MEM_DYNAMIC_POOL_SIZE  = 2 * 1024 * 1024;
constexpr size_t AUDIO_QUEUE_MAX_DEPTH  = 300;
constexpr int    VOLUME_MIN             = 0;
constexpr int    VOLUME_MAX             = 100;
constexpr int    VOLUME_DEFAULT         = 50;

/*===========================================================================*/
/* 错误码                                                                     */
/*===========================================================================*/

enum class AudioError
{
    NONE = 0,           // 无错误
    NOT_INIT,           // 未初始化
    ALREADY_INIT,       // 重复初始化
    ALREADY_RUNNING,    // 流已运行
    NO_DEVICE,          // 无可用设备
    STREAM_OPEN_ERR,    // 流打开失败
    STREAM_START_ERR,   // 流启动失败
    ENCODE_ERR,         // 编码失败
    DECODE_ERR,         // 解码失败
    ALLOC_ERR,          // 内存分配失败
    FILE_ERR,           // 文件操作失败
    PARAM_ERR           // 参数错误
};

/*===========================================================================*/
/* 流方向                                                                     */
/*===========================================================================*/

enum class StreamDirection
{
    INPUT,   // 采集
    OUTPUT   // 播放
};

/*===========================================================================*/
/* 资源释放器                                                                 */
/*===========================================================================*/

struct OpusEncoderDeleter { void operator()(OpusEncoder* p) const { if (p) opus_encoder_destroy(p); } };
struct OpusDecoderDeleter { void operator()(OpusDecoder* p) const { if (p) opus_decoder_destroy(p); } };
struct SrcStateDeleter    { void operator()(SRC_STATE* p)   const { if (p) src_delete(p); } };
struct SpeexStateDeleter  { void operator()(SpeexPreprocessState* p) const { if (p) speex_preprocess_state_destroy(p); } };
struct PaStreamDeleter    { void operator()(PaStream* p)    const { if (p) { Pa_StopStream(p); Pa_CloseStream(p); } } };

using OpusEncoderPtr = std::unique_ptr<OpusEncoder, OpusEncoderDeleter>;
using OpusDecoderPtr = std::unique_ptr<OpusDecoder, OpusDecoderDeleter>;
using SrcStatePtr    = std::unique_ptr<SRC_STATE, SrcStateDeleter>;
using SpeexStatePtr  = std::unique_ptr<SpeexPreprocessState, SpeexStateDeleter>;
using PaStreamPtr    = std::unique_ptr<PaStream, PaStreamDeleter>;

/*===========================================================================*/
/* 音频帧                                                                     */
/*===========================================================================*/

struct AudioFrame
{
    uint8_t* data      = nullptr;   // 数据指针
    size_t   capacity  = 0;         // 容量(字节)
    size_t   size      = 0;         // 有效长度(字节)
    uint64_t timestamp = 0;         // 时间戳(us)

    template <typename T = int16_t> T*       get_data()       { return reinterpret_cast<T*>(data); }
    template <typename T = int16_t> const T* get_data() const { return reinterpret_cast<const T*>(data); }
    size_t get_sample_count() const { return size / sizeof(int16_t); }
};

using AudioFramePtr = std::shared_ptr<AudioFrame>;

/*===========================================================================*/
/* 内存池配置                                                                 */
/*===========================================================================*/

struct MemoryPoolConfig
{
    size_t fixed_block_size  = MEM_FIXED_BLOCK_SIZE;
    size_t fixed_block_count = MEM_FIXED_BLOCK_COUNT;
    size_t dynamic_pool_size = MEM_DYNAMIC_POOL_SIZE;
};

/*===========================================================================*/
/* 音频内存池                                                                 */
/*===========================================================================*/

class AudioMemoryPool
{
public:
    explicit AudioMemoryPool(const MemoryPoolConfig& cfg);
    ~AudioMemoryPool();

    AudioMemoryPool(const AudioMemoryPool&)            = delete;
    AudioMemoryPool& operator=(const AudioMemoryPool&) = delete;

    AudioFramePtr allocate(size_t size);

    struct Stats
    {
        std::atomic<uint64_t> fixed_hits{0};
        std::atomic<uint64_t> dynamic_hits{0};
        std::atomic<uint64_t> total_allocs{0};
        std::atomic<uint64_t> failures{0};
        double get_fixed_hit_rate() const;
    };

    void get_stats(Stats& out) const;
    void reset_stats();
    void log_stats() const;

private:
    struct FixedPool;
    MemoryPoolConfig                          cfg_;
    std::unique_ptr<FixedPool>                fixed_;
    std::unique_ptr<tool::memory::MemoryPool> dynamic_;
    Stats                                     stats_;

    AudioFramePtr alloc_fixed(size_t size);
    AudioFramePtr alloc_dynamic(size_t size);
};

/*===========================================================================*/
/* 音频系统配置                                                               */
/*===========================================================================*/

struct AudioConfig
{
    /* 基本参数 */
    int sample_rate       = AUDIO_SAMPLE_RATE_48K;
    int channels          = AUDIO_CHANNELS_MONO;
    int frame_ms          = AUDIO_FRAME_MS_20;

    /* Opus编码 */
    int opus_bitrate      = OPUS_BITRATE_64K;

    /* 3A算法 */
    bool  denoise_en      = true;
    bool  agc_en          = true;
    float agc_level       = 8000.0f;

    /* 重采样 */
    bool  resample_en     = false;
    int   resample_rate   = AUDIO_SAMPLE_RATE_16K;

    /* 音量 */
    int volume            = VOLUME_DEFAULT;

    /* 队列深度 */
    size_t rec_queue_max  = AUDIO_QUEUE_MAX_DEPTH;
    size_t play_queue_max = AUDIO_QUEUE_MAX_DEPTH;

    /* 录音路径 */
    std::string wav_path  = "/tmp/audio/";

    /* 内存池 */
    MemoryPoolConfig mem_cfg;
};

/*===========================================================================*/
/* 回调                                                                       */
/*===========================================================================*/

using AudioFrameCallback = std::function<void(AudioFramePtr)>;

/*===========================================================================*/
/* 音频系统                                                                   */
/*===========================================================================*/

class AudioSystem
{
public:
    explicit AudioSystem(const AudioConfig& cfg = AudioConfig());
    ~AudioSystem();

    AudioSystem(const AudioSystem&)            = delete;
    AudioSystem& operator=(const AudioSystem&) = delete;

    /* 初始化/销毁 */
    AudioError init();
    void       deinit();
    bool       is_init() const;

    /* 流控制 */
    AudioError start_stream(StreamDirection dir);
    AudioError stop_stream(StreamDirection dir);
    bool       stream_running(StreamDirection dir) const;

    /* 采集队列 */
    AudioFramePtr get_frame(std::chrono::milliseconds timeout = std::chrono::milliseconds(100));
    void          clear_rec_queue();

    /* 播放队列 */
    void push_frame(AudioFramePtr frame);
    void clear_play_queue();

    /* WAV录制 */
    AudioError start_wav(const std::string& path = "", int sec = 0);
    AudioError stop_wav();
    bool       wav_running() const;

    /* Opus编解码 */
    AudioFramePtr opus_encode(const int16_t* pcm, size_t samples);
    AudioFramePtr opus_decode(const uint8_t* data, size_t len);

    /* 重采样 */
    AudioFramePtr resample(const int16_t* in, size_t samples, int src_fs, int dst_fs);

    /* 音量 */
    void set_volume(int vol);
    int  get_volume() const;

    /* 回调 */
    void set_callback(AudioFrameCallback cb);

    /* 统计 */
    struct Stats
    {
        AudioMemoryPool::Stats mem;
        std::atomic<uint64_t>  rec_frames{0};
        std::atomic<uint64_t>  play_frames{0};
        std::atomic<uint64_t>  drop_frames{0};
        std::atomic<uint64_t>  enc_cnt{0};
        std::atomic<uint64_t>  dec_cnt{0};
    };

    void get_stats(Stats& out) const;
    void reset_stats();
    void log_stats() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace app::media::audio

#endif // AUDIO_HPP
