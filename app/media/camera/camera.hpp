/**
 * @file camera.hpp
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <atomic>
#include <mutex>
#include <thread>
#include <array>

// sample_comm.h
#if __has_include("sample_comm.h")
#define CAMERA_HAS_SAMPLE_COMM 1
#include "sample_comm.h"
#elif __has_include("rkmedia/sample_comm.h")
#define CAMERA_HAS_SAMPLE_COMM 1
#include "rkmedia/sample_comm.h"
#else
#define CAMERA_HAS_SAMPLE_COMM 0
#endif

#if !CAMERA_HAS_SAMPLE_COMM
extern "C"
{
    using RK_S32                           = int;
    using RK_U32                           = unsigned int;
    using RK_U64                           = unsigned long long;
    using RK_BOOL                          = int;
    constexpr RK_S32  RK_SUCCESS           = 0;
    constexpr RK_S32  RK_ERR_VI_NOT_CONFIG = -1;
    constexpr RK_S32  RK_ERR_VENC_BUSY     = -2;
    constexpr RK_BOOL RK_TRUE              = 1;
    constexpr RK_BOOL RK_FALSE             = 0;
    using MB_BLK                           = void*;

    struct VI_DEV_ATTR_S
    {
        int reserved{};
    };

    struct VI_DEV_BIND_PIPE_S
    {
        RK_U32 u32Num    = 0;
        RK_S32 PipeId[4] = {0};
    };

    struct VI_CHN_ATTR_S
    {
        struct
        {
            RK_U32 u32BufCount = 0;
            int    enMemoryType{0};
        } stIspOpt;
        struct
        {
            RK_U32 u32Width  = 0;
            RK_U32 u32Height = 0;
        } stSize;
        int    enPixelFormat{0};
        int    enCompressMode{0};
        RK_U32 u32Depth = 0;
    };

    struct VENC_ATTR_JPEGE_S
    {
        bool bSupportDCF{false};
        struct
        {
            unsigned char u8LargeThumbNailNum{0};
        } stMPFCfg;
        int enReceiveMode{0};
    };

    struct VENC_ATTR_S
    {
        int               enType{0};
        int               enPixelFormat{0};
        RK_U32            u32PicWidth{0};
        RK_U32            u32PicHeight{0};
        RK_U32            u32VirWidth{0};
        RK_U32            u32VirHeight{0};
        RK_U32            u32BufSize{0};
        RK_U32            u32StreamBufCnt{0};
        int               enMirror{0};
        RK_U32            u32Profile{0};
        VENC_ATTR_JPEGE_S stAttrJpege{};
    };

    struct VENC_RC_ATTR_S
    {
        int enRcMode{0};
        struct
        {
            RK_U32 u32BitRate{0};
            RK_U32 u32Gop{0};
        } stH264Cbr;
        struct
        {
            RK_U32 u32BitRate{0};
            RK_U32 u32Gop{0};
        } stH265Cbr;
        struct
        {
            RK_U32 u32Qfactor{0};
        } stMjpegFixQp;
    };

    struct VENC_CHN_ATTR_S
    {
        VENC_ATTR_S    stVencAttr{};
        VENC_RC_ATTR_S stRcAttr{};
    };

    struct VENC_PACK_S
    {
        RK_U32 u32Len{0};
        MB_BLK pMbBlk{nullptr};
        struct
        {
            int enH264EType{0};
            int enH265EType{0};
        } DataType;
        RK_U64 u64PTS{0};
    };

    struct VENC_STREAM_S
    {
        VENC_PACK_S* pstPack{nullptr};
        RK_U32       u32PackCount{0};
    };

    struct VENC_RECV_PIC_PARAM_S
    {
        RK_S32 s32RecvPicNum{0};
    };

    struct MPP_CHN_S
    {
        int    enModId{0};
        RK_S32 s32DevId{0};
        RK_S32 s32ChnId{0};
    };

    struct VIDEO_FRAME_INFO_S
    {
        struct
        {
            MB_BLK pMbBlk{nullptr};
            RK_U32 u32Width{0};
            RK_U32 u32Height{0};
            RK_U32 u32VirWidth{0};
            RK_U32 u32VirHeight{0};
            int    enPixelFormat{0};
            RK_U64 u64PTS{0};
        } stVFrame;
    };

    enum
    {
        RK_ID_VI   = 0,
        RK_ID_VENC = 1
    };

    enum
    {
        RK_FMT_YUV420SP = 0
    };

    enum
    {
        COMPRESS_MODE_NONE = 0
    };

    enum
    {
        VI_V4L2_MEMORY_TYPE_DMABUF = 0
    };

    enum
    {
        MIRROR_NONE = 0
    };

    enum RK_CODEC_ID_E
    {
        RK_VIDEO_ID_AVC   = 0,
        RK_VIDEO_ID_HEVC  = 1,
        RK_VIDEO_ID_MJPEG = 2
    };

    enum
    {
        VENC_RC_MODE_H264CBR    = 0,
        VENC_RC_MODE_H265CBR    = 1,
        VENC_RC_MODE_MJPEGFIXQP = 2
    };

    enum
    {
        VENC_PIC_RECEIVE_SINGLE = 0
    };

    enum
    {
        H264E_PROFILE_BASELINE = 0
    };

    enum
    {
        H264E_NALU_IDRSLICE = 5,
        H265E_NALU_IDRSLICE = 19
    };

    int   RK_MPI_SYS_Init();
    int   RK_MPI_SYS_Exit();
    int   RK_MPI_SYS_Malloc(MB_BLK* blk, RK_U32 size);
    void* RK_MPI_MB_Handle2VirAddr(MB_BLK blk);
    int   RK_MPI_SYS_Free(MB_BLK blk);
    int   RK_MPI_VI_GetDevAttr(int devId, VI_DEV_ATTR_S* attr);
    int   RK_MPI_VI_SetDevAttr(int devId, const VI_DEV_ATTR_S* attr);
    int   RK_MPI_VI_GetDevIsEnable(int devId);
    int   RK_MPI_VI_EnableDev(int devId);
    int   RK_MPI_VI_DisableDev(int devId);
    int   RK_MPI_VI_SetDevBindPipe(int devId, const VI_DEV_BIND_PIPE_S* pipe);
    int   RK_MPI_VI_SetChnAttr(int devId, int chnId, const VI_CHN_ATTR_S* attr);
    int   RK_MPI_VI_EnableChn(int devId, int chnId);
    int   RK_MPI_VI_DisableChn(int devId, int chnId);
    int   RK_MPI_VENC_CreateChn(int chnId, const VENC_CHN_ATTR_S* attr);
    int   RK_MPI_VENC_DestroyChn(int chnId);
    int   RK_MPI_VENC_StartRecvFrame(int chnId, const VENC_RECV_PIC_PARAM_S* param);
    int   RK_MPI_VENC_StopRecvFrame(int chnId);
    int   RK_MPI_VENC_GetStream(int chnId, VENC_STREAM_S* stream, int timeout);
    int   RK_MPI_VENC_ReleaseStream(int chnId, VENC_STREAM_S* stream);
    int   RK_MPI_VENC_GetChnAttr(int chnId, VENC_CHN_ATTR_S* attr);
    int   RK_MPI_VENC_SetChnAttr(int chnId, const VENC_CHN_ATTR_S* attr);
    int   RK_MPI_SYS_Bind(const MPP_CHN_S* src, const MPP_CHN_S* dest);
    int   RK_MPI_SYS_UnBind(const MPP_CHN_S* src, const MPP_CHN_S* dest);
    int RK_MPI_VI_GetChnFrame(int devId, int chnId, VIDEO_FRAME_INFO_S* pstFrameInfo, int timeout);
    int RK_MPI_VI_ReleaseChnFrame(int devId, int chnId, VIDEO_FRAME_INFO_S* pstFrameInfo);
} // extern "C"
#endif

#if !CAMERA_HAS_SAMPLE_COMM
#undef CAMERA_HAS_SAMPLE_COMM
#define CAMERA_HAS_SAMPLE_COMM 0
#else
#undef CAMERA_HAS_SAMPLE_COMM
#define CAMERA_HAS_SAMPLE_COMM 1
#endif

// RKAIQ 头文件
#if __has_include("rkaiq/uAPI2/rk_aiq_user_api2_imgproc.h")
#include "rkaiq/uAPI2/rk_aiq_user_api2_imgproc.h"
#include "rkaiq/uAPI2/rk_aiq_user_api2_sysctl.h"
#else
extern "C"
{
    struct rk_aiq_sys_ctx_s
    {
        int reserved{};
    };
    using rk_aiq_sys_ctx_t = rk_aiq_sys_ctx_s;

    struct rk_aiq_sensor_info_t
    {
        const char* sensor_name{nullptr};
    };

    struct rk_aiq_static_info_t
    {
        rk_aiq_sensor_info_t sensor_info;
    };

    struct paRange_t
    {
        float min{0.0F};
        float max{0.0F};
    };

    struct rk_aiq_wb_gain_t
    {
        float rgain{1.0F};
        float bgain{1.0F};
        float grgain{1.0F};
        float gbgain{1.0F};
    };

    enum opMode_t
    {
        OP_AUTO   = 0,
        OP_MANUAL = 1
    };

    enum rk_aiq_working_mode_t
    {
        RK_AIQ_WORKING_MODE_NORMAL = 0
    };

    enum XCamReturn
    {
        XCAM_RETURN_NO_ERROR = 0,
        XCAM_RETURN_ERROR    = -1
    };

    int               rk_aiq_uapi2_sysctl_enumStaticMetas(int cam_id, rk_aiq_static_info_t* info);
    rk_aiq_sys_ctx_t* rk_aiq_uapi2_sysctl_init(const char* sns_ent_name, const char* iq_dir,
                                               void* unused1, void* unused2);
    int rk_aiq_uapi2_sysctl_prepare(rk_aiq_sys_ctx_t* ctx, int, int, rk_aiq_working_mode_t mode);
    int rk_aiq_uapi2_sysctl_start(rk_aiq_sys_ctx_t* ctx);
    int rk_aiq_uapi2_sysctl_stop(rk_aiq_sys_ctx_t* ctx, bool);
    int rk_aiq_uapi2_sysctl_deinit(rk_aiq_sys_ctx_t* ctx);
    XCamReturn rk_aiq_uapi2_setExpMode(rk_aiq_sys_ctx_t* ctx, opMode_t mode);
    XCamReturn rk_aiq_uapi2_getExpMode(rk_aiq_sys_ctx_t* ctx, opMode_t* mode);
    XCamReturn rk_aiq_uapi2_setExpGainRange(rk_aiq_sys_ctx_t* ctx, const paRange_t* range);
    XCamReturn rk_aiq_uapi2_getExpGainRange(rk_aiq_sys_ctx_t* ctx, paRange_t* range);
    XCamReturn rk_aiq_uapi2_setExpTimeRange(rk_aiq_sys_ctx_t* ctx, const paRange_t* range);
    XCamReturn rk_aiq_uapi2_getExpTimeRange(rk_aiq_sys_ctx_t* ctx, paRange_t* range);
    XCamReturn rk_aiq_uapi2_setAeLock(rk_aiq_sys_ctx_t* ctx, bool lock);
    XCamReturn rk_aiq_uapi2_setWBMode(rk_aiq_sys_ctx_t* ctx, opMode_t mode);
    XCamReturn rk_aiq_uapi2_getWBMode(rk_aiq_sys_ctx_t* ctx, opMode_t* mode);
    XCamReturn rk_aiq_uapi2_setMWBGain(rk_aiq_sys_ctx_t* ctx, const rk_aiq_wb_gain_t* gain);
    XCamReturn rk_aiq_uapi2_getWBGain(rk_aiq_sys_ctx_t* ctx, rk_aiq_wb_gain_t* gain);
    XCamReturn rk_aiq_uapi2_setMWBCT(rk_aiq_sys_ctx_t* ctx, unsigned int ct);
    XCamReturn rk_aiq_uapi2_getWBCT(rk_aiq_sys_ctx_t* ctx, unsigned int* ct);
    XCamReturn rk_aiq_uapi2_lockAWB(rk_aiq_sys_ctx_t* ctx);
    XCamReturn rk_aiq_uapi2_unlockAWB(rk_aiq_sys_ctx_t* ctx);
    XCamReturn rk_aiq_uapi2_setBrightness(rk_aiq_sys_ctx_t* ctx, unsigned int level);
    XCamReturn rk_aiq_uapi2_getBrightness(rk_aiq_sys_ctx_t* ctx, unsigned int* level);
    XCamReturn rk_aiq_uapi2_setContrast(rk_aiq_sys_ctx_t* ctx, unsigned int level);
    XCamReturn rk_aiq_uapi2_getContrast(rk_aiq_sys_ctx_t* ctx, unsigned int* level);
    XCamReturn rk_aiq_uapi2_setSaturation(rk_aiq_sys_ctx_t* ctx, unsigned int level);
    XCamReturn rk_aiq_uapi2_getSaturation(rk_aiq_sys_ctx_t* ctx, unsigned int* level);
    XCamReturn rk_aiq_uapi2_setHue(rk_aiq_sys_ctx_t* ctx, unsigned int level);
    XCamReturn rk_aiq_uapi2_getHue(rk_aiq_sys_ctx_t* ctx, unsigned int* level);
    XCamReturn rk_aiq_uapi2_setSharpness(rk_aiq_sys_ctx_t* ctx, unsigned int level);
    XCamReturn rk_aiq_uapi2_getSharpness(rk_aiq_sys_ctx_t* ctx, unsigned int* level);
    XCamReturn rk_aiq_uapi2_setDehazeEnable(rk_aiq_sys_ctx_t* ctx, bool enable);
    XCamReturn rk_aiq_uapi2_setMDehazeStrth(rk_aiq_sys_ctx_t* ctx, unsigned int level);
    XCamReturn rk_aiq_uapi2_getMDehazeStrth(rk_aiq_sys_ctx_t* ctx, unsigned int* level);
}
#endif

#include "../sync/sync.hpp"
#include "../../tool/memory/memory.hpp"
#include "tool/file/file.hpp"

namespace app::media::camera
{

    // ============================================================================
    // 错误类型枚举
    // ============================================================================

            enum class VideoError
            {
                NONE = 0,            // 无错误
                INVALID_PARAM,       // 无效参数
                ALLOC_FAILED,        // 内存分配失败
                INIT_FAILED,         // 初始化失败
                NOT_INITIALIZED,     // 未初始化
                ALREADY_INITIALIZED, // 已初始化
                ALREADY_STARTED,     // 已启动
                NOT_STARTED,         // 未启动
                INVALID_STATE,       // 无效状态
                TIMEOUT,             // 超时
                ENCODE_FAILED,       // 编码失败
                FILE_OPEN_FAILED,    // 文件打开失败
                RKMPI_ERROR,         // RKMPI错误
                NOT_SUPPORTED,       // 功能暂不支持
                UNKNOWN              // 未知错误
            };

            // ============================================================================
            // 编码格式枚举
            // ============================================================================

            enum class EncodeFormat
            {
                JPEG = 0, // JPEG格式
                H264,     // H.264格式
                H265      // H.265格式
            };

            // ============================================================================
            // 视频状态枚举
            // ============================================================================

            enum class VideoState
            {
                IDLE = 0,  // 空闲状态
                STREAMING, // 视频流运行中
                PHOTO,     // 拍照中
                RECORD,    // 录像中
                RTSP       // RTSP推流中
            };

            // ============================================================================
            // 配置结构体
            // ============================================================================

            struct VideoConfig
            {
                // 基本参数
                int          width  = 1920;               // 图像宽度
                int          height = 1080;               // 图像高度
                int          fps    = 30;                 // 帧率
                EncodeFormat format = EncodeFormat::H264; // 编码格式

                // 编码参数
                int bitrate = 10 * 1024; // 码率 (kbps)
                int gop     = 30;        // GOP大小
                int quality = 77;        // JPEG质量（1-100）

                // 拍照参数
                int         photo_skip_frames = 3; // 拍照跳过帧数（等待曝光稳定）
                std::string photo_path        = "/root/picture/"; // 照片保存路径

                // 录像参数
                std::string record_path = "/root/video/"; // 录像保存路径

                // 内存池配置
                size_t fixed_pool_size      = 100;              // 固定池块数
                size_t fixed_block_size     = 512 * 1024;       // 固定池块大小
                size_t dynamic_pool_size    = 10 * 1024 * 1024; // 动态池大小
                bool   enable_dma_zero_copy = true;             // 启用DMA零拷贝
            };

            // ============================================================================
            // 视频帧结构体
            // ============================================================================

            struct VideoFrame
            {
                uint8_t*     data        = nullptr;            // 帧数据指针
                size_t       size        = 0;                  // 数据大小
                uint64_t     timestamp   = 0;                  // 时间戳（微秒）
                uint64_t     pts         = 0;                  // 原始PTS
                EncodeFormat format      = EncodeFormat::H264; // 编码格式
                bool         is_keyframe = false;              // 是否关键帧
                int          frame_index = -1;                 // 内存池块索引

                // DMA相关
                bool   is_dma_buffer = false;   // 是否为DMA缓冲区
                MB_BLK dma_mb_blk    = nullptr; // RKMPI MediaBuffer句柄

                // 删除器函数
                std::function<void(int)> deleter;

                VideoFrame()                             = default;
                VideoFrame(const VideoFrame&)            = delete;
                VideoFrame& operator=(const VideoFrame&) = delete;
                VideoFrame(VideoFrame&&)                 = default;
                VideoFrame& operator=(VideoFrame&&)      = default;
            };

            using VideoFramePtr = std::shared_ptr<VideoFrame>;

            // ============================================================================
            // 原始YUV帧结构体
            // ============================================================================

            struct RawVideoFrame
            {
                uint8_t* data      = nullptr; // YUV420SP数据指针
                size_t   size      = 0;       // 数据大小（字节）
                uint32_t width     = 0;       // 图像宽度
                uint32_t height    = 0;       // 图像高度
                uint64_t timestamp = 0;       // 时间戳（微秒）
                uint64_t pts       = 0;       // PTS

                // DMA相关
                bool   is_dma_buffer = false;   // 是否为DMA缓冲区
                MB_BLK dma_mb_blk    = nullptr; // RKMPI MediaBuffer句柄

                // 删除器函数
                std::function<void()> deleter;

                RawVideoFrame()                                = default;
                RawVideoFrame(const RawVideoFrame&)            = delete;
                RawVideoFrame& operator=(const RawVideoFrame&) = delete;
                RawVideoFrame(RawVideoFrame&&)                 = default;
                RawVideoFrame& operator=(RawVideoFrame&&)      = default;
            };

            using RawVideoFramePtr = std::shared_ptr<RawVideoFrame>;

            // ============================================================================
            // 视频内存池
            // ============================================================================

            class VideoMemoryPool
            {
            public:
                struct Stats
                {
                    std::atomic<uint64_t> fixed_pool_hits{0};
                    std::atomic<uint64_t> dynamic_pool_hits{0};
                    std::atomic<uint64_t> total_allocations{0};
                    std::atomic<uint64_t> allocation_failures{0};
                };

                struct Config
                {
                    size_t fixed_block_size  = 512 * 1024;
                    size_t fixed_block_count = 100;
                    size_t dynamic_pool_size = 10 * 1024 * 1024;
                    size_t alignment         = 64;
                };

                explicit VideoMemoryPool(const Config& config);
                ~VideoMemoryPool();

                VideoMemoryPool(const VideoMemoryPool&)            = delete;
                VideoMemoryPool& operator=(const VideoMemoryPool&) = delete;

                VideoFramePtr allocate(size_t size);
                void          get_stats(Stats& out_stats) const;
                void          reset_stats();
                void          log_stats() const;

            private:
                struct FixedPool
                {
                    static constexpr size_t MAX_BLOCKS        = 256;
                    static constexpr size_t BITMAP_WORD_COUNT = 4;
                    static constexpr size_t BITS_PER_WORD     = 64;

                    size_t block_size;
                    size_t block_count;

                    alignas(64) std::atomic<uint64_t> allocation_bitmap_[BITMAP_WORD_COUNT];
                    alignas(64) std::vector<uint8_t> buffer;
                    VideoFrame frame_objects[MAX_BLOCKS];

                    FixedPool(size_t block_sz, size_t block_cnt);
                    ~FixedPool();

                    int      alloc_block();
                    void     dealloc_block(int index);
                    uint8_t* get_block_ptr(int index) const;
                };

                std::unique_ptr<FixedPool>                fixed_pool_;
                std::unique_ptr<tool::memory::MemoryPool> dynamic_pool_;
                Stats                                     stats_;
                Config                                    config_;
            };

            // ============================================================================
            // ISP包装器
            // ============================================================================

            class ISPWrapper
            {
            public:
                ISPWrapper(int camera_id, const std::string& iq_dir);
                ~ISPWrapper();

                ISPWrapper(const ISPWrapper&)            = delete;
                ISPWrapper& operator=(const ISPWrapper&) = delete;

                bool valid() const
                {
                    return valid_;
                }

                // 曝光控制
                VideoError set_exposure_mode(opMode_t mode);
                VideoError set_exp_gain_range(float min_gain, float max_gain);
                VideoError set_exp_time_range(float min_time, float max_time);
                VideoError lock_ae(bool lock);

                // 白平衡控制
                VideoError set_white_balance_mode(opMode_t mode);
                VideoError set_white_balance_gain(float r_gain, float b_gain);
                VideoError set_color_temp(unsigned int ct);
                VideoError lock_awb(bool lock);

                // 图像质量控制
                VideoError set_brightness(unsigned int level);
                VideoError set_contrast(unsigned int level);
                VideoError set_saturation(unsigned int level);
                VideoError set_sharpness(unsigned int level);

            private:
                int               camera_id_;
                bool              valid_   = false;
                rk_aiq_sys_ctx_t* aiq_ctx_ = nullptr;
            };

            // ============================================================================
            // VI设备包装器
            // ============================================================================

            class VIDeviceWrapper
            {
            public:
                explicit VIDeviceWrapper(int dev_id);
                ~VIDeviceWrapper();

                VIDeviceWrapper(const VIDeviceWrapper&)            = delete;
                VIDeviceWrapper& operator=(const VIDeviceWrapper&) = delete;

                bool valid() const
                {
                    return valid_;
                }

            private:
                int  dev_id_;
                bool valid_ = false;
            };

            // ============================================================================
            // VI通道包装器
            // ============================================================================

            class VIChannelWrapper
            {
            public:
                VIChannelWrapper(int dev_id, int chn_id, int width, int height, int depth = 0);
                ~VIChannelWrapper();

                VIChannelWrapper(const VIChannelWrapper&)            = delete;
                VIChannelWrapper& operator=(const VIChannelWrapper&) = delete;

                bool valid() const
                {
                    return valid_;
                }
                VideoError get_raw_frame(RawVideoFramePtr& frame, VideoMemoryPool& pool,
                                         int timeout_ms = -1);

            private:
                int  dev_id_;
                int  chn_id_;
                bool valid_ = false;
            };

            // ============================================================================
            // VENC编码器包装器
            // ============================================================================

            class VENCWrapper
            {
            public:
                VENCWrapper(int chn_id, int width, int height, EncodeFormat format, int bitrate,
                            int gop);
                ~VENCWrapper();

                VENCWrapper(const VENCWrapper&)            = delete;
                VENCWrapper& operator=(const VENCWrapper&) = delete;

                bool valid() const
                {
                    return valid_;
                }

                VideoError get_stream(VideoFramePtr& frame, VideoMemoryPool& pool, int timeout_ms);
                VideoError get_stream_zero_copy(VideoFramePtr& frame, int timeout_ms);
                VideoError set_bitrate(int bitrate_kbps);
                VideoError set_gop(int gop);
                VideoError set_jpeg_quality(int quality);

            private:
                int          chn_id_;
                bool         valid_ = false;
                EncodeFormat current_format_;
                int          current_width_;
                int          current_height_;
                int          current_bitrate_;
                int          current_gop_;
                int          current_jpeg_quality_;
            };

            // ============================================================================
            // 文件包装器（简化版）
            // ============================================================================

            using FileWrapper = tool::file::FileWrapper;

            // ============================================================================
            // VideoSystem 主类
            // ============================================================================

            class VideoSystem
            {
            public:
                explicit VideoSystem(const VideoConfig& config = VideoConfig());
                ~VideoSystem();

                VideoSystem(const VideoSystem&)            = delete;
                VideoSystem& operator=(const VideoSystem&) = delete;

                // 生命周期管理
                VideoError init(std::shared_ptr<sync_context_t> sync_ctx = nullptr);
                void       deinit();
                bool       is_init() const
                {
                    return is_initialized_.load();
                }

                // 视频流控制
                VideoError start_stream();
                VideoError stop_stream();
                bool       is_streaming() const
                {
                    return is_streaming_.load();
                }

                // 拍照功能
                VideoError take_photo(const std::string& filename = "");
                bool       is_photo_capturing() const
                {
                    return is_photo_capturing_.load();
                }

                // 录像功能
                VideoError start_record(const std::string& filename = "", int duration_sec = 0);
                VideoError stop_record();
                bool       is_recording() const
                {
                    return is_recording_.load();
                }

                // RTSP推流
                VideoError start_rtsp(int port = 554, const std::string& path = "/live/0");
                VideoError stop_rtsp();
                bool       is_rtsp_streaming() const
                {
                    return is_rtsp_streaming_.load();
                }

                // 状态获取
                VideoState get_state() const
                {
                    return state_.load();
                }
                float get_current_fps() const;

                // 原始帧获取
                VideoError get_raw_frame(RawVideoFramePtr& frame, int timeout_ms = -1);

                // 编码参数设置
                VideoError set_bitrate(int bitrate_kbps);
                VideoError set_gop(int gop);
                VideoError set_jpeg_quality(int quality);

                // ISP控制
                VideoError set_exposure_mode(opMode_t mode);
                VideoError set_brightness(unsigned int level);
                VideoError set_contrast(unsigned int level);
                VideoError set_saturation(unsigned int level);
                VideoError set_sharpness(unsigned int level);

                // 统计信息
                struct Stats
                {
                    VideoMemoryPool::Stats mem_stats;
                    std::atomic<uint64_t>  frames_captured{0};
                    std::atomic<uint64_t>  photos_taken{0};
                    std::atomic<uint64_t>  record_duration_ms{0};
                };

                void get_stats(Stats& out_stats) const;
                void reset_stats();
                void log_stats() const;

            private:
                class Impl;
                std::unique_ptr<Impl> pImpl_;

                std::atomic<bool>       is_initialized_{false};
                std::atomic<bool>       is_streaming_{false};
                std::atomic<bool>       is_photo_capturing_{false};
                std::atomic<bool>       is_recording_{false};
                std::atomic<bool>       is_rtsp_streaming_{false};
                std::atomic<VideoState> state_{VideoState::IDLE};
    };

} // namespace app::media::camera

#endif // CAMERA_H_
