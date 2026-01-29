/**
 * @file camera.cc
 */

#include "camera.hpp"
#include "tool/log/log.hpp"
#include "../media_config.hpp"
#include "protocol/rtsp/rtsp.h"
#include <cstring>
#include <algorithm>
#include <chrono>
#include <sys/stat.h>
#include <fstream>
#include <thread>

namespace app::media::camera
{
    using namespace tool::log;
    using namespace tool::file;

            namespace
            {
                constexpr const char* LOG_TAG              = "CAMERA";
                constexpr int         JPEG_DEFAULT_QUALITY = 77;
                constexpr int         JPEG_MIN_QUALITY     = 1;
                constexpr int         JPEG_MAX_QUALITY     = 100;
                constexpr int         JPEG_QP_SCALE        = 99;
                constexpr int         JPEG_QP_OFFSET       = 50;
                constexpr int         JPEG_QP_DIVISOR      = 100;
                constexpr double      BYTES_PER_MEGABYTE   = 1024.0 * 1024.0;

                // 创建目录
                void create_directory(const std::string& path)
                {
                    struct stat st
                    {
                    };
                    if (stat(path.c_str(), &st) == -1)
                    {
                        mkdir(path.c_str(), 0755);
                    }
                }
            } // namespace

            // ============================================================================
            // VideoMemoryPool::FixedPool
            // ============================================================================

            VideoMemoryPool::FixedPool::FixedPool(size_t block_sz, size_t block_cnt)
                : block_size(block_sz)
                , block_count(block_cnt)
            {
                if (block_count > MAX_BLOCKS)
                {
                    block_count = MAX_BLOCKS;
                    LOG_WARN(LOG_TAG, "块数量限制为 %zu", MAX_BLOCKS);
                }

                for (auto& bitmap_word : allocation_bitmap_)
                    bitmap_word.store(0, std::memory_order_relaxed);

                buffer.resize(block_size * block_count);

                for (size_t i = 0; i < block_count; i++)
                    frame_objects[i].frame_index = static_cast<int>(i);

                LOG_INFO(LOG_TAG, "固定池: %zu块 × %zu字节 = %.2fMB", block_count, block_size,
                         static_cast<double>(block_count * block_size) / BYTES_PER_MEGABYTE);
            }

            VideoMemoryPool::FixedPool::~FixedPool() = default;

            int VideoMemoryPool::FixedPool::alloc_block()
            {
                int bitmap_count =
                    static_cast<int>((block_count + BITS_PER_WORD - 1) / BITS_PER_WORD);

                for (int bitmap_index = 0; bitmap_index < bitmap_count; bitmap_index++)
                {
                    auto&    bitmap_atomic = allocation_bitmap_[bitmap_index];
                    uint64_t bitmap        = bitmap_atomic.load(std::memory_order_acquire);
                    int      base_index    = bitmap_index * static_cast<int>(BITS_PER_WORD);
                    int      max_blocks    = std::min(static_cast<int>(BITS_PER_WORD),
                                                      static_cast<int>(block_count) - base_index);

                    if (max_blocks <= 0)
                        break;

                    while (bitmap != UINT64_MAX)
                    {
                        uint64_t inverted = ~bitmap;
                        if (inverted == 0)
                            break;

                        int free_bit = __builtin_ctzll(inverted);
                        if (free_bit >= max_blocks)
                            break;

                        uint64_t new_bitmap = bitmap | (1ULL << free_bit);
                        if (bitmap_atomic.compare_exchange_weak(bitmap, new_bitmap,
                                                                std::memory_order_acq_rel,
                                                                std::memory_order_acquire))
                            return base_index + free_bit;
                    }
                }
                return -1;
            }

            void VideoMemoryPool::FixedPool::dealloc_block(int index)
            {
                if (index < 0 || index >= static_cast<int>(block_count))
                {
                    LOG_ERROR(LOG_TAG, "无效的块索引: %d", index);
                    return;
                }

                int      bitmap_index = index / static_cast<int>(BITS_PER_WORD);
                int      bit_position = index % static_cast<int>(BITS_PER_WORD);
                uint64_t mask         = ~(1ULL << bit_position);

                allocation_bitmap_[bitmap_index].fetch_and(mask, std::memory_order_release);
            }

            uint8_t* VideoMemoryPool::FixedPool::get_block_ptr(int index) const
            {
                if (index < 0 || index >= static_cast<int>(block_count))
                    return nullptr;
                return const_cast<uint8_t*>(buffer.data() +
                                            (static_cast<size_t>(index) * block_size));
            }

            // ============================================================================
            // VideoMemoryPool
            // ============================================================================

            VideoMemoryPool::VideoMemoryPool(const Config& config)
                : config_(config)
            {
                LOG_INFO(LOG_TAG, "初始化视频内存池...");

                fixed_pool_ =
                    std::make_unique<FixedPool>(config.fixed_block_size, config.fixed_block_count);
                dynamic_pool_ = std::make_unique<tool::memory::MemoryPool>(config.dynamic_pool_size,
                                                                           config.alignment, 1.5);

                LOG_INFO(LOG_TAG, "视频内存池初始化完成");
            }

            VideoMemoryPool::~VideoMemoryPool()
            {
                log_stats();
            }

            VideoFramePtr VideoMemoryPool::allocate(size_t size)
            {
                stats_.total_allocations.fetch_add(1, std::memory_order_relaxed);

                // 固定池
                if (size <= config_.fixed_block_size)
                {
                    int block_index = fixed_pool_->alloc_block();
                    if (block_index >= 0)
                    {
                        stats_.fixed_pool_hits.fetch_add(1, std::memory_order_relaxed);

                        auto* frame_obj        = &fixed_pool_->frame_objects[block_index];
                        frame_obj->data        = fixed_pool_->get_block_ptr(block_index);
                        frame_obj->size        = size;
                        frame_obj->frame_index = block_index;

                        auto pool_ptr      = fixed_pool_.get();
                        frame_obj->deleter = [pool_ptr](int idx)
                        {
                            pool_ptr->dealloc_block(idx);
                        };

                        return VideoFramePtr(frame_obj,
                                             [](VideoFrame* f)
                                             {
                                                 if (f && f->deleter && f->frame_index >= 0)
                                                     f->deleter(f->frame_index);
                                             });
                    }
                }

                // 动态池
                auto* mem = dynamic_pool_->allocate(size + sizeof(VideoFrame));
                if (mem)
                {
                    stats_.dynamic_pool_hits.fetch_add(1, std::memory_order_relaxed);

                    auto* frame        = new (mem) VideoFrame();
                    frame->data        = static_cast<uint8_t*>(mem) + sizeof(VideoFrame);
                    frame->size        = size;
                    frame->frame_index = -1;

                    auto pool_ptr  = dynamic_pool_.get();
                    frame->deleter = [pool_ptr, mem](int)
                    {
                        pool_ptr->deallocate(mem);
                    };

                    return VideoFramePtr(frame,
                                         [](VideoFrame* f)
                                         {
                                             if (f && f->deleter)
                                             {
                                                 f->~VideoFrame();
                                                 f->deleter(0);
                                             }
                                         });
                }

                stats_.allocation_failures.fetch_add(1, std::memory_order_relaxed);
                LOG_ERROR(LOG_TAG, "内存分配失败: %zu", size);
                return nullptr;
            }

            void VideoMemoryPool::get_stats(Stats& out_stats) const
            {
                out_stats.fixed_pool_hits = stats_.fixed_pool_hits.load(std::memory_order_relaxed);
                out_stats.dynamic_pool_hits =
                    stats_.dynamic_pool_hits.load(std::memory_order_relaxed);
                out_stats.total_allocations =
                    stats_.total_allocations.load(std::memory_order_relaxed);
                out_stats.allocation_failures =
                    stats_.allocation_failures.load(std::memory_order_relaxed);
            }

            void VideoMemoryPool::reset_stats()
            {
                stats_.fixed_pool_hits.store(0);
                stats_.dynamic_pool_hits.store(0);
                stats_.total_allocations.store(0);
                stats_.allocation_failures.store(0);
            }

            void VideoMemoryPool::log_stats() const
            {
                uint64_t total = stats_.total_allocations.load(std::memory_order_relaxed);
                if (total == 0)
                    return;

                uint64_t fixed   = stats_.fixed_pool_hits.load();
                uint64_t dynamic = stats_.dynamic_pool_hits.load();
                uint64_t fail    = stats_.allocation_failures.load();

                LOG_INFO(LOG_TAG, "内存池统计: 总=%zu, 固定=%zu(%.1f%%), 动态=%zu, 失败=%zu", total,
                         fixed, static_cast<double>(fixed) * 100.0 / static_cast<double>(total),
                         dynamic, fail);
            }

            // ============================================================================
            // ISPWrapper
            // ============================================================================

            ISPWrapper::ISPWrapper(int camera_id, const std::string& iq_dir)
                : camera_id_(camera_id)
            {
                LOG_INFO(LOG_TAG, "初始化ISP (camera %d)", camera_id);

                system("RkLunch-stop.sh 2>/dev/null");

                rk_aiq_static_info_t static_info;
                memset(&static_info, 0, sizeof(static_info));

                if (rk_aiq_uapi2_sysctl_enumStaticMetas(camera_id, &static_info) != 0)
                {
                    LOG_ERROR(LOG_TAG, "枚举相机静态元数据失败");
                    return;
                }

                const char* sns_ent_name = static_info.sensor_info.sensor_name;
                LOG_INFO(LOG_TAG, "传感器: %s", sns_ent_name);

                aiq_ctx_ = rk_aiq_uapi2_sysctl_init(sns_ent_name, iq_dir.c_str(), nullptr, nullptr);
                if (!aiq_ctx_)
                {
                    LOG_ERROR(LOG_TAG, "AIQ初始化失败");
                    return;
                }

                if (rk_aiq_uapi2_sysctl_prepare(aiq_ctx_, 0, 0, RK_AIQ_WORKING_MODE_NORMAL) != 0)
                {
                    LOG_ERROR(LOG_TAG, "AIQ准备失败");
                    rk_aiq_uapi2_sysctl_deinit(aiq_ctx_);
                    aiq_ctx_ = nullptr;
                    return;
                }

                if (rk_aiq_uapi2_sysctl_start(aiq_ctx_) != 0)
                {
                    LOG_ERROR(LOG_TAG, "AIQ启动失败");
                    rk_aiq_uapi2_sysctl_deinit(aiq_ctx_);
                    aiq_ctx_ = nullptr;
                    return;
                }

                valid_ = true;
                LOG_INFO(LOG_TAG, "ISP初始化成功");
            }

            ISPWrapper::~ISPWrapper()
            {
                if (aiq_ctx_)
                {
                    rk_aiq_uapi2_sysctl_stop(aiq_ctx_, false);
                    rk_aiq_uapi2_sysctl_deinit(aiq_ctx_);
                }
            }

            VideoError ISPWrapper::set_exposure_mode(opMode_t mode)
            {
                if (!aiq_ctx_)
                    return VideoError::NOT_INITIALIZED;
                return (rk_aiq_uapi2_setExpMode(aiq_ctx_, mode) == XCAM_RETURN_NO_ERROR)
                           ? VideoError::NONE
                           : VideoError::RKMPI_ERROR;
            }

            VideoError ISPWrapper::set_exp_gain_range(float min_gain, float max_gain)
            {
                if (!aiq_ctx_)
                    return VideoError::NOT_INITIALIZED;
                paRange_t gain{min_gain, max_gain};
                return (rk_aiq_uapi2_setExpGainRange(aiq_ctx_, &gain) == XCAM_RETURN_NO_ERROR)
                           ? VideoError::NONE
                           : VideoError::RKMPI_ERROR;
            }

            VideoError ISPWrapper::set_exp_time_range(float min_time, float max_time)
            {
                if (!aiq_ctx_)
                    return VideoError::NOT_INITIALIZED;
                paRange_t time{min_time, max_time};
                return (rk_aiq_uapi2_setExpTimeRange(aiq_ctx_, &time) == XCAM_RETURN_NO_ERROR)
                           ? VideoError::NONE
                           : VideoError::RKMPI_ERROR;
            }

            VideoError ISPWrapper::lock_ae(bool lock)
            {
                if (!aiq_ctx_)
                    return VideoError::NOT_INITIALIZED;
                return (rk_aiq_uapi2_setAeLock(aiq_ctx_, lock) == XCAM_RETURN_NO_ERROR)
                           ? VideoError::NONE
                           : VideoError::RKMPI_ERROR;
            }

            VideoError ISPWrapper::set_white_balance_mode(opMode_t mode)
            {
                if (!aiq_ctx_)
                    return VideoError::NOT_INITIALIZED;
                return (rk_aiq_uapi2_setWBMode(aiq_ctx_, mode) == XCAM_RETURN_NO_ERROR)
                           ? VideoError::NONE
                           : VideoError::RKMPI_ERROR;
            }

            VideoError ISPWrapper::set_white_balance_gain(float r_gain, float b_gain)
            {
                if (!aiq_ctx_)
                    return VideoError::NOT_INITIALIZED;
                rk_aiq_wb_gain_t gain{r_gain, b_gain, 1.0f, 1.0f};
                return (rk_aiq_uapi2_setMWBGain(aiq_ctx_, &gain) == XCAM_RETURN_NO_ERROR)
                           ? VideoError::NONE
                           : VideoError::RKMPI_ERROR;
            }

            VideoError ISPWrapper::set_color_temp(unsigned int ct)
            {
                if (!aiq_ctx_)
                    return VideoError::NOT_INITIALIZED;
                return (rk_aiq_uapi2_setMWBCT(aiq_ctx_, ct) == XCAM_RETURN_NO_ERROR)
                           ? VideoError::NONE
                           : VideoError::RKMPI_ERROR;
            }

            VideoError ISPWrapper::lock_awb(bool lock)
            {
                if (!aiq_ctx_)
                    return VideoError::NOT_INITIALIZED;
                return ((lock ? rk_aiq_uapi2_lockAWB(aiq_ctx_)
                              : rk_aiq_uapi2_unlockAWB(aiq_ctx_)) == XCAM_RETURN_NO_ERROR)
                           ? VideoError::NONE
                           : VideoError::RKMPI_ERROR;
            }

            VideoError ISPWrapper::set_brightness(unsigned int level)
            {
                if (!aiq_ctx_)
                    return VideoError::NOT_INITIALIZED;
                return (rk_aiq_uapi2_setBrightness(aiq_ctx_, level) == XCAM_RETURN_NO_ERROR)
                           ? VideoError::NONE
                           : VideoError::RKMPI_ERROR;
            }

            VideoError ISPWrapper::set_contrast(unsigned int level)
            {
                if (!aiq_ctx_)
                    return VideoError::NOT_INITIALIZED;
                return (rk_aiq_uapi2_setContrast(aiq_ctx_, level) == XCAM_RETURN_NO_ERROR)
                           ? VideoError::NONE
                           : VideoError::RKMPI_ERROR;
            }

            VideoError ISPWrapper::set_saturation(unsigned int level)
            {
                if (!aiq_ctx_)
                    return VideoError::NOT_INITIALIZED;
                return (rk_aiq_uapi2_setSaturation(aiq_ctx_, level) == XCAM_RETURN_NO_ERROR)
                           ? VideoError::NONE
                           : VideoError::RKMPI_ERROR;
            }

            VideoError ISPWrapper::set_sharpness(unsigned int level)
            {
                if (!aiq_ctx_)
                    return VideoError::NOT_INITIALIZED;
                return (rk_aiq_uapi2_setSharpness(aiq_ctx_, level) == XCAM_RETURN_NO_ERROR)
                           ? VideoError::NONE
                           : VideoError::RKMPI_ERROR;
            }

            // ============================================================================
            // VIDeviceWrapper
            // ============================================================================

            VIDeviceWrapper::VIDeviceWrapper(int dev_id)
                : dev_id_(dev_id)
            {
                LOG_INFO(LOG_TAG, "初始化VI设备 %d", dev_id);

                VI_DEV_ATTR_S      dev_attr{};
                VI_DEV_BIND_PIPE_S bind_pipe{};

                RK_S32 ret = RK_MPI_VI_GetDevAttr(dev_id, &dev_attr);
                if (ret == RK_ERR_VI_NOT_CONFIG)
                {
                    ret = RK_MPI_VI_SetDevAttr(dev_id, &dev_attr);
                    if (ret != RK_SUCCESS)
                    {
                        LOG_ERROR(LOG_TAG, "SetDevAttr失败: 0x%x", ret);
                        return;
                    }
                }

                ret = RK_MPI_VI_GetDevIsEnable(dev_id);
                if (ret != RK_SUCCESS)
                {
                    ret = RK_MPI_VI_EnableDev(dev_id);
                    if (ret != RK_SUCCESS)
                    {
                        LOG_ERROR(LOG_TAG, "EnableDev失败: 0x%x", ret);
                        return;
                    }

                    bind_pipe.u32Num    = 1;
                    bind_pipe.PipeId[0] = dev_id;
                    ret                 = RK_MPI_VI_SetDevBindPipe(dev_id, &bind_pipe);
                    if (ret != RK_SUCCESS)
                    {
                        RK_MPI_VI_DisableDev(dev_id);
                        return;
                    }
                }

                valid_ = true;
                LOG_INFO(LOG_TAG, "VI设备初始化成功");
            }

            VIDeviceWrapper::~VIDeviceWrapper()
            {
                if (valid_)
                    RK_MPI_VI_DisableDev(dev_id_);
            }

            // ============================================================================
            // VIChannelWrapper
            // ============================================================================

            VIChannelWrapper::VIChannelWrapper(int dev_id, int chn_id, int width, int height,
                                               int depth)
                : dev_id_(dev_id)
                , chn_id_(chn_id)
            {
                LOG_INFO(LOG_TAG, "初始化VI通道 %d (%dx%d)", chn_id, width, height);

                VI_CHN_ATTR_S vi_chn_attr{};
                vi_chn_attr.stIspOpt.u32BufCount  = 4;
                vi_chn_attr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
                vi_chn_attr.stSize.u32Width       = static_cast<RK_U32>(width);
                vi_chn_attr.stSize.u32Height      = static_cast<RK_U32>(height);
                vi_chn_attr.enPixelFormat         = RK_FMT_YUV420SP;
                vi_chn_attr.enCompressMode        = COMPRESS_MODE_NONE;
                vi_chn_attr.u32Depth              = static_cast<RK_U32>(depth);

                RK_S32 ret = RK_MPI_VI_SetChnAttr(dev_id, chn_id, &vi_chn_attr);
                if (ret != RK_SUCCESS)
                {
                    LOG_ERROR(LOG_TAG, "SetChnAttr失败: 0x%x", ret);
                    return;
                }

                ret = RK_MPI_VI_EnableChn(dev_id, chn_id);
                if (ret != RK_SUCCESS)
                {
                    LOG_ERROR(LOG_TAG, "EnableChn失败: 0x%x", ret);
                    return;
                }

                valid_ = true;
                LOG_INFO(LOG_TAG, "VI通道初始化成功");
            }

            VIChannelWrapper::~VIChannelWrapper()
            {
                if (valid_)
                    RK_MPI_VI_DisableChn(dev_id_, chn_id_);
            }

            VideoError VIChannelWrapper::get_raw_frame(RawVideoFramePtr& frame,
                                                       VideoMemoryPool& pool, int timeout_ms)
            {
                if (!valid_)
                    return VideoError::NOT_INITIALIZED;

                VIDEO_FRAME_INFO_S frame_info{};
                RK_S32 ret = RK_MPI_VI_GetChnFrame(dev_id_, chn_id_, &frame_info, timeout_ms);

                if (ret != RK_SUCCESS)
                    return VideoError::RKMPI_ERROR;

                void* vi_data = RK_MPI_MB_Handle2VirAddr(frame_info.stVFrame.pMbBlk);
                if (!vi_data)
                {
                    RK_MPI_VI_ReleaseChnFrame(dev_id_, chn_id_, &frame_info);
                    return VideoError::RKMPI_ERROR;
                }

                size_t frame_size =
                    frame_info.stVFrame.u32VirWidth * frame_info.stVFrame.u32VirHeight * 3 / 2;

                VideoFramePtr video_frame = pool.allocate(frame_size);
                if (!video_frame)
                {
                    RK_MPI_VI_ReleaseChnFrame(dev_id_, chn_id_, &frame_info);
                    return VideoError::ALLOC_FAILED;
                }

                std::memcpy(video_frame->data, vi_data, frame_size);
                RK_MPI_VI_ReleaseChnFrame(dev_id_, chn_id_, &frame_info);

                auto* raw_frame          = new RawVideoFrame();
                raw_frame->data          = video_frame->data;
                raw_frame->size          = frame_size;
                raw_frame->width         = frame_info.stVFrame.u32Width;
                raw_frame->height        = frame_info.stVFrame.u32Height;
                raw_frame->pts           = frame_info.stVFrame.u64PTS;
                raw_frame->timestamp     = frame_info.stVFrame.u64PTS;
                raw_frame->is_dma_buffer = false;

                auto* video_frame_ptr = new VideoFramePtr(video_frame);
                raw_frame->deleter    = [video_frame_ptr]()
                {
                    if (video_frame_ptr)
                    {
                        video_frame_ptr->reset();
                        delete video_frame_ptr;
                    }
                };

                frame = RawVideoFramePtr(raw_frame,
                                         [](RawVideoFrame* f)
                                         {
                                             if (f)
                                             {
                                                 if (f->deleter)
                                                     f->deleter();
                                                 delete f;
                                             }
                                         });

                return VideoError::NONE;
            }

            // ============================================================================
            // VENCWrapper
            // ============================================================================

            VENCWrapper::VENCWrapper(int chn_id, int width, int height, EncodeFormat format,
                                     int bitrate, int gop)
                : chn_id_(chn_id)
                , current_format_(format)
                , current_width_(width)
                , current_height_(height)
                , current_bitrate_(bitrate)
                , current_gop_(gop)
                , current_jpeg_quality_(JPEG_DEFAULT_QUALITY)
            {
                LOG_INFO(LOG_TAG, "初始化VENC通道 %d (%dx%d)", chn_id, width, height);

                VENC_CHN_ATTR_S venc_attr{};
                RK_CODEC_ID_E   codec_type{};

                switch (format)
                {
                case EncodeFormat::H264:
                    codec_type                              = RK_VIDEO_ID_AVC;
                    venc_attr.stRcAttr.enRcMode             = VENC_RC_MODE_H264CBR;
                    venc_attr.stRcAttr.stH264Cbr.u32BitRate = static_cast<RK_U32>(bitrate);
                    venc_attr.stRcAttr.stH264Cbr.u32Gop     = static_cast<RK_U32>(gop);
                    venc_attr.stVencAttr.u32Profile         = H264E_PROFILE_BASELINE;
                    venc_attr.stVencAttr.u32BufSize = static_cast<RK_U32>(bitrate * 2 * 1024);
                    break;
                case EncodeFormat::H265:
                    codec_type                              = RK_VIDEO_ID_HEVC;
                    venc_attr.stRcAttr.enRcMode             = VENC_RC_MODE_H265CBR;
                    venc_attr.stRcAttr.stH265Cbr.u32BitRate = static_cast<RK_U32>(bitrate);
                    venc_attr.stRcAttr.stH265Cbr.u32Gop     = static_cast<RK_U32>(gop);
                    venc_attr.stVencAttr.u32BufSize = static_cast<RK_U32>(width * height * 3 / 2);
                    break;
                case EncodeFormat::JPEG:
                    codec_type                                     = RK_VIDEO_ID_MJPEG;
                    venc_attr.stRcAttr.enRcMode                    = VENC_RC_MODE_MJPEGFIXQP;
                    venc_attr.stRcAttr.stMjpegFixQp.u32Qfactor     = 70;
                    venc_attr.stVencAttr.stAttrJpege.bSupportDCF   = RK_FALSE;
                    venc_attr.stVencAttr.stAttrJpege.enReceiveMode = VENC_PIC_RECEIVE_SINGLE;
                    venc_attr.stVencAttr.u32BufSize = static_cast<RK_U32>(width * height * 3);
                    break;
                }

                venc_attr.stVencAttr.enType          = codec_type;
                venc_attr.stVencAttr.enPixelFormat   = RK_FMT_YUV420SP;
                venc_attr.stVencAttr.u32PicWidth     = static_cast<RK_U32>(width);
                venc_attr.stVencAttr.u32PicHeight    = static_cast<RK_U32>(height);
                venc_attr.stVencAttr.u32VirWidth     = static_cast<RK_U32>(width);
                venc_attr.stVencAttr.u32VirHeight    = static_cast<RK_U32>(height);
                venc_attr.stVencAttr.u32StreamBufCnt = 3;
                venc_attr.stVencAttr.enMirror        = MIRROR_NONE;

                RK_S32 ret = RK_MPI_VENC_CreateChn(chn_id, &venc_attr);
                if (ret != RK_SUCCESS)
                {
                    LOG_ERROR(LOG_TAG, "CreateChn失败: 0x%x", ret);
                    return;
                }

                VENC_RECV_PIC_PARAM_S recv_param{};
                recv_param.s32RecvPicNum = -1;

                ret = RK_MPI_VENC_StartRecvFrame(chn_id, &recv_param);
                if (ret != RK_SUCCESS)
                {
                    LOG_ERROR(LOG_TAG, "StartRecvFrame失败: 0x%x", ret);
                    RK_MPI_VENC_DestroyChn(chn_id);
                    return;
                }

                valid_ = true;
                LOG_INFO(LOG_TAG, "VENC初始化成功");
            }

            VENCWrapper::~VENCWrapper()
            {
                if (valid_)
                {
                    RK_MPI_VENC_StopRecvFrame(chn_id_);
                    RK_MPI_VENC_DestroyChn(chn_id_);
                }
            }

            VideoError VENCWrapper::get_stream(VideoFramePtr& frame, VideoMemoryPool& pool,
                                               int timeout_ms)
            {
                if (!valid_)
                    return VideoError::NOT_INITIALIZED;

                VENC_STREAM_S venc_stream{};
                venc_stream.pstPack = static_cast<VENC_PACK_S*>(malloc(sizeof(VENC_PACK_S)));
                if (!venc_stream.pstPack)
                    return VideoError::ALLOC_FAILED;

                RK_S32 ret = RK_MPI_VENC_GetStream(chn_id_, &venc_stream, timeout_ms);
                if (ret != RK_SUCCESS)
                {
                    free(venc_stream.pstPack);
                    return (ret == RK_ERR_VENC_BUSY) ? VideoError::TIMEOUT
                                                     : VideoError::ENCODE_FAILED;
                }

                if (venc_stream.u32PackCount == 0 || !venc_stream.pstPack)
                {
                    RK_MPI_VENC_ReleaseStream(chn_id_, &venc_stream);
                    free(venc_stream.pstPack);
                    return VideoError::ENCODE_FAILED;
                }

                size_t total_size = 0;
                for (uint32_t i = 0; i < venc_stream.u32PackCount; i++)
                    total_size += venc_stream.pstPack[i].u32Len;

                frame = pool.allocate(total_size);
                if (!frame)
                {
                    RK_MPI_VENC_ReleaseStream(chn_id_, &venc_stream);
                    free(venc_stream.pstPack);
                    return VideoError::ALLOC_FAILED;
                }

                size_t offset = 0;
                for (uint32_t i = 0; i < venc_stream.u32PackCount; i++)
                {
                    void* data_ptr = RK_MPI_MB_Handle2VirAddr(venc_stream.pstPack[i].pMbBlk);
                    if (data_ptr)
                    {
                        std::memcpy(frame->data + offset, data_ptr, venc_stream.pstPack[i].u32Len);
                        offset += venc_stream.pstPack[i].u32Len;
                    }
                }

                frame->size = total_size;
                frame->pts  = venc_stream.pstPack[0].u64PTS;
                frame->is_keyframe =
                    (venc_stream.pstPack[0].DataType.enH264EType == H264E_NALU_IDRSLICE ||
                     venc_stream.pstPack[0].DataType.enH265EType == H265E_NALU_IDRSLICE);
                frame->is_dma_buffer = false;

                RK_MPI_VENC_ReleaseStream(chn_id_, &venc_stream);
                free(venc_stream.pstPack);

                return VideoError::NONE;
            }

            VideoError VENCWrapper::get_stream_zero_copy(VideoFramePtr& frame, int timeout_ms)
            {
                if (!valid_)
                    return VideoError::NOT_INITIALIZED;

                auto* venc_stream    = new VENC_STREAM_S();
                venc_stream->pstPack = static_cast<VENC_PACK_S*>(malloc(sizeof(VENC_PACK_S)));
                if (!venc_stream->pstPack)
                {
                    delete venc_stream;
                    return VideoError::ALLOC_FAILED;
                }

                RK_S32 ret = RK_MPI_VENC_GetStream(chn_id_, venc_stream, timeout_ms);
                if (ret != RK_SUCCESS)
                {
                    free(venc_stream->pstPack);
                    delete venc_stream;
                    return (ret == RK_ERR_VENC_BUSY) ? VideoError::TIMEOUT
                                                     : VideoError::ENCODE_FAILED;
                }

                if (venc_stream->u32PackCount == 0 || !venc_stream->pstPack)
                {
                    RK_MPI_VENC_ReleaseStream(chn_id_, venc_stream);
                    free(venc_stream->pstPack);
                    delete venc_stream;
                    return VideoError::ENCODE_FAILED;
                }

                size_t total_size = 0;
                for (uint32_t i = 0; i < venc_stream->u32PackCount; i++)
                    total_size += venc_stream->pstPack[i].u32Len;

                VideoFrame* video_frame = new VideoFrame();

                if (venc_stream->u32PackCount == 1)
                {
                    video_frame->data = static_cast<uint8_t*>(
                        RK_MPI_MB_Handle2VirAddr(venc_stream->pstPack[0].pMbBlk));
                    video_frame->dma_mb_blk = venc_stream->pstPack[0].pMbBlk;
                }
                else
                {
                    RK_MPI_VENC_ReleaseStream(chn_id_, venc_stream);
                    free(venc_stream->pstPack);
                    delete venc_stream;
                    delete video_frame;
                    return VideoError::ENCODE_FAILED;
                }

                video_frame->size = total_size;
                video_frame->pts  = venc_stream->pstPack[0].u64PTS;
                video_frame->is_keyframe =
                    (venc_stream->pstPack[0].DataType.enH264EType == H264E_NALU_IDRSLICE ||
                     venc_stream->pstPack[0].DataType.enH265EType == H265E_NALU_IDRSLICE);
                video_frame->is_dma_buffer = true;

                int venc_chn         = chn_id_;
                video_frame->deleter = [venc_chn, venc_stream](int)
                {
                    RK_MPI_VENC_ReleaseStream(venc_chn, venc_stream);
                    if (venc_stream)
                    {
                        free(venc_stream->pstPack);
                        delete venc_stream;
                    }
                };

                frame = VideoFramePtr(video_frame,
                                      [](VideoFrame* f)
                                      {
                                          if (f)
                                          {
                                              if (f->deleter)
                                                  f->deleter(0);
                                              delete f;
                                          }
                                      });

                return VideoError::NONE;
            }

            VideoError VENCWrapper::set_bitrate(int bitrate_kbps)
            {
                if (!valid_)
                    return VideoError::NOT_INITIALIZED;

                VENC_CHN_ATTR_S channel_attr{};
                RK_S32          ret = RK_MPI_VENC_GetChnAttr(chn_id_, &channel_attr);
                if (ret != RK_SUCCESS)
                    return VideoError::ENCODE_FAILED;

                switch (current_format_)
                {
                case EncodeFormat::H264:
                    channel_attr.stRcAttr.stH264Cbr.u32BitRate = static_cast<RK_U32>(bitrate_kbps);
                    break;
                case EncodeFormat::H265:
                    channel_attr.stRcAttr.stH265Cbr.u32BitRate = static_cast<RK_U32>(bitrate_kbps);
                    break;
                case EncodeFormat::JPEG:
                    return VideoError::INVALID_PARAM;
                }

                ret = RK_MPI_VENC_SetChnAttr(chn_id_, &channel_attr);
                if (ret == RK_SUCCESS)
                {
                    current_bitrate_ = bitrate_kbps;
                    return VideoError::NONE;
                }
                return VideoError::ENCODE_FAILED;
            }

            VideoError VENCWrapper::set_gop(int gop)
            {
                if (!valid_)
                    return VideoError::NOT_INITIALIZED;

                VENC_CHN_ATTR_S channel_attr{};
                RK_S32          ret = RK_MPI_VENC_GetChnAttr(chn_id_, &channel_attr);
                if (ret != RK_SUCCESS)
                    return VideoError::ENCODE_FAILED;

                switch (current_format_)
                {
                case EncodeFormat::H264:
                    channel_attr.stRcAttr.stH264Cbr.u32Gop = static_cast<RK_U32>(gop);
                    break;
                case EncodeFormat::H265:
                    channel_attr.stRcAttr.stH265Cbr.u32Gop = static_cast<RK_U32>(gop);
                    break;
                case EncodeFormat::JPEG:
                    current_gop_ = 1;
                    return VideoError::NONE;
                }

                ret = RK_MPI_VENC_SetChnAttr(chn_id_, &channel_attr);
                if (ret == RK_SUCCESS)
                {
                    current_gop_ = gop;
                    return VideoError::NONE;
                }
                return VideoError::ENCODE_FAILED;
            }

            VideoError VENCWrapper::set_jpeg_quality(int quality)
            {
                if (!valid_)
                    return VideoError::NOT_INITIALIZED;

                quality = std::max(JPEG_MIN_QUALITY, std::min(JPEG_MAX_QUALITY, quality));
                current_jpeg_quality_ = quality;

                if (current_format_ != EncodeFormat::JPEG)
                    return VideoError::NONE;

                VENC_CHN_ATTR_S channel_attr{};
                RK_S32          ret = RK_MPI_VENC_GetChnAttr(chn_id_, &channel_attr);
                if (ret != RK_SUCCESS)
                    return VideoError::ENCODE_FAILED;

                if (channel_attr.stRcAttr.enRcMode == VENC_RC_MODE_MJPEGFIXQP)
                {
                    int quality_param = std::max(
                        JPEG_MIN_QUALITY,
                        std::min(JPEG_QP_SCALE,
                                 (quality * JPEG_QP_SCALE + JPEG_QP_OFFSET) / JPEG_QP_DIVISOR));
                    channel_attr.stRcAttr.stMjpegFixQp.u32Qfactor =
                        static_cast<RK_U32>(quality_param);
                    RK_MPI_VENC_SetChnAttr(chn_id_, &channel_attr);
                }

                return VideoError::NONE;
            }

            // ============================================================================
            // VideoSystem::Impl
            // ============================================================================

            class VideoSystem::Impl
            {
            public:
                Impl(const VideoConfig& config)
                    : config_(config)
                    , memory_pool_(VideoMemoryPool::Config{config.fixed_block_size,
                                                           config.fixed_pool_size,
                                                           config.dynamic_pool_size, 64})
                    , quit_flag_(false)
                    , current_fps_(0.0f)
                    , photo_id_(0)
                    , record_id_(0)
                {
                }

                ~Impl()
                {
                    deinit();
                }

                VideoError init(std::shared_ptr<sync_context_t> sync_ctx)
                {
                    std::lock_guard<std::mutex> lock(mutex_);

                    LOG_INFO(LOG_TAG, "初始化视频系统 (%dx%d @ %dfps)", config_.width,
                             config_.height, config_.fps);

                    sync_ctx_ = sync_ctx;
                    create_directory(config_.photo_path);
                    create_directory(config_.record_path);

                    if (RK_MPI_SYS_Init() != RK_SUCCESS)
                    {
                        LOG_ERROR(LOG_TAG, "RKMPI初始化失败");
                        return VideoError::INIT_FAILED;
                    }
                    rkmpi_initialized_ = true;

                    isp_ = std::make_unique<ISPWrapper>(0, ISP_PATH);
                    if (!isp_->valid())
                        return VideoError::INIT_FAILED;

                    vi_dev_ = std::make_unique<VIDeviceWrapper>(0);
                    if (!vi_dev_->valid())
                        return VideoError::INIT_FAILED;

                    vi_chn_ =
                        std::make_unique<VIChannelWrapper>(0, 0, config_.width, config_.height, 0);
                    if (!vi_chn_->valid())
                        return VideoError::INIT_FAILED;

                    vi_chn_raw_ =
                        std::make_unique<VIChannelWrapper>(0, 1, config_.width, config_.height, 2);
                    if (!vi_chn_raw_->valid())
                    {
                        LOG_WARN(LOG_TAG, "VI通道1初始化失败，原始帧获取功能不可用");
                    }

                    venc_ =
                        std::make_unique<VENCWrapper>(0, config_.width, config_.height,
                                                      config_.format, config_.bitrate, config_.gop);
                    if (!venc_->valid())
                        return VideoError::INIT_FAILED;

                    MPP_CHN_S src{RK_ID_VI, 0, 0}, dest{RK_ID_VENC, 0, 0};
                    if (RK_MPI_SYS_Bind(&src, &dest) != RK_SUCCESS)
                    {
                        LOG_ERROR(LOG_TAG, "模块绑定失败");
                        return VideoError::INIT_FAILED;
                    }
                    modules_bound_ = true;

                    LOG_INFO(LOG_TAG, "视频系统初始化成功");
                    return VideoError::NONE;
                }

                void deinit()
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    LOG_INFO(LOG_TAG, "关闭视频系统...");

                    stop_stream_internal();
                    stop_record_internal();
                    stop_rtsp_internal();

                    if (modules_bound_)
                    {
                        MPP_CHN_S src{RK_ID_VI, 0, 0}, dest{RK_ID_VENC, 0, 0};
                        RK_MPI_SYS_UnBind(&src, &dest);
                        modules_bound_ = false;
                    }

                    venc_.reset();
                    vi_chn_raw_.reset();
                    vi_chn_.reset();
                    vi_dev_.reset();
                    isp_.reset();

                    if (rkmpi_initialized_)
                    {
                        RK_MPI_SYS_Exit();
                        rkmpi_initialized_ = false;
                    }

                    LOG_INFO(LOG_TAG, "视频系统已关闭");
                }

                VideoError start_stream()
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    return start_stream_internal();
                }

                VideoError start_stream_internal()
                {
                    if (stream_thread_ && stream_thread_->joinable())
                        return VideoError::ALREADY_STARTED;
                    quit_flag_.store(false);
                    stream_thread_ =
                        std::make_unique<std::thread>(&Impl::stream_process_thread, this);
                    LOG_INFO(LOG_TAG, "视频流已启动");
                    return VideoError::NONE;
                }

                VideoError stop_stream()
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    return stop_stream_internal();
                }

                VideoError stop_stream_internal()
                {
                    if (!stream_thread_ || !stream_thread_->joinable())
                        return VideoError::NOT_STARTED;
                    quit_flag_.store(true);
                    stream_thread_->join();
                    stream_thread_.reset();
                    LOG_INFO(LOG_TAG, "视频流已停止");
                    return VideoError::NONE;
                }

                void stream_process_thread()
                {
                    auto last_stat_time = std::chrono::steady_clock::now();
                    int  frame_count    = 0;

                    while (!quit_flag_.load(std::memory_order_acquire))
                    {
                        VideoFramePtr frame;
                        VideoError    err = config_.enable_dma_zero_copy
                                                ? venc_->get_stream_zero_copy(frame, 100)
                                                : venc_->get_stream(frame, memory_pool_, 100);

                        if (err == VideoError::TIMEOUT)
                            continue;
                        if (err != VideoError::NONE || !frame)
                            continue;

                        frame_count++;
                        stats_.frames_captured.fetch_add(1);

                        frame->timestamp =
                            sync_ctx_ ? sync_get_timestamp(sync_ctx_.get(), frame->pts, false)
                                      : frame->pts;

                        // 拍照处理
                        if (is_photo_capturing_.load())
                        {
                            handle_photo_frame(frame);
                        }

                        // 录像处理
                        if (is_recording_.load())
                        {
                            handle_record_frame(frame);
                        }

                        // RTSP处理
                        if (is_rtsp_streaming_.load())
                        {
                            handle_rtsp_frame(frame);
                        }

                        // FPS计算
                        auto current_time = std::chrono::steady_clock::now();
                        auto elapsed      = std::chrono::duration_cast<std::chrono::microseconds>(
                                           current_time - last_stat_time)
                                           .count();

                        if (elapsed >= 1000000)
                        {
                            current_fps_ = static_cast<float>(frame_count) * 1000000.0f /
                                           static_cast<float>(elapsed);
                            last_stat_time = current_time;
                            frame_count    = 0;
                        }
                    }
                }

                void handle_photo_frame(VideoFramePtr& frame)
                {
                    photo_skip_count_++;
                    if (photo_skip_count_ < config_.photo_skip_frames)
                        return;

                    std::string filename =
                        photo_filename_.empty()
                            ? config_.photo_path + "photo_" + std::to_string(photo_id_++) + ".jpg"
                            : photo_filename_;

                    FileWrapper file(filename, FileMode::WRITE);
                    if (file.valid() && file.write(frame->data, frame->size))
                    {
                        file.flush();
                        LOG_INFO(LOG_TAG, "照片已保存: %s", filename.c_str());
                        stats_.photos_taken.fetch_add(1);
                    }

                    photo_skip_count_ = 0;
                    is_photo_capturing_.store(false);
                    photo_filename_.clear();
                }

                VideoError take_photo(const std::string& filename)
                {
                    if (is_photo_capturing_.load())
                        return VideoError::ALREADY_STARTED;

                    photo_filename_   = filename;
                    photo_skip_count_ = 0;
                    is_photo_capturing_.store(true);

                    LOG_INFO(LOG_TAG, "开始拍照...");
                    return VideoError::NONE;
                }

                void handle_record_frame(VideoFramePtr& frame)
                {
                    std::lock_guard<std::mutex> lock(record_mutex_);
                    if (record_file_ && record_file_->valid())
                    {
                        if (record_file_->write(frame->data, frame->size))
                        {
                            record_file_->flush();
                            auto elapsed =
                                std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - record_start_time_)
                                    .count();
                            stats_.record_duration_ms.store(static_cast<uint64_t>(elapsed));

                            if (record_duration_sec_ > 0 && elapsed >= record_duration_sec_ * 1000)
                                stop_record_internal();
                        }
                    }
                }

                VideoError start_record(const std::string& filename, int duration_sec)
                {
                    std::lock_guard<std::mutex> lock(record_mutex_);
                    if (is_recording_.load())
                        return VideoError::ALREADY_STARTED;

                    std::string record_filename = filename.empty()
                                                      ? config_.record_path + "record_" +
                                                            std::to_string(record_id_++) + ".h264"
                                                      : filename;

                    record_file_ = std::make_unique<FileWrapper>(record_filename, FileMode::WRITE);
                    if (!record_file_->valid())
                    {
                        record_file_.reset();
                        return VideoError::FILE_OPEN_FAILED;
                    }

                    record_duration_sec_ = duration_sec;
                    record_start_time_   = std::chrono::steady_clock::now();
                    is_recording_.store(true);

                    LOG_INFO(LOG_TAG, "录像已启动: %s", record_filename.c_str());
                    return VideoError::NONE;
                }

                VideoError stop_record()
                {
                    std::lock_guard<std::mutex> lock(record_mutex_);
                    return stop_record_internal();
                }

                VideoError stop_record_internal()
                {
                    if (!is_recording_.load())
                        return VideoError::NOT_STARTED;

                    is_recording_.store(false);

                    if (record_file_)
                    {
                        record_file_->flush();
                        record_file_.reset();
                    }

                    LOG_INFO(LOG_TAG, "录像已停止");
                    return VideoError::NONE;
                }

                void handle_rtsp_frame(VideoFramePtr& frame)
                {
                    std::lock_guard<std::mutex> lock(rtsp_mutex_);
                    if (rtsp_demo_ && rtsp_session_)
                    {
                        uint64_t ts = rtsp_get_reltime();
                        rtsp_tx_video(rtsp_session_, frame->data, static_cast<int>(frame->size),
                                      ts);
                        rtsp_do_event(rtsp_demo_);
                    }
                }

                VideoError start_rtsp(int port, const std::string& path)
                {
                    if (is_rtsp_streaming_.load())
                        return VideoError::ALREADY_STARTED;
                    std::lock_guard<std::mutex> lock(rtsp_mutex_);

                    rtsp_demo_ = rtsp_new_demo(port);
                    if (!rtsp_demo_)
                    {
                        LOG_ERROR(LOG_TAG, "创建RTSP服务器失败");
                        return VideoError::INIT_FAILED;
                    }

                    rtsp_session_ = rtsp_new_session(rtsp_demo_, path.c_str());
                    if (!rtsp_session_)
                    {
                        LOG_ERROR(LOG_TAG, "创建RTSP会话失败");
                        rtsp_del_demo(rtsp_demo_);
                        rtsp_demo_ = nullptr;
                        return VideoError::INIT_FAILED;
                    }

                    int codec_id = RTSP_CODEC_ID_VIDEO_H264;
                    if (config_.format == EncodeFormat::H265)
                        codec_id = RTSP_CODEC_ID_VIDEO_H265;
                    else if (config_.format == EncodeFormat::JPEG)
                    {
                        LOG_ERROR(LOG_TAG, "RTSP不支持JPEG格式");
                        rtsp_del_session(rtsp_session_);
                        rtsp_del_demo(rtsp_demo_);
                        rtsp_session_ = nullptr;
                        rtsp_demo_    = nullptr;
                        return VideoError::NOT_SUPPORTED;
                    }

                    if (rtsp_set_video(rtsp_session_, codec_id, nullptr, 0) != 0)
                    {
                        LOG_ERROR(LOG_TAG, "设置RTSP视频编码器失败");
                        rtsp_del_session(rtsp_session_);
                        rtsp_del_demo(rtsp_demo_);
                        rtsp_session_ = nullptr;
                        rtsp_demo_    = nullptr;
                        return VideoError::INIT_FAILED;
                    }

                    rtsp_sync_video_ts(rtsp_session_, rtsp_get_reltime(), rtsp_get_ntptime());

                    is_rtsp_streaming_.store(true);

                    LOG_INFO(LOG_TAG, "RTSP推流已启动: rtsp://<ip>:%d%s", port, path.c_str());
                    return VideoError::NONE;
                }

                VideoError stop_rtsp()
                {
                    std::lock_guard<std::mutex> lock(rtsp_mutex_);
                    return stop_rtsp_internal();
                }

                VideoError stop_rtsp_internal()
                {
                    if (!is_rtsp_streaming_.load())
                        return VideoError::NOT_STARTED;

                    if (rtsp_session_)
                    {
                        rtsp_del_session(rtsp_session_);
                        rtsp_session_ = nullptr;
                    }

                    if (rtsp_demo_)
                    {
                        rtsp_del_demo(rtsp_demo_);
                        rtsp_demo_ = nullptr;
                    }

                    is_rtsp_streaming_.store(false);

                    LOG_INFO(LOG_TAG, "RTSP推流已停止");
                    return VideoError::NONE;
                }

                VideoError get_raw_frame(RawVideoFramePtr& frame, int timeout_ms)
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!vi_chn_raw_ || !vi_chn_raw_->valid())
                        return VideoError::NOT_INITIALIZED;
                    return vi_chn_raw_->get_raw_frame(frame, memory_pool_, timeout_ms);
                }

                void get_stats(VideoSystem::Stats& out_stats) const
                {
                    memory_pool_.get_stats(out_stats.mem_stats);
                    out_stats.frames_captured    = stats_.frames_captured.load();
                    out_stats.photos_taken       = stats_.photos_taken.load();
                    out_stats.record_duration_ms = stats_.record_duration_ms.load();
                }

                void log_stats() const
                {
                    LOG_INFO(LOG_TAG, "统计: 帧=%zu, 照片=%zu, 录像=%zums",
                             stats_.frames_captured.load(), stats_.photos_taken.load(),
                             stats_.record_duration_ms.load());
                    memory_pool_.log_stats();
                }

                float get_current_fps() const
                {
                    return current_fps_;
                }

                VideoConfig     config_;
                VideoMemoryPool memory_pool_;

                std::unique_ptr<ISPWrapper>       isp_;
                std::unique_ptr<VIDeviceWrapper>  vi_dev_;
                std::unique_ptr<VIChannelWrapper> vi_chn_;
                std::unique_ptr<VIChannelWrapper> vi_chn_raw_;
                std::unique_ptr<VENCWrapper>      venc_;

                std::shared_ptr<sync_context_t> sync_ctx_;

                std::unique_ptr<std::thread> stream_thread_;
                std::atomic<bool>            quit_flag_;
                std::atomic<float>           current_fps_;

                std::atomic<bool> is_photo_capturing_{false};
                int               photo_skip_count_ = 0;
                int               photo_id_;
                std::string       photo_filename_;

                std::atomic<bool>                     is_recording_{false};
                std::unique_ptr<FileWrapper>          record_file_;
                int                                   record_id_;
                int                                   record_duration_sec_ = 0;
                std::chrono::steady_clock::time_point record_start_time_;
                std::mutex                            record_mutex_;

                std::atomic<bool>   is_rtsp_streaming_{false};
                rtsp_demo_handle    rtsp_demo_    = nullptr;
                rtsp_session_handle rtsp_session_ = nullptr;
                std::mutex          rtsp_mutex_;

                bool rkmpi_initialized_ = false;
                bool modules_bound_     = false;

                struct
                {
                    std::atomic<uint64_t> frames_captured{0};
                    std::atomic<uint64_t> photos_taken{0};
                    std::atomic<uint64_t> record_duration_ms{0};
                } stats_;

                mutable std::mutex mutex_;
            };

            // ============================================================================
            // VideoSystem公开接口
            // ============================================================================

            VideoSystem::VideoSystem(const VideoConfig& config)
                : pImpl_(std::make_unique<Impl>(config))
            {
            }

            VideoSystem::~VideoSystem()
            {
                deinit();
            }

            VideoError VideoSystem::init(std::shared_ptr<sync_context_t> sync_ctx)
            {
                VideoError err = pImpl_->init(sync_ctx);
                if (err == VideoError::NONE)
                    is_initialized_.store(true);
                return err;
            }

            void VideoSystem::deinit()
            {
                if (is_initialized_.load())
                {
                    pImpl_->deinit();
                    is_initialized_.store(false);
                }
            }

            VideoError VideoSystem::start_stream()
            {
                VideoError err = pImpl_->start_stream();
                if (err == VideoError::NONE)
                    is_streaming_.store(true);
                return err;
            }

            VideoError VideoSystem::stop_stream()
            {
                VideoError err = pImpl_->stop_stream();
                if (err == VideoError::NONE)
                    is_streaming_.store(false);
                return err;
            }

            VideoError VideoSystem::take_photo(const std::string& filename)
            {
                if (!is_init())
                    return VideoError::NOT_INITIALIZED;
                VideoError err = pImpl_->take_photo(filename);
                if (err == VideoError::NONE)
                    is_photo_capturing_.store(true);
                return err;
            }

            VideoError VideoSystem::start_record(const std::string& filename, int duration_sec)
            {
                if (!is_init())
                    return VideoError::NOT_INITIALIZED;
                VideoError err = pImpl_->start_record(filename, duration_sec);
                if (err == VideoError::NONE)
                    is_recording_.store(true);
                return err;
            }

            VideoError VideoSystem::stop_record()
            {
                VideoError err = pImpl_->stop_record();
                if (err == VideoError::NONE)
                    is_recording_.store(false);
                return err;
            }

            VideoError VideoSystem::start_rtsp(int port, const std::string& path)
            {
                if (!is_init())
                    return VideoError::NOT_INITIALIZED;
                if (!is_streaming())
                {
                    VideoError err = start_stream();
                    if (err != VideoError::NONE)
                        return err;
                }
                VideoError err = pImpl_->start_rtsp(port, path);
                if (err == VideoError::NONE)
                    is_rtsp_streaming_.store(true);
                return err;
            }

            VideoError VideoSystem::stop_rtsp()
            {
                pImpl_->stop_rtsp();
                is_rtsp_streaming_.store(false);
                return VideoError::NONE;
            }

            float VideoSystem::get_current_fps() const
            {
                return pImpl_->get_current_fps();
            }

            VideoError VideoSystem::get_raw_frame(RawVideoFramePtr& frame, int timeout_ms)
            {
                return is_init() ? pImpl_->get_raw_frame(frame, timeout_ms)
                                 : VideoError::NOT_INITIALIZED;
            }

            VideoError VideoSystem::set_bitrate(int bitrate_kbps)
            {
                if (!is_init() || !pImpl_->venc_ || !pImpl_->venc_->valid())
                    return VideoError::NOT_INITIALIZED;
                return pImpl_->venc_->set_bitrate(bitrate_kbps);
            }

            VideoError VideoSystem::set_gop(int gop)
            {
                if (!is_init() || !pImpl_->venc_ || !pImpl_->venc_->valid())
                    return VideoError::NOT_INITIALIZED;
                return pImpl_->venc_->set_gop(gop);
            }

            VideoError VideoSystem::set_jpeg_quality(int quality)
            {
                if (!is_init() || !pImpl_->venc_ || !pImpl_->venc_->valid())
                    return VideoError::NOT_INITIALIZED;
                return pImpl_->venc_->set_jpeg_quality(quality);
            }

            VideoError VideoSystem::set_exposure_mode(opMode_t mode)
            {
                return (is_init() && pImpl_->isp_ && pImpl_->isp_->valid())
                           ? pImpl_->isp_->set_exposure_mode(mode)
                           : VideoError::NOT_INITIALIZED;
            }

            VideoError VideoSystem::set_brightness(unsigned int level)
            {
                return (is_init() && pImpl_->isp_ && pImpl_->isp_->valid())
                           ? pImpl_->isp_->set_brightness(level)
                           : VideoError::NOT_INITIALIZED;
            }

            VideoError VideoSystem::set_contrast(unsigned int level)
            {
                return (is_init() && pImpl_->isp_ && pImpl_->isp_->valid())
                           ? pImpl_->isp_->set_contrast(level)
                           : VideoError::NOT_INITIALIZED;
            }

            VideoError VideoSystem::set_saturation(unsigned int level)
            {
                return (is_init() && pImpl_->isp_ && pImpl_->isp_->valid())
                           ? pImpl_->isp_->set_saturation(level)
                           : VideoError::NOT_INITIALIZED;
            }

            VideoError VideoSystem::set_sharpness(unsigned int level)
            {
                return (is_init() && pImpl_->isp_ && pImpl_->isp_->valid())
                           ? pImpl_->isp_->set_sharpness(level)
                           : VideoError::NOT_INITIALIZED;
            }

            void VideoSystem::get_stats(Stats& out_stats) const
            {
                pImpl_->get_stats(out_stats);
            }

            void VideoSystem::reset_stats()
            {
                pImpl_->stats_.frames_captured.store(0);
                pImpl_->stats_.photos_taken.store(0);
                pImpl_->stats_.record_duration_ms.store(0);
                pImpl_->memory_pool_.reset_stats();
            }

            void VideoSystem::log_stats() const
    {
        pImpl_->log_stats();
    }

} // namespace app::media::camera
