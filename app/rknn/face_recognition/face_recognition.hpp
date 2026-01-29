/**
 * @file face_recognition.hpp
 * @brief 人脸识别模块
 */

#ifndef FACE_RECOGNITION_HPP
#define FACE_RECOGNITION_HPP

#include <vector>
#include <cstdint>
#include <cmath>
#include <string>
#include "../rknn.hpp"
#include "../rknn_config.hpp"
#include "../../tool/log/log.hpp"

namespace app::rknn::face_recognition
{
            using namespace tool::log;

            /**
             * @brief 人脸特征向量
             */
            struct FaceFeature
            {
                std::vector<float> data;
                size_t             size() const
                {
                    return data.size();
                }

                /**
                 * @brief L2归一化
                 */
                void normalize()
                {
                    float norm = 0.0f;
                    for (float val : data)
                    {
                        norm += val * val;
                    }
                    norm = std::sqrt(norm);
                    if (norm > FaceRecognitionConst::NORMALIZE_THRESHOLD)
                    {
                        for (float& val : data)
                        {
                            val /= norm;
                        }
                    }
                }
            };

            /**
             * @brief 识别结果
             */
            struct RecognitionResult
            {
                int   person_id  = -1;
                float similarity = 0.0f;
                bool  recognized = false;
            };

            /**
             * @brief 人脸识别配置
             */
            using RecognitionConfig = FaceRecognitionConfig::RecognitionConfig;

            /**
             * @brief 人脸识别类
             */
            class FaceRecognition
            {
            public:
                /**
                 * @brief 构造函数
                 */
                FaceRecognition();

                /**
                 * @brief 析构函数
                 */
                ~FaceRecognition();

                // 禁用拷贝和移动
                FaceRecognition(const FaceRecognition&)            = delete;
                FaceRecognition& operator=(const FaceRecognition&) = delete;
                FaceRecognition(FaceRecognition&&)                 = delete;
                FaceRecognition& operator=(FaceRecognition&&)      = delete;

                /**
                 * @brief 初始化识别模型
                 * @param model_path 模型文件路径
                 * @param config 识别配置
                 * @return 错误码
                 */
                RKNNError init(const std::string&       model_path,
                               const RecognitionConfig& config = RecognitionConfig());

                /**
                 * @brief 反初始化
                 */
                void deinit();

                /**
                 * @brief 检查是否已初始化
                 */
                bool is_init() const
                {
                    return initialized_;
                }

                /**
                 * @brief 提取人脸特征
                 * @param image_data 输入图像数据
                 * @param width 图像宽度
                 * @param height 图像高度
                 * @param feature 输出特征向量
                 * @return 错误码
                 */
                RKNNError extract_feature(const uint8_t* image_data, int width, int height,
                                          FaceFeature& feature);

                /**
                 * @brief 提取人脸特征
                 * @param image_data 输入图像数据
                 * @param feature 输出特征向量
                 * @return 错误码
                 */
                RKNNError extract_feature(const uint8_t* image_data, FaceFeature& feature);

                /**
                 * @brief 计算两个特征的相似度
                 * @param feature1 特征1
                 * @param feature2 特征2
                 * @return 相似度
                 */
                static float calc_similarity(const FaceFeature& feature1,
                                             const FaceFeature& feature2);

                /**
                 * @brief 识别
                 * @param query_feature 查询特征
                 * @param feature_db 特征库
                 * @param result 识别结果
                 * @return 错误码
                 */
                RKNNError recognize(const FaceFeature&                              query_feature,
                                    const std::vector<std::pair<int, FaceFeature>>& feature_db,
                                    RecognitionResult&                              result);

                /**
                 * @brief 获取模型输入宽度
                 */
                int get_model_width() const
                {
                    return model_ ? model_->get_model_width() : 0;
                }

                /**
                 * @brief 获取模型输入高度
                 */
                int get_model_height() const
                {
                    return model_ ? model_->get_model_height() : 0;
                }

                /**
                 * @brief 获取模型输入通道数
                 */
                int get_model_channel() const
                {
                    return model_ ? model_->get_model_channel() : 0;
                }

                /**
                 * @brief 获取特征维度
                 */
                int get_feature_dim() const
                {
                    return config_.feature_dim;
                }

                /**
                 * @brief 设置识别配置
                 */
                void set_config(const RecognitionConfig& config)
                {
                    config_ = config;
                }

                /**
                 * @brief 获取识别配置
                 */
                const RecognitionConfig& get_config() const
                {
                    return config_;
                }

                /**
                 * @brief 分配临时缓冲区
                 * @param size 缓冲区大小（字节）
                 * @return 缓冲区指针，失败返回nullptr
                 */
                void* alloc_temp_buf(size_t size);

                /**
                 * @brief 释放临时缓冲区
                 * @param ptr 缓冲区指针
                 */
                void dealloc_temp_buf(void* ptr);

            private:
                void preprocess_image(const uint8_t* src_data, int src_width, int src_height,
                                      uint8_t* dst_data, int dst_width, int dst_height);

                inline float dequantize(int8_t qnt, int32_t zp, float scale) const
                {
                    return (static_cast<float>(qnt) - static_cast<float>(zp)) * scale;
                }

            private:
                std::unique_ptr<RKNNModel> model_;
                RecognitionConfig          config_;
                bool                       initialized_;
            };

} // namespace app::rknn::face_recognition

#endif // FACE_RECOGNITION_HPP
